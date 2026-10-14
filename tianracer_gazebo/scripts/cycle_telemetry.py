"""Bounded nonblocking JSONL telemetry. Observes control, never issues commands."""
import copy
import json
import math
import os
import queue
import threading


def finite_json(value):
    if isinstance(value, dict):
        return {str(k): finite_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [finite_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class CycleTelemetry:
    def __init__(self, filename, capacity=2048):
        self.filename = os.path.expanduser(filename)
        os.makedirs(os.path.dirname(os.path.abspath(self.filename)), exist_ok=True)
        # Never silently replace an earlier experiment's evidence.
        self.stream = open(self.filename, 'x')
        self.queue = queue.Queue(maxsize=capacity)
        self.accepted = self.written = self.dropped = 0
        self.error = None
        self.closed = False
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def append(self, row):
        if self.closed or self.error is not None:
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait(copy.deepcopy(row))
            self.accepted += 1
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _write(self):
        try:
            while True:
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    if self.closed:
                        break
                    continue
                try:
                    if isinstance(row, threading.Event):
                        self.stream.flush()
                        row.set()
                        continue
                    self.stream.write(json.dumps(finite_json(row), allow_nan=False,
                                                  separators=(',', ':')) + '\n')
                    self.written += 1
                    if self.written % 50 == 0:
                        self.stream.flush()
                finally:
                    self.queue.task_done()
        except (OSError, ValueError, TypeError) as error:
            self.error = str(error)
        finally:
            self.stream.close()

    def close(self, timeout=2.0):
        self.closed = True
        self.worker.join(timeout=timeout)
        return not self.worker.is_alive()

    def flush(self, timeout=2.0):
        if self.error is not None or self.closed:
            return False
        signal = threading.Event()
        try:
            self.queue.put(signal, timeout=timeout)
        except queue.Full:
            return False
        return signal.wait(timeout)

    def status(self):
        return dict(filename=self.filename, accepted=self.accepted,
                    written=self.written, dropped=self.dropped, error=self.error,
                    flushed=self.closed and not self.worker.is_alive())
