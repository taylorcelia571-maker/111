"""Simulation-time summaries of controller behavior, not official race results."""
import math
from collections import deque


class CycleTiming:
    """Bounded wall-time diagnostics, separate from simulation race timing."""
    def __init__(self):
        self.rows = {}

    def observe(self, **seconds):
        for name, value in seconds.items():
            if not math.isfinite(value) or value < 0:
                continue
            row = self.rows.setdefault(name, dict(count=0, total=0., peak=0., recent=deque(maxlen=256)))
            row['count'] += 1
            row['total'] += value
            row['peak'] = max(row['peak'], value)
            row['recent'].append(value)

    def report(self):
        return {name: dict(samples=row['count'], mean_ms=1000*row['total']/row['count'],
                           peak_ms=1000*row['peak'],
                           recent_p95_ms=1000*sorted(row['recent'])[int(.95*(len(row['recent'])-1))])
                for name, row in self.rows.items()}


class RunMetrics:
    def __init__(self):
        self.elapsed=0.0
        self.last=None
        self.bins={}
        self.reference_loops=[]
        self.peak_speed=0.0
        self.peak_error=0.0

    def observe(self,elapsed,speed,command,error,reason,departure,curve,loops):
        values=(elapsed,speed,command,error,curve,loops)
        if not all(math.isfinite(v) for v in values) or elapsed<self.elapsed:
            raise ValueError('Invalid metrics sample/time')
        dt=elapsed-self.elapsed
        if self.last is not None:
            previous=self.last
            for name in (previous['reason'],'phase:departure' if previous['departure'] else 'phase:tracking',
                         'road:curve' if abs(previous['curve'])>=.15 else 'road:straight_or_gentle',
                         'lap:first_reference' if previous['loops']<1 else 'lap:subsequent_reference'):
                row=self.bins.setdefault(name,dict(seconds=0.0,speed_integral=0.0,command_integral=0.0))
                row['seconds']+=dt
                row['speed_integral']+=.5*(previous['speed']+speed)*dt
                row['command_integral']+=previous['command']*dt
        self.last=dict(speed=speed,command=command,reason=reason,departure=departure,curve=curve,loops=loops)
        self.elapsed=elapsed
        self.peak_speed=max(self.peak_speed,speed)
        self.peak_error=max(self.peak_error,error)
        changed=False
        while loops>=len(self.reference_loops)+1:
            self.reference_loops.append(elapsed);changed=True
        return changed

    def report(self):
        durations=[];previous=0.0
        for end in self.reference_loops:
            durations.append(end-previous);previous=end
        return dict(observed_simulation_seconds=self.elapsed,official_result=False,
                    interval_attribution='previous command/reason; trapezoidal measured speed',
                    reference_loop_seconds=durations,peak_measured_speed=self.peak_speed,
                    peak_reference_tracking_error=self.peak_error,
                    time_by_state={name:dict(seconds=row['seconds'],
                        mean_measured_speed=row['speed_integral']/row['seconds'] if row['seconds'] else 0,
                        mean_command_speed=row['command_integral']/row['seconds'] if row['seconds'] else 0)
                        for name,row in self.bins.items()})
