"""Offline motion-chain metrics; flags are evidence, not automatic PID tuning."""
import math
import numpy as np
from wheel_kinematics import equivalent_steer


def series(rows, key):
    usable = [(float(r['t']), float(r[key])) for r in rows if key in r
              and math.isfinite(r['t']) and math.isfinite(r[key])]
    # Keep the last sample at a repeated simulation timestamp.
    unique = dict(usable)
    return np.array(sorted(unique.items()), dtype=float).reshape(-1, 2)


def summarize(samples, max_speed=1.8, max_yaw_rate=3.6, wheelbase=0.26):
    for rows in samples.values():
        times = [row['t'] for row in rows if 't' in row]
        if any(b < a for a,b in zip(times,times[1:])):
            return {'insufficient_data': 'Simulation clock reset during capture; record a new run'}
    result = {}
    cmd = series(samples.get('cmd_vel', []), 'speed')
    angle = series(samples.get('cmd_vel', []), 'steer')
    odom = series(samples.get('odom', []), 'speed')
    actuator = series(samples.get('ackermann', []), 'speed')
    if len(cmd) < 2 or len(angle) < 2:
        return {'insufficient_data': 'Need command speed and steering samples'}
    t, v = cmd[:, 0], np.abs(cmd[:, 1])
    steer = np.interp(t, angle[:, 0], angle[:, 1])
    weights = np.r_[np.diff(t), 0]
    def fraction(mask):
        return float(np.sum(weights[mask]) / max(np.sum(weights), 1e-9))
    for name, mask in [('straight', np.abs(steer) < 0.06), ('corner', np.abs(steer) > 0.12)]:
        item = {'duration_s': float(weights[mask].sum()),
                'command_speed_mean_m_s': float(np.average(v[mask], weights=weights[mask]))
                if weights[mask].sum() > 0 else None}
        if len(odom) > 1:
            valid = mask & (t >= odom[0, 0]) & (t <= odom[-1, 0])
            measured = np.abs(np.interp(t, odom[:, 0], odom[:, 1]))
            target_v = v
            if len(actuator) > 1:
                valid &= (t >= actuator[0, 0]) & (t <= actuator[-1, 0])
                target_v = np.abs(np.interp(t, actuator[:, 0], actuator[:, 1]))
            if weights[valid].sum() > 0:
                item['actual_speed_mean_m_s'] = float(np.average(measured[valid], weights=weights[valid]))
                item['actuator_speed_mean_m_s'] = float(np.average(target_v[valid], weights=weights[valid]))
                item['planner_to_actuator_reduction_mean_m_s'] = float(np.average(
                    np.maximum(0, v[valid]-target_v[valid]), weights=weights[valid]))
                item['speed_deficit_mean_m_s'] = float(np.average(np.maximum(0, target_v[valid]-measured[valid]),
                                                                                weights=weights[valid]))
        result[name] = item
    intervals = np.diff(t)
    result['cmd_interval_p95_s'] = float(np.percentile(intervals, 95))
    result['cmd_interval_max_s'] = float(intervals.max())
    result['cmd_gaps_above_0_2s'] = int(np.sum(intervals > 0.2))
    result['near_zero_command_fraction'] = fraction(v < 0.05)
    result['speed_ceiling_fraction'] = fraction(v >= 0.95*max_speed)
    yaw = v*np.abs(np.tan(steer))/wheelbase
    result['yaw_budget_fraction'] = fraction(yaw >= 0.95*max_yaw_rate)
    dt = np.diff(angle[:, 0])
    rate = np.diff(angle[:, 1])/dt
    result['steering_rate_rms_rad_s'] = float(np.sqrt(np.mean(rate**2)))
    result['steering_step_p95_rad'] = float(np.percentile(np.abs(np.diff(angle[:, 1])), 95))
    # Direction changes can come from real chicanes: do not label all as oscillation.
    significant = np.diff(angle[:, 1])[np.abs(np.diff(angle[:, 1])) > 0.01]
    result['steering_trend_reversals'] = int(np.sum(significant[1:]*significant[:-1] < 0))
    joints = samples.get('joints', [])
    actual = []
    for row in joints:
        if all(k in row for k in ('left', 'right', 't')):
            try:
                value = equivalent_steer(row['left'], row['right'], wheelbase)
                if math.isfinite(value): actual.append({'t': row['t'], 'steer': value})
            except (ValueError, ZeroDivisionError): pass
    hinge = series(actual, 'steer')
    target = series(samples.get('ackermann', []), 'steer')
    if len(hinge) >= 20 and len(target) >= 20 and np.std(target[:, 1]) > 0.02:
        estimates = []
        for lag in np.arange(0, 0.301, 0.01):
            valid = (hinge[:, 0]-lag >= target[0, 0]) & (hinge[:, 0]-lag <= target[-1, 0])
            if valid.sum() >= 20:
                error = hinge[valid, 1]-np.interp(hinge[valid, 0]-lag, target[:, 0], target[:, 1])
                estimates.append((float(np.sqrt(np.mean(error**2))), float(lag)))
        if estimates:
            rmse, lag = min(estimates)
            result['steering_lag_estimate_s'] = lag
            result['steering_tracking_rmse_rad'] = rmse
    return result
