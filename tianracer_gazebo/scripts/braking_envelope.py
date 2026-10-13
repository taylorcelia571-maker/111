"""Cyclic delayed-braking reachability; assumes the configured braking is achievable."""
import math
import numpy as np


def delayed_envelope(caps, ds, braking, acceleration, reaction):
    caps=np.asarray(caps,dtype=float)
    if caps.ndim!=1 or len(caps)<3 or not np.isfinite(caps).all() or np.any(caps<=0):
        raise ValueError('Need positive finite cyclic speed caps')
    if not all(math.isfinite(v) and v>0 for v in (ds,braking,acceleration)) or not math.isfinite(reaction) or not 0<=reaction<=.5:
        raise ValueError('Invalid delayed braking budget')
    speed=caps.copy()
    # Each downstream constraint reserves coasting once, then constant braking:
    # d >= v*reaction + (v*v-u*u)/(2*b), when v>u.
    horizon=float(caps.max())*reaction+float(caps.max())**2/(2*braking)
    count=min(len(caps)-1,int(math.ceil(horizon/ds)))
    for offset in range(1,count+1):
        target=np.roll(caps,-offset)
        reachable=-braking*reaction+np.sqrt((braking*reaction)**2+target**2+2*braking*offset*ds)
        speed=np.minimum(speed,np.maximum(target,reachable))
    # Forward acceleration propagation across the seam. Never raise a cap.
    for _ in range(2):
        for i in range(len(caps)):
            speed[i]=min(speed[i],math.sqrt(speed[(i-1)%len(caps)]**2+2*acceleration*ds))
    return speed
