"""Frozen 23b469f scalar laser protection oracle; no production imports."""
import math
import numpy as np

def reference_clearance(path, progress, pose, scan, laser_pose, speed, braking=3.0,
                        half_width=0.16, reaction=0.15, diagnostics=None, half_length=0.23, sample=None):
    """Check laser hits along the *planned* turn, rather than extrapolating one steer."""
    report = diagnostics if diagnostics is not None else {}
    report.clear()
    report['reason'] = 'invalid_scan_metadata'
    r = np.asarray(scan.ranges, dtype=float)
    if len(r)<3 or not math.isfinite(scan.angle_increment) or scan.angle_increment<=0:
        return 0.0
    if not all(math.isfinite(v) for v in (scan.angle_min,scan.range_min,scan.range_max,*laser_pose)):
        return 0.0
    if scan.range_min < 0 or scan.range_max <= scan.range_min: return 0.0
    x,y,yaw,_ = pose
    lx,ly,lyaw = laser_pose
    angles = scan.angle_min+np.arange(len(r))*scan.angle_increment+lyaw
    forward = np.cos(angles)>=0
    valid = np.isfinite(r)&(r>=scan.range_min)&(r<=scan.range_max)
    known_fraction = float(np.mean((valid|np.isposinf(r))[forward])) if forward.any() else 0.0
    report.update(reason='invalid_scan_coverage', known_forward=known_fraction,
                  below_min=int(np.sum(np.isfinite(r)&(r<scan.range_min))),
                  above_max=int(np.sum(np.isfinite(r)&(r>scan.range_max))),
                  nan=int(np.sum(np.isnan(r))), negative_inf=int(np.sum(np.isneginf(r))))
    if known_fraction < 0.9: return 0.0
    report['reason'] = 'clear'
    select = valid  # also protect the sides in a tight turn
    bx = lx+r[select]*np.cos(angles[select]); by = ly+r[select]*np.sin(angles[select])
    obstacles = np.column_stack([x+math.cos(yaw)*bx-math.sin(yaw)*by,
                                 y+math.sin(yaw)*bx+math.cos(yaw)*by])
    horizon = max(0.6,speed*reaction+speed**2/(2*braking)+0.35)
    arcs = np.linspace(0,horizon,int(math.ceil(horizon/0.04))+1)
    sample = path.sample if sample is None else sample
    rear = np.array([sample(progress+d) for d in arcs])
    tangents = np.array([sample(progress+d+0.04)-sample(progress+d-0.04) for d in arcs])
    tangents /= np.maximum(np.linalg.norm(tangents,axis=1)[:,None],1e-9)
    centers = rear+0.13*tangents  # base center lies ahead of rear axle
    actual_center = np.array([x+0.13*math.cos(yaw),y+0.13*math.sin(yaw)])
    centers += (actual_center-centers[0])*np.maximum(0,1-arcs[:,None]/0.6)
    if not len(obstacles): return float('inf')
    # A disk overstates lateral width while understating front/rear extent.
    # Use an oriented rectangle enclosing the existing costmap (.15 square)
    # and TEB (.18 x .10 half-extents) footprints, with additional margin.
    headings = np.arctan2(tangents[:,1],tangents[:,0])
    heading_error = math.atan2(math.sin(yaw-headings[0]),math.cos(yaw-headings[0]))
    headings += heading_error*np.maximum(0,1-arcs/0.6)
    relative = obstacles[:,None,:]-centers[None,:,:]
    longitudinal = relative[:,:,0]*np.cos(headings)+relative[:,:,1]*np.sin(headings)
    lateral = -relative[:,:,0]*np.sin(headings)+relative[:,:,1]*np.cos(headings)
    inside = (np.abs(longitudinal)<=half_length)&(np.abs(lateral)<=half_width)
    hits = np.where(np.any(inside,axis=0))[0]
    if not len(hits): return float('inf')
    report.update(reason='obstacle_in_corridor', hit_arc=float(arcs[hits[0]]),
                  closest=float(np.min(np.linalg.norm(relative,axis=2))))
    free=max(0,float(arcs[hits[0]])-0.20)
    return max(0,math.sqrt((braking*reaction)**2+2*braking*free)-braking*reaction)
