"""Forward-only closed-loop tracking and reference-corridor laser clearance."""
import math
import numpy as np
from race_line import LineFollower, positive


class PredictiveFollower(LineFollower):
    def __init__(self, path, max_speed=2.0, lateral_accel=5.0, acceleration=2.0,
                 braking=3.0, max_error=0.15, wheelbase=0.26, max_steer=0.48,
                 steering_rate=2.0, yaw_rate=3.2):
        super().__init__(path, max_speed, lateral_accel, acceleration, braking,
                         wheelbase, max_steer, max_error)
        self.steering_rate = positive('steering_rate', steering_rate)
        positive('yaw_rate', yaw_rate)
        curve = np.abs(path.curvature)
        delta = np.arctan(wheelbase*path.curvature)
        derivative = np.maximum(np.abs(np.roll(delta,-1)-delta), np.abs(delta-np.roll(delta,1)))/path.ds
        path.speeds = np.minimum.reduce([path.speeds, yaw_rate/np.maximum(curve,1e-6),
                                        steering_rate/np.maximum(derivative,1e-6)])
        for _ in range(2):
            for i in range(path.n-1,-1,-1):
                path.speeds[i] = min(path.speeds[i], math.sqrt(path.speeds[(i+1)%path.n]**2+2*braking*path.ds))
            for i in range(path.n):
                path.speeds[i] = min(path.speeds[i], math.sqrt(path.speeds[(i-1)%path.n]**2+2*acceleration*path.ds))
        self.steer = 0.0
        self.departure = True

    def start(self, x, y, yaw):
        super().start(x, y, yaw)
        self.departure = True

    def stop(self):
        super().stop()
        self.steer = 0.0

    def step(self, x, y, yaw, measured_speed, dt, projection_dt=None):
        if not all(math.isfinite(v) for v in (x,y,yaw,measured_speed,dt)) or not 0 < dt <= 0.2:
            self.stop(); raise ValueError('Invalid feedback/control step: dt=%.6fs, pose=(%s,%s,%s), speed=%s' % (dt,x,y,yaw,measured_speed))
        elapsed = dt if projection_dt is None else projection_dt
        if not math.isfinite(elapsed) or not 0 < elapsed <= 0.3:
            self.stop(); raise ValueError('Invalid projection interval: %.6fs' % elapsed)
        if measured_speed < -0.1:
            self.stop(); raise ValueError('Unexpected backward vehicle motion')
        if self.progress is None: self.start(x,y,yaw)
        s,self.error = self.path.project(x,y,self.progress-0.1,self.progress+self.max_speed*elapsed*2+0.20)
        if self.error > self.max_error:
            global_s,global_error = self.path.project(x,y)
            details = ('error=%.4fm limit=%.3fm pose=(%.4f,%.4f,%.4f) '
                       'progress=%.4fm projected=%.4fm global_projected=%.4fm global_error=%.4fm '
                       'measured_speed=%.4f command_speed=%.4f steer=%.4f dt=%.6fs elapsed=%.6fs') % (
                       self.error,self.max_error,x,y,yaw,self.progress,s,global_s,global_error,
                       measured_speed,self.speed,self.steer,dt,elapsed)
            self.stop(); raise ValueError('Tracking error exceeds safe corridor: '+details)
        self.progress = max(s,self.progress)
        i = int(self.progress/self.path.ds)%self.path.n
        # Velocity-scaled preview is short in tight turns, longer on straights.
        curve = abs(self.path.curvature[i])
        preview = min(0.80, max(0.28, (0.35+0.18*max(0,measured_speed))/(1+0.55*curve)))
        target = self.path.sample(self.progress+preview)
        dx,dy = target-[x,y]
        forward = math.cos(yaw)*dx+math.sin(yaw)*dy
        if forward <= 0: self.stop(); raise ValueError('Reference behind vehicle; reposition instead of reversing')
        k = 2*(-math.sin(yaw)*dx+math.cos(yaw)*dy)/max(dx*dx+dy*dy,1e-6)
        desired_steer = float(np.clip(math.atan(self.wheelbase*k),-self.max_steer,self.max_steer))
        self.steer += float(np.clip(desired_steer-self.steer,-self.steering_rate*dt,self.steering_rate*dt))
        desired = min(float(self.path.speeds[i]), math.sqrt(self.lateral_accel/max(abs(k),1e-6)))
        # Small localization corrections should not cause repeated strong braking.
        desired *= max(0.4, 1-0.6*(max(0,self.error-0.05)/max(0.01,self.max_error-0.05))**2)
        if self.departure:
            tangent = self.path.sample(self.progress+0.08)-self.path.sample(self.progress)
            heading = math.atan2(tangent[1],tangent[0])
            heading_error = math.atan2(math.sin(heading-yaw),math.cos(heading-yaw))
            if self.error < 0.035 and abs(heading_error) < 0.15:
                self.departure = False
            else:
                # Establish tracking once at departure, not at each lap seam.
                # Keep the existing corridor/laser limits; never creep outside them.
                desired = min(desired,0.35)
        self.speed = min(desired,self.speed+self.acceleration*dt)
        return self.speed,self.steer


def reference_clearance(path, progress, pose, scan, laser_pose, speed, braking=3.0,
                        half_width=0.16, reaction=0.15, diagnostics=None, half_length=0.23):
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
    rear = np.array([path.sample(progress+d) for d in arcs])
    tangents = np.array([path.sample(progress+d+0.04)-path.sample(progress+d-0.04) for d in arcs])
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
