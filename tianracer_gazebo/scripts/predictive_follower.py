"""Forward-only closed-loop tracking and reference-corridor laser clearance."""
import math
import copy
import numpy as np
from race_line import LineFollower, positive


class PredictiveFollower(LineFollower):
    def __init__(self, path, max_speed=2.0, lateral_accel=5.0, acceleration=2.0,
                 braking=3.0, max_error=0.15, wheelbase=0.26, max_steer=0.48,
                 steering_rate=2.0, yaw_rate=3.2, fast_start=False, start_blend=False,
                 response_delay=0.0, lateral_control='pursuit',
                 startup_speed=1.2, startup_acceleration=1.5):
        super().__init__(path, max_speed, lateral_accel, acceleration, braking,
                         wheelbase, max_steer, max_error)
        self.steering_rate = positive('steering_rate', steering_rate)
        self.yaw_rate = positive('yaw_rate', yaw_rate)
        if not math.isfinite(response_delay) or not 0 <= response_delay <= 0.5:
            raise ValueError('response_delay must be between 0 and 0.5 seconds')
        self.response_delay = float(response_delay)
        if lateral_control not in ('pursuit','smooth_pursuit','curvature_feedback'):
            raise ValueError('Unknown lateral_control: '+str(lateral_control))
        if lateral_control == 'curvature_feedback' and start_blend:
            raise ValueError('Selected lateral controller requires the unmodified reference path')
        self.lateral_control = lateral_control
        self.measured_steer = None
        self.startup_speed=positive('startup_speed',startup_speed)
        self.startup_acceleration=positive('startup_acceleration',startup_acceleration)
        self.launch_limited=False
        self.launch_ready=0.0
        self.launch_guard_distance=5.0
        self.alignment_ready = 0.0
        curve = np.abs(path.curvature)
        delta = np.arctan(wheelbase*path.curvature)
        derivative = np.maximum(np.abs(np.roll(delta,-1)-delta), np.abs(delta-np.roll(delta,1)))/path.ds
        preview_curve = np.maximum.reduce([np.roll(curve,j) for j in range(-2,3)])
        self.speed_caps = dict(straight=np.full(path.n,max_speed),
                               lateral=np.sqrt(lateral_accel/np.maximum(preview_curve,1e-6)),
                               yaw_rate=yaw_rate/np.maximum(curve,1e-6),
                               steering_rate=steering_rate/np.maximum(derivative,1e-6))
        path.speeds = np.minimum.reduce([path.speeds,self.speed_caps['yaw_rate'],self.speed_caps['steering_rate']])
        for _ in range(2):
            for i in range(path.n-1,-1,-1):
                path.speeds[i] = min(path.speeds[i], math.sqrt(path.speeds[(i+1)%path.n]**2+2*braking*path.ds))
            for i in range(path.n):
                path.speeds[i] = min(path.speeds[i], math.sqrt(path.speeds[(i-1)%path.n]**2+2*acceleration*path.ds))
        self.steer = 0.0
        self.departure = True
        if not isinstance(fast_start,bool): raise ValueError('fast_start must be boolean')
        self.fast_start = fast_start
        self.start_blend = start_blend
        if not isinstance(start_blend,bool): raise ValueError('start_blend must be boolean')
        self.connector_offset = None
        self.connector_length = 2.5
        self.last_report = {}

    def plan_report(self):
        names=list(self.speed_caps)
        caps=np.vstack([self.speed_caps[name] for name in names])
        reason=np.argmin(caps,axis=0)
        envelope=self.path.speeds < caps.min(axis=0)-1e-6
        weights=self.path.ds/self.path.speeds
        fractions={name:float(weights[(reason==i)&~envelope].sum()/weights.sum()) for i,name in enumerate(names)}
        fractions['acceleration_braking']=float(weights[envelope].sum()/weights.sum())
        # Planning-time sensitivity only: no extra optimization in the driving
        # loop. Use the same route and solver, with independent mutable paths.
        budgets=dict(max_speed=self.max_speed,lateral_accel=self.lateral_accel,
                     acceleration=self.acceleration,braking=self.braking,
                     steering_rate=self.steering_rate,yaw_rate=self.yaw_rate)
        sensitivity={}
        for label,key,value in (('max_speed_plus_1','max_speed',self.max_speed+1),
                                ('lateral_plus_10_percent','lateral_accel',self.lateral_accel*1.1),
                                ('acceleration_plus_20_percent','acceleration',self.acceleration*1.2),
                                ('braking_plus_20_percent','braking',self.braking*1.2)):
            trial=dict(budgets);trial[key]=value
            other=PredictiveFollower(copy.deepcopy(self.path),**trial)
            seconds=float(5*np.sum(other.path.ds/other.path.speeds))
            sensitivity[label]=dict(parameter=key,value=value,model_five_seconds=seconds,
                                    model_seconds_saved=float(5*weights.sum()-seconds))
        return dict(length=self.path.length,model_lap_seconds=float(weights.sum()),
                    model_five_seconds=float(5*weights.sum()),model_includes_startup=False,
                    required_mean_speed_for_60s=self.path.length/12,
                    response_delay_seconds=self.response_delay,
                    response_delay_is_measured=False,
                    model_includes_response_preview=False,
                    lateral_control=self.lateral_control,
                    startup_limits=dict(speed=self.startup_speed,acceleration=self.startup_acceleration,
                                        acceleration_guard_distance=self.launch_guard_distance),
                    peak_planned_speed=float(self.path.speeds.max()),
                    speed_ceiling_binding=bool(np.any(self.path.speeds>=self.max_speed-1e-6)),
                    same_route_parameter_sensitivity=sensitivity,
                    sensitivity_is_physical_validation=False,
                    limits=dict(max_speed=self.max_speed,lateral_accel=self.lateral_accel,
                                acceleration=self.acceleration,braking=self.braking,
                                steering_rate=self.steering_rate,yaw_rate=self.yaw_rate),
                    time_fraction_by_limit=fractions)

    def start(self, x, y, yaw):
        super().start(x, y, yaw)
        self.departure = True
        self.alignment_ready = 0.0
        self.connector_offset = None
        self.launch_limited=False
        self.launch_ready=0.0
        self.start_diagnostics = dict(mode='alignment',reason='connector_disabled_or_geometry_not_eligible')
        self.last_report = {}
        tangent=self.path.sample(self.origin+.08)-self.path.sample(self.origin)
        difference=math.atan2(math.sin(math.atan2(tangent[1],tangent[0])-yaw),
                              math.cos(math.atan2(tangent[1],tangent[0])-yaw))
        initial_error=math.hypot(*(np.array([x,y])-self.path.sample(self.origin)))
        ids=np.arange(int(self.origin/self.path.ds),int((self.origin+self.connector_length)/self.path.ds)+1)%self.path.n
        heading_limit=.05 if self.lateral_control=='smooth_pursuit' else .15
        if self.start_blend and initial_error<=.10 and abs(difference)<=heading_limit and np.max(abs(self.path.curvature[ids]))<.20:
            self.connector_offset=np.array([x,y])-self.path.sample(self.origin)
            self.departure=False
            self.start_diagnostics=dict(mode='connector',length=self.connector_length,initial_error=initial_error)
            self.launch_limited=self.lateral_control=='smooth_pursuit'

    def sample_tracking(self, distance):
        point=self.path.sample(distance)
        if self.connector_offset is not None:
            u=float(np.clip((distance-self.origin)/self.connector_length,0,1))
            blend=1-10*u**3+15*u**4-6*u**5
            point=point+blend*self.connector_offset
        return point

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
        reference=self.path.sample(self.progress)
        tangent=self.path.sample(self.progress+.08)-self.path.sample(self.progress-.08)
        connector_active=(self.connector_offset is not None and
                          self.progress-self.origin<self.connector_length)
        control_error=self.error
        if connector_active and self.lateral_control=='smooth_pursuit':
            # Track the validated connector consistently, instead of treating
            # its intentional starting offset as an error to correct urgently.
            reference=self.sample_tracking(self.progress)
            tangent=self.sample_tracking(self.progress+.08)-self.sample_tracking(self.progress-.08)
            control_error=math.hypot(x-reference[0],y-reference[1])
        tangent/=max(float(np.linalg.norm(tangent)),1e-9)
        heading=math.atan2(tangent[1],tangent[0])
        heading_error=math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))
        signed_error=float((x-reference[0])*(-tangent[1])+(y-reference[1])*tangent[0])
        outward=math.copysign(1,signed_error)*math.sin(heading_error) if abs(signed_error)>1e-6 else 0
        recovery_heading_error=heading_error
        recovering=(control_error>.02 and outward>.12) or (control_error>.06 and outward>.05)
        preview = min(0.80, max(0.28, (0.35+0.18*max(0,measured_speed))/(1+0.55*curve)))
        if self.lateral_control=='smooth_pursuit':
            severity=float(np.clip(max(0,control_error-.02)/.04*max(0,outward)/.12,0,1))
            preview+=(min(preview,.30)-preview)*severity
            if self.departure:
                preview=max(preview,.55)
        elif recovering:
            # A long preview can let an outward heading consume the remaining
            # corridor before steering catches up. Tighten only during drift.
            preview=min(preview,.30)
        target = self.sample_tracking(self.progress+preview)
        dx,dy = target-[x,y]
        forward = math.cos(yaw)*dx+math.sin(yaw)*dy
        if forward <= 0: self.stop(); raise ValueError('Reference behind vehicle; reposition instead of reversing')
        k = 2*(-math.sin(yaw)*dx+math.cos(yaw)*dy)/max(dx*dx+dy*dy,1e-6)
        desired_steer = float(np.clip(math.atan(self.wheelbase*k),-self.max_steer,self.max_steer))
        feedforward=None
        if self.lateral_control=='smooth_pursuit':
            advance=min(.25,max(0,measured_speed)*.10)
            index=(self.progress+advance)/self.path.ds
            a=int(math.floor(index));fraction=index-a
            ahead=float((1-fraction)*self.path.curvature[a%self.path.n]+
                        fraction*self.path.curvature[(a+1)%self.path.n])
            current=float(self.path.curvature[i])
            if connector_active:
                def tracking_curvature(distance):
                    a=self.sample_tracking(distance)-self.sample_tracking(distance-.3)
                    b=self.sample_tracking(distance+.3)-self.sample_tracking(distance)
                    area=a[0]*b[1]-a[1]*b[0]
                    return 2*area/max(float(np.linalg.norm(a)*np.linalg.norm(b)*np.linalg.norm(a+b)),1e-9)
                current=tracking_curvature(self.progress)
                ahead=tracking_curvature(self.progress+advance)
            feedforward=.4*(ahead-current)
            k+=feedforward
            desired_steer=float(np.clip(math.atan(self.wheelbase*k),-self.max_steer,self.max_steer))
        if self.lateral_control == 'curvature_feedback':
            # Curvature feedforward plus continuous heading/cross-track feedback.
            # Avoid switching to a short pursuit target during an outward drift:
            # that switch amplifies tiny pose errors into large alternating turns.
            advance=min(.25,max(0,measured_speed)*.10)
            index=(self.progress+advance)/self.path.ds
            a=int(math.floor(index)); fraction=index-a
            feedforward=float((1-fraction)*self.path.curvature[a%self.path.n]+
                              fraction*self.path.curvature[(a+1)%self.path.n])
            correction=-.8*heading_error-math.atan(.8*signed_error/(max(0,measured_speed)+.5))
            desired_steer=float(np.clip(math.atan(self.wheelbase*feedforward)+correction,
                                        -self.max_steer,self.max_steer))
            k=math.tan(desired_steer)/self.wheelbase
        self.steer += float(np.clip(desired_steer-self.steer,-self.steering_rate*dt,self.steering_rate*dt))
        live_lateral = math.sqrt(self.lateral_accel/max(abs(k),1e-6))
        live_yaw = self.yaw_rate/max(abs(k),1e-6)
        # The spatial envelope assumes instantaneous execution. Real wheels keep
        # moving after a lower command: advance the braking envelope by measured
        # travel during the response allowance. Never substitute command speed
        # here, since it can already be much lower than the actual speed.
        preview_distance=max(0,measured_speed)*self.response_delay
        count=min(self.path.n-1,int(math.ceil(preview_distance/self.path.ds)))
        envelope=float(np.min(self.path.speeds[(i+np.arange(count+1))%self.path.n]))
        desired = min(envelope,live_lateral,live_yaw)
        reason = 'planned_envelope'
        if envelope < self.path.speeds[i]-1e-6: reason='response_preview'
        if live_lateral < envelope: reason='tracking_lateral'
        if live_yaw < min(envelope,live_lateral): reason='tracking_yaw'

        # Small localization corrections should not cause repeated strong braking.
        error_factor = max(0.4, 1-0.6*(max(0,control_error-0.05)/max(0.01,self.max_error-0.05))**2)
        desired *= error_factor
        if error_factor < 0.99: reason='tracking_error'
        steering_mismatch=None
        steering_risk=None
        if self.lateral_control=='smooth_pursuit':
            actual=self.steer if self.measured_steer is None else self.measured_steer
            if not math.isfinite(actual):
                self.stop();raise ValueError('Nonfinite measured steering')
            steering_mismatch=abs(desired_steer-actual)
            predicted_error=control_error+abs(math.sin(heading_error))*max(0,measured_speed)*self.response_delay
            steering_risk=float(np.clip((predicted_error-.04)/.06,0,1))
            steering_cap=max(.35,envelope/(1+steering_risk*(max(0,steering_mismatch-.04)/.10)**2))
            if steering_cap<desired:
                desired=steering_cap
                reason='steering_slew'
        if self.lateral_control == 'curvature_feedback':
            # Include command slew even if an offline model has no joint feedback.
            actual=self.steer if self.measured_steer is None else self.measured_steer
            if not math.isfinite(actual):
                self.stop();raise ValueError('Nonfinite measured steering')
            steering_mismatch=abs(desired_steer-actual)
            steering_cap=max(.35,self.max_speed/(1+(max(0,steering_mismatch-.04)/.08)**2))
            heading_cap=max(.35,self.max_speed/(1+(heading_error/.25)**2))
            if min(steering_cap,heading_cap)<desired:
                desired=min(steering_cap,heading_cap)
                reason='steering_alignment'
        recovery_cap=None
        if self.lateral_control in ('smooth_pursuit','curvature_feedback'):
            # Continuous time-to-boundary cap, without the old on/off recovery
            # switch. Preserve the same corridor and reaction allowance.
            remaining=max(0,self.max_error-self.error-.025)
            drift=outward;reaction=.3
            if self.lateral_control=='smooth_pursuit':
                # Smooth the error-side sign across the centerline rather than
                # switching a fixed 1m/s cap on/off for millimeter perturbations.
                drift=max(0,math.sin(heading_error)*math.tanh(signed_error/.02));reaction=.5
            recovery_cap=max(.2,min(self.max_speed,remaining/(reaction*max(drift,1e-6))))
            if recovery_cap<desired:
                desired=recovery_cap
                reason='outward_error_recovery'
        elif recovering:
            remaining=max(0,self.max_error-self.error-.025)
            recovery_cap=max(.2,min(1.0,remaining/(.30*max(outward,.05))))
            if recovery_cap<desired:
                desired=recovery_cap
                reason='outward_error_recovery'
        if self.departure:
            tangent = self.path.sample(self.progress+0.08)-self.path.sample(self.progress)
            heading = math.atan2(tangent[1],tangent[0])
            heading_error = math.atan2(math.sin(heading-yaw),math.cos(heading-yaw))
            ready=self.error<.035 and abs(heading_error)<.15
            if self.lateral_control=='smooth_pursuit':
                ready=(self.error<.025 and abs(heading_error)<.05 and
                       abs(desired_steer)<.12 and abs(desired_steer-self.steer)<.04)
                self.alignment_ready=self.alignment_ready+dt if ready else 0.0
                ready=self.alignment_ready>=.20
            if ready:
                self.departure = False
            else:
                # Establish tracking once at departure, not at each lap seam.
                # Keep the existing corridor/laser limits; never creep outside them.
                departure_limit = 0.35
                if self.fast_start:
                    # Increase pace progressively as lateral and heading alignment
                    # improve. Large errors retain the successful .35m/s fallback.
                    lateral_quality = float(np.clip((0.10-self.error)/0.065,0,1))
                    heading_quality = float(np.clip((0.35-abs(heading_error))/0.20,0,1))
                    departure_limit += 0.65*min(lateral_quality,heading_quality)
                if departure_limit < desired: reason='departure'
                desired = min(desired,departure_limit)
        if self.launch_limited:
            actual=self.steer if self.measured_steer is None else self.measured_steer
            ready=(not connector_active and control_error<.025 and
                   abs(recovery_heading_error)<.05 and abs(desired_steer-actual)<.05)
            self.launch_ready=self.launch_ready+dt if ready else 0.0
            if self.launch_ready>=.3:
                self.launch_limited=False
            else:
                if self.startup_speed<desired:reason='startup_connector'
                desired=min(desired,self.startup_speed)
        active_acceleration=self.acceleration
        if self.lateral_control=='smooth_pursuit' and self.start_blend and (
                self.launch_limited or self.progress-self.origin<self.launch_guard_distance):
            active_acceleration=min(active_acceleration,self.startup_acceleration)
        if self.lateral_control=='smooth_pursuit' and self.departure:
            active_acceleration=min(active_acceleration,2.0)
        acceleration_limit=self.speed+active_acceleration*dt
        if acceleration_limit < desired: reason='acceleration'
        self.speed = min(desired,acceleration_limit)
        self.last_report=dict(reason=reason,planned_speed=float(self.path.speeds[i]),
                              live_curvature=float(k),measured_speed=measured_speed,
                              command_speed=self.speed,steer=self.steer,tracking_error=self.error,
                              start_connector=self.connector_offset is not None and self.progress-self.origin<self.connector_length,
                              outward_heading=outward,heading_error=recovery_heading_error,recovery_speed_cap=recovery_cap)
        self.last_report.update(response_delay=self.response_delay,
                                preview_distance=preview_distance,preview_speed=envelope,
                                speed_excess=max(0,measured_speed-self.speed),
                                lateral_control=self.lateral_control,desired_steer=desired_steer,
                                measured_steer=self.measured_steer,steering_mismatch=steering_mismatch,
                                steering_alignment_risk=steering_risk,
                                feedforward_curvature=feedforward)
        self.last_report.update(preview_length=preview,departure=self.departure,
                                control_tracking_error=control_error,
                                launch_limited=self.launch_limited,launch_ready_seconds=self.launch_ready,
                                startup_speed=self.startup_speed,startup_acceleration=self.startup_acceleration,
                                alignment_ready_seconds=self.alignment_ready,
                                active_acceleration=active_acceleration)
        return self.speed,self.steer


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
