#!/usr/bin/env python3
"""Track a closed path using localization, odometry and live LaserScan."""
from collections import deque
import copy
import json
import math
import os
import threading
import numpy as np
import rospy
import rosgraph
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_srvs.srv import Trigger, TriggerResponse
from race_line import ClosedPath, LineFollower, clearance_speed
from run_metrics import RunMetrics


class ContinuousRace:
    def __init__(self, path=None, clearance_guard=None, start_path_guard=None):
        self.start_path_guard = start_path_guard
        self.clearance_guard = clearance_guard
        self.generated = path is not None
        self.feedback_pose = {}
        self.predict_pose=rospy.get_param('~predict_pose',False) if self.generated else False
        if path is None:
            filename = os.path.expanduser(rospy.get_param('~path_file'))
            path = ClosedPath(np.loadtxt(filename, delimiter=',', comments='#'))
        self.path = path
        self.reference_sha256 = path.fingerprint()
        follower_type = LineFollower
        if self.generated:
            from predictive_follower import PredictiveFollower
            follower_type = PredictiveFollower
        defaults = (('max_speed', 2.0), ('lateral_accel', 5.0), ('acceleration', 2.0),
                    ('braking', 3.0), ('max_error', 0.15)) if self.generated else (
                    ('max_speed', 3.0), ('lateral_accel', 3.0), ('acceleration', 2.5),
                    ('braking', 3.5), ('max_error', 0.6))
        options = {k: rospy.get_param('~' + k, v) for k,v in defaults}
        if self.generated:
            options.update(fast_start=rospy.get_param('~fast_start',False),
                           start_blend=rospy.get_param('~start_blend',False),
                           response_delay=rospy.get_param('~response_delay',0.0),
                           lateral_control=rospy.get_param('~lateral_control','pursuit'),
                           startup_speed=rospy.get_param('~startup_speed',1.2),
                           startup_acceleration=rospy.get_param('~startup_acceleration',1.5),
                           steering_rate=rospy.get_param('~steering_rate',2.0),
                           yaw_rate=rospy.get_param('~yaw_rate',3.2))
        self.follower = follower_type(self.path, **options)
        self.require_joint_feedback=self.generated and self.follower.lateral_control=='curvature_feedback'
        self.laps = rospy.get_param('~laps', 5)
        if isinstance(self.laps, bool) or not isinstance(self.laps, int) or self.laps < 1:
            raise ValueError('laps must be a positive integer')
        self.frame = rospy.get_param('~map_frame', 'map')
        robot = rospy.get_namespace().strip('/')
        self.base = rospy.get_param('~base_frame', robot + '/base_footprint')
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.lock = threading.RLock()
        self.scan = self.odom = None
        self.joints = self.joints_time = None
        self.laser_cache = None
        self.scan_history = deque(maxlen=16)
        self.scan_time = self.odom_time = None
        self.active = False
        self.prepared_steer = None
        self.last_fault = None
        self.start_metrics = {}
        self.started_at = None
        self.run_metrics=None
        self.run_report_file=os.path.expanduser(rospy.get_param('~run_report',''))
        self.report_lock=threading.Lock()
        self.previous = rospy.Time.now()
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.on_scan, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=1)
        if self.require_joint_feedback or (self.generated and self.follower.lateral_control=='smooth_pursuit'):
            self.joint_sub=rospy.Subscriber('joint_states',JointState,self.on_joints,queue_size=1)
        self.start_service = rospy.Service('~start', Trigger, self.start)
        self.prepare_service = rospy.Service('~prepare',Trigger,self.prepare)
        self.stop_service = rospy.Service('~stop', Trigger, self.stop)
        self.status_service = rospy.Service('~status', Trigger, self.status)
        rospy.on_shutdown(lambda: self.stop(None))
        rospy.loginfo('Loaded %.1fm closed path. Idle; prepare referee then call ~start.', self.path.length)
        if self.generated:
            rospy.loginfo('Controller: %s; speed budget %.2fm/s; steering joint feedback required=%s',
                          self.follower.lateral_control,self.follower.max_speed,self.require_joint_feedback)

    def on_joints(self,msg):
        with self.lock:
            self.joints,self.joints_time=msg,rospy.Time.now()

    def on_scan(self, msg):
        with self.lock:
            self.scan, self.scan_time = msg, rospy.Time.now()
            self.scan_history.append(msg)

    def on_odom(self, msg):
        with self.lock:
            self.odom, self.odom_time = msg, rospy.Time.now()

    def sensors(self):
        now = rospy.Time.now()
        if now.to_sec() <= 0:
            raise ValueError('Waiting for initialized ROS clock; start/unpause simulation')
        if self.generated and self.follower.lateral_control=='smooth_pursuit':
            # Optional enhancement to the already-working pursuit adapter.
            # Never retain expired steering, and do not require an extra stream
            # to run the command-slew fallback. Scan/odom/TF guards remain mandatory.
            self.follower.measured_steer=None
            msg=self.joints
            if msg is not None and self.joints_time is not None:
                age=(now-msg.header.stamp).to_sec()
                if msg.header.stamp.to_sec()>0 and -.02<=age<=.3 and -.02<=(now-self.joints_time).to_sec()<=.3:
                    positions=dict(zip(msg.name,msg.position))
                    left=positions.get('left_steering_hinge_joint');right=positions.get('right_steering_hinge_joint')
                    if left is not None and right is not None and all(math.isfinite(v) and abs(v)<=.65 for v in (left,right)):
                        from wheel_kinematics import equivalent_steer
                        self.follower.measured_steer=equivalent_steer(left,right)
        if self.require_joint_feedback:
            msg=self.joints
            if msg is None or self.joints_time is None:
                raise ValueError('Steering joint feedback missing on joint_states')
            age=(now-msg.header.stamp).to_sec()
            if msg.header.stamp.to_sec()<=0 or not -.02<=age<=.3 or not -.02<=(now-self.joints_time).to_sec()<=.3:
                raise ValueError('Steering joint feedback stale: age=%.6fs' % age)
            positions=dict(zip(msg.name,msg.position))
            try:
                left=positions['left_steering_hinge_joint'];right=positions['right_steering_hinge_joint']
                if not all(math.isfinite(v) and abs(v)<=.65 for v in (left,right)):
                    raise ValueError('Invalid steering hinge position')
                from wheel_kinematics import equivalent_steer
                self.follower.measured_steer=equivalent_steer(left,right)
            except KeyError:
                raise ValueError('Steering hinges absent from joint_states')
        for label,msg,received in (('scan',self.scan,self.scan_time),('odom',self.odom,self.odom_time)):
            if msg is None or received is None:
                raise ValueError('%s missing on %s' % (label,rospy.resolve_name(label)))
            received_age = (now-received).to_sec()
            stamp_age = (now-msg.header.stamp).to_sec()
            # Separate topics can arrive just ahead of /clock over ROS transport.
            # Allow only 20ms future skew; keep the 300ms stale-data limit.
            if not -0.02 <= received_age <= 0.3 or not -0.02 <= stamp_age <= 0.3 or msg.header.stamp.to_sec() <= 0:
                raise ValueError('%s timing invalid: now=%.6f stamp=%.6f stamp_age=%.6fs received_age=%.6fs '
                                 '(allowed -0.020..0.300s); check /clock and topic publisher' %
                                 (label,now.to_sec(),msg.header.stamp.to_sec(),stamp_age,received_age))
        stamp = self.listener.getLatestCommonTime(self.frame, self.base)
        age = (now-stamp).to_sec()
        if not -0.02 <= age <= 0.3:
            raise ValueError('Localization TF timing invalid: age=%.6fs (allowed -0.020..0.300s)' % age)
        position, rotation = self.listener.lookupTransform(self.frame, self.base, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        if not all(math.isfinite(v) for v in (*position[:2], yaw, self.odom.twist.twist.linear.x)):
            raise ValueError('Nonfinite localization or odometry')
        offset = 0.13 if self.generated else 0.0
        x,y=position[0]-offset*math.cos(yaw),position[1]-offset*math.sin(yaw)
        speed=self.odom.twist.twist.linear.x
        self.feedback_pose=dict(raw_rear_pose=[x,y,yaw],tf_age_seconds=age,
                                prediction_applied=False)
        angular=getattr(self.odom.twist.twist,'angular',None)
        omega=getattr(angular,'z',None)
        lateral=getattr(self.odom.twist.twist.linear,'y',None)
        self.feedback_pose.update(
            odometry_yaw_rate=omega if omega is not None and math.isfinite(omega) else None,
            odometry_lateral_speed=lateral if lateral is not None and math.isfinite(lateral) else None)
        # Official odometry publishes at 20Hz. Compensate only a short,
        # timestamp-matched acquisition delay using public odometry feedback.
        if self.predict_pose and omega is not None:
            if not math.isfinite(omega): raise ValueError('Nonfinite odometry yaw rate')
            if 0<age<=.08 and abs((self.odom.header.stamp-stamp).to_sec())<=.02:
                from motion_prediction import predict_rear_pose
                base_lateral=getattr(self.odom.twist.twist.linear,'y',None)
                rear_lateral=0 if base_lateral is None else base_lateral-omega*offset
                x,y,yaw=predict_rear_pose(x,y,yaw,speed,omega,age,rear_lateral)
                self.feedback_pose['prediction_applied']=True
        self.feedback_pose['control_rear_pose']=[x,y,yaw]
        return x,y,yaw,speed

    def status(self, request):
        with self.lock:
            state=dict(active=self.active,prepared_steer=self.prepared_steer,control=getattr(self.follower,'last_report',{}),
                       startup=self.start_metrics,feedback=self.feedback_pose,last_fault=self.last_fault)
            if self.run_metrics is not None:state['run_metrics']=self.run_metrics.report()
            try:
                pose = self.sensors()
                state.update(pose=pose,feedback=self.feedback_pose,
                             message='Feedback ready; start still checks controller conflicts and pose')
                return TriggerResponse(True,json.dumps(state))
            except (ValueError,tf.Exception) as error:
                state['sensor_error']=str(error)
                return TriggerResponse(False,json.dumps(state))

    def record_fault(self,error):
        self.last_fault=dict(reason=str(error),simulation_time=rospy.Time.now().to_sec(),
                             control=dict(getattr(self.follower,'last_report',{})),
                             feedback=dict(self.feedback_pose))

    def prepare(self,request):
        """Pre-position steering while stopped; does not drive or start timing."""
        with self.lock:
            if self.active:return TriggerResponse(False,'Already active; stop before preparing')
            if not self.generated or self.follower.lateral_control not in ('smooth_pursuit','curvature_feedback'):
                return TriggerResponse(False,'prepare requires smooth_pursuit or curvature_feedback')
            try:
                publishers,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                topic=rospy.resolve_name('cmd_vel')
                others=[name for published,names in publishers if published==topic
                        for name in names if name!=rospy.get_name()]
                if others:raise ValueError('Other cmd_vel publishers active: '+', '.join(others))
                pose=self.sensors()
                if abs(pose[3])>.05:raise ValueError('Vehicle must be stationary')
                if self.clearance_guard is not None:self.clearance_guard(pose)
                probe=copy.deepcopy(self.follower)
                probe.start(*pose[:3]);probe.stop()
                if self.start_path_guard is not None and probe.connector_offset is not None:
                    if not self.start_path_guard(probe):
                        probe.connector_offset=None;probe.departure=True
                probe.step(*pose,.02)
                self.prepared_steer=probe.last_report['desired_steer']
                msg=Twist();msg.angular.z=self.prepared_steer
                self.pub.publish(msg)  # linear.x remains zero throughout preparation
                return TriggerResponse(True,'Steering prepared with zero speed; arm referee, then call start')
            except (ValueError,tf.Exception,rosgraph.MasterError,rosgraph.MasterFailure,OSError) as error:
                if self.prepared_steer is not None:self.stop(None)
                return TriggerResponse(False,str(error))

    def start(self, request):
        with self.lock:
            if self.active: return TriggerResponse(False, 'Already active')
            try:
                publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
                topic = rospy.resolve_name('cmd_vel')
                others = [name for published, names in publishers if published == topic
                          for name in names if name != rospy.get_name()]
                if others: raise ValueError('Other cmd_vel publishers active: ' + ', '.join(others))
                pose = self.sensors()
                if abs(pose[3]) > 0.05: raise ValueError('Vehicle must be stationary')
                if self.clearance_guard is not None:self.clearance_guard(pose)
                prepared_used=self.prepared_steer is not None
                self.follower.start(*pose[:3])
                self.follower.stop()
                if self.prepared_steer is not None:
                    initial=self.follower.measured_steer
                    if initial is None:initial=self.prepared_steer
                    self.follower.steer=float(np.clip(initial,-self.follower.max_steer,self.follower.max_steer))
                if self.start_path_guard is not None and self.follower.connector_offset is not None:
                    if not self.start_path_guard(self.follower):
                        self.follower.connector_offset=None
                        self.follower.departure=True
                        self.follower.start_diagnostics=dict(mode='alignment',reason='connector_map_clearance_failed')
                self.previous = rospy.Time.now() - rospy.Duration(0.02)
                self.active = True
                self.prepared_steer = None
                self.last_fault = None
                self.started_at=rospy.Time.now()
                self.start_metrics=dict(prepare_used=prepared_used,
                    start_mode=dict(getattr(self.follower,'start_diagnostics',{})),
                    initial_reference_error=self.follower.error)
                self.run_metrics=RunMetrics()
                i=int(self.follower.progress/self.path.ds)%self.path.n
                self.run_metrics.observe(0,pose[3],0,self.follower.error,'startup_wait',
                    getattr(self.follower,'departure',False),float(self.path.curvature[i]),0)
                rospy.loginfo('Race start mode: %s',json.dumps(getattr(self.follower,'start_diagnostics',{})))
                rospy.loginfo('Race start: rear_pose=(%.4f,%.4f,%.4f) measured_v=%.4f progress=%.4fm',
                              *pose, self.follower.progress)
                return TriggerResponse(True, 'Started; official referee determines valid laps')
            except (ValueError, tf.Exception, rosgraph.MasterError, rosgraph.MasterFailure, OSError) as error:
                return TriggerResponse(False, str(error))

    def stop(self, request):
        with self.lock:
            self.active = False
            self.prepared_steer = None
            self.follower.stop()
            self.pub.publish(Twist())
            self.save_run_report()
            return TriggerResponse(True, 'Stopped')

    def save_run_report(self,background=False):
        if not self.run_report_file or self.run_metrics is None:return
        report=self.run_metrics.report()
        report.update(reference_sha256=self.reference_sha256,
                      controller=getattr(self.follower,'lateral_control','legacy'),
                      startup=self.start_metrics,last_fault=self.last_fault,
                      limits={name:getattr(self.follower,name) for name in
                              ('max_speed','lateral_accel','acceleration','braking','max_error',
                               'yaw_rate','steering_rate','response_delay') if hasattr(self.follower,name)})
        report=copy.deepcopy(report)
        if background:
            # Never wait for storage in the 50Hz control loop. Skip a periodic
            # snapshot when the previous writer is busy; stop saves the final one.
            if not self.report_lock.acquire(blocking=False):return
            def write():
                try:self.write_run_report(report)
                finally:self.report_lock.release()
            worker=threading.Thread(target=write,daemon=True)
            try:worker.start()
            except RuntimeError as error:
                self.report_lock.release();rospy.logwarn('Could not start report writer: %s',error)
        else:
            with self.report_lock:self.write_run_report(report)

    def write_run_report(self,report):
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.run_report_file)),exist_ok=True)
            temporary=self.run_report_file+'.tmp'
            with open(temporary,'w') as stream:json.dump(report,stream,indent=2,sort_keys=True)
            os.replace(temporary,self.run_report_file)
        except OSError as error:
            rospy.logwarn('Could not save controller run report: %s',error)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                if self.active:
                    try:
                        now = rospy.Time.now()
                        dt = (now - self.previous).to_sec()
                        if not math.isfinite(dt) or dt < 0:
                            raise ValueError('Control clock invalid/reset: dt=%.6fs' % dt)
                        # /clock can repeat while paused or between Gazebo updates.
                        # Do not integrate, advance progress, or latch a fault at dt=0.
                        if dt > 0:
                            self.control_cycle(now, dt)
                    except (ValueError, tf.Exception) as error:
                        self.record_fault(error)
                        self.stop(None)
                        rospy.logerr('Race stopped: %s; last_feedback=%s', error,json.dumps(self.feedback_pose))
                elif self.prepared_steer is not None:
                    try:
                        pose=self.sensors()
                        if abs(pose[3])>.05:raise ValueError('Vehicle moved during stationary preparation')
                        if self.clearance_guard is not None:self.clearance_guard(pose)
                        msg=Twist();msg.angular.z=self.prepared_steer
                        self.pub.publish(msg)
                    except (ValueError,tf.Exception) as error:
                        self.record_fault(error);self.stop(None)
                        rospy.logerr('Preparation stopped: %s',error)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.record_fault('ROS simulation clock moved backwards')
                self.stop(None)
                rate = rospy.Rate(50)

    def laser_feedback(self, now):
        # Select the newest short-age scan whose acquisition transform exists.
        # A 40Hz scan and 20Hz odometry need not share the latest timestamp.
        for scan in reversed(self.scan_history):
            age=(now-scan.header.stamp).to_sec()
            if not -.02<=age<=.1: continue
            if self.listener.canTransform(self.frame,scan.header.frame_id,scan.header.stamp):
                position,rotation=self.listener.lookupTransform(self.frame,scan.header.frame_id,scan.header.stamp)
                self.laser_cache=(scan,position,rotation)
                return self.laser_cache
        if self.laser_cache is not None and -.02<=(now-self.laser_cache[0].header.stamp).to_sec()<=.1:
            return self.laser_cache
        scan=self.scan
        if scan is None: raise ValueError('No laser scan available')
        age=(now-scan.header.stamp).to_sec()
        if not -.02<=age<=.3:
            raise ValueError('Current laser scan stale: age=%.6fs' % age)
        # Cache expiry alone is not a sensor fault. Try bounded TF acquisition
        # before stopping; never return a scan with an unverified transform.
        self.listener.waitForTransform(self.frame,scan.header.frame_id,scan.header.stamp,
                                       rospy.Duration(.05),rospy.Duration(.005))
        position,rotation=self.listener.lookupTransform(self.frame,scan.header.frame_id,scan.header.stamp)
        age=(rospy.Time.now()-scan.header.stamp).to_sec()
        if not -.02<=age<=.3:
            raise ValueError('Laser scan expired during TF acquisition: age=%.6fs' % age)
        self.laser_cache=(scan,position,rotation)
        return self.laser_cache

    def control_cycle(self, now, dt):
        pose = self.sensors()
        if self.generated:
            scan,laser_pos,laser_rot=self.laser_feedback(now)
            # A bounded TF wait may have advanced simulation time. Refresh pose
            # and integration interval before calculating a motion command.
            now=rospy.Time.now()
            dt=(now-self.previous).to_sec()
            if not math.isfinite(dt) or dt<=0:
                raise ValueError('Control clock invalid after TF acquisition: dt=%.6fs' % dt)
            pose=self.sensors()
        maximum_gap = 0.3 if self.generated else 0.2
        if dt > maximum_gap:
            raise ValueError('Control scheduling gap: dt=%.6fs, limit=%.3fs' % (dt, maximum_gap))
        self.previous = now
        if self.clearance_guard is not None:
            self.clearance_guard(pose)
        if self.generated:
            # Fresh feedback is mandatory. Project over the actual
            # elapsed arc, but never catch up acceleration/steering
            # by integrating an entire delayed interval at once.
            speed, steer = self.follower.step(*pose, min(dt, 0.04), projection_dt=dt)
            if dt > 0.1:
                rospy.logwarn_throttle(1, 'Control scheduling delay: dt=%.6fs; fresh feedback accepted', dt)
        else:
            speed, steer = self.follower.step(*pose, dt)
        if self.generated:
            from predictive_follower import reference_clearance
            # A scan is already allowed to be up to 300ms old. Transform
            # its points at acquisition time, not at the latest car pose.
            laser_yaw = tf.transformations.euler_from_quaternion(laser_rot)[2]
            dx,dy = laser_pos[0]-pose[0],laser_pos[1]-pose[1]
            relative_laser = (math.cos(pose[2])*dx+math.sin(pose[2])*dy,
                              -math.sin(pose[2])*dx+math.cos(pose[2])*dy,
                              laser_yaw-pose[2])
            report = {}
            obstacle_limit = reference_clearance(
                self.path, self.follower.progress, pose, scan,
                relative_laser, max(speed, abs(pose[3])),
                self.follower.braking, diagnostics=report, sample=self.follower.sample_tracking,
                reaction=max(.15,self.follower.response_delay))
            report.update(scan_age=(now-scan.header.stamp).to_sec(),
                          speed_limit=obstacle_limit if math.isfinite(obstacle_limit) else None,
                          tracking_error=self.follower.error)
            if obstacle_limit < speed:
                rospy.logwarn_throttle(1, 'Laser speed restriction: %s', json.dumps(report,sort_keys=True))
            if obstacle_limit < 0.05:
                raise ValueError('Laser protection: '+json.dumps(report,sort_keys=True))
        else:
            laser_pos, laser_rot = self.listener.lookupTransform(
                self.base, self.scan.header.frame_id, rospy.Time(0))
            laser_yaw = tf.transformations.euler_from_quaternion(laser_rot)[2]
            obstacle_limit = clearance_speed(
                self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                self.scan.range_min, self.scan.range_max, max(speed, abs(pose[3])), steer,
                braking=self.follower.braking,
                laser_x=laser_pos[0], laser_y=laser_pos[1], laser_yaw=laser_yaw)
            if obstacle_limit < 0.05:
                raise ValueError('Obstacle/invalid scan: stopped; inspect then call start again')
        speed = min(speed, obstacle_limit)
        self.follower.speed = speed
        if self.generated and pose[3] > speed + 0.5:
            rospy.logwarn_throttle(1, 'Vehicle above requested speed: measured=%.3f command=%.3f; control=%s',
                                   pose[3], speed, json.dumps(self.follower.last_report,sort_keys=True))
        if self.generated:
            self.follower.last_report['output_speed']=speed
            if obstacle_limit < self.follower.last_report['command_speed']:
                self.follower.last_report['reason']='laser'
        progress = self.follower.progress - self.follower.origin
        elapsed=(now-self.started_at).to_sec() if self.started_at is not None else 0
        if pose[3]>.05 and 'first_measured_motion_seconds' not in self.start_metrics:
            self.start_metrics['first_measured_motion_seconds']=elapsed
            rospy.loginfo('Startup: measured motion after %.3fs',elapsed)
        if self.generated and not self.follower.departure and 'alignment_released_seconds' not in self.start_metrics:
            self.start_metrics['alignment_released_seconds']=elapsed
        if progress>=self.path.length and 'first_reference_loop_seconds' not in self.start_metrics:
            self.start_metrics['first_reference_loop_seconds']=elapsed
            rospy.loginfo('First reference loop: %.3fs (official referee time is separate)',elapsed)
        if self.run_metrics is not None:
            i=int(self.follower.progress/self.path.ds)%self.path.n
            changed=self.run_metrics.observe(elapsed,pose[3],speed,self.follower.error,
                getattr(self.follower,'last_report',{}).get('reason','legacy'),
                getattr(self.follower,'departure',False),float(self.path.curvature[i]),progress/self.path.length)
            if changed:self.save_run_report(background=True)
        if progress >= self.laps * self.path.length:
            self.stop(None)
            rospy.loginfo('Reference loops complete; check OFFICIAL referee laps separately')
        else:
            msg = Twist()
            msg.linear.x, msg.angular.z = speed, steer
            self.pub.publish(msg)
            rospy.loginfo_throttle(2, 'Reference %.2f/%d loops, v=%.2f measured=%.2f error=%.2fm limit=%s',
                                  progress / self.path.length,self.laps,speed,pose[3],self.follower.error,
                                  getattr(self.follower,'last_report',{}).get('reason','legacy'))

if __name__ == '__main__':
    rospy.init_node('continuous_race')
    try:
        ContinuousRace().run()
    except (ValueError, OSError) as error:
        rospy.logfatal('Cannot initialize continuous race: %s', error)
        raise SystemExit(1)
