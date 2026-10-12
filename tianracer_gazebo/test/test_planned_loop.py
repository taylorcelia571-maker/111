"""Actual map geometry and ideal bicycle tracking; these are not Gazebo race results."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from loop_planner import LoopPlanner
from predictive_follower import PredictiveFollower, reference_clearance
from race_line import ClosedPath


def rear_path(center):
    d=np.roll(center.points,-1,axis=0)-np.roll(center.points,1,axis=0)
    d/=np.linalg.norm(d,axis=1)[:,None]
    p=center.points-0.13*d
    return ClosedPath(np.vstack([p,p[0]]),spacing=0.04)


class PlannedLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        meta=yaml.safe_load((PACKAGE/'maps/tianracer_racetrack.yaml').read_text())
        with open(PACKAGE/'maps'/meta['image'],'rb') as f:
            assert f.readline().strip()==b'P5'
            line=f.readline()
            while line.startswith(b'#'): line=f.readline()
            width,height=map(int,line.split()); assert f.readline().strip()==b'255'
            img=np.frombuffer(f.read(),dtype=np.uint8).reshape(height,width)[::-1]
        cls.grid=np.where(img>=250,0,100)
        cls.planner=LoopPlanner(cls.grid,meta['resolution'],meta['origin'][:2])
        data=yaml.safe_load((PACKAGE/'scripts/waypoint_race/tianracer_racetrack_points.yaml').read_text())
        cls.anchors=np.array([[v['pose']['position'][k] for k in ('x','y')] for v in data['waypoints']])
        cls.center=cls.planner.build(cls.anchors)

    def test_actual_course_clearance_curvature_and_order(self):
        self.assertGreater(self.planner.segment_check(self.center.points),0.26)
        self.assertLess(abs(self.center.curvature).max(),math.tan(.48)/.26)
        self.assertGreater(self.center.length,40)
        self.assertLess(self.center.length,50)
        positions=[self.center.project(*p)[0] for p in self.anchors]
        distances=np.diff(np.r_[positions,positions[0]+self.center.length])%self.center.length
        self.assertAlmostEqual(distances.sum(),self.center.length,places=5)
        self.assertTrue(all(self.center.project(*p)[1]<.4 for p in self.anchors))

    def test_cyclic_velocity_and_steering_constraints(self):
        p=rear_path(self.center); f=PredictiveFollower(p,max_speed=2.2)
        self.assertTrue(np.all(p.speeds<=2.2))
        self.assertTrue(np.all(p.speeds**2*abs(p.curvature)<=5.0+1e-8))
        self.assertTrue(np.all(p.speeds**2-np.roll(p.speeds,-1)**2<=2*f.braking*p.ds+1e-8))
        self.assertTrue(np.all(p.speeds**2-np.roll(p.speeds,1)**2<=2*f.acceleration*p.ds+1e-8))
        delta=np.arctan(.26*p.curvature)
        derivative=np.maximum(abs(np.roll(delta,-1)-delta),abs(delta-np.roll(delta,1)))/p.ds
        self.assertTrue(np.all(derivative*p.speeds<=f.steering_rate+1e-8))

    def test_five_forward_laps_from_official_spawn(self):
        p=rear_path(self.center); f=PredictiveFollower(p,max_speed=2.0)
        yaw=1.54; x,y=-.13*math.cos(yaw),-.13*math.sin(yaw)
        f.start(x,y,yaw);speed=steer=0;seams=0;old_loop=0
        for _ in range(8000):
            previous_steer=steer
            speed,steer=f.step(x,y,yaw,speed,.02)
            self.assertGreaterEqual(speed,0)
            self.assertLessEqual(abs(steer-previous_steer),f.steering_rate*.02+1e-8)
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            c=[[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]]
            self.assertGreater(self.planner.clearance_at(c)[0],.23)
            loop=int((f.progress-f.origin)/p.length)
            if loop>old_loop:
                self.assertGreater(speed,1.0)  # seam does not introduce a stop
                old_loop=loop;seams+=1
            if seams==5:break
        self.assertEqual(seams,5)

    def test_time_optimized_course_and_fast_start_against_baseline(self):
        center=self.planner.build(self.anchors,optimize_time=True,max_speed=4)
        optimized=rear_path(center); base=rear_path(self.center)
        PredictiveFollower(base,max_speed=4); PredictiveFollower(optimized,max_speed=4)
        self.assertLessEqual(float(np.sum(optimized.ds/optimized.speeds)),float(np.sum(base.ds/base.speeds)))
        self.assertGreaterEqual(self.planner.segment_check(center.points),.26)
        self.assertLess(abs(center.curvature).max(),math.tan(.48)/.26)
        locations=[center.project(*p)[0] for p in self.anchors]
        self.assertAlmostEqual(float((np.diff(np.r_[locations,locations[0]+center.length])%center.length).sum()),center.length)

        def simulate(p, fast):
            f=PredictiveFollower(p,max_speed=4,fast_start=fast)
            x,y,yaw=.0081,-.1494,1.5709; f.start(x,y,yaw);speed=steer=0;first=None
            for tick in range(8000):
                v,d=f.step(x,y,yaw,speed,.02)
                self.assertGreaterEqual(v,0)
                speed+=(v-speed)*.02/.08;steer+=(d-steer)*.02/.08
                x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02;yaw+=speed*math.tan(steer)/.26*.02
                self.assertLess(f.error,.15)
                self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
                if first is None and f.progress-f.origin>=p.length: first=(tick+1)*.02
                if f.progress-f.origin>=5*p.length: return first,(tick+1)*.02
            self.fail('Did not complete five ideal-model reference loops')
        baseline=simulate(base,False)
        fast=simulate(rear_path(self.center),True)
        combined=simulate(rear_path(center),True)
        self.assertLess(fast[0],baseline[0])
        self.assertLess(combined[1],baseline[1])

    def test_fast_start_keeps_low_speed_fallback_for_larger_initial_error(self):
        p=rear_path(self.center); f=PredictiveFollower(p,max_speed=4,fast_start=True)
        x,y=p.points[0];d=p.sample(.08)-p.sample(0);yaw=math.atan2(d[1],d[0])
        x-=.11*math.sin(yaw);y+=.11*math.cos(yaw)
        f.start(x,y,yaw)
        for _ in range(20):
            v,_=f.step(x,y,yaw,0,.02)
            self.assertLessEqual(v,.35)

    def test_rapid_profile_limits_report_and_five_ideal_laps(self):
        budgets=dict(max_speed=5.0,lateral_accel=6.0,acceleration=3.0,braking=3.0,yaw_rate=4.0,steering_rate=2.0)
        center=self.planner.build(self.anchors,optimize_time=True,**budgets)
        p=rear_path(center);f=PredictiveFollower(p,fast_start=True,**budgets)
        report=f.plan_report()
        self.assertEqual(report['limits'],budgets)
        self.assertAlmostEqual(sum(report['time_fraction_by_limit'].values()),1)
        self.assertGreater(report['model_five_seconds'],60)  # do not claim the minute target
        delta=np.arctan(.26*p.curvature)
        derivative=np.maximum(abs(np.roll(delta,-1)-delta),abs(delta-np.roll(delta,1)))/p.ds
        self.assertTrue(np.all(p.speeds*abs(p.curvature)<=4+1e-8))
        self.assertTrue(np.all(p.speeds**2*abs(p.curvature)<=6+1e-8))
        self.assertTrue(np.all(p.speeds*derivative<=2+1e-8))
        x,y,yaw=.0081,-.1494,1.5709;f.start(x,y,yaw);speed=steer=0
        for _ in range(8000):
            v,d=f.step(x,y,yaw,speed,.02)
            self.assertLessEqual(v*abs(f.last_report['live_curvature']),4+1e-8)
            self.assertLessEqual(v*v*abs(f.last_report['live_curvature']),6+1e-8)
            speed+=(v-speed)*.02/.08;steer+=(d-steer)*.02/.08
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02;yaw+=speed*math.tan(steer)/.26*.02
            self.assertLess(f.error,.15)
            self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
            if f.progress-f.origin>=5*p.length:break
        self.assertGreaterEqual(f.progress-f.origin,5*p.length)

    def test_refined_route_and_connector_under_actual_user_budgets(self):
        budgets=dict(max_speed=8.0,lateral_accel=6.5,acceleration=5.0,braking=3.0,yaw_rate=3.8)
        center=self.planner.build(self.anchors,optimize_time=True,refine_line=True,**budgets)
        selection=self.planner.selection
        self.assertTrue(selection['route_changed'])
        self.assertGreater(selection['refinement_moves'],0)
        self.assertGreater(selection['refinement_gain_seconds'],0)
        self.assertGreaterEqual(self.planner.segment_check(center.points),.26)
        positions=[center.project(*p)[0] for p in self.anchors]
        self.assertAlmostEqual(float((np.diff(np.r_[positions,positions[0]+center.length])%center.length).sum()),center.length)
        p=rear_path(center)
        def simulate(blend):
            f=PredictiveFollower(p,fast_start=True,start_blend=blend,**budgets)
            x,y,yaw=.0081,-.1494,1.5709;f.start(x,y,yaw)
            if blend:
                self.assertIsNotNone(f.connector_offset)
                self.assertLess(np.linalg.norm(f.sample_tracking(f.origin)-[x,y]),1e-8)
                self.assertLess(np.linalg.norm(f.sample_tracking(f.origin+2.5)-p.sample(f.origin+2.5)),1e-8)
                self.assertLess(np.linalg.norm(f.sample_tracking(f.origin+p.length)-p.sample(f.origin+p.length)),1e-8)
            speed=steer=0;first=None
            for tick in range(8000):
                v,d=f.step(x,y,yaw,speed,.02)
                speed+=(v-speed)*.02/.08;steer+=(d-steer)*.02/.08
                x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02;yaw+=speed*math.tan(steer)/.26*.02
                self.assertLess(f.error,.15)
                self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
                if first is None and f.progress-f.origin>=p.length:first=(tick+1)*.02
                if f.progress-f.origin>=5*p.length:return first
            self.fail('Reference loops incomplete')
        self.assertLess(simulate(True),simulate(False))

    def test_start_connector_falls_back_for_curve_or_wrong_heading(self):
        p=rear_path(self.center);f=PredictiveFollower(p,start_blend=True,fast_start=True)
        i=int(np.argmax(abs(p.curvature)));x,y=p.points[i]
        d=p.sample(i*p.ds+.08)-p.sample(i*p.ds);yaw=math.atan2(d[1],d[0])
        f.start(x,y,yaw)
        self.assertIsNone(f.connector_offset)
        self.assertTrue(f.departure)

    def test_sprint_trial_with_delayed_feedback_prediction(self):
        from motion_prediction import predict_rear_pose
        budgets=dict(max_speed=8.0,lateral_accel=9.0,acceleration=6.0,braking=6.0,yaw_rate=5.5)
        center=self.planner.build(self.anchors,optimize_time=True,refine_line=True,**budgets)
        p=rear_path(center);f=PredictiveFollower(p,fast_start=True,start_blend=True,**budgets)
        x,y,yaw=.0081,-.1494,1.5709;f.start(x,y,yaw);speed=steer=0
        history=[(x,y,yaw,0,0)]*3
        for tick in range(8000):
            old_x,old_y,old_yaw,old_speed,omega=history[-3]
            predicted=predict_rear_pose(old_x,old_y,old_yaw,old_speed,omega,.04)
            v,d=f.step(*predicted,old_speed,.02)
            self.assertGreaterEqual(v,0)
            self.assertLessEqual(v*v*abs(f.last_report['live_curvature']),9+1e-8)
            self.assertLessEqual(v*abs(f.last_report['live_curvature']),5.5+1e-8)
            speed+=(v-speed)*.02/.08;steer+=(d-steer)*.02/.08
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02;yaw+=speed*math.tan(steer)/.26*.02
            history.append((x,y,yaw,speed,speed*math.tan(steer)/.26))
            self.assertLess(f.error,.15)
            self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
            if f.progress-f.origin>=5*p.length:break
        self.assertGreaterEqual(f.progress-f.origin,5*p.length)
        self.assertLess((tick+1)*.02,65)  # ideal-model regression bound, not an official result

    def test_departure_aligns_at_low_speed_without_relaxing_error_limit(self):
        p=rear_path(self.center); f=PredictiveFollower(p,max_speed=1.8)
        yaw=1.3; x,y=.04-.13*math.cos(yaw),-.13*math.sin(yaw)
        f.start(x,y,yaw); speed=steer=0; was_limited=False
        for _ in range(500):
            aligning=f.departure
            desired_speed,desired_steer=f.step(x,y,yaw,speed,.02)
            if aligning and f.departure:
                self.assertLessEqual(desired_speed,.35)
                was_limited=True
            speed+=(desired_speed-speed)*.02/.12
            steer+=(desired_steer-steer)*.02/.12
            x+=speed*math.cos(yaw)*.02; y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            self.assertLessEqual(f.error,.15)
        self.assertTrue(was_limited)
        self.assertFalse(f.departure)
        self.assertGreater(speed,.35)

    def test_departure_does_not_bypass_tracking_stop_and_reports_pose(self):
        p=rear_path(self.center); f=PredictiveFollower(p)
        x,y=p.points[0]; d=p.sample(.08)-p.sample(0);yaw=math.atan2(d[1],d[0])
        f.start(x,y,yaw)
        with self.assertRaisesRegex(ValueError,'global_error=.*measured_speed=.*elapsed='):
            f.step(x-.5*math.sin(yaw),y+.5*math.cos(yaw),yaw,0,.02)
        self.assertEqual(f.speed,0)
        self.assertEqual(f.steer,0)

    def test_tracking_with_assumed_actuator_delay(self):
        # Hypothetical 80ms first-order response; not a measured vehicle model.
        p=rear_path(self.center); f=PredictiveFollower(p,max_speed=2.0)
        yaw=1.54; x,y=-.13*math.cos(yaw),-.13*math.sin(yaw)
        f.start(x,y,yaw); speed=steer=0
        for _ in range(8000):
            desired_speed,desired_steer=f.step(x,y,yaw,speed,.02)
            speed+=(desired_speed-speed)*.02/.08
            steer+=(desired_steer-steer)*.02/.08
            x+=speed*math.cos(yaw)*.02; y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            self.assertLess(f.error,.10)
            self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
            if f.progress-f.origin>=5*p.length:break
        self.assertGreaterEqual(f.progress-f.origin,5*p.length)

    def test_static_wall_scan_follows_reference_turn_without_false_braking(self):
        p=rear_path(self.center)
        angles=np.linspace(-math.pi,math.pi,361); steps=np.arange(.05,3.5,.025)
        for progress in np.linspace(0,p.length,20,endpoint=False):
            rear=p.sample(progress);d=p.sample(progress+.04)-p.sample(progress-.04)
            yaw=math.atan2(d[1],d[0]);base=rear+.13*np.array([math.cos(yaw),math.sin(yaw)])
            points=base[None,None,:]+steps[None,:,None]*np.column_stack([np.cos(angles+yaw),np.sin(angles+yaw)])[:,None,:]
            cells=np.floor((points-self.planner.origin)/self.planner.resolution).astype(int)
            hits=self.grid[cells[:,:,1],cells[:,:,0]]!=0
            ranges=np.where(hits.any(axis=1),steps[hits.argmax(axis=1)],np.inf)
            scan=NS(ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/360,range_min=.05,range_max=10)
            limit=reference_clearance(p,progress,(*rear,yaw,2),scan,(.13,0,0),2)
            self.assertGreaterEqual(limit,2.0)

    def test_side_clearance_uses_body_shape_and_front_obstacle_stops(self):
        p=rear_path(self.center); q=p.sample(0)
        d=p.sample(.04)-p.sample(-.04); yaw=math.atan2(d[1],d[0])
        scan=NS(ranges=[float('inf'),float('inf'),.19],angle_min=-math.pi/2,
                angle_increment=math.pi/2,range_min=.05,range_max=10)
        report={}
        # 19cm to the side is outside the padded .16m body half-width;
        # the former .22m disk would classify it as an immediate collision.
        limit=reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1,diagnostics=report)
        self.assertGreaterEqual(limit,1)
        self.assertEqual(report['reason'],'clear')
        scan.ranges=[float('inf'),.19,float('inf')]
        self.assertEqual(reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1,diagnostics=report),0)
        self.assertEqual(report['reason'],'obstacle_in_corridor')
        scan.ranges=[0,0,0]
        self.assertEqual(reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1,diagnostics=report),0)
        self.assertEqual(report['reason'],'invalid_scan_coverage')

    def test_obstacle_invalid_scan_and_disconnected_map_stop(self):
        p=rear_path(self.center);q=p.sample(0);d=p.sample(.04)-p.sample(-.04);yaw=math.atan2(d[1],d[0])
        scan=NS(ranges=[0,0,0],angle_min=-.1,angle_increment=.1,range_min=.05,range_max=10)
        self.assertEqual(reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1),0)
        scan.ranges=[float('inf'),.25,float('inf')]
        self.assertLess(reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1),1)
        wall=LoopPlanner(np.full((100,100),100),.1,[0,0])
        with self.assertRaises(ValueError):wall.edge([2,2],[3,3])


if __name__=='__main__': unittest.main()
