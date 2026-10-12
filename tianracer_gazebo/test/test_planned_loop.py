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

    def test_obstacle_invalid_scan_and_disconnected_map_stop(self):
        p=rear_path(self.center);q=p.sample(0);d=p.sample(.04)-p.sample(-.04);yaw=math.atan2(d[1],d[0])
        scan=NS(ranges=[0,0,0],angle_min=-.1,angle_increment=.1,range_min=.05,range_max=10)
        self.assertEqual(reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1),0)
        scan.ranges=[float('inf'),.25,float('inf')]
        self.assertLess(reference_clearance(p,0,(*q,yaw,1),scan,(.13,0,0),1),1)
        wall=LoopPlanner(np.full((100,100),100),.1,[0,0])
        with self.assertRaises(ValueError):wall.edge([2,2],[3,3])


if __name__=='__main__': unittest.main()
