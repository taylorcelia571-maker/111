"""Regress the logged second-lap wall stop; no official-result claim."""
from pathlib import Path
import sys
import math
import unittest
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from race60_audit import load_course, rear_path, simulate

PACKAGE=Path(__file__).resolve().parents[1]


class WallReserveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner, cls.anchors=load_course(PACKAGE)
        cls.planner.margin=.34
        cls.planner.allowed=cls.planner.clearance>=.34
        cls.budgets=dict(max_speed=2.2,lateral_accel=3.2,acceleration=1.5,braking=2.,
                         yaw_rate=2.5,steering_rate=2.)
        cls.center=cls.planner.build(cls.anchors,optimize_time=True,refine_line=True,
                                   response_delay=.25,**cls.budgets)

    def test_log_pose_reproduces_wall_stop_with_original_threshold(self):
        pose=(-.30400394991316954,3.79559726964633,1.9922489691317387,1.87)
        with self.assertRaisesRegex(ValueError,'center_clearance=0.2276m stop_limit=0.2300m'):
            self.planner.check_vehicle_clearance(pose)

    def test_rapid_log_pose_reproduces_wall_stop_not_stale_tf(self):
        pose=(-1.8800011666554266,-.16872605762874826,-.5347856781980944,2.0)
        with self.assertRaisesRegex(ValueError,'center_clearance=0.2288m stop_limit=0.2300m'):
            self.planner.check_vehicle_clearance(pose)

    def test_rapid_entry_forwards_the_same_wall_reserve(self):
        launch=ET.parse(PACKAGE/'launch/rapid_race.launch').getroot()
        args={x.get('name'):x.get('default') for x in launch.findall('arg')}
        forwarded={x.get('name'):x.get('value') for x in launch.find('include').findall('arg')}
        self.assertEqual(args['route_margin'],'0.34')
        self.assertEqual(forwarded['route_margin'],'$(arg route_margin)')

    def test_route_has_tracking_reserve_for_eight_centimeters_and_heading_error(self):
        path=rear_path(self.center)
        tangent=np.roll(path.points,-1,axis=0)-np.roll(path.points,1,axis=0)
        headings=np.arctan2(tangent[:,1],tangent[:,0])
        for error in (-.08,.08):
            for angle in (-.08,.08):
                yaw=headings+angle
                rear=path.points+error*np.column_stack([-np.sin(headings),np.cos(headings)])
                actual=rear+.13*np.column_stack([np.cos(yaw),np.sin(yaw)])
                self.assertGreater(float(self.planner.clearance_at(actual).min()),.23)
        self.assertGreaterEqual(self.planner.segment_check(self.center.points),.34)

    def test_five_loops_keep_wall_clearance_with_three_response_lags(self):
        for lag in (.08,.15,.25):
            with self.subTest(lag=lag):
                row=simulate(self.center,self.planner,self.budgets,.25,lag)
                self.assertTrue(row['completed'],row)
                self.assertGreaterEqual(row['minimum_center_clearance'],.23)
                self.assertLess(row['peak_error'],.15)

    def test_candidate_launch_passes_route_reserve_to_planner(self):
        launch=ET.parse(PACKAGE/'launch/optimized_race.launch').getroot()
        args={x.get('name'):x.get('default') for x in launch.findall('arg')}
        self.assertEqual(args['route_margin'],'0.34')
        forwarded={x.get('name'):x.get('value') for x in launch.find('include').findall('arg')}
        self.assertEqual(forwarded['route_margin'],'$(arg route_margin)')
        planned=ET.parse(PACKAGE/'launch/planned_race.launch').getroot()
        params={x.get('name'):x.get('value') for x in planned.find('node').findall('param')}
        self.assertEqual(params['route_margin'],'$(arg route_margin)')


if __name__=='__main__':unittest.main()
