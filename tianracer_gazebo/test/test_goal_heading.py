import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import yaml
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from goal_heading import arrival_yaw


def make_goal(w):
    position=NS(**w['pose']['position'])
    orientation=NS(**w['pose']['orientation'])
    return NS(target_pose=NS(header=NS(frame_id=w['frame_id']),pose=NS(position=position,orientation=orientation)))


class HeadingGeometryTests(unittest.TestCase):
    def test_arrival_heading_uses_real_path_not_waypoint_chord(self):
        yaw=arrival_yaw([(0,0),(1,0),(1,1),(1,2)],(1,2))
        self.assertAlmostEqual(yaw,math.pi/2)

    def test_invalid_or_wrong_endpoint_rejected(self):
        for points in ([],[(0,0)],[(0,0),(0,0)],[(0,0),(99,99)],[(0,0),(float('nan'),0)]):
            self.assertIsNone(arrival_yaw(points,(1,0)))


class HeadingAdapterTests(unittest.TestCase):
    def setUp(self):
        self.ros=Mock();self.ros.is_shutdown.return_value=False
        self.ros.ServiceException=RuntimeError
        self.action=Mock();self.action.SimpleActionClient.return_value.get_state.return_value=3
        utils=NS(get_waypoints=lambda f:yaml.safe_load(Path(f).read_text())['waypoints'],
                 create_move_base_goal=make_goal,create_viz_markers=lambda w:None)
        modules={'rospy':self.ros,'actionlib':self.action,'rospkg':Mock(),
                 'actionlib_msgs.msg':NS(GoalStatus=NS(SUCCEEDED=3,ACTIVE=1,PENDING=0)),
                 'geometry_msgs.msg':NS(Twist=object),'move_base_msgs.msg':NS(MoveBaseAction=object),
                 'visualization_msgs.msg':NS(MarkerArray=object),'waypoint_race':NS(utils=utils)}
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('heading_waypoints',SCRIPTS/'waypoint_laps.py')
            self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.filename=str(SCRIPTS/'waypoint_race/tianracer_racetrack_points.yaml')

    def race(self,align=True):
        with patch.dict(sys.modules,{'nav_msgs.srv':NS(GetPlan=object)}):
            return self.module.WaypointLaps(self.filename,align_goal_yaw=align)

    def path(self,goal):
        x,y=goal.pose.position.x,goal.pose.position.y
        return NS(plan=NS(header=NS(frame_id='map'),poses=[NS(pose=NS(position=NS(x=x+.5,y=y))),
                                NS(pose=NS(position=NS(x=x,y=y)))]))

    def test_cached_heading_changes_yaw_only(self):
        race=self.race()
        race._make_plan.side_effect=lambda start,goal,tolerance:self.path(goal)
        race._prepare_headings()
        self.assertTrue(race._visit(0,allow_early=True))
        goal=race._client.send_goal.call_args.args[0].target_pose
        self.assertAlmostEqual(goal.pose.orientation.z,1)
        self.assertAlmostEqual(goal.pose.orientation.w,0)
        self.assertEqual(goal.pose.position.x,race.waypoints[0]['pose']['position']['x'])
        self.assertEqual(goal.header.frame_id,'map')

    def test_preparation_queries_all_edges_before_any_action(self):
        race=self.race()
        race._make_plan.side_effect=lambda start,goal,tolerance:self.path(goal)
        race._prepare_headings()
        self.assertEqual(race._make_plan.call_count,len(race.waypoints))
        race._client.send_goal.assert_not_called()
        args=race._make_plan.call_args_list[0].args
        self.assertEqual(args[0].pose.position.x,race.waypoints[-1]['pose']['position']['x'])

    def test_empty_plan_falls_back_to_original_goal(self):
        race=self.race();race._make_plan.return_value=NS(plan=NS(header=NS(frame_id='map'),poses=[]))
        race._prepare_headings()
        self.assertTrue(race._visit(0,allow_early=True))
        goal=race._client.send_goal.call_args.args[0]
        self.assertEqual(goal.target_pose.pose.orientation.z,race.waypoints[0]['pose']['orientation']['z'])

    def test_running_visits_never_query_planning_service(self):
        race=self.race()
        self.assertTrue(race._visit(1,allow_early=True))
        race._make_plan.assert_not_called()

    def test_wrong_frame_does_not_change_orientation(self):
        race=self.race();goal=make_goal(race.waypoints[0]).target_pose
        response=self.path(goal);response.plan.header.frame_id='odom'
        race._make_plan.return_value=response
        race._prepare_headings()
        self.assertFalse(race._arrival_headings)

    def test_disabled_mode_does_not_create_planning_service(self):
        race=self.race(False)
        self.ros.ServiceProxy.assert_not_called()
        self.assertTrue(race._visit(0,allow_early=False))
        self.assertIsNone(race._client.send_goal.call_args.kwargs['feedback_cb'])


if __name__=='__main__':unittest.main()
