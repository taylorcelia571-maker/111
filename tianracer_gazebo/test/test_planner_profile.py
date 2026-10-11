"""Verify effective planner configuration and launch forwarding offline."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as E
import yaml
ROOT = Path(__file__).resolve().parents[1]


class PlannerProfileTests(unittest.TestCase):
    def test_effective_planner_and_robot_constraints(self):
        base = yaml.safe_load((ROOT/'config/teb/teb_local_planner_params.yaml').read_text())['TebLocalPlannerROS']
        profile = yaml.safe_load((ROOT/'config/teb/planner_race_profile.yaml').read_text())
        effective = dict(base, **profile['TebLocalPlannerROS'])
        self.assertEqual(profile['base_global_planner'], 'global_planner/GlobalPlanner')
        self.assertEqual(effective['footprint_model'], base['footprint_model'])
        self.assertEqual(effective['min_obstacle_dist'], base['min_obstacle_dist'])
        self.assertEqual(effective['min_turning_radius'], base['min_turning_radius'])
        self.assertTrue(effective['cmd_angle_instead_rotvel'])
        self.assertGreater(effective['max_global_plan_lookahead_dist'], base['max_global_plan_lookahead_dist'])
        self.assertGreater(effective['no_inner_iterations']*effective['no_outer_iterations'],
                           base['no_inner_iterations']*base['no_outer_iterations'])
        self.assertLess(effective['dt_hysteresis'], effective['dt_ref'])

    def test_override_order_and_speed_argument(self):
        node = E.parse(ROOT/'launch/includes/teb_base.launch.xml').find('.//node')
        children = list(node)
        loads = [(i,x.attrib['file']) for i,x in enumerate(children) if x.tag=='rosparam']
        profile = next(i for i,f in loads if f.endswith('/planner_race_profile.yaml'))
        baseline = next(i for i,f in loads if f.endswith('/teb_local_planner_params.yaml'))
        speed = next(i for i,x in enumerate(children) if x.attrib.get('name')=='TebLocalPlannerROS/max_vel_x'
                     and x.attrib.get('if')=='$(arg planner_profile)')
        self.assertLess(baseline, profile)
        self.assertLess(profile, speed)
        self.assertEqual(children[speed].attrib['value'], '$(arg planner_max_speed)')

    def test_both_namespaces_forward_profile(self):
        includes = E.parse(ROOT/'launch/demo_tianracer_teb_nav.launch').findall('.//include')
        planners = [i for i in includes if i.attrib['file'].endswith('/teb_base.launch.xml')]
        self.assertEqual(len(planners), 2)
        for i in planners:
            names = {a.attrib['name']: a.attrib['value'] for a in i.findall('arg')}
            self.assertEqual(names['planner_profile'], '$(arg planner_profile)')
            self.assertEqual(names['planner_max_speed'], '$(arg planner_max_speed)')

    def test_wrapper_has_no_stacked_filter_or_recording(self):
        include = E.parse(ROOT/'launch/planner_race_demo.launch').find('include')
        args = {a.attrib['name']:a.attrib['value'] for a in include.findall('arg')}
        self.assertEqual(args['race_profile'], 'false')
        self.assertEqual(args['planner_profile'], 'true')
        self.assertEqual(args['command_watchdog'], 'true')
        self.assertEqual(args['world'], 'tianracer_racetrack')


if __name__ == '__main__': unittest.main()
