from pathlib import Path
import unittest
import yaml
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]


class FlowProfileTests(unittest.TestCase):
    def test_collision_geometry_and_checks_are_preserved(self):
        base=yaml.safe_load((ROOT/'config/teb/teb_local_planner_params.yaml').read_text())['TebLocalPlannerROS']
        tuning=yaml.safe_load((ROOT/'config/teb/flow_tuning.yaml').read_text())['TebLocalPlannerROS']
        merged=dict(base,**tuning)
        for key in ('min_obstacle_dist','min_turning_radius','footprint_model','include_costmap_obstacles'):
            self.assertEqual(merged[key],base[key])
        self.assertGreaterEqual(merged['feasibility_check_no_poses']*merged['dt_ref'],
                                base['feasibility_check_no_poses']*base['dt_ref'])
        self.assertGreater(merged['no_inner_iterations']*merged['no_outer_iterations'],
                           base['no_inner_iterations']*base['no_outer_iterations'])

    def test_wrapper_disables_the_regressed_output_guard(self):
        include=E.parse(ROOT/'launch/flow_race_demo.launch').find('include')
        args={a.attrib['name']:a.attrib['value'] for a in include.findall('arg')}
        self.assertEqual(args['corner_speed_control'],'false')
        self.assertEqual(args['race_profile'],'false')
        self.assertEqual(args['wheel_feedforward'],'true')
        self.assertEqual(args['path_bias'],'false')

    def test_flow_parameter_forwarded_and_loaded_last(self):
        includes=E.parse(ROOT/'launch/demo_tianracer_teb_nav.launch').findall('.//include')
        for i in [i for i in includes if i.attrib['file'].endswith('/teb_base.launch.xml')]:
            self.assertIn('flow_tuning',[a.attrib['name'] for a in i.findall('arg')])
        node=E.parse(ROOT/'launch/includes/teb_base.launch.xml').find('.//node')
        last=node.findall('rosparam')[-1]
        self.assertTrue(last.attrib['file'].endswith('/flow_tuning.yaml'))
        self.assertEqual(last.attrib['if'],'$(arg flow_tuning)')


if __name__=='__main__':unittest.main()
