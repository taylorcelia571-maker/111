from pathlib import Path
import unittest
import yaml
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]


class CornerTests(unittest.TestCase):
    def test_corner_changes_preserve_feasibility_constraints(self):
        base=yaml.safe_load((ROOT/'config/teb/teb_local_planner_params.yaml').read_text())['TebLocalPlannerROS']
        profile=yaml.safe_load((ROOT/'config/teb/corner_tuning.yaml').read_text())
        merged=dict(base,**profile['TebLocalPlannerROS'])
        for key in ('footprint_model','min_obstacle_dist','min_turning_radius','weight_obstacle',
                    'weight_kinematics_nh','weight_kinematics_turning_radius',
                    'global_plan_viapoint_sep','no_inner_iterations','no_outer_iterations'):
            self.assertEqual(merged[key],base[key])
        self.assertNotIn('base_global_planner',profile)
        self.assertNotIn('local_costmap',profile)
        self.assertNotIn('max_vel_x',profile['TebLocalPlannerROS'])

    def test_actuator_and_local_args_reach_both_namespace_paths(self):
        root=E.parse(ROOT/'launch/demo_tianracer_teb_nav.launch').getroot()
        for ending,args in [('tianracer_control.launch',['wheel_feedforward']),('teb_base.launch.xml',['corner_tuning','path_bias'])]:
            includes=[i for i in root.findall('.//include') if i.attrib['file'].endswith(ending)]
            self.assertEqual(len(includes),2)
            for include in includes:
                names={a.attrib['name']:a.attrib['value'] for a in include.findall('arg')}
                for arg in args:self.assertEqual(names[arg],'$(arg '+arg+')')

    def test_experimental_path_bias_is_separate_and_disabled(self):
        root=E.parse(ROOT/'launch/smooth_race_demo.launch').getroot()
        defaults={a.attrib['name']:a.attrib.get('default') for a in root.findall('arg')}
        self.assertEqual(defaults['path_bias'],'false')
        self.assertEqual(defaults['corner_tuning'],'false')
        node=E.parse(ROOT/'launch/includes/teb_base.launch.xml').find('.//node')
        bias=next(x for x in node.findall('rosparam') if x.attrib['file'].endswith('/path_bias.yaml'))
        self.assertEqual(bias.attrib['if'],'$(arg path_bias)')


if __name__=='__main__':unittest.main()
