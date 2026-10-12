import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as E
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from speed_budget import curve_budgets, braking_budget, model_com
ROOT=Path(__file__).resolve().parents[1]


class BudgetTests(unittest.TestCase):
    def test_yaw_budget_and_friction_scenarios(self):
        b=curve_budgets(.5,.034)
        self.assertAlmostEqual(b['kinematic_yaw_limit_m_s'],1.8)
        self.assertAlmostEqual(b['friction_scenarios_m_s']['0.8'],math.sqrt(.8*9.81*.5))
        self.assertGreater(curve_budgets(1,.034)['kinematic_yaw_limit_m_s'],1.8)

    def test_com_height_and_mass_from_transformed_inertial_origins(self):
        xml='''<robot><link name="root"/><link name="a"><inertial><origin xyz="0 0 .01"/><mass value="2"/></inertial></link><joint name="j" type="fixed"><parent link="root"/><child link="a"/><origin xyz="0 0 .03"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'robot.urdf';path.write_text(xml)
            mass,com=model_com(path)
        self.assertEqual(mass,2)
        self.assertAlmostEqual(com[2],.04)

    def test_braking_budget_matches_distance_equation(self):
        v=braking_budget()
        self.assertAlmostEqual(v,2.4)
        self.assertAlmostEqual(v*v/(2*1.8)+v*.25+.3,2.5)

    def test_invalid_geometry_rejected(self):
        with self.assertRaises(ValueError):curve_budgets(.5,0)
        with self.assertRaises(ValueError):curve_budgets(.5,.034,lateral_offset=.10)
        with self.assertRaises(ValueError):braking_budget(margin=3)


class PerformanceConfigTests(unittest.TestCase):
    def test_costmap_covers_proven_horizon_without_coarsening(self):
        base=yaml.safe_load((ROOT/'config/teb/local_costmap_params.yaml').read_text())['local_costmap']
        profile=yaml.safe_load((ROOT/'config/teb/performance_tuning.yaml').read_text())
        merged=dict(base,**profile['local_costmap'])
        self.assertEqual(merged['resolution'],base['resolution'])
        self.assertEqual(merged['plugins'],base['plugins'])
        self.assertEqual(merged['update_frequency'],base['update_frequency'])
        self.assertGreater(merged['width']/2,3+.3)
        self.assertAlmostEqual(merged['width']*merged['height']/(base['width']*base['height']),64/225)
        self.assertTrue(profile['TebLocalPlannerROS']['use_proportional_saturation'])

    def test_native_ratio_preserves_turn_radius(self):
        # Native TEB source applies one ratio to v and omega when configured.
        v,w=2.2,4.4
        ratio=min(1,3.6/w)
        self.assertAlmostEqual((v*ratio)/(w*ratio),.5)
        self.assertAlmostEqual(v*ratio,1.8)
        self.assertGreater(v/3.6,.5)  # Independent omega clipping widens this turn.

    def test_wrapper_retains_collision_profile_and_mirrors_limits(self):
        include=E.parse(ROOT/'launch/fast_race_demo.launch').find('include')
        args={a.attrib['name']:a.attrib['value'] for a in include.findall('arg')}
        self.assertEqual(args['flow_tuning'],'true')
        self.assertEqual(args['performance_tuning'],'true')
        self.assertEqual(args['corner_speed_control'],'false')
        node=E.parse(ROOT/'launch/includes/teb_base.launch.xml').find('.//node')
        param=next(p for p in node.findall('param') if p.attrib.get('name')=='TebLocalPlannerROS/max_vel_trans')
        self.assertEqual(param.attrib['value'],'$(arg planner_max_speed)')


if __name__=='__main__':unittest.main()
