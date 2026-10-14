"""Ensure the public tuning entry really forwards the bounded prediction flag."""
from pathlib import Path
import unittest
from xml.etree import ElementTree as ET

LAUNCH=Path(__file__).resolve().parents[1]/'launch'


class PredictionLaunchTests(unittest.TestCase):
    def test_default_is_unchanged_and_option_is_forwarded_to_typed_param(self):
        for filename,target in [('race_tuning_demo.launch','race_tuning.launch'),
                                ('race_tuning.launch','planned_race.launch')]:
            root=ET.parse(LAUNCH/filename).getroot()
            args={a.attrib['name']:a.attrib for a in root.findall('arg')}
            self.assertEqual(args['predict_pose']['default'],'false')
            includes=[i for i in root.findall('include') if i.attrib['file'].endswith(target)]
            self.assertEqual(len(includes),1)
            forwarded={a.attrib['name']:a.attrib['value'] for a in includes[0].findall('arg')}
            self.assertEqual(forwarded['predict_pose'],'$(arg predict_pose)')
        params={p.attrib['name']:p.attrib for p in ET.parse(LAUNCH/'planned_race.launch').getroot().iter('param')}
        self.assertEqual(params['predict_pose']['type'],'bool')
        self.assertEqual(params['predict_pose']['value'],'$(arg predict_pose)')

    def test_protected_budgets_and_static_defaults_remain_intact(self):
        root=ET.parse(LAUNCH/'race_tuning.launch').getroot()
        forwarded={a.attrib['name']:a.attrib['value'] for i in root.findall('include') for a in i.findall('arg')}
        self.assertEqual(forwarded['route_margin'],'0.34')
        self.assertEqual(forwarded['response_delay'],'0.25')
        self.assertEqual(forwarded['max_speed'],'10.0')
        args={a.attrib['name']:a.attrib['default'] for a in root.findall('arg')}
        self.assertEqual(args['startup_acceleration'],'2.0')
        self.assertEqual(args['yaw_rate'],'3.8')


if __name__=='__main__':unittest.main()
