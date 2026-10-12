from pathlib import Path
import unittest
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]


class AdaptiveLaunchTests(unittest.TestCase):
    def test_wrapper_keeps_baseline_planning_and_turning_speed(self):
        root=E.parse(ROOT/'launch/adaptive_race_demo.launch').getroot()
        defaults={x.attrib['name']:x.attrib['default'] for x in root.findall('arg')}
        self.assertEqual(defaults['corner_speed'],'1.8')
        self.assertEqual(defaults['straight_speed'],'2.0')
        self.assertEqual(defaults['corner_tuning'],'false')
        args={a.attrib['name']:a.attrib['value'] for a in root.find('include').findall('arg')}
        self.assertEqual(args['planner_max_speed'],'$(arg straight_speed)')
        self.assertEqual(args['race_profile'],'false')
        self.assertEqual(args['corner_speed_control'],'true')
        self.assertEqual(args['world'],'tianracer_racetrack')

    def test_control_arguments_forwarded_in_both_namespace_paths(self):
        includes=E.parse(ROOT/'launch/demo_tianracer_teb_nav.launch').findall('.//include')
        controls=[i for i in includes if i.attrib['file'].endswith('/tianracer_control.launch')]
        self.assertEqual(len(controls),2)
        for i in controls:
            args={a.attrib['name']:a.attrib['value'] for a in i.findall('arg')}
            for name in ['corner_speed_control','straight_speed','corner_speed']:
                self.assertEqual(args[name],'$(arg '+name+')')

    def test_existing_launches_do_not_enable_guard_implicitly(self):
        root=E.parse(ROOT/'launch/demo_tianracer_teb_nav.launch').getroot()
        arg=next(a for a in root.findall('arg') if a.attrib['name']=='corner_speed_control')
        self.assertEqual(arg.attrib['default'],'false')


if __name__=='__main__':unittest.main()
