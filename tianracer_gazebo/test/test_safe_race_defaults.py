"""Regression: observed startup failure must not be re-enabled by wrapper defaults."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
LAUNCH=Path(__file__).resolve().parents[1]/'launch'


class LaunchDefaultsTests(unittest.TestCase):
    def test_experimental_startup_and_path_switches_default_off_and_are_forwarded(self):
        for filename in ['rapid_race.launch','sprint_trial.launch']:
            root=ET.parse(LAUNCH/filename).getroot()
            defaults={x.get('name'):x.get('default') for x in root.findall('arg')}
            forwarded={x.get('name'):x.get('value') for x in root.find('include').findall('arg')}
            for name in ['start_blend','predict_pose','refine_line']:
                self.assertEqual(defaults[name],'false')
                self.assertEqual(forwarded[name],'$(arg '+name+')')

    def test_recovery_entry_keeps_existing_validated_speed_and_timing_defaults(self):
        root=ET.parse(LAUNCH/'planned_race.launch').getroot()
        defaults={x.get('name'):x.get('default') for x in root.findall('arg')}
        self.assertEqual(defaults['braking'],'3.0')
        self.assertEqual(defaults['steering_rate'],'2.0')
        self.assertEqual(defaults['start_blend'],'false')
        self.assertEqual(defaults['predict_pose'],'false')


if __name__=='__main__':unittest.main()
