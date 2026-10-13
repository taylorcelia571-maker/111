"""Projection geometry must agree with feedback, including seam corrections."""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from race_line import ClosedPath
from predictive_follower import PredictiveFollower


class ProjectionConsistencyTests(unittest.TestCase):
    def path(self):
        a = np.linspace(0, 2*math.pi, 401)
        return ClosedPath(np.column_stack((10*np.cos(a), 10*np.sin(a))))

    def test_window_endpoints_and_seam_return_matching_distance(self):
        p = self.path()
        for begin in (.013, p.length-.047):
            end = begin+.083
            for query in (begin-.2, begin+.04, end+.2):
                x, y = p.sample(query)
                s, error = p.project(x, y, begin, end)
                self.assertGreaterEqual(s, begin)
                self.assertLessEqual(s, end)
                self.assertAlmostEqual(error, np.linalg.norm(p.sample(s)-[x,y]), places=9)

    def test_local_backward_correction_does_not_become_control_error(self):
        p = self.path()
        f = PredictiveFollower(p, lateral_control='race_feedback', max_error=.15,
                               start_blend=True)
        s = 1.0
        x,y = p.sample(s)
        tangent = p.sample(s+.08)-p.sample(s-.08)
        yaw = math.atan2(tangent[1], tangent[0])
        f.start(x,y,yaw)
        self.assertIsNotNone(f.connector_offset)
        f.progress = s+.08
        f.step(x,y,yaw,1.,.02)
        self.assertGreaterEqual(f.progress,s+.08)
        self.assertLess(f.last_report['control_tracking_error'],1e-8)
        self.assertLess(abs(f.last_report['signed_tracking_error']),1e-8)
        self.assertAlmostEqual(f.last_report['projected_progress'],s,places=8)

    def test_reference_corridor_guard_remains_enforced(self):
        p = self.path()
        f = PredictiveFollower(p, lateral_control='race_feedback', max_error=.15)
        x,y = p.sample(1.)
        tangent = p.sample(1.08)-p.sample(.92)
        f.start(x,y,math.atan2(tangent[1],tangent[0]))
        with self.assertRaisesRegex(ValueError,'safe corridor'):
            f.step(x*1.05,y*1.05,math.atan2(tangent[1],tangent[0]),1.,.02)


if __name__ == '__main__':
    unittest.main()
