"""Safety equivalence of batched sampling and obstacle pruning to the old guard."""
import math
import unittest
from types import SimpleNamespace as NS
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent/'fixtures'))
from race_line import ClosedPath
from predictive_follower import PredictiveFollower,reference_clearance
from clearance_scalar import reference_clearance as scalar_guard

class BatchClearanceTests(unittest.TestCase):
    def setUp(self):
        a=np.linspace(0,2*math.pi,1000)
        self.path=ClosedPath(3*np.column_stack([np.cos(a),np.sin(a)]),spacing=.04)
        self.f=PredictiveFollower(self.path,start_blend=True,lateral_control='smooth_pursuit')
        self.f.start(3,0,math.pi/2)

    def test_cyclic_samples_and_connector_match_scalar_at_seams(self):
        values=np.r_[np.linspace(-2,3*self.path.length,1001),self.path.length,0]
        np.testing.assert_allclose(self.path.sample_many(values),[self.path.sample(s) for s in values],atol=1e-12)
        self.f.connector_offset=np.array([.07,-.02])
        np.testing.assert_allclose(self.f.sample_tracking_many(values),[self.f.sample_tracking(s) for s in values],atol=1e-12)
        self.assertEqual(self.path.sample_many([]).shape,(0,2))
        with self.assertRaises(ValueError):self.path.sample_many([math.nan])

    def test_random_scans_heading_offsets_and_connectors_preserve_first_hit(self):
        rng=np.random.RandomState(812)
        for case in range(180):
            progress=float(rng.uniform(-1,2*self.path.length))
            q=self.path.sample(progress)
            yaw=math.atan2(q[1],q[0])+math.pi/2+float(rng.uniform(-.12,.12))
            pose=(q[0]+float(rng.uniform(-.06,.06)),q[1]+float(rng.uniform(-.06,.06)),yaw,0)
            ranges=rng.uniform(.08,8,360)
            ranges[rng.choice(len(ranges),40,replace=False)]=np.inf
            if case%8==0:ranges[0:50]=np.nan
            if case%9==0:ranges[:]=np.inf
            scan=NS(ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/len(ranges),range_min=.05,range_max=10.)
            laser=(float(rng.uniform(0,.25)),float(rng.uniform(-.03,.03)),float(rng.uniform(-.05,.05)))
            self.f.connector_offset=np.array([.03,-.02]) if case%2 else None
            kwargs=dict(reaction=.25,sample=self.f.sample_tracking)
            old,new={},{}
            speed=float(rng.uniform(.1,6.))
            expected=scalar_guard(self.path,progress,pose,scan,laser,speed,diagnostics=old,**kwargs)
            actual=reference_clearance(self.path,progress,pose,scan,laser,speed,diagnostics=new,
                                      sample_many=self.f.sample_tracking_many,**kwargs)
            self.assertEqual(new['reason'],old['reason'])
            self.assertAlmostEqual(actual,expected,places=10)
            for name in ['known_forward','hit_arc','closest']:
                if name in old:self.assertAlmostEqual(new[name],old[name],places=10)

    def test_custom_scalar_sampler_is_not_silently_bypassed(self):
        calls=[]
        def sample(s):calls.append(s);return self.path.sample(s)
        scan=NS(ranges=np.ones(100),angle_min=-math.pi,angle_increment=2*math.pi/100,range_min=.05,range_max=10.)
        reference_clearance(self.path,0,(3,0,math.pi/2,1),scan,(0,0,0),1,sample=sample)
        self.assertTrue(calls)

if __name__=='__main__':unittest.main()
