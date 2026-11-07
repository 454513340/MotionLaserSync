import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
from m0m5_prepare_scans import unwrap_sensor_timestamp

class ClockRolloverTest(unittest.TestCase):
    def test_continuous_128_wrap(self):
        seq=[(0,127.99),(1,127.995),(2,0.0001),(3,0.0051),(4,0.0101)]
        prev=float('-inf');rawprev=None;offset=0;oprev=-1;rollovers=0
        got=[]
        for ordinal,t in seq:
            value,offset,w=unwrap_sensor_timestamp(t,rawprev,prev,offset,ordinal,oprev,ordinal)
            got.append(value);rollovers+=int(w);prev=value;rawprev=t;oprev=ordinal
        self.assertEqual(rollovers,1)
        self.assertAlmostEqual(got[2],128.0001)
        self.assertTrue(all(b>a for a,b in zip(got,got[1:])))
    def test_reject_unrelated_time_backstep(self):
        with self.assertRaises(ValueError):
            unwrap_sensor_timestamp(100.0,100.1,100.1,0,11,10,11)
    def test_reject_large_gap(self):
        with self.assertRaises(ValueError):
            unwrap_sensor_timestamp(101.0,100.0,100.0,0,11,10,11)
    def test_reject_wrap_without_continuous_ordinal(self):
        with self.assertRaises(ValueError):
            unwrap_sensor_timestamp(.005,127.995,127.995,0,13,10,13)

if __name__=='__main__':unittest.main()
