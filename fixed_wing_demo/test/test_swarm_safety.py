import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from swarm_safety import closest_approach,avoidance_targets


class SafetyTests(unittest.TestCase):
    def test_head_on(self):
        distance,predicted,t=closest_approach((0,0,60),(18,0,0),(180,0,60),(-18,0,0))
        self.assertEqual(distance,180)
        self.assertAlmostEqual(predicted,0)
        self.assertAlmostEqual(t,5)

    def test_crossing_with_vertical_separation(self):
        _,predicted,_=closest_approach((-90,0,60),(18,0,0),(0,-90,78),(0,18,0))
        self.assertAlmostEqual(predicted,18)

    def test_receding_and_stationary(self):
        self.assertEqual(closest_approach((0,0,0),(-1,0,0),(6,0,0),(1,0,0)),(6,6,0))
        self.assertEqual(closest_approach((0,0,0),(0,0,0),(18,0,0),(0,0,0)),(18,18,0))

    def test_three_aircraft_conflict_keeps_stack_separated(self):
        height,speed=avoidance_targets([60,78,96],[(0,1),(1,2)])
        self.assertLess(height[0],60)
        self.assertGreater(height[2],96)
        self.assertTrue(all(b-a>=12 for a,b in zip(height,height[1:])))
        self.assertTrue(all(16<=s<=20 for s in speed))


if __name__=='__main__':unittest.main()
