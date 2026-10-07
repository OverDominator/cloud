import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_safety_geometry import swept_rectangle_clear, swept_fire_clear


class SafetyGeometryTests(unittest.TestCase):
    def test_clear_motion(self):
        self.assertTrue(swept_rectangle_clear([], 1, .5, .6, 2.1, 2.8, .35))

    def test_forward_wall(self):
        self.assertFalse(swept_rectangle_clear([(2.6, 0)], 1, 0, .6, 2.1, 2.8, .35))

    def test_rear_wall(self):
        self.assertFalse(swept_rectangle_clear([(-2.6, 0)], -1, 0, .6, 2.1, 2.8, .35))

    def test_intrusive_rotation(self):
        self.assertFalse(swept_rectangle_clear([(2.4, 2.8)], 0, -.9, .6, 2.1, 2.8, .35))

    def test_fire_approach(self):
        self.assertFalse(swept_fire_clear([(5,0,1)], (0,0,0), 1,0,1,3.5))

    def test_nonfinite_command(self):
        self.assertFalse(swept_rectangle_clear([], float('nan'),0,.6,2.1,2.8,.35))
