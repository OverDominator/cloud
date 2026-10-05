import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS


class NavigationSafetyTurnLimitTests(unittest.TestCase):
    def setUp(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "scripts/navigation_safety_guard.py"
        )
        tree = ast.parse(path.read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        method = next(
            node for node in cls.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "safe_reduced_turn"
        )
        self.wall_attempts = []

        def wall_clear(_points, _v, w, *_geometry):
            self.wall_attempts.append(w)
            return abs(w) <= 0.4

        env = {
            "math": math,
            "swept_rectangle_clear": wall_clear,
            "swept_fire_clear": lambda *_args: True,
        }
        exec(
            compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"),
            env,
        )
        self.method = env["safe_reduced_turn"]
        self.guard = NS(
            in_place_linear_threshold=0.08,
            safe_turn_retry_speeds=[0.55, 0.4, 0.25, 0.15],
            motion_horizon=0.6,
            half_length=0.675,
            half_width=0.9,
            margin=0.30,
            zones={},
        )

    def test_chooses_fastest_safe_speed_in_requested_direction(self):
        selected = self.method(self.guard, [], (0, 0, 0), 0.0, -0.9)
        self.assertAlmostEqual(selected, -0.4)
        self.assertEqual(self.wall_attempts, [-0.55, -0.4])

    def test_does_not_shape_translating_command(self):
        selected = self.method(self.guard, [], (0, 0, 0), 0.2, 0.9)
        self.assertIsNone(selected)
        self.assertEqual(self.wall_attempts, [])

    def test_does_not_increase_an_already_slow_turn(self):
        selected = self.method(self.guard, [], (0, 0, 0), 0.0, 0.1)
        self.assertIsNone(selected)
        self.assertEqual(self.wall_attempts, [])


if __name__ == "__main__":
    unittest.main()
