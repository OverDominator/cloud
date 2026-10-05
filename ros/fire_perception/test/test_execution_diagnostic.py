import ast
import unittest
from pathlib import Path
from types import SimpleNamespace as NS


class Stamp(float):
    def __sub__(self, value):
        return Stamp(float(self)-float(value))
    def to_sec(self):
        return float(self)


class ExecutionDiagnosticTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/navigation_safety_guard.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'execution_diagnostic')
        env = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
        self.method = env['execution_diagnostic']

    def test_missing_feedback_is_not_zero_motion(self):
        obj = NS(unity_command=None, unity_status=('unavailable',None), odom=None)
        result = self.method(obj, Stamp(10))
        self.assertIn('unity_command=unavailable', result)
        self.assertIn('odom_vx_vy_w=unavailable', result)

    def test_command_and_actual_motion_remain_distinct(self):
        twist = NS(linear=NS(x=.05,y=0), angular=NS(z=.1))
        measured = NS(linear=NS(x=.001,y=0), angular=NS(z=0))
        obj = NS(unity_command=NS(header=NS(stamp=Stamp(9)),twist=twist),
                 unity_status=('normal',Stamp(9.5)),
                 odom=NS(twist=NS(twist=measured),pose=NS(pose=NS(position=NS(x=1,y=2)))))
        result = self.method(obj, Stamp(10))
        self.assertIn('unity_command=(0.0500,0.1000)', result)
        self.assertIn('odom_vx_vy_w=(0.0010,0.0000,0.0000)', result)
        self.assertIn('unity_age=1.000', result)
