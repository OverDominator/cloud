import ast
import json
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace
import math

tree = ast.parse((Path(__file__).parents[1] / 'scripts/fire_keepout_cloud.py').read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'layout_callback')
namespace = dict(json=json, math=math, rospy=SimpleNamespace(logwarn=lambda *a: None))
exec(compile(ast.Module(body=[method], type_ignores=[]), '<layout_callback>', 'exec'), namespace)

class FireLayoutResetTests(unittest.TestCase):
    def test_new_layout_replaces_old_zones_even_outside_merge_radius(self):
        robot = SimpleNamespace(require_simulated_truth=True, lock=threading.Lock(),
                                ground_z=.1, zones=[[99,99,0]])
        def send(x):
            namespace['layout_callback'](robot, SimpleNamespace(data=json.dumps(
                dict(objects=[dict(name='FireSource_01',position=dict(x=x,z=5))]))))
        send(2)
        self.assertEqual(robot.zones, [[5,-2,.1]])
        send(20)
        self.assertEqual(robot.zones, [[5,-20,.1]])

    def test_empty_layout_removes_old_fire(self):
        robot = SimpleNamespace(require_simulated_truth=True, lock=threading.Lock(),
                                ground_z=0, zones=[[1,2,0]])
        namespace['layout_callback'](robot, SimpleNamespace(data='{"objects": []}'))
        self.assertEqual(robot.zones, [])
