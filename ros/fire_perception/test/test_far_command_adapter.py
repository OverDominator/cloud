"""Transport-stubbed regression checks; optional reference is external legacy source.

python test_far_command_adapter.py --reference /path/to/original/controller.py
This is not a ROS integration test or a replay of the historical trials.
"""
import argparse
import ast
import copy
import random
import threading
from pathlib import Path
from types import SimpleNamespace as NS


class Stamp(float):
    @classmethod
    def now(cls):
        return cls(100)


def message(**kwargs):
    return NS(header=NS(seq=0, stamp=Stamp(0), frame_id=''),
              twist=NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.)), **kwargs)


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


params = {'~normal_max_forward_speed': 1.12}
ros = NS(Time=Stamp, Duration=Stamp, Publisher=Publisher,
         get_param=lambda key, default=None: params.get(key, default),
         Subscriber=lambda *a, **kw: None, Timer=lambda *a: None,
         logerr_throttle=lambda *a: None)


def load(path, classname, selected=None):
    tree = ast.parse(path.read_text(encoding='utf-8-sig'))
    tree.body = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    if selected:
        tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == classname or isinstance(n, ast.FunctionDef) and n.name == 'clamp']
        for n in tree.body:
            if isinstance(n, ast.ClassDef):
                n.body = [m for m in n.body if isinstance(m, ast.FunctionDef) and m.name in selected]
    space = dict(rospy=ros, threading=threading, TwistStamped=message, Odometry=message,
                 Bool=message, String=message, UInt32=message)
    exec(compile(tree, str(path), 'exec'), space)
    return space[classname]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args()
    adapter = load(Path(__file__).resolve().parents[1]/'scripts/far_command_adapter.py', 'FarCommandAdapter')
    a = adapter()
    assert [p.messages[0].data for p in a.status_publishers] == ['normal', False, 0, '']
    a.odom_callback(message())
    command = message(); command.twist.linear.x = 3.; command.twist.angular.z = 2.
    a.raw_callback(command); a.control(None)
    assert a.cmd_pub.messages[-1].twist.linear.x == 1.12
    assert a.cmd_pub.messages[-1].twist.angular.z == .45
    a.mission_active_callback(message(data=False))
    assert a.raw_command is None and a.cmd_pub.messages[-1].twist.linear.x == 0
    a.mission_active_callback(message(data=True)); a.control(None)
    assert a.cmd_pub.messages[-1].twist.linear.x == 0
    params['~enable_active_recovery'] = True
    try:
        adapter()
    except RuntimeError:
        pass
    else:
        raise AssertionError('active recovery accepted')
    del params['~enable_active_recovery']
    print('PASS: status, speed limits, mission stop/restart, active-recovery rejection')
    if args.reference:
        legacy = load(args.reference, 'LocalWallRecoveryController', {'control', 'raw_copy', 'make_command'})
        rng = random.Random(200)
        cases = 0
        for active in (False, True):
            for odom_age in (None, 0., .8, .800001, 2.):
                for cmd_age in (None, 0., .6, .600001, 2.):
                    for speed in (-2., -.55, 0., .749999, .75, 1.12, 3.):
                        for yaw in (-2., 0., 2.):
                            a = adapter(); a.mission_active = active
                            a.odom = None if odom_age is None else message()
                            a.odom_time = Stamp(100-(odom_age or 0))
                            m = message(); m.header.seq = 17; m.header.frame_id = rng.choice(['', 'vehicle', 'map'])
                            m.twist.linear.x = speed; m.twist.angular.z = yaw
                            m.twist.linear.y = .1; m.twist.angular.x = -.2
                            a.raw_command = None if cmd_age is None else m
                            a.raw_command_time = Stamp(100-(cmd_age or 0))
                            b = legacy.__new__(legacy)
                            b.__dict__.update(a.__dict__)
                            b.cmd_pub = Publisher(); b.enable_active_recovery = False
                            b.obstacle_ranges = lambda: (10., 10., 10.)
                            a.control(None); b.control(None)
                            assert vars(a.cmd_pub.messages[-1]) == vars(b.cmd_pub.messages[-1]), (active, odom_age, cmd_age, speed, yaw)
                            cases += 1
        print('PASS: legacy inactive-path command equivalence,', cases, 'cases')


if __name__ == '__main__':
    main()
