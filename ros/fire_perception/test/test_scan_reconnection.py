import ast
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock


class ScanReconnectTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/navigation_safety_guard.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'check_scan_connection')
        self.clock = NS(monotonic=MagicMock(return_value=10.0))
        env = dict(time=self.clock, rospy=MagicMock())
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
        self.check = env['check_scan_connection']
        self.robot = NS(lock=threading.RLock(), scan_received_at=0., scan_retry_at=0.,
                        scan_subscriber=MagicMock(), subscribe_scan=MagicMock(), scan=('old', []))

    def test_missing_messages_reconnect_without_refreshing_scan(self):
        old = self.robot.scan_subscriber
        self.check(self.robot, None)
        old.unregister.assert_called_once()
        self.robot.subscribe_scan.assert_called_once()
        self.assertEqual(self.robot.scan, ('old', []))
        self.check(self.robot, None)
        self.assertEqual(self.robot.subscribe_scan.call_count, 1)

    def test_healthy_connection_is_untouched(self):
        self.robot.scan_received_at = 9.
        self.check(self.robot, None)
        self.robot.subscribe_scan.assert_not_called()

    def test_failed_registration_retries_later(self):
        self.robot.subscribe_scan.side_effect = RuntimeError('endpoint restarting')
        self.check(self.robot, None)
        self.clock.monotonic.return_value = 16.
        self.check(self.robot, None)
        self.assertEqual(self.robot.subscribe_scan.call_count, 2)
