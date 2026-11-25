"""Exercise sender callbacks with fake ROS imports, clocks and socket; no I/O."""

import importlib
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


def stub_module(name, **members):
    module = ModuleType(name)
    module.__dict__.update(members)
    return module


modules = {
    "rclpy": stub_module("rclpy"),
    "rclpy.node": stub_module("rclpy.node", Node=object),
    "rclpy.clock": stub_module("rclpy.clock", Clock=object, ClockType=object),
    "rclpy.executors": stub_module("rclpy.executors", ExternalShutdownException=RuntimeError),
    "geometry_msgs": stub_module("geometry_msgs"),
    "geometry_msgs.msg": stub_module("geometry_msgs.msg", TwistStamped=object),
}
with patch.dict(sys.modules, modules):
    bridge_module = importlib.import_module("bit_standalone.can_bridge")
CanBridge = bridge_module.CanBridge


class CanBridgeChecks(unittest.TestCase):
    def setUp(self):
        self.node = CanBridge.__new__(CanBridge)
        self.node.scale, self.node.timeout = 1.0, 0.1
        self.node.command, self.node.received_at, self.node.stamp_ns = (0, 0), float("-inf"), 0
        self.node.socket = Mock()
        self.node.socket.send.return_value = 16
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
        self.node.get_logger = Mock()
        self.node.timer = Mock()

    def message(self, sec=10, frame="base_link"):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=0)),
            twist=SimpleNamespace(linear=SimpleNamespace(x=0.5, y=-0.5)),
        )

    def test_startup_sends_zero_and_recent_command_sends_scaled_bytes(self):
        with patch.object(bridge_module.time, "monotonic", return_value=1.0):
            self.node.send_frame()
            self.assertEqual(self.node.socket.send.call_args.args[0][8:], bytes(8))
            self.node.on_command(self.message())
            self.node.send_frame()
            self.assertEqual(self.node.socket.send.call_args.args[0][8:], bytes([63, 193, 0, 0, 0, 0, 0, 0]))

    def test_no_new_message_expires_by_monotonic_time(self):
        with patch.object(bridge_module.time, "monotonic", return_value=1.0):
            self.node.on_command(self.message())
        with patch.object(bridge_module.time, "monotonic", return_value=1.101):
            self.node.send_frame()
        self.assertEqual(self.node.socket.send.call_args.args[0][8:], bytes(8))

    def test_delayed_message_does_not_become_fresh_on_receipt(self):
        with patch.object(bridge_module.time, "monotonic", return_value=1.0):
            self.node.on_command(self.message(sec=9))
            self.node.send_frame()
        self.assertEqual(self.node.socket.send.call_args.args[0][8:], bytes(8))

    def test_wrong_coordinate_frame_is_rejected(self):
        with self.assertRaises(ValueError):
            self.node.on_command(self.message(frame="odom"))
        self.assertEqual(self.node.received_at, float("-inf"))

    def test_bus_error_is_propagated(self):
        self.node.socket.send.side_effect = OSError("bus unavailable")
        with self.assertRaises(OSError):
            self.node.send_frame()

    def test_shutdown_attempts_zero_then_closes(self):
        self.node.close()
        self.assertEqual(self.node.socket.send.call_args.args[0][8:], bytes(8))
        self.node.socket.close.assert_called_once()

    def test_shutdown_reports_failed_zero_and_still_closes(self):
        self.node.socket.send.side_effect = OSError("bus unavailable")
        self.node.close()
        self.node.get_logger.return_value.error.assert_called_once()
        self.node.socket.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
