"""Exercise the legacy velocity callbacks without ROS or hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ('teleop.py', 'spring_teleop.py')


class ServoCommand:
    def __init__(self):
        self.index = []
        self.cmd = []


def make_node(filename):
    spec = importlib.util.spec_from_file_location('tested_' + filename[:-3], ROOT / 'scripts' / filename)
    module = importlib.util.module_from_spec(spec)
    modules = {
        'numpy': NS(),
        'rospy': NS(logwarn_throttle=Mock()),
        'spinal.msg': NS(ServoControlCmd=ServoCommand, ServoStates=object, Imu=object,
                         ServoTorqueCmd=object, ServoTorqueStates=object),
        'geometry_msgs.msg': NS(Twist=object),
        'sensor_msgs.msg': NS(Joy=object),
    }
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    node = module.Teleop.__new__(module.Teleop)
    node.vel_rate = 250.
    node.max_val = 250.
    node.cmd_pub = Mock()
    return node


class LegacyVelocityTests(unittest.TestCase):
    def command(self, node, velocity):
        node._twistCallback(NS(linear=NS(x=velocity)))
        message = node.cmd_pub.publish.call_args[0][0]
        self.assertEqual(message.index, [0])
        self.assertEqual(len(message.cmd), 1)
        self.assertIsInstance(message.cmd[0], int)
        return message.cmd[0]

    def test_scaled_commands_respect_positive_and_negative_limits(self):
        for filename in SCRIPTS:
            node = make_node(filename)
            for velocity, expected in ((0., 0), (.4, 100), (-.4, -100),
                                       (1., 250), (-1., -250), (2., 250), (-2., -250)):
                with self.subTest(script=filename, velocity=velocity):
                    self.assertEqual(self.command(node, velocity), expected)

    def test_custom_rate_and_limit_apply_to_the_output(self):
        for filename in SCRIPTS:
            node = make_node(filename)
            node.vel_rate, node.max_val = 50., 75.
            for velocity, expected in ((.5, 25), (-.5, -25), (2., 75), (-2., -75)):
                with self.subTest(script=filename, velocity=velocity):
                    self.assertEqual(self.command(node, velocity), expected)

    def test_nonfinite_input_or_scaled_overflow_sends_zero_then_recovers(self):
        for filename in SCRIPTS:
            for velocity in (float('nan'), float('inf'), -float('inf'), 1e308, -1e308):
                with self.subTest(script=filename, velocity=velocity):
                    node = make_node(filename)
                    self.assertEqual(self.command(node, .4), 100)
                    self.assertEqual(self.command(node, velocity), 0)
                    self.assertEqual(self.command(node, -.4), -100)
                    self.assertEqual(node.cmd_pub.publish.call_count, 3)


if __name__ == '__main__':
    unittest.main()
