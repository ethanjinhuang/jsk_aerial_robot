#!/usr/bin/env python3
"""Real ROS motion transport test, isolated master and test topics; no hardware.

The node runs with a paused ROS clock to exercise the wall-clock watchdog.
"""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-script', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'scripts/sonic_teleop.py')
    args = parser.parse_args()
    port = 11428
    try:
        connection = socket.create_connection(('127.0.0.1', port), timeout=.2)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError('test master port 11428 is already in use')
    output = Path(tempfile.mkdtemp(prefix='fish-motion-ros-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11428', ROS_IP='127.0.0.1',
                      ROS_HOME=str(output), ROS_LOG_DIR=str(output / 'log'))
    master_log = (output / 'master.log').open('w')
    node_log = (output / 'node.log').open('w')
    master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log, stderr=subprocess.STDOUT)
    node = None
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if master.poll() is not None:
                raise RuntimeError('isolated master failed; see ' + str(output))
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/motion_smoke')
                break
            except OSError:
                time.sleep(.05)
        else:
            raise RuntimeError('isolated master startup timeout')
        import rospy
        from sensor_msgs.msg import Joy
        from spinal.msg import ServoControlCmd
        rospy.init_node('sonic_motion_smoke', disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        lock = threading.Lock()
        received = []

        def receive(message):
            assert list(message.index) == [0] and len(message.cmd) == 1
            with lock:
                received.append((time.monotonic(), message.cmd[0]))

        sub = rospy.Subscriber('/motion_test/targets', ServoControlCmd, receive, queue_size=100)
        pub = rospy.Publisher('/motion_test/joy', Joy, queue_size=1)
        node = subprocess.Popen(
            [sys.executable, str(args.node_script), '__name:=sonic_motion_under_test',
             '/joy:=/motion_test/joy', '/servo/target_states:=/motion_test/targets',
             '_joy_timeout_s:=0.35', '_auto_frequency:=2.0'],
            stdout=node_log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if node.poll() is not None:
                raise RuntimeError('motion node failed; see ' + str(output))
            if pub.get_num_connections() and sub.get_num_connections():
                break
            time.sleep(.02)
        else:
            raise RuntimeError('ROS test topic connection timeout')
        time.sleep(.1)
        assert not received, 'node emitted a command before receiving Joy'

        def send(duration, pressed=(), steering=0., malformed=False):
            begin = time.monotonic()
            while time.monotonic() - begin < duration:
                msg = Joy()
                if not malformed:
                    msg.axes = [0., 0., steering, 1., 1.]
                    msg.buttons = [int(i in pressed) for i in range(8)]
                pub.publish(msg)
                time.sleep(.025)
            with lock:
                return [(t, value) for t, value in received if t >= begin + .06]

        def centered(rows):
            assert len(rows) >= 3 and all(value == 2048 for _, value in rows), rows

        rows = send(.7, pressed=(3,))
        assert len(rows) >= 20 and max(v for _, v in rows) > 2450 and min(v for _, v in rows) < 1650
        intervals = sorted(b[0] - a[0] for a, b in zip(rows, rows[1:]))
        median = intervals[len(intervals) // 2]
        assert .01 < median < .04, median
        assert all(1024 <= value <= 3072 for _, value in rows)
        print('PASS: automatic waveform and approximately 50 Hz output with paused ROS clock', flush=True)
        centered(send(.2, pressed=(1,)))
        centered(send(.2, pressed=(3,)))
        send(.1)
        centered(send(.15, pressed=(0,)))
        for steering, target in [(-.25, 2304), (-1., 3072), (1., 1024), (0., 2048)]:
            rows = send(.15, steering=steering)
            assert len(rows) >= 3 and all(v == target for _, v in rows), rows
        send(.12, pressed=(3,))
        for steering, target in [(-1., 3072), (1., 1024)]:
            rows = send(.15, steering=steering)
            assert len(rows) >= 3 and all(v == target for _, v in rows), rows
        rows = send(.6, steering=-.25)
        assert len(rows) >= 15 and max(v for _, v in rows) > 2750 and min(v for _, v in rows) < 1850, rows
        print('PASS: latched stop, direct manual positioning and direct auto center adjustment', flush=True)
        centered(send(.2, malformed=True))
        centered(send(.15, pressed=(3,)))
        send(.1)
        send(.1, pressed=(0,))
        rows = send(.2, steering=-1.)
        assert rows[-1][1] == 3072
        time.sleep(.55)
        with lock:
            tail = list(received[-5:])
        centered(tail)
        centered(send(.2, pressed=(3,)))
        send(.1)
        rows = send(.35, pressed=(3,))
        assert any(abs(value - 2048) > 150 for _, value in rows)
        centered(send(.2, pressed=(1,)))
        print('PASS: malformed input and Joy loss stop motion; fresh messages cannot silently restart it', flush=True)
        node.send_signal(signal.SIGINT)
        assert node.wait(timeout=5) == 0
        pub.unregister()
        sub.unregister()
        rospy.signal_shutdown('motion smoke complete')
        print('PASS: clean motion node shutdown; logs: ' + str(output), flush=True)
    finally:
        for process in (node, master):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        master_log.close()
        node_log.close()


if __name__ == '__main__':
    main()
