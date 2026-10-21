#!/usr/bin/env python3
"""Isolated ROS cycle-steering test with synthetic ADC/Joy, no hardware."""
import argparse
from pathlib import Path
import os
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
    port = 11429
    try:
        connection = socket.create_connection(('127.0.0.1', port), timeout=.2)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError('test master port 11429 is already in use')
    output = Path(tempfile.mkdtemp(prefix='fish-cycle-ros-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_IP='127.0.0.1',
                      ROS_HOME=str(output), ROS_LOG_DIR=str(output / 'log'))
    os.environ.pop('ROS_HOSTNAME', None)
    logs = []
    processes = []
    try:
        log = (output / 'master.log').open('w'); logs.append(log)
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=subprocess.STDOUT)
        processes.append(master)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if master.poll() is not None:
                raise RuntimeError('isolated master failed; see ' + str(output))
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/cycle_smoke')
                break
            except OSError:
                time.sleep(.05)
        else:
            raise RuntimeError('isolated master startup timeout')
        import rospy
        from sensor_msgs.msg import Joy
        from spinal.msg import ServoControlCmd
        from robotic_fish_sonic.msg import AdcSample, AdcSampleArray, AcousticSteeringState
        from robotic_fish_sonic.srv import SetAcousticSteering
        rospy.init_node('cycle_steering_smoke', disable_signals=True)

        for case, (frequency, polarity) in enumerate(((1.5, 1), (2., -1))):
            name = 'cycle_motion_' + str(case)
            prefix = '/cycle_test_' + str(case)
            log = (output / (name + '.log')).open('w'); logs.append(log)
            lock = threading.Lock()
            states, targets = [], []
            def receive_state(msg):
                with lock:
                    states.append((time.monotonic(), msg))
            def receive_target(msg):
                assert list(msg.index) == [0] and len(msg.cmd) == 1
                with lock:
                    targets.append(msg.cmd[0])
            sub = rospy.Subscriber(prefix + '/state', AcousticSteeringState, receive_state, queue_size=200)
            target_sub = rospy.Subscriber(prefix + '/targets', ServoControlCmd, receive_target, queue_size=200)
            joy_pub = rospy.Publisher(prefix + '/joy', Joy, queue_size=1)
            adc_pub = rospy.Publisher(prefix + '/adc', AdcSampleArray, queue_size=1)
            node = subprocess.Popen(
                [sys.executable, str(args.node_script), '__name:=' + name,
                 '/joy:=' + prefix + '/joy', '/servo/target_states:=' + prefix + '/targets',
                 '/robotic_fish/adc/samples:=' + prefix + '/adc',
                 '/robotic_fish/acoustic/steering_state:=' + prefix + '/state',
                 '_acoustic_steering/enabled:=true', '_acoustic_steering/bias:=128.0',
                 '_acoustic_steering/left_bias_sign:=' + str(polarity),
                 '_auto_frequency:=' + str(frequency)], stdout=log, stderr=subprocess.STDOUT)
            processes.append(node)
            driving = {'button': None, 'axis': 0., 'left': 2., 'right': 1., 'valid': True, 'adc': True}
            finish = threading.Event()

            def drive():
                while not finish.is_set() and not rospy.is_shutdown():
                    with lock:
                        values = dict(driving)
                    joy = Joy(axes=[0., 0., values['axis'], 1., 1.],
                              buttons=[int(i == values['button']) for i in range(14)])
                    joy_pub.publish(joy)
                    if values['adc']:
                        stamp = rospy.Time.now()
                        samples = [AdcSample(channel_id=i, timestamp=stamp, adc_code=100,
                                             volt_raw=v, volt_cali=v, status_cali=values['valid'],
                                             cali_id='synthetic', status_dac_feedback=True, dac_volt=.9)
                                   for i, v in enumerate((1., values['left'], values['right']))]
                        adc_pub.publish(AdcSampleArray(samples=samples))
                    finish.wait(.025)

            def set_input(**kwargs):
                with lock:
                    driving.update(kwargs)

            def wait_state(predicate, timeout=3.):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if node.poll() is not None:
                        raise RuntimeError('motion node exited; see ' + str(output))
                    with lock:
                        last = states[-1][1] if states else None
                    if last is not None and predicate(last):
                        return last
                    time.sleep(.01)
                raise AssertionError('state timeout; last=' + str(last) + '; logs: ' + str(output))

            def wait_target(value):
                deadline = time.monotonic() + 1.
                while time.monotonic() < deadline:
                    with lock:
                        if targets and targets[-1] == value:
                            return
                    time.sleep(.01)
                raise AssertionError('target timeout; expected ' + str(value))

            def share_report():
                path = output / (name + '.log')
                marker = 'Sonic status (Share):'
                before = path.read_text().count(marker)
                set_input(button=8)
                deadline = time.monotonic() + 2.
                while time.monotonic() < deadline:
                    content = path.read_text()
                    if content.count(marker) == before + 1:
                        break
                    if node.poll() is not None:
                        raise RuntimeError('node exited during Share report; see ' + str(output))
                    time.sleep(.01)
                else:
                    raise AssertionError('Share did not print status; see ' + str(output))
                time.sleep(.12)
                content = path.read_text()
                assert content.count(marker) == before + 1, 'held Share printed repeatedly'
                set_input(button=None)
                return content.rsplit(marker, 1)[1]

            thread = threading.Thread(target=drive, daemon=True)
            try:
                deadline = time.monotonic() + 8
                while not (joy_pub.get_num_connections() and adc_pub.get_num_connections()):
                    if node.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('test topic connection failed; see ' + str(output))
                    time.sleep(.02)
                thread.start()
                wait_state(lambda s: s.motion_mode == 'manual')
                set_input(axis=.25)
                time.sleep(.15)
                with lock:
                    assert targets[-1] == 1792, targets[-5:]
                # Triangle always selects ordinary auto, even without acoustic data.
                set_input(button=3, adc=False)
                wait_state(lambda s: s.motion_mode == 'auto' and s.command_bias == -256.)
                set_input(button=None)
                time.sleep(.3)
                with lock:
                    assert states[-1][1].motion_mode == 'auto'
                set_input(button=1)
                wait_state(lambda s: s.motion_mode == 'stopped' and s.status == 'stop_button')
                set_input(axis=0., button=None, adc=True)
                time.sleep(.12)
                set_input(button=2)
                wait_state(lambda s: s.motion_mode == 'tracking' and s.status == 'warmup')
                set_input(button=None)
                state = wait_state(lambda s: s.cycle_index >= 1 and s.status == 'active')
                assert abs(state.window_s - 1. / frequency) < 1e-9
                assert abs((state.window_end - state.window_start).to_sec() - 1. / frequency) < 1e-6
                assert abs(state.normalized_difference - 1 / 3) < 1e-9
                assert state.command_bias == polarity * 128.
                with lock:
                    updates = [s for _, s in states if s.decision_updated]
                assert len(updates) == 1, updates
                report = share_report()
                assert 'ADC instant raw [V]: head=1.00000 left=2.00000 right=1.00000' in report
                assert 'Cycle calibrated mean [V]: head=1.00000 left=2.00000 right=1.00000' in report
                assert 'delta [V]=+1.00000 normalized=+0.3333' in report
                assert 'Direction=left last_decision=left' in report
                # Re-press Circle: keep tracking and the existing phase/window.
                set_input(button=2)
                time.sleep(.1)
                with lock:
                    assert states[-1][1].motion_mode == 'tracking'
                    assert states[-1][1].status == 'active'
                set_input(button=None)
                rospy.wait_for_service('/' + name + '/set_acoustic_steering', timeout=3)
                configure = rospy.ServiceProxy('/' + name + '/set_acoustic_steering', SetAcousticSteering)
                assert not configure(-1., 64.).success
                assert configure(.6, 64.).success
                wait_state(lambda s: s.parameters_pending)
                state = wait_state(lambda s: s.cycle_index >= 2 and not s.parameters_pending)
                assert state.command_bias == 0. and state.turn_bias == 64.
                assert configure(.1, 64.).success
                state = wait_state(lambda s: s.cycle_index >= 3 and s.command_bias == polarity * 64.)
                set_input(left=1., right=3.)
                wait_state(lambda s: s.command_bias == -polarity * 64. and s.status == 'active')
                with lock:
                    updates = [s for _, s in states if s.decision_updated]
                for before, after in zip(updates, updates[1:]):
                    assert after.cycle_index == before.cycle_index + 1
                    assert abs((after.window_end - before.window_end).to_sec() - 1. / frequency) < 1e-6
                set_input(left=1., right=1.)
                wait_state(lambda s: s.command_bias == 0. and s.status == 'active')
                set_input(button=1)
                wait_state(lambda s: s.motion_mode == 'stopped' and s.status == 'stop_button')
                wait_target(2048)
                report = share_report()
                assert 'historical=True' in report and 'Direction=stopped' in report
                set_input(button=None)
                time.sleep(.1)
                set_input(button=2)
                wait_state(lambda s: s.motion_mode == 'tracking' and s.status == 'warmup')
                set_input(button=None)
                wait_state(lambda s: s.cycle_index == 1 and s.status == 'active')
                # Hold Circle before the fault: a released frame followed by
                # a new press after stopping would be legitimate recovery.
                set_input(button=2)
                time.sleep(.1)
                set_input(valid=False)
                wait_state(lambda s: s.motion_mode == 'stopped' and 'calibration_fallback' in s.status)
                set_input(valid=True)
                time.sleep(.15)
                with lock:
                    assert states[-1][1].motion_mode == 'stopped'
                    assert targets[-1] == 2048
                set_input(button=None)
                time.sleep(.1)
                set_input(button=2)
                wait_state(lambda s: s.motion_mode == 'tracking' and s.status == 'warmup')
                set_input(button=None)
                wait_state(lambda s: s.cycle_index == 1 and s.status == 'active')
                set_input(adc=False)
                wait_state(lambda s: s.motion_mode == 'stopped' and 'adc_timeout' in s.status)
                set_input(adc=True)
                time.sleep(.15)
                with lock:
                    assert states[-1][1].motion_mode == 'stopped'
                    assert all(1024 <= value <= 3072 for value in targets)
                set_input(button=0)
                wait_state(lambda s: s.motion_mode == 'manual')
                set_input(button=1)
                wait_state(lambda s: s.motion_mode == 'stopped' and s.status == 'stop_button')
                print('PASS: f=%.1f Hz, W=1/f, one decision/cycle, polarity=%d, '
                      'Circle tracking, Triangle ordinary auto, Cross stop, '
                      'Share snapshots, live settings and fault recovery' % (frequency, polarity), flush=True)
            finally:
                finish.set()
                if thread.is_alive(): thread.join(timeout=2)
                joy_pub.unregister(); adc_pub.unregister(); sub.unregister(); target_sub.unregister()
                if node.poll() is None:
                    node.send_signal(signal.SIGINT)
                    assert node.wait(timeout=5) == 0
        rospy.signal_shutdown('cycle steering smoke complete')
        print('PASS: clean shutdown; isolated logs: ' + str(output), flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=2)
        for log in logs: log.close()


if __name__ == '__main__':
    main()
