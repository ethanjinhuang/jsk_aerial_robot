#!/usr/bin/env python3
"""Isolated ROS cycle-steering test with synthetic ADC/Joy, no hardware."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client
from xml.etree import ElementTree


RECORD_TOPICS = {'/robotic_fish/adc/samples': 'adc',
                 '/robotic_fish/acoustic/steering_state': 'state',
                 '/robotic_fish/sonic/motion_state': 'motion',
                 '/robotic_fish/sonic/config': 'config',
                 '/robotic_fish/acoustic/steering_events': 'events',
                 '/robotic_fish/sonic/status_snapshot': 'snapshot',
                 '/servo/target_states': 'targets', '/joy': 'joy'}


def verify_recording(directory):
    """Read a bag made by the real record.launch, including late latched config."""
    import rosbag
    paths = list(directory.glob('sonic_*.bag'))
    assert len(paths) == 1 and not list(directory.glob('*.bag.active')), paths
    recorded = defaultdict(list)
    with rosbag.Bag(str(paths[0])) as bag:
        for topic, message, _ in bag.read_messages():
            # rosbag stores the requested topic name, even when its subscription is remapped.
            recorded[RECORD_TOPICS.get(topic, topic)].append(message)
    for topic in ('adc', 'state', 'motion', 'config', 'events', 'targets', 'snapshot'):
        assert recorded[topic], 'missing recording: ' + topic
    assert recorded['/rosout'], 'record.launch did not record ROS logs'
    configs = [json.loads(m.config_json) for m in recorded['config']]
    assert all(c['schema_version'] == 4 and
               c['settings']['acoustic_steering']['difference_reference'] == 'channel_mean' and
               c['settings']['acoustic_steering']['voltage_offset_reference'] == 'left_minus_right' and
               c['settings']['acoustic_steering']['threshold_inclusive'] for c in configs)
    assert configs[0]['settings']['acoustic_steering']['bias'] == 128., 'late startup config lost'
    assert configs[0]['pending_request'] is None
    assert any(c['pending_request'] for c in configs), 'pending configuration lost'
    assert configs[-1]['pending_request'] is None and configs[-1]['applied_request_id'] > 0
    motions = recorded['motion']
    assert {'manual', 'auto', 'tracking', 'stopped'} <= {m.motion_mode for m in motions}
    assert any(not m.command_published and math.isnan(m.target_position) for m in motions)
    assert {'valid', 'calibration_fallback', 'gain_unknown', 'stale_sample'} <= {m.adc_quality for m in motions}
    assert any(m.reason == 'acoustic_adc_timeout' for m in motions)
    for m in motions:
        if not m.command_published:
            continue
        assert m.target_command == int(m.target_position)
        if m.motion_mode in ('auto', 'tracking'):
            assert 0 <= m.phase_rad < 2 * math.pi
            expected = 2048. + m.applied_bias + m.effective_amplitude * math.sin(m.phase_rad)
            assert abs(m.target_position - expected) < 1e-7
        else:
            assert math.isnan(m.phase_rad)
    states = recorded['state']
    assert all(m.difference_reference == 'channel_mean' for m in states)
    decisions = [m for m in states if m.decision_updated]
    assert decisions and all(m.has_decision and m.decision_current and m.head_mean == 1. for m in decisions)
    assert all(m.voltage_difference == m.left_mean - m.right_mean for m in decisions)
    for m in decisions:
        assert abs(m.compensated_voltage_difference - (m.voltage_difference + m.lr_voltage_offset_v)) < 1e-9
        assert abs(m.compensated_left_mean - (m.left_mean + m.lr_voltage_offset_v / 2.)) < 1e-9
        assert abs(m.compensated_right_mean - (m.right_mean - m.lr_voltage_offset_v / 2.)) < 1e-9
        assert abs(m.compensated_voltage_difference -
                   (m.compensated_left_mean - m.compensated_right_mean)) < 1e-9
        assert abs(m.compensated_difference - m.compensated_voltage_difference / ((m.left_mean + m.right_mean) / 2)) < 1e-9
    assert any(m.direction == 'holding' and m.has_decision and not m.decision_current for m in states)
    events = recorded['events']
    assert all(m.difference_reference == 'channel_mean' for m in events)
    assert {'accepted', 'rejected', 'applied'} <= {m.event for m in events}
    for applied in (m for m in events if m.event == 'applied'):
        assert applied.cycle_index > 0
        assert any(m.event == 'accepted' and m.request_id == applied.request_id and
                   (m.threshold, m.turn_bias, m.lr_voltage_offset_v) ==
                   (applied.threshold, applied.turn_bias, applied.lr_voltage_offset_v) for m in events)
    print('PASS: actual record.launch bag readback: motion/phase/ADC quality, cycle means, '
          'late latched config, parameter request/application events and ROS logs; ' + str(paths[0]), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-script', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'scripts/sonic_teleop.py')
    parser.add_argument('--console-script', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'robotic_fish_sonic/scripts/status_console.py')
    parser.add_argument('--record-launch', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'launch/record.launch')
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
        from std_msgs.msg import String
        from spinal.msg import ServoControlCmd
        from robotic_fish_sonic.msg import AdcSample, AdcSampleArray, AcousticSteeringState
        from robotic_fish_sonic.srv import SetAcousticSteering
        rospy.init_node('cycle_steering_smoke', disable_signals=True)

        for case, (frequency, polarity) in enumerate(((1.5, 1), (2., -1))):
            name = 'cycle_motion_' + str(case)
            prefix = '/cycle_test_' + str(case)
            log = (output / (name + '.log')).open('w'); logs.append(log)
            lock = threading.Lock()
            states, targets, snapshots = [], [], []
            def receive_state(msg):
                with lock:
                    states.append((time.monotonic(), msg))
            def receive_target(msg):
                assert list(msg.index) == [0] and len(msg.cmd) == 1
                with lock:
                    targets.append(msg.cmd[0])
            def receive_snapshot(msg):
                with lock:
                    snapshots.append(msg.data)
            sub = rospy.Subscriber(prefix + '/state', AcousticSteeringState, receive_state, queue_size=200)
            target_sub = rospy.Subscriber(prefix + '/targets', ServoControlCmd, receive_target, queue_size=200)
            status_sub = rospy.Subscriber(prefix + '/snapshot', String, receive_snapshot, queue_size=10)
            joy_pub = rospy.Publisher(prefix + '/joy', Joy, queue_size=1)
            adc_pub = rospy.Publisher(prefix + '/adc', AdcSampleArray, queue_size=1)
            node = subprocess.Popen(
                [sys.executable, str(args.node_script), '__name:=' + name,
                 '/joy:=' + prefix + '/joy', '/servo/target_states:=' + prefix + '/targets',
                 '/robotic_fish/adc/samples:=' + prefix + '/adc',
                 '/robotic_fish/acoustic/steering_state:=' + prefix + '/state',
                 '/robotic_fish/sonic/status_snapshot:=' + prefix + '/snapshot',
                 '/robotic_fish/sonic/motion_state:=' + prefix + '/motion',
                 '/robotic_fish/sonic/config:=' + prefix + '/config',
                 '/robotic_fish/acoustic/steering_events:=' + prefix + '/events',
                 '_acoustic_steering/enabled:=true', '_acoustic_steering/bias:=128.0',
                 '_acoustic_steering/left_bias_sign:=' + str(polarity),
                 '_auto_frequency:=' + str(frequency)], stdout=log, stderr=subprocess.STDOUT)
            processes.append(node)
            console_log = (output / (name + '_console.log')).open('w'); logs.append(console_log)
            console = subprocess.Popen(
                [sys.executable, str(args.console_script), '__name:=' + name + '_console',
                 '/robotic_fish/sonic/status_snapshot:=' + prefix + '/snapshot',
                 '_color:=' + ('auto' if case == 0 else 'always')],
                stdout=console_log, stderr=subprocess.STDOUT)
            processes.append(console)
            driving = {'button': None, 'axis': 0., 'left': 2., 'right': 1., 'valid': True,
                       'adc': True, 'gain': .9, 'mixed_gain': False,
                       'gain_known': True, 'saturated': False}
            remote_joy_skew = .067 if case == 0 else 60.
            finish = threading.Event()

            def drive():
                while not finish.is_set() and not rospy.is_shutdown():
                    with lock:
                        values = dict(driving)
                    joy = Joy(axes=[0., 0., values['axis'], 1., 1.],
                              buttons=[int(i == values['button']) for i in range(14)])
                    # PC time may differ; VIM4 must use its local Joy receipt time.
                    joy.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(remote_joy_skew)
                    joy_pub.publish(joy)
                    if values['adc']:
                        stamp = rospy.Time.now()
                        samples = [AdcSample(channel_id=i, timestamp=stamp,
                                             adc_code=32767 if values['saturated'] else 100,
                                             volt_raw=v * .5, volt_cali=v, status_cali=values['valid'],
                                             cali_id='synthetic' if values['valid'] else '',
                                             status_dac_feedback=values['gain_known'],
                                             dac_volt=(values['gain'] + (.05 if values['mixed_gain'] and i == 0 else 0.)
                                                       if values['gain_known'] else float('nan')))
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
                path = output / (name + '_console.log')
                marker = 'SONIC STATUS / SHARE'
                with lock:
                    before = len(snapshots)
                set_input(button=8)
                deadline = time.monotonic() + 2.
                while time.monotonic() < deadline:
                    content = path.read_text()
                    with lock:
                        delivered = len(snapshots) == before + 1
                    if delivered and content.count(marker) == before + 1:
                        break
                    if node.poll() is not None or console.poll() is not None:
                        raise RuntimeError('node exited during Share report; see ' + str(output))
                    time.sleep(.01)
                else:
                    raise AssertionError('Share did not print status; see ' + str(output))
                time.sleep(.12)
                content = path.read_text()
                assert content.count(marker) == before + 1, 'held Share printed repeatedly'
                with lock:
                    assert len(snapshots) == before + 1
                    report = snapshots[-1]
                assert '\033[' not in report
                assert report in re.sub(r'\033\[[0-9;]*m', '', content)
                assert ('\033[' in content) == (case == 1)
                set_input(button=None)
                return report

            thread = threading.Thread(target=drive, daemon=True)
            recorder = None
            try:
                deadline = time.monotonic() + 8
                while not (joy_pub.get_num_connections() and adc_pub.get_num_connections()):
                    if node.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('test topic connection failed; see ' + str(output))
                    time.sleep(.02)
                if case == 0:
                    recorder_log = (output / 'recorder.log').open('w'); logs.append(recorder_log)
                    # Use an explicit launch scope for isolated recorder subscriptions.
                    launch = ElementTree.Element('launch')
                    for source, dest in RECORD_TOPICS.items():
                        ElementTree.SubElement(launch, 'remap', {'from': source, 'to': prefix + '/' + dest})
                    include = ElementTree.SubElement(launch, 'include', {'file': str(args.record_launch.resolve())})
                    ElementTree.SubElement(include, 'arg', {'name': 'bag_prefix', 'value': str(output / 'sonic')})
                    wrapper = output / 'record_test.launch'
                    ElementTree.ElementTree(launch).write(str(wrapper), encoding='unicode')
                    recorder = subprocess.Popen(['roslaunch', str(wrapper)],
                        stdout=recorder_log, stderr=subprocess.STDOUT)
                    processes.append(recorder)
                    deadline = time.monotonic() + 10
                    while adc_pub.get_num_connections() < 2 or joy_pub.get_num_connections() < 2:
                        if recorder.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('recorder failed to subscribe; see ' + str(output))
                        time.sleep(.02)
                    time.sleep(.1)  # Capture idle telemetry before the first synthetic Joy.
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
                assert abs(state.normalized_difference - 2 / 3) < 1e-9
                assert state.lr_voltage_offset_v == 0.
                assert abs(state.compensated_difference - 2 / 3) < 1e-9
                assert state.command_bias == polarity * 128.
                with lock:
                    updates = [s for _, s in states if s.decision_updated]
                assert len(updates) == 1, updates
                report = share_report()
                assert re.search(r'Raw instant\s*\|\s*0.50000\s*\|\s*1.00000\s*\|\s*0.50000', report)
                assert re.search(r'Raw cycle mean\s*\|\s*0.50000\s*\|\s*1.00000\s*\|\s*0.50000', report)
                assert re.search(r'Calibrated cycle mean\s*\|\s*1.00000\s*\|\s*2.00000\s*\|\s*1.00000', report)
                assert re.search(r'Diff after/T \[%\]\s*:\s*\(2.00000-1.00000\+\(\+0.00000\)\)/AVG = \+66.67% / 10.00%', report)
                assert re.search(r'Direction\s*:\s*LEFT', report)
                assert set(map(len, report.splitlines())) == {78}
                assert len(report.splitlines()) == 17
                # Re-press Circle: keep tracking and the existing phase/window.
                set_input(button=2)
                time.sleep(.1)
                with lock:
                    assert states[-1][1].motion_mode == 'tracking'
                    assert states[-1][1].status == 'active'
                set_input(button=None)
                rospy.wait_for_service('/' + name + '/set_acoustic_steering', timeout=3)
                configure = rospy.ServiceProxy('/' + name + '/set_acoustic_steering', SetAcousticSteering)
                assert not configure(-1., 64., 0.).success
                response = configure(1.2, 64., 0.)
                assert response.difference_reference == 'channel_mean'
                assert response.success and response.parameters_pending
                assert response.motion_mode == 'tracking' and response.direction == 'left'
                assert response.adc_quality == 'valid'
                assert (response.active_threshold, response.active_bias, response.active_lr_voltage_offset_v) == (.1, 128., 0.)
                assert (response.pending_threshold, response.pending_bias, response.pending_lr_voltage_offset_v) == (1.2, 64., 0.)
                assert response.request_id == response.pending_request_id
                wait_state(lambda s: s.parameters_pending)
                state = wait_state(lambda s: s.cycle_index >= 2 and not s.parameters_pending)
                assert state.command_bias == 0. and state.turn_bias == 64.
                assert configure(.1, 64., 0.).success
                state = wait_state(lambda s: s.cycle_index >= 3 and s.command_bias == polarity * 64.)
                previous_end = state.window_end
                assert not configure(.15, 96., float('nan')).success
                request_time = rospy.Time.now()
                command = subprocess.run(
                    ['rosservice', 'call', '/' + name + '/set_acoustic_steering',
                     '{threshold: 0.15, bias: 96.0, lr_voltage_offset_v: -1.50}'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=5)
                assert command.returncode == 0 and 'success: True' in command.stdout, command.stdout
                (output / (name + '_steering_response.yaml')).write_text(command.stdout)
                import yaml
                feedback = yaml.safe_load(command.stdout)
                assert feedback['motion_mode'] == 'tracking' and feedback['adc_quality'] == 'valid'
                assert feedback['parameters_pending'] and feedback['pending_bias'] == 96.
                assert (feedback['active_threshold'], feedback['active_bias'], feedback['active_lr_voltage_offset_v']) == (.1, 64., 0.)
                assert feedback['pending_threshold'] == .15 and feedback['pending_lr_voltage_offset_v'] == -1.5
                assert feedback['difference_reference'] == 'channel_mean'
                state = wait_state(lambda s: s.lr_voltage_offset_v == -1.5 and s.command_bias == -polarity * 96.)
                assert (state.threshold, state.turn_bias, state.lr_voltage_offset_v) == (.15, 96., -1.5)
                with lock:
                    settings = {(s.threshold, s.turn_bias, s.lr_voltage_offset_v)
                                for _, s in states if s.header.stamp >= request_time}
                assert settings <= {(.1, 64., 0.), (.15, 96., -1.5)}, settings
                assert abs(state.normalized_difference - 2 / 3) < 1e-9
                assert abs(state.compensated_difference + 1 / 3) < 1e-9
                phase_cycles = (state.window_end - previous_end).to_sec() * frequency
                assert abs(phase_cycles - round(phase_cycles)) < 1e-6
                report = share_report()
                assert re.search(r'LR offset \[V\]\s*:\s*-1.50000', report)
                assert re.search(r'Raw cycle mean\s*\|\s*0.50000\s*\|\s*1.00000\s*\|\s*0.50000', report)
                assert re.search(r'Calibrated cycle mean\s*\|\s*1.00000\s*\|\s*2.00000\s*\|\s*1.00000', report)
                assert re.search(r'Diff before \[%\]\s*:\s*\(2.00000-1.00000\)/AVG = \+66.67%', report)
                assert re.search(r'Diff after/T \[%\]\s*:\s*\(2.00000-1.00000\+\(-1.50000\)\)/AVG = -33.33% / 15.00%', report)
                assert re.search(r'Direction\s*:\s*RIGHT', report)
                previous_cycle = state.cycle_index
                assert configure(.1, 64., -1.5).success
                state = wait_state(lambda s: s.cycle_index > previous_cycle and not s.parameters_pending
                                   and (s.threshold, s.turn_bias, s.lr_voltage_offset_v) == (.1, 64., -1.5))
                assert state.lr_voltage_offset_v == -1.5 and state.command_bias == -polarity * 64.
                assert configure(.1, 64., 0.).success
                wait_state(lambda s: s.lr_voltage_offset_v == 0. and s.command_bias == polarity * 64.)
                set_input(left=1., right=3.)
                state = wait_state(lambda s: s.command_bias == -polarity * 64. and s.status == 'active')
                with lock:
                    updates = [s for _, s in states if s.decision_updated]
                for before, after in zip(updates, updates[1:]):
                    assert after.cycle_index == before.cycle_index + 1
                    assert abs((after.window_end - before.window_end).to_sec() - 1. / frequency) < 1e-6
                before_hold = state
                held_bias = state.command_bias
                response = configure(.2, 96., -10.)
                assert response.success and response.parameters_pending
                state = wait_state(lambda s: s.status == 'voltage_compensation_out_of_range')
                assert state.motion_mode == 'tracking' and state.command_bias == held_bias
                assert state.parameters_pending and state.turn_bias == 64. and state.lr_voltage_offset_v == 0.
                report = share_report()
                assert 'HISTORICAL' in report and re.search(r'Direction\s*:\s*HOLDING', report)
                assert re.search(r'Pending LR \[V\]\s*:\s*-10.00000', report)
                assert configure(.1, 64., 0.).success
                state = wait_state(lambda s: s.status == 'active' and not s.parameters_pending)
                assert state.command_bias == held_bias
                skipped = (state.window_end - before_hold.window_end).to_sec() * frequency
                assert abs(skipped - round(skipped)) < 1e-6
                # Changing gain preserves motion/phase and holds the last bias.
                # Mixed groups are excluded; stable data must fill an aligned cycle.
                with lock:
                    before_gain = states[-1][1]
                    held_bias = before_gain.command_bias
                    target_start = len(targets)
                set_input(gain=1., mixed_gain=True, left=3., right=1.)
                wait_state(lambda s: s.motion_mode == 'tracking' and s.status == 'gain_warmup')
                report = share_report()
                assert re.search(r'Direction\s*:\s*HOLDING', report)
                assert 'HISTORICAL' in report
                with lock:
                    assert states[-1][1].command_bias == held_bias
                    assert len(set(targets[target_start:])) > 2, 'gait stopped during gain adjustment'
                stable_at = rospy.Time.now().to_sec()
                set_input(mixed_gain=False)
                state = wait_state(lambda s: s.status == 'active' and s.command_bias == polarity * 64.)
                assert state.window_start.to_sec() >= stable_at - .01
                assert abs(state.left_mean - 3.) < 1e-9 and abs(state.right_mean - 1.) < 1e-9
                elapsed_cycles = (state.window_end - before_gain.window_end).to_sec() * frequency
                assert abs(elapsed_cycles - round(elapsed_cycles)) < 1e-6, 'gain adjustment reset gait phase'
                with lock:
                    gain_states = [s for _, s in states if s.header.stamp >= before_gain.header.stamp]
                assert all(s.motion_mode == 'tracking' for s in gain_states)
                assert not any(s.decision_updated for s in gain_states if s.status == 'gain_warmup')
                set_input(left=1., right=1.)
                wait_state(lambda s: s.command_bias == 0. and s.status == 'active')
                set_input(button=1)
                wait_state(lambda s: s.motion_mode == 'stopped' and s.status == 'stop_button')
                wait_target(2048)
                report = share_report()
                assert 'HISTORICAL' in report and re.search(r'Direction\s*:\s*STOPPED', report)
                set_input(button=None)
                time.sleep(.1)
                set_input(button=2)
                wait_state(lambda s: s.motion_mode == 'tracking' and s.status == 'warmup')
                set_input(button=None)
                wait_state(lambda s: s.cycle_index == 1 and s.status == 'active')
                set_input(left=3., right=1.)
                wait_state(lambda s: s.status == 'active' and s.command_bias == polarity * 64.)
                for change, waiting in [({'valid': False}, 'calibration_fallback'),
                                        ({'gain_known': False}, 'gain_unknown')]:
                    with lock:
                        before_quality = states[-1][1]
                        target_start = len(targets)
                    set_input(**change)
                    wait_state(lambda s: s.motion_mode == 'tracking' and s.status == waiting)
                    report = share_report()
                    assert re.search(r'Direction\s*:\s*HOLDING', report)
                    assert re.search(r'ADC quality\s*:\s*' + waiting, report)
                    time.sleep(1. / frequency + .1)
                    with lock:
                        assert states[-1][1].motion_mode == 'tracking'
                        assert states[-1][1].command_bias == before_quality.command_bias
                        assert len(set(targets[target_start:])) > 2
                        assert states[-1][1].cycle_index == before_quality.cycle_index
                    recovered_at = rospy.Time.now().to_sec()
                    set_input(valid=True, gain_known=True)
                    wait_state(lambda s: s.motion_mode == 'tracking' and s.status == 'data_warmup')
                    state = wait_state(lambda s: s.status == 'active' and s.cycle_index > before_quality.cycle_index)
                    assert state.window_start.to_sec() >= recovered_at - .01
                    phase_cycles = (state.window_end - before_quality.window_end).to_sec() * frequency
                    assert abs(phase_cycles - round(phase_cycles)) < 1e-6
                # Hold Circle before the fault: a released frame followed by
                # a new press after stopping would be legitimate recovery.
                set_input(button=2)
                time.sleep(.1)
                set_input(saturated=True)
                wait_state(lambda s: s.motion_mode == 'stopped' and 'saturated' in s.status)
                set_input(saturated=False)
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
                      'gain adjustment and unusable quality hold without stopping, '
                      'remote aligned/color Share snapshots, '
                      'live voltage compensation, settings and fault recovery, '
                      'remote Joy stamp %.3f s behind local clock' % (frequency, polarity, remote_joy_skew), flush=True)
            finally:
                finish.set()
                if thread.is_alive(): thread.join(timeout=2)
                joy_pub.unregister(); adc_pub.unregister(); sub.unregister(); target_sub.unregister(); status_sub.unregister()
                if node.poll() is None:
                    node.send_signal(signal.SIGINT)
                    assert node.wait(timeout=5) == 0
                if console.poll() is None:
                    console.send_signal(signal.SIGINT)
                    assert console.wait(timeout=5) == 0
                if recorder is not None:
                    recorder.send_signal(signal.SIGINT)
                    assert recorder.wait(timeout=10) == 0
                node_output = (output / (name + '.log')).read_text()
                assert 'SONIC STATUS / SHARE' not in node_output
                assert 'Sonic manual: bias' not in node_output
                assert 'threshold=0.1500 bias=96.0 lr_voltage_offset_v=-1.500000 V' in node_output
            if recorder is not None:
                verify_recording(output)
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
