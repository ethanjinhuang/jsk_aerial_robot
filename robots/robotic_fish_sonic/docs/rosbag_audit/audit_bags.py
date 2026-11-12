#!/usr/bin/env python3
"""Read-only ROS1 bag audit, using embedded message definitions; no ROS master.

Run: python3 audit_bags.py INPUT_DIRECTORY --output NEW_JSON_FILE
Payload byte counts exclude bag framing, indexes and compression. Gap thresholds
are diagnostic observations, not assertions of packet loss. Input bags are never
rewritten. The output is created exclusively, so prior evidence is preserved.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path

import rosbag


ADC = '/robotic_fish/adc/'
SONIC = '/robotic_fish/sonic/'
STEERING = '/robotic_fish/acoustic/'


def digest(data):
    return hashlib.sha256(data).hexdigest()


def serialized(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return stream.getvalue()


def distribution(values):
    rows = sorted(v for v in values if math.isfinite(v))
    if not rows:
        return {'n': 0}

    def percentile(p):
        index = (len(rows) - 1) * p
        low = int(index)
        return rows[low] + (rows[min(low + 1, len(rows) - 1)] - rows[low]) * (index - low)

    return dict(n=len(rows), min=rows[0], median=percentile(.5),
                p95=percentile(.95), max=rows[-1])


def stamp(message):
    if hasattr(message, 'header'):
        return message.header.stamp.to_sec()
    for field in ('timestamp', 'stamp'):
        value = getattr(message, field, None)
        if hasattr(value, 'to_sec'):
            return value.to_sec()
    return None


def text(value):
    return value.decode() if isinstance(value, bytes) else value


def audit(path):
    before = path.stat()
    topics = defaultdict(lambda: dict(count=0, payload_bytes=0, intervals=[],
                                    source_intervals=[], latency=[], large_gaps=[],
                                    source_nonincreasing=0))
    scalar_hashes, array_hashes = Counter(), Counter()
    rosout_hashes, aggregate_hashes = Counter(), Counter()
    rosout_keys, aggregate_keys, requested_topics = Counter(), Counter(), set()
    servo_fields = defaultdict(Counter)
    configs, events, clock_steps, mode_segments = [], [], [], []
    modes, reasons, quality, steering_status, directions = (Counter() for _ in range(5))
    diagnostics, log_messages = Counter(), Counter()
    channel_values = defaultdict(list)
    adc_checks, calibration_ids, gain_values = Counter(), Counter(), Counter()
    steering_checks = Counter()
    group_latencies, group_spans, intervals, previous_motion = [], [], [], None
    previous_channels = {}
    configs_by_device = Counter()

    with rosbag.Bag(str(path), 'r') as bag:
        connections = [dict(topic=c.topic, type=c.datatype, md5=c.md5sum,
                            callerid=text(c.header.get('callerid')),
                            latching=text(c.header.get('latching')),
                            definition_sha256=digest(c.msg_def.encode()))
                       for c in bag._get_connections()]
        for topic, raw, timestamp in bag.read_messages(raw=True):
            typename, data, md5, _, cls = raw
            t = timestamp.to_sec()
            row = topics[topic]
            if row['count']:
                delta = t - row['last']
                row['intervals'].append(delta)
                if delta > 1.:
                    row['large_gaps'].append(dict(before=row['last'], after=t, seconds=delta))
            else:
                row['first'] = t
            row['last'] = t
            row['count'] += 1
            row['payload_bytes'] += len(data)
            row['type'], row['md5'] = typename, md5
            if topic == ADC + 'sample':
                scalar_hashes[digest(data)] += 1
                continue
            if topic == '/rosout':
                rosout_hashes[digest(data)] += 1
            if topic == '/rosout_agg':
                aggregate_hashes[digest(data)] += 1
            message = cls().deserialize(data)
            if topic in ('/rosout', '/rosout_agg'):
                # /rosout reassigns header.seq when forwarding to /rosout_agg.
                key = (message.header.stamp.to_nsec(), message.level, message.name,
                       message.msg, message.file, message.function, message.line,
                       tuple(message.topics))
                (rosout_keys if topic == '/rosout' else aggregate_keys)[key] += 1
                if message.name.startswith('/record_') and message.msg.startswith('Subscribing to '):
                    requested_topics.add(message.msg[len('Subscribing to '):])
            source = stamp(message)
            if source is not None:
                row['latency'].append(t - source)
                if 'last_source' in row:
                    row['source_intervals'].append(source - row['last_source'])
                    row['source_nonincreasing'] += int(source <= row['last_source'])
                row['last_source'] = source

            if topic == ADC + 'samples':
                samples = message.samples
                adc_checks['groups'] += 1
                adc_checks['invalid_channels'] += int(sorted(s.channel_id for s in samples) != [0, 1, 2])
                times = [s.timestamp.to_sec() for s in samples]
                if times:
                    group_latencies.append(t - max(times))
                    group_spans.append(max(times) - min(times))
                adc_checks['fallback_groups'] += int(any(not s.status_cali for s in samples))
                adc_checks['unknown_gain_groups'] += int(any(not s.status_dac_feedback for s in samples))
                adc_checks['mixed_gain_groups'] += int(len({(s.status_dac_feedback, s.dac_volt) for s in samples}) > 1)
                adc_checks['saturated_groups'] += int(any(abs(s.adc_code) >= 32767 for s in samples))
                for sample in samples:
                    array_hashes[digest(serialized(sample))] += 1
                    channel_values[sample.channel_id].append(sample.volt_raw)
                    calibration_ids[sample.cali_id] += 1
                    gain_values[str(sample.dac_volt)] += 1
                    if sample.channel_id in previous_channels:
                        old = previous_channels[sample.channel_id]
                        adc_checks['channel_stamp_nonincreasing'] += int(sample.timestamp.to_nsec() <= old[0])
                        adc_checks['channel_serial_discontinuities'] += int((sample.serial_num-old[1]) % (2**32) != 3)
                    previous_channels[sample.channel_id] = (sample.timestamp.to_nsec(), sample.serial_num)
            elif topic.endswith('/config') and hasattr(message, 'config_json'):
                config = json.loads(message.config_json)
                configs_by_device[message.device] += 1
                calibration = config.pop('calibration', None)
                if calibration:
                    config['calibration_summary'] = dict(
                        id=calibration.get('calibration_id'),
                        sha256=digest(json.dumps(calibration, sort_keys=True).encode()),
                        ranges=[(c['input_min_v'], c['input_max_v']) for c in calibration['curves']])
                configs.append(dict(topic=topic, record_s=t, source_s=source,
                                    device=message.device, config=config))
            elif topic == SONIC + 'motion_state':
                modes[message.motion_mode] += 1
                reasons[message.reason] += 1
                quality[message.adc_quality] += 1
                intervals.append(message.control_interval_s)
                if not mode_segments or mode_segments[-1]['mode'] != message.motion_mode or mode_segments[-1]['reason'] != message.reason:
                    mode_segments.append(dict(mode=message.motion_mode, reason=message.reason,
                                              ros_start=source, monotonic_start=message.monotonic_s,
                                              monotonic_end=message.monotonic_s, count=1))
                else:
                    mode_segments[-1]['monotonic_end'] = message.monotonic_s
                    mode_segments[-1]['count'] += 1
                current = (source, message.monotonic_s, t)
                if previous_motion:
                    ros_delta = source - previous_motion[0]
                    mono_delta = message.monotonic_s - previous_motion[1]
                    if abs(ros_delta - mono_delta) > .05:
                        clock_steps.append(dict(ros_before=previous_motion[0], ros_after=source,
                                                monotonic_before=previous_motion[1], monotonic_after=message.monotonic_s,
                                                ros_delta_s=ros_delta, monotonic_delta_s=mono_delta,
                                                offset_change_s=ros_delta-mono_delta,
                                                bag_delta_s=t-previous_motion[2]))
                else:
                    motion_first = message.monotonic_s
                previous_motion = current
            elif topic == STEERING + 'steering_state':
                steering_status[message.status] += 1
                directions[message.direction] += 1
                steering_checks['decision_updates'] += int(message.decision_updated)
                steering_checks['current_decision_messages'] += int(message.decision_current)
                steering_checks['held_decision_messages'] += int(message.has_decision and not message.decision_current)
                if message.decision_updated:
                    steering_checks['updated_' + message.decision_direction] += 1
            elif topic == STEERING + 'steering_events':
                events.append(dict(record_s=t, request_id=message.request_id, event=message.event,
                                   detail=message.detail, threshold=message.threshold,
                                   bias=message.turn_bias, offset_v=message.lr_voltage_offset_v,
                                   cycle=message.cycle_index))
            elif topic == '/diagnostics':
                for status in message.status:
                    diagnostics[(status.name, status.level, status.message)] += 1
            elif topic == '/servo/states':
                for servo in message.servos:
                    for field in ('angle', 'temp', 'load', 'error'):
                        servo_fields[field][getattr(servo, field)] += 1
            elif topic == '/rosout' and not message.name.startswith('/record_'):
                log_messages[(message.name, message.level, message.msg)] += 1

        for row in topics.values():
            row['record_interval_s'] = distribution(row.pop('intervals'))
            row['source_interval_s'] = distribution(row.pop('source_intervals'))
            row['record_minus_source_s'] = distribution(row.pop('latency'))
            row['count_over_span_hz'] = ((row['count']-1)/(row['last']-row['first'])
                                         if row['last'] > row['first'] else None)
            row.pop('last_source', None)
        result = dict(file=str(path.resolve()), bytes=before.st_size,
                      start_utc=datetime.fromtimestamp(bag.get_start_time(), timezone.utc).isoformat(),
                      end_utc=datetime.fromtimestamp(bag.get_end_time(), timezone.utc).isoformat(),
                      bag_span_s=bag.get_end_time()-bag.get_start_time(),
                      messages=bag.get_message_count(), topics=dict(sorted(topics.items())),
                      connections=connections, configs=configs, configs_by_device=configs_by_device,
                      adc=dict(checks=adc_checks, raw_v={str(c): distribution(v) for c, v in channel_values.items()},
                               group_span_s=distribution(group_spans), record_minus_latest_sample_s=distribution(group_latencies),
                               calibration_ids=calibration_ids, dac_values=gain_values,
                               scalar_messages=sum(scalar_hashes.values()), array_samples=sum(array_hashes.values()),
                               byte_identical_overlap=sum((scalar_hashes & array_hashes).values()),
                               scalar_only=sum((scalar_hashes-array_hashes).values()),
                               array_only=sum((array_hashes-scalar_hashes).values())),
                      logs=dict(rosout_agg_byte_identical_overlap=sum((rosout_hashes & aggregate_hashes).values()),
                                rosout_agg_same_event_ignoring_seq=sum((rosout_keys & aggregate_keys).values()),
                                messages=[dict(node=k[0], level=k[1], message=k[2], count=v) for k, v in log_messages.items()]),
                      requested_topics_observed_in_recorder_logs=sorted(requested_topics),
                      servo_fields={field: dict(count=sum(values.values()), min=min(values), max=max(values),
                                                most_common=values.most_common(5))
                                    for field, values in servo_fields.items()},
                      motion=dict(modes=modes, reasons=reasons, adc_quality=quality,
                                  control_interval_s=distribution(intervals), segments=mode_segments,
                                  monotonic_span_s=previous_motion[1]-motion_first if previous_motion else None,
                                  clock_steps=clock_steps),
                      steering=dict(statuses=steering_status, directions=directions, checks=steering_checks),
                      events=events,
                      diagnostics=[dict(name=k[0], level=k[1], message=k[2], count=v) for k, v in diagnostics.items()])
    with path.open('rb') as stream:
        file_hash = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            file_hash.update(chunk)
    result['sha256'] = file_hash.hexdigest()
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise RuntimeError('Input changed during audit: ' + str(path))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = sorted(args.input.glob('*.bag')) if args.input.is_dir() else [args.input]
    if not paths:
        parser.error('No .bag files found')
    # Fail before scanning if the evidence output already exists.
    if args.output.exists():
        parser.error('Output already exists: ' + str(args.output))
    results = []
    for path in paths:
        result = audit(path)
        results.append(result)
        print(path.name, result['messages'], 'messages;',
              round(result['bag_span_s'], 3), 'bag seconds;',
              round(result['motion']['monotonic_span_s'], 3)
              if result['motion']['monotonic_span_s'] is not None else None,
              'motion monotonic seconds', flush=True)
    payload = json.dumps(dict(schema_version=1, inputs=str(args.input.resolve()),
                             gap_observation_threshold_s=1.0, clock_step_threshold_s=.05,
                             bags=results), indent=2, allow_nan=False)
    with args.output.open('x', encoding='utf-8') as output:
        output.write(payload + '\n')


if __name__ == '__main__':
    main()
