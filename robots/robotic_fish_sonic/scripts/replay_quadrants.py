#!/usr/bin/env python3
"""Decode embedded ROS1 definitions; no hardware, ROS master or source-bag writes."""
import argparse
import json
from collections import Counter
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from robotic_fish_sonic.acoustic_core import CLASSES, Estimator, Model, Status


def recalibrate(message, calibration):
    """Replace decoded values in memory; preserve raw values and acquisition times."""
    if any(not hasattr(s, 'volt_raw') for s in message.samples):
        raise ValueError('--calibration requires the volt_raw ADC message schema')
    samples = sorted(message.samples, key=lambda s: s.channel_id)
    if [s.channel_id for s in samples] != [0, 1, 2]:
        raise ValueError('Expected ADC0, ADC1, ADC2 for recalibration')
    raw = [s.volt_raw for s in samples]
    try:
        corrected, _ = calibration.apply(raw)
        applied = True
    except ValueError:
        corrected, applied = raw, False
    for s, value in zip(samples, corrected):
        s.volt_cali = value
        s.status_cali = applied
        s.cali_id = calibration.calibration_id if applied else ''
        s.diff_cali = value - s.volt_raw if applied else float('nan')


def records(path):
    try:
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ModuleNotFoundError as exc:
        if exc.name != 'rosbags':
            raise
        # ROS installations can decode embedded definitions without the optional
        # rosbags dependency or locally generated custom message modules.
        import rosbag
        found = False
        with rosbag.Bag(str(path)) as bag:
            for _, message, stamp in bag.read_messages(topics=['/robotic_fish/adc/samples']):
                found = True
                yield stamp.to_sec(), message
        if not found:
            raise ValueError('bag has no ADC arrays: ' + str(path))
        return
    with Reader(path) as reader:
        store = get_typestore(Stores.ROS1_NOETIC)
        connections = [c for c in reader.connections if c.topic == '/robotic_fish/adc/samples']
        if not connections:
            raise ValueError('bag has no ADC arrays: ' + str(path))
        for connection in connections:
            definition = connection.msgdef
            store.register(get_types_from_msg(definition if isinstance(definition, str) else definition.data, connection.msgtype))
        for connection, stamp, raw in reader.messages(connections=connections):
            yield stamp / 1e9, store.deserialize_ros1(raw, connection.msgtype)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', required=True, type=Path, help='New output directory; never overwrite')
    parser.add_argument('--model')
    parser.add_argument('--calibration', type=Path, help='Recalibrate raw ADC values in memory before comparing')
    parser.add_argument('--window-s', type=float, default=.5)
    args = parser.parse_args()
    paths = sorted(p for p in args.input.glob('*.bag') if not p.name.startswith('._')) if args.input.is_dir() else [args.input]
    if not paths:
        parser.error('no bags found')
    model = Model(args.model) if args.model else None
    calibration = None
    if args.calibration:
        from robotic_fish_sonic.adc_calibration import AdcCalibration
        calibration = AdcCalibration.load(args.calibration)
    args.output.mkdir(parents=True, exist_ok=False)
    summaries = []
    for path in paths:
        engine = Estimator(model, args.window_s)
        counts, flags = Counter(), Counter()
        elapsed = 0.
        with (args.output / (path.stem + '.jsonl')).open('x') as output:
            for now, message in records(path):
                if calibration:
                    recalibrate(message, calibration)
                start = time.perf_counter()
                result = engine.push(message, now)
                elapsed += time.perf_counter() - start
                counts['groups'] += 1
                counts['updated'] += int(result['updated'])
                counts['feature_rows'] += int('features' in result)
                counts['comparison_updates'] += int(result['comparison']['updated'])
                counts['reliable_comparisons'] += int(result['comparison']['reliable'])
                for flag in Status:
                    if result['status'] & flag:
                        flags[flag.name] += 1
                p = result['probabilities']
                if p is not None:
                    assert all(0 <= v <= 1 for v in p) and abs(sum(p) - 1) < 1e-12
                result.update(record_time=now, probability_left=p[2]+p[3] if p else None,
                              probability_right=p[0]+p[1] if p else None)
                output.write(json.dumps(result, allow_nan=False) + '\n')
        summary = dict(bag=str(path), counts=dict(counts), flags=dict(flags),
                       mean_compute_ms=1000 * elapsed / max(counts['groups'], 1),
                       model_id=model.id if model else 'unavailable', classes=CLASSES,
                       calibration_id=calibration.calibration_id if calibration else 'as_recorded')
        summaries.append(summary)
        print(json.dumps(summary))
    (args.output / 'summary.json').write_text(json.dumps(summaries, indent=2) + '\n')


if __name__ == '__main__':
    main()
