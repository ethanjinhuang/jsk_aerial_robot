#!/usr/bin/env python3
"""Audit a DAC sweep; optionally fit relative equal-input ADC transfer curves.

Requires ROS1 rosbag and numpy. Reads embedded bag definitions without a master.
Never edits a bag or deploys a calibration. --common-input is an experimental
condition assertion, not something that can be inferred from ADC measurements.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from robotic_fish_sonic.adc_calibration import AdcCalibration


def plateaus(rows, settle_s=1., guard_s=.25, maximum_v=3.3):
    """Split contiguous confirmed DAC settings; retain the ascending sweep only."""
    starts = np.r_[0, np.where(np.diff(rows[:, 1]) != 0)[0] + 1]
    ends = np.r_[starts[1:], len(rows)]
    accepted, excluded = [], []
    previous = -float('inf')
    recovery = False
    for start, end in zip(starts, ends):
        segment = rows[start:end]
        gain = segment[0, 1]
        recovery = recovery or gain < previous
        previous = gain
        stable = segment[(segment[:, 0] >= segment[0, 0] + settle_s) &
                         (segment[:, 0] <= segment[-1, 0] - guard_s)]
        reason = None
        if recovery:
            reason = 'after DAC decrease / possible safety recovery'
        elif len(stable) < 16:
            reason = 'fewer than 16 settled samples'
        elif np.max(stable[:, 2:5]) > maximum_v:
            reason = 'above selected calibration ceiling'
        elif np.any(stable[:, 2:5] <= 0):
            reason = 'nonpositive input'
        if reason:
            excluded.append(dict(dac_v=float(gain), reason=reason))
        else:
            accepted.append(stable)
    return accepted, excluded


def fit(groups, calibration_id):
    medians = np.array([np.median(g[:, 2:5], axis=0) for g in groups])
    if len(groups) < 3 or np.any(np.diff(medians, axis=0) <= 0):
        raise ValueError('Need at least three strictly increasing per-channel plateaus')
    reference = np.median(medians, axis=1)
    document = dict(schema_version=1, calibration_id=calibration_id,
                    calibration_type='per_channel_piecewise_linear',
                    channel_order=['ADC0', 'ADC1', 'ADC2'],
                    conditions=dict(common_input_required=True,
                                    absolute_voltage_reference=False,
                                    reference_type='three_channel_adc_consensus'),
                    processing=dict(interpolation='piecewise_linear_per_channel',
                                    runtime_dac_dependency=False,
                                    generation_dac_dependency=True,
                                    offset_correction=False,
                                    reference='median_of_three_channel_medians',
                                    segmentation='confirmed_DAC_contiguous_plateaus',
                                    validation_split='first_half_fit_second_half_holdout'),
                    validity=dict(endpoint_tolerance_v=.0025,
                                  out_of_range='reject_group_and_fallback_raw'), curves=[])
    for channel in range(3):
        nodes = [dict(raw_voltage_v=float(row[channel]), reference_voltage_v=float(ref),
                      steady_sample_count=len(group))
                 for row, ref, group in zip(medians, reference, groups)]
        document['curves'].append(dict(channel='ADC'+str(channel), nodes=nodes,
                                      input_min_v=nodes[0]['raw_voltage_v'],
                                      input_max_v=nodes[-1]['raw_voltage_v']))
    return AdcCalibration(document)


def metrics(values):
    values = np.asarray(values)
    if not len(values):
        return dict(count=0)
    spread = np.ptp(values, axis=1)
    relative = 100 * spread / np.median(values, axis=1)
    return dict(count=len(values), median_spread_v=float(np.median(spread)),
                p95_spread_v=float(np.percentile(spread, 95)),
                median_relative_spread_pct=float(np.median(relative)),
                p95_relative_spread_pct=float(np.percentile(relative, 95)))


def validate(groups, calibration):
    old, new, raw = [], [], []
    rejected = 0
    for group in groups:
        for row in group[len(group)//2:]:
            try:
                corrected, _ = calibration.apply(row[2:5].tolist())
            except ValueError:
                rejected += 1
                continue
            raw.append(row[2:5])
            old.append(row[5:8])
            new.append(corrected)
    return dict(raw=metrics(raw), recorded=metrics(old), candidate=metrics(new),
                rejected_holdout_groups=rejected,
                note='Temporal holdout within the same sweep; not independent experimental validation')


def main():
    import rosbag
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='New directory')
    parser.add_argument('--common-input', action='store_true', help='Assert fixed equal input on all channels')
    args = parser.parse_args()
    rows, acoustic_status, configs = [], {}, {}
    invalid = 0
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                '/robotic_fish/adc/samples', '/robotic_fish/acoustic/quadrant_probabilities',
                '/robotic_fish/gain_control/config']):
            if topic.endswith('/config'):
                configs[message.device] = json.loads(message.config_json)
            elif topic.endswith('/quadrant_probabilities'):
                key = '{}:has_estimate={}:model={}'.format(message.status, message.has_estimate, message.model_id)
                acoustic_status[key] = acoustic_status.get(key, 0) + 1
            else:
                samples = sorted(message.samples, key=lambda sample: sample.channel_id)
                if ([s.channel_id for s in samples] != [0, 1, 2] or
                        not all(s.status_dac_feedback for s in samples) or
                        len(set(s.dac_volt for s in samples)) != 1):
                    invalid += 1
                    continue
                row = [max(s.timestamp.to_sec() for s in samples), samples[0].dac_volt,
                       *[s.volt_raw for s in samples], *[s.volt_cali for s in samples],
                       int(all(s.status_cali for s in samples))]
                if not np.isfinite(row).all():
                    invalid += 1
                    continue
                rows.append(row)
    if not rows:
        parser.error('No usable ADC groups')
    rows = np.array(rows)
    if np.any(np.diff(rows[:, 0]) <= 0):
        parser.error('ADC timestamps must be strictly increasing')
    groups, excluded = plateaus(rows)
    summary = dict(bag=str(args.bag.resolve()), sha256=hashlib.sha256(args.bag.read_bytes()).hexdigest(),
                   groups=len(rows), invalid_groups=invalid,
                   calibration_fallback_groups=int(np.sum(rows[:, -1] == 0)),
                   acoustic_status_counts=acoustic_status, accepted_plateaus=len(groups),
                   excluded_plateaus=excluded, common_input_confirmed=args.common_input,
                   settling_s=1., end_guard_s=.25, calibration_ceiling_v=3.3)
    calibration = None
    if args.common_input:
        training = [g[:len(g)//2] for g in groups]
        trial = fit(training, 'holdout_only')
        summary['validation'] = validate(groups, trial)
        calibration = fit(groups, 'adc_independent_transfer_' + args.bag.stem)
        calibration.document['source'] = dict(file=str(args.bag.resolve()), sha256=summary['sha256'])
        calibration.document['validation'] = summary['validation']
        calibration.document['processing'].update(settling_time_ms=1000, end_guard_time_ms=250)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (args.output / 'recorded_config.json').write_text(json.dumps(configs, indent=2) + '\n')
    with (args.output / 'plateaus.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['dac_v', 'samples', 'raw0', 'raw1', 'raw2', 'recorded0', 'recorded1', 'recorded2', 'recorded_spread_pct'])
        for g in groups:
            medians = np.median(g[:, 2:8], axis=0)
            writer.writerow([g[0, 1], len(g), *medians,
                             100*np.ptp(medians[3:])/np.median(medians[3:])])
    if calibration:
        (args.output / 'candidate_calibration.json').write_text(json.dumps(calibration.document, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
