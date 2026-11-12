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
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from robotic_fish_sonic.adc_calibration import AdcCalibration
from robotic_fish_sonic.adc_limits import MAX_RAW, MAX_VOLTAGE_V


def package_config_path():
    source_path = Path(__file__).resolve().parents[1] / 'config' / 'io.yaml'
    if source_path.is_file():
        return source_path
    # Installed executables live in lib/<package>, while config is in share/<package>.
    import rospkg
    return Path(rospkg.RosPack().get_path('robotic_fish_sonic')) / 'config' / 'io.yaml'


def safety_limit(recorded_settings=(), explicit_v=None, config_path=None):
    """Use the recorded effective threshold; never silently override it."""
    def checked(value):
        if isinstance(value, bool):
            raise ValueError('ADC safety limit must be a finite voltage in (0, 4.096]')
        value = float(value)
        if not np.isfinite(value) or not 0 < value <= 4.096:
            raise ValueError('ADC safety limit must be a finite voltage in (0, 4.096]')
        return value

    values = {checked(settings['safety_limit_v']) for settings in recorded_settings
              if 'safety_limit_v' in settings}
    explicit = None if explicit_v is None else checked(explicit_v)
    if len(values) > 1:
        raise ValueError('ADC safety threshold changed during recording; split the experiment')
    if values:
        value = values.pop()
        if explicit is not None and not np.isclose(explicit, value, rtol=0, atol=1e-9):
            raise ValueError('Explicit ADC safety limit conflicts with the recorded threshold')
        return value, 'recorded_gain_control_config'
    if explicit is not None:
        return explicit, 'explicit_argument_no_recorded_threshold'
    path = Path(config_path) if config_path is not None else package_config_path()
    with path.open() as stream:
        settings = yaml.safe_load(stream)
    return checked(settings['gain_control']['adc_safety_limit_v']), 'package_config_no_recorded_threshold'


def read_sweep_bag(path):
    """Audit ALL raw observations before filtering by DAC feedback validity."""
    import rosbag
    rows, configs, gain_settings, acoustic_status = [], {}, [], {}
    audit = dict(total_groups=0, invalid_feedback_or_data_groups=0,
                 nonfinite_raw_groups=0, saturated_raw_groups=0, peak_adc_raw_v=None)
    with rosbag.Bag(str(path)) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                '/robotic_fish/adc/samples', '/robotic_fish/acoustic/quadrant_probabilities',
                '/robotic_fish/gain_control/config']):
            if topic.endswith('/config'):
                configs[message.device] = json.loads(message.config_json)
                if message.device == 'gain_control':
                    gain_settings.append(configs[message.device].get('settings', {}))
                continue
            if topic.endswith('/quadrant_probabilities'):
                key = '{}:has_estimate={}:model={}'.format(message.status, message.has_estimate, message.model_id)
                acoustic_status[key] = acoustic_status.get(key, 0) + 1
                continue
            audit['total_groups'] += 1
            samples = sorted(message.samples, key=lambda sample: sample.channel_id)
            raw = np.array([s.volt_raw for s in samples])
            finite_raw = raw[np.isfinite(raw)]
            if len(finite_raw):
                peak = float(np.max(finite_raw))
                previous = audit['peak_adc_raw_v']
                audit['peak_adc_raw_v'] = peak if previous is None else max(peak, previous)
            audit['nonfinite_raw_groups'] += int(not np.isfinite(raw).all())
            audit['saturated_raw_groups'] += int(any(abs(s.adc_code) >= MAX_RAW for s in samples))
            if ([s.channel_id for s in samples] != [0, 1, 2] or
                    not all(s.status_dac_feedback for s in samples) or
                    len({s.dac_volt for s in samples}) != 1):
                audit['invalid_feedback_or_data_groups'] += 1
                continue
            row = [max(s.timestamp.to_sec() for s in samples), samples[0].dac_volt,
                   *raw, *[s.volt_cali for s in samples], int(all(s.status_cali for s in samples))]
            if not np.isfinite(row).all():
                audit['invalid_feedback_or_data_groups'] += 1
                continue
            rows.append(row)
    if not rows:
        raise ValueError('No usable ADC groups')
    rows = np.array(rows)
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('ADC timestamps must be strictly increasing')
    return rows, configs, gain_settings, acoustic_status, audit


def require_raw_audit(audit, limit):
    """Reject unsafe training sessions, including observations with invalid feedback."""
    if audit['nonfinite_raw_groups']:
        raise ValueError('Nonfinite raw ADC observation before DAC feedback filtering')
    if audit['saturated_raw_groups']:
        raise ValueError('Saturated raw ADC code before DAC feedback filtering')
    if audit['peak_adc_raw_v'] is None or audit['peak_adc_raw_v'] >= limit:
        raise ValueError('Raw ADC reached session ceiling before DAC feedback filtering; use a separate valid sweep')


def plateaus(rows, settle_s=1., guard_s=.25, maximum_v=None, dac_max_v=None):
    """Split contiguous confirmed DAC settings; retain the ascending sweep only."""
    rows = np.asarray(rows)
    if (rows.ndim != 2 or rows.shape[1] < 8 or not len(rows) or
            not np.isfinite(rows).all() or np.any(np.diff(rows[:, 0]) <= 0)):
        raise ValueError('Expected finite ADC rows with strictly increasing timestamps')
    if maximum_v is None:
        maximum_v, _ = safety_limit()
    if not np.isfinite(maximum_v) or not 0 < maximum_v <= 4.096:
        raise ValueError('Invalid ADC safety limit')
    if dac_max_v is not None and (not np.isfinite(dac_max_v) or not 0 <= dac_max_v <= 5):
        raise ValueError('Experimental DAC maximum must be within 0..5 V')
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
        elif dac_max_v is not None and gain > dac_max_v + 1e-6:
            # Feedback uses float32: 1.30 may be represented as 1.29999995.
            reason = 'above experimental DAC ceiling'
        elif np.max(segment[:, 2:5]) >= maximum_v:
            # Reject even transient overrange during settling, not just medians.
            reason = 'at or above ADC safety ceiling'
        elif len(stable) < 16:
            reason = 'fewer than 16 settled samples'
        elif np.any(stable[:, 2:5] <= 0):
            reason = 'nonpositive input'
        if reason:
            excluded.append(dict(dac_v=float(gain), reason=reason))
        else:
            accepted.append(stable)
    return accepted, excluded


def fit(groups, calibration_id, include_measured_endpoints=False):
    if len(groups) < 3 or any(len(g) == 0 or not np.isfinite(g[:, 2:5]).all() for g in groups):
        raise ValueError('Need at least three nonempty finite training plateaus')
    medians = np.array([np.median(g[:, 2:5], axis=0) for g in groups])
    if np.any(np.diff(medians, axis=0) <= 0):
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
        median_min, median_max = nodes[0]['raw_voltage_v'], nodes[-1]['raw_voltage_v']
        measured_min = float(np.min(groups[0][:, 2+channel]))
        measured_max = float(np.max(groups[-1][:, 2+channel]))
        if include_measured_endpoints:
            # Estimate the boundary response from the nearest two median nodes,
            # but only inside the raw voltages observed in the TRAINING windows.
            # The independent validation set never determines these bounds.
            if not 0 < measured_min <= median_min < median_max <= measured_max < MAX_VOLTAGE_V:
                raise ValueError('Measured endpoint support must be positive, ordered, and unsaturated')
            first, second = nodes[:2]
            before_last, last = nodes[-2:]

            def boundary_node(raw, a, b, count):
                slope = ((b['reference_voltage_v']-a['reference_voltage_v']) /
                         (b['raw_voltage_v']-a['raw_voltage_v']))
                ref = a['reference_voltage_v'] + (raw-a['raw_voltage_v'])*slope
                if not np.isfinite(ref) or ref <= 0:
                    raise ValueError('Invalid estimated boundary reference voltage')
                return dict(raw_voltage_v=raw, reference_voltage_v=float(ref),
                            steady_sample_count=count, node_kind='measured_support_boundary',
                            reference_estimated=True)

            if measured_min < median_min:
                nodes.insert(0, boundary_node(measured_min, first, second, len(groups[0])))
            if measured_max > median_max:
                nodes.append(boundary_node(measured_max, before_last, last, len(groups[-1])))
        document['curves'].append(dict(channel='ADC'+str(channel), nodes=nodes,
                                      input_min_v=nodes[0]['raw_voltage_v'],
                                      input_max_v=nodes[-1]['raw_voltage_v']))
        if include_measured_endpoints:
            document['curves'][-1]['measured_support'] = dict(
                input_min_v=measured_min, input_max_v=measured_max,
                median_input_min_v=median_min, median_input_max_v=median_max,
                source='first_and_last_settled_training_plateaus',
                boundary_reference='nearest_two_median_nodes_linear_estimate')
    if include_measured_endpoints:
        document['processing']['boundary_policy'] = 'linear_nodes_within_measured_training_support'
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


def require_holdout(validation):
    accepted = validation['candidate']['count']
    total = accepted + validation['rejected_holdout_groups']
    if not accepted or accepted / total < .99:
        raise ValueError('Temporal holdout calibrated coverage below 99%; candidate not generated')
    if validation['candidate']['p95_spread_v'] > validation['raw']['p95_spread_v'] + 1e-12:
        raise ValueError('Temporal holdout P95 spread worsened; candidate not generated')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='New directory')
    parser.add_argument('--common-input', action='store_true', help='Assert fixed equal input on all channels')
    parser.add_argument('--adc-safety-limit-v', type=float,
                        help='Fallback threshold for old bags; must agree with recorded settings when present')
    parser.add_argument('--dac-max-v', type=float,
                        help='Experiment-only DAC ceiling (e.g. 1.30 for the 5 mVpp setup); does not change runtime limits')
    parser.add_argument('--include-measured-endpoints', action='store_true',
                        help='Add estimated boundary nodes at observed training endpoints; never use validation data for bounds')
    args = parser.parse_args()
    try:
        rows, configs, gain_settings, acoustic_status, audit = read_sweep_bag(args.bag)
        limit, limit_source = safety_limit(gain_settings, args.adc_safety_limit_v)
        groups, excluded = plateaus(rows, maximum_v=limit, dac_max_v=args.dac_max_v)
    except (ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    summary = dict(bag=str(args.bag.resolve()), sha256=hashlib.sha256(args.bag.read_bytes()).hexdigest(),
                   groups=len(rows), invalid_groups=audit['invalid_feedback_or_data_groups'],
                   raw_observation_audit=audit,
                   calibration_fallback_groups=int(np.sum(rows[:, -1] == 0)),
                   acoustic_status_counts=acoustic_status, accepted_plateaus=len(groups),
                   excluded_plateaus=excluded, common_input_confirmed=args.common_input,
                   settling_s=1., end_guard_s=.25, calibration_ceiling_v=limit,
                   adc_safety_limit_v=limit, safety_limit_source=limit_source,
                   experimental_dac_max_v=args.dac_max_v)
    calibration = None
    if args.common_input:
        try:
            require_raw_audit(audit, limit)
            training = [g[:len(g)//2] for g in groups]
            trial = fit(training, 'holdout_only', args.include_measured_endpoints)
            summary['validation'] = validate(groups, trial)
            require_holdout(summary['validation'])
            summary['candidate_checks'] = dict(status='passed', minimum_holdout_coverage=.99,
                                               p95_spread_must_not_worsen=True,
                                               independent_validation_required=True)
            calibration = fit(groups, 'adc_independent_transfer_' + args.bag.stem,
                              args.include_measured_endpoints)
        except (ValueError, TypeError, KeyError) as exc:
            parser.error(str(exc))
        calibration.document['source'] = dict(file=str(args.bag.resolve()), sha256=summary['sha256'])
        calibration.document['validation'] = summary['validation']
        calibration.document['processing'].update(settling_time_ms=1000, end_guard_time_ms=250)
        calibration.document['conditions'].update(adc_safety_limit_v=limit,
                                                  experimental_dac_max_v=args.dac_max_v)
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
