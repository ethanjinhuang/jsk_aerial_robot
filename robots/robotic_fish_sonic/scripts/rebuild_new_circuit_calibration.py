#!/usr/bin/env python3
"""Merge an overlap-checked high-end supplement, preserving per-run thresholds."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from calibrate_from_bag import (fit, metrics, plateaus, read_sweep_bag,
                               require_holdout, require_raw_audit, safety_limit, validate)


def read_run(path):
    rows, _, settings, _, audit = read_sweep_bag(path)
    if not settings or any('safety_limit_v' not in config for config in settings):
        raise ValueError('Recorded gain-control threshold is required for reproducible merging')
    limit, source = safety_limit(settings)
    require_raw_audit(audit, limit)
    groups, excluded = plateaus(rows, maximum_v=limit, dac_max_v=1.30)
    metadata = dict(file=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    adc_safety_limit_v=limit, safety_limit_source=source,
                    invalid_groups=audit['invalid_feedback_or_data_groups'],
                    accepted_plateaus=len(groups), excluded_plateaus=excluded,
                    peak_adc_raw_v=audit['peak_adc_raw_v'])
    return groups, metadata


def require_levels(groups, expected, label):
    """This experiment has a fixed acquisition plan, including overlap levels."""
    actual = [float(g[0, 1]) for g in groups]
    if len(actual) != len(expected) or not np.allclose(actual, expected, rtol=0, atol=1e-6):
        raise ValueError('Unexpected stable DAC plateaus in ' + label)


def assess(groups, calibration, enforce=True):
    raw, corrected, per_plateau = [], [], []
    rejected = 0
    failures = []
    for group in groups:
        before, after = [], []
        rejected_here = 0
        for row in group:
            try:
                result, _ = calibration.apply(row[2:5].tolist())
            except ValueError:
                rejected += 1
                rejected_here += 1
                continue
            before.append(row[2:5])
            after.append(result)
        if not after:
            raise ValueError('Independent plateau has no calibrated observations')
        if rejected_here / len(group) > .05:
            failures.append('Independent plateau coverage below 95% at DAC ' + str(group[0, 1]))
        before_metrics, after_metrics = metrics(before), metrics(after)
        if after_metrics['p95_spread_v'] >= before_metrics['p95_spread_v']:
            failures.append('Independent P95 did not improve at DAC ' + str(group[0, 1]))
        raw.extend(before)
        corrected.extend(after)
        per_plateau.append(dict(dac_v=float(group[0, 1]), raw=before_metrics,
                                calibrated=after_metrics, rejected_stable_groups=rejected_here,
                                total_stable_groups=len(group),
                                sample_start_s=float(group[0, 0]), sample_end_s=float(group[-1, 0]),
                                calibrated_coverage=len(after)/len(group)))
    if not corrected or rejected / (len(corrected)+rejected) > .01:
        failures.append('Independent calibrated coverage below 99%')
    if failures and enforce:
        raise ValueError('; '.join(failures))
    return dict(raw=metrics(raw), calibrated=metrics(corrected),
                accepted_plateaus=len(groups), rejected_stable_groups=rejected,
                per_plateau=per_plateau, recommended_for_deployment=not failures,
                status='rejected' if failures else 'passed', failures=failures,
                acceptance_criteria=dict(minimum_per_plateau_coverage=.95, minimum_overall_coverage=.99,
                                         p95_improves_on_each_plateau=True),
                metric_scope='raw and calibrated metrics use only groups accepted by the calibration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base', type=Path)
    parser.add_argument('supplement', type=Path)
    parser.add_argument('--endpoint', type=Path)
    parser.add_argument('--save-rejected', action='store_true')
    parser.add_argument('--include-measured-endpoints', action='store_true')
    parser.add_argument('--output', type=Path, help='New output directory; never overwrites a prior report')
    args = parser.parse_args()
    base, source_base = read_run(args.base / 'ascending_sweep.bag')
    supplement, source_extra = read_run(args.supplement / 'supplement_120_130.bag')
    require_levels(base, np.arange(125)/100, 'base training')
    require_levels(supplement, np.arange(120, 131)/100, 'supplement training')
    endpoint = None
    if args.endpoint:
        endpoint, endpoint_source = read_run(args.endpoint / 'endpoint_training.bag')
        require_levels(endpoint, [1.30], 'extended endpoint training')
        supplement = [g for g in supplement if round(g[0, 1], 2) != 1.30] + endpoint
        source_extra['endpoint_refinement'] = endpoint_source
    by_level = {round(g[0, 1], 2): g for g in base}
    overlap = []
    additions = []
    for group in supplement:
        level = round(group[0, 1], 2)
        if level in by_level:
            old = np.median(by_level[level][:, 2:5], axis=0)
            new = np.median(group[:, 2:5], axis=0)
            difference = abs(new-old)
            relative = difference/old
            if np.max(relative) > .02:
                raise ValueError('Overlap drift exceeds 2% at ' + str(level))
            overlap.append(dict(dac_v=level, base_median_v=old.tolist(),
                                supplement_median_v=new.tolist(),
                                absolute_drift_v=difference.tolist(),
                                relative_drift_pct=(relative*100).tolist()))
        else:
            additions.append(group)
    if len(overlap) != 5 or len(additions) != 6:
        raise ValueError('Expected five overlap and six new plateaus')
    groups = base + additions
    identifier = ('adc_independent_transfer_20261112_060020_new_circuit_psu_130' +
                  ('_observed_support' if args.include_measured_endpoints else ''))
    trial = fit([g[:len(g)//2] for g in groups], 'holdout_only', args.include_measured_endpoints)
    internal = validate(groups, trial)
    require_holdout(internal)
    if internal['candidate']['count'] < 100:
        raise ValueError('Merged holdout has insufficient calibrated coverage')
    calibration = fit(groups, identifier, args.include_measured_endpoints)
    original_repeat, original_repeat_source = read_run(args.base / 'independent_repeat.bag')
    high_repeat, high_repeat_source = read_run(args.supplement / 'independent_high_repeat.bag')
    require_levels(original_repeat, [i/100 for i in range(5, 116, 10)]+[1.23], 'base independent repeat')
    require_levels(high_repeat, [1.25, 1.27, 1.29, 1.30], 'high independent repeat')
    repeats = original_repeat+high_repeat
    repeat_sources = [original_repeat_source, high_repeat_source]
    if args.endpoint:
        last_repeat, last_repeat_source = read_run(args.endpoint / 'endpoint_independent.bag')
        require_levels(last_repeat, [1.30], 'endpoint independent repeat')
        repeats += last_repeat
        repeat_sources.append(last_repeat_source)
    independent = assess(repeats, calibration, enforce=not args.save_rejected)
    independent.update(validation_type='independent_repeats_same_5mvpp_same_regulated_supply',
                       sources=repeat_sources, calibration_id=identifier,
                       limitations=['Input frequency and ADC VDD not reported by operator',
                                    'No oscilloscope clipping verification',
                                    'No weaker-input/higher-gain, temperature, or supply validation'])
    doc = calibration.document
    doc['conditions'].update(input_amplitude_vpp=.005, power_source='regulated_bench_supply',
                             frequency_hz=None, frequency_status='not_reported_by_operator',
                             hardware_vdd_v=None, experimental_dac_min_v=0.0,
                             experimental_dac_max_v=1.30,
                             operational_adc_safety_limit_v=4.0,
                             supplement_adc_saturation_limit_v=source_extra['adc_safety_limit_v'])
    doc['source'] = dict(file=source_base['file'], sha256=source_base['sha256'],
                         supplementary=source_extra,
                         merge='retain base 0..1.24; append supplement 1.25..1.29 and extended endpoint 1.30')
    doc['processing'].update(settling_time_ms=1000, end_guard_time_ms=250)
    doc['processing']['nominal_plateau_count'] = len(groups)
    doc['validation'] = dict(**internal, independent_repeat=independent, overlap_check=overlap)
    out = args.output or args.supplement / ('measured_support_candidate' if args.include_measured_endpoints else
                            ('rejected_extended_candidate' if independent['status']=='rejected' else 'merged_candidate'))
    out.mkdir(exist_ok=False)
    (out/'candidate_calibration.json').write_text(json.dumps(doc, indent=2)+'\n')
    summary = dict(calibration_id=identifier, nodes_per_channel=[len(c) for c in calibration.curves],
                   source_base=source_base, source_supplement=source_extra,
                   overlap=overlap, validation=independent,
                   ranges=calibration.input_ranges)
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({key:summary[key] for key in ['calibration_id','nodes_per_channel','ranges']}))
    print(json.dumps({key:independent[key] for key in ['raw','calibrated','accepted_plateaus','rejected_stable_groups']}))


if __name__ == '__main__':
    main()
