"""Aligned Share snapshots and optional ANSI colors; no ROS dependencies."""
import math
import os
import re
import sys
import textwrap


WIDTH = 78
TITLE = 'SONIC STATUS / SHARE'
COLOR_MODES = ('auto', 'always', 'never')


def numeric(value, precision=5, signed=False):
    if value is None or not math.isfinite(value):
        return '--'
    return format(value, ('+' if signed else '') + '.' + str(precision) + 'f')


def percent(value, signed=False):
    return numeric(None if value is None else value * 100., 2, signed)


def format_status(data):
    """Render a snapshot captured by the motion node, without recalculating it."""
    border = '+' + '-' * (WIDTH - 2) + '+'
    lines = [border, '| ' + TITLE.center(WIDTH - 4) + ' |', border]

    def row(label, value):
        for part in textwrap.wrap('%-16s: %s' % (label, value), WIDTH - 4,
                                  subsequent_indent=' ' * 18):
            lines.append('| ' + part.ljust(WIDTH - 4) + ' |')

    def voltages(label, values):
        cells = [label] + ([numeric(v) for v in values] if values is not None else ['--'] * 3)
        widths = (22, 15, 15, 13)
        lines.append('| ' + ' | '.join(cell.ljust(width) if i == 0 else cell.rjust(width)
                                       for i, (cell, width) in enumerate(zip(cells, widths))) + ' |')

    row('Mode / state', '%s / %s' % (data['mode'], data['status']))
    row('Direction', data['direction'].upper())
    row('Bias / limit', '(%s/%s)' % (numeric(data['bias'], 1, True), numeric(data['bias_limit'], 1)))
    row('Reason', data['reason'])
    row('Snapshot ROS [s]', numeric(data['stamp'], 3))
    lines.append(border)
    lines.append('| %-22s | %15s | %15s | %13s |' %
                 ('Voltage [V]', 'Head (ADC0)', 'Left (ADC1)', 'Right (ADC2)'))
    voltages('Raw instant', data['raw'])
    voltages('Calibrated / fallback', data['calibrated'])
    voltages('Previous cycle mean', data['means'])
    voltages('Compensated mean', data['compensated_means'])
    lines.append(border)
    row('ADC quality', data['quality'])
    row('Source / Rx age', '%s / %s s' % (numeric(data['sample_age'], 3), numeric(data['receipt_age'], 3)))
    if data['means'] is None:
        row('Cycle', 'unavailable (no completed tracking cycle)')
    else:
        row('Cycle', '#%d %s; W=%.3f s; N=%d' %
            (data['cycle'], 'HISTORICAL' if data['historical'] else 'CURRENT', data['window'], data['count']))
    row('L-R [V]', numeric(data['delta'], 5, True))
    row('Comp. L-R [V]', numeric(data['compensated_delta'], 5, True))
    row('Raw diff [%]', percent(data['difference'], True))
    row('LR offset [V]', numeric(data['lr_voltage_offset_v'], 5, True))
    row('Pending LR [V]', numeric(data['pending_lr_voltage_offset_v'], 5, True))
    row('Diff / T [%]', '%s / %s' % (percent(data['compensated_difference'], True),
                                     percent(data['threshold'])))
    row('Last decision', data['judged'].upper())
    lines.append(border)
    return '\n'.join(lines)


def use_color(mode, stream, environ):
    if mode not in COLOR_MODES:
        raise ValueError('color must be auto, always or never')
    if mode != 'auto':
        return mode == 'always'
    return (stream.isatty() and environ.get('TERM') != 'dumb'
            and 'NO_COLOR' not in environ)


_TOKEN = re.compile(r'\b(?:left|right|straight|stopped|tracking|active|valid|warmup|unavailable|'
                    r'historical|current|disabled|not_tracking|invalid_input|no_data|holding|gain_warmup|data_warmup|'
                    r'calibration_fallback|stale_sample|future_sample|gain_unknown|saturated|'
                    r'voltage_compensation_out_of_range)\b'
                    r'|(?<![\w.])[+-]?\d+(?:\.\d+)?', re.IGNORECASE)


def colorize_status(plain):
    def token(match):
        word = match.group().lower()
        if word in ('stopped', 'invalid_input', 'calibration_fallback', 'stale_sample',
                    'future_sample', 'gain_unknown', 'saturated', 'no_data'):
            code = '91'
        elif word in ('warmup', 'gain_warmup', 'data_warmup', 'holding', 'unavailable', 'historical',
                      'disabled', 'not_tracking', 'left', 'voltage_compensation_out_of_range'):
            code = '93'
        elif word == 'right':
            code = '95'
        elif word in ('straight', 'tracking', 'active', 'valid', 'current'):
            code = '92'
        else:
            code = '96'
        return '\033[' + code + 'm' + match.group() + '\033[0m'

    return '\n'.join('\033[90m' + line + '\033[0m' if line.startswith('+') else
                     '\033[1;97m' + line + '\033[0m' if TITLE in line else
                     _TOKEN.sub(token, line) for line in plain.splitlines())


def write_snapshot(plain, mode='auto', stream=None, environ=None):
    stream = sys.stdout if stream is None else stream
    environ = os.environ if environ is None else environ
    stream.write('\n' + (colorize_status(plain) if use_color(mode, stream, environ) else plain) + '\n')
    stream.flush()
