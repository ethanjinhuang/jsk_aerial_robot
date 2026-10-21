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
    row('Direction', '%s; bias=(%s/%s)' %
        (data['direction'].upper(), numeric(data['bias'], 1, True), numeric(data['bias_limit'], 1)))
    row('ADC quality', data['quality'])
    if data['mode'] == 'stopped':
        row('Reason', data['reason'])
    lines.append(border)
    lines.append('| %-22s | %15s | %15s | %13s |' %
                 ('Voltage [V]', 'Head (ADC0)', 'Left (ADC1)', 'Right (ADC2)'))
    voltages('Raw instant', data['raw'])
    voltages('Raw cycle mean', data['raw_means'])
    voltages('Calibrated cycle mean', data['means'])
    lines.append(border)
    if data['means'] is None:
        row('Cycle', 'UNAVAILABLE (no completed tracking cycle)')
    elif data['historical']:
        row('Cycle', 'HISTORICAL (held means and difference)')
    row('Diff before [%]', percent(data['difference'], True))
    row('Diff after/T [%]', '%s / %s' % (percent(data['compensated_difference'], True),
                                     percent(data['threshold'])))
    row('LR offset [V]', numeric(data['lr_voltage_offset_v'], 5, True))
    if data['pending_lr_voltage_offset_v'] is not None:
        row('Pending LR [V]', numeric(data['pending_lr_voltage_offset_v'], 5, True))
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
