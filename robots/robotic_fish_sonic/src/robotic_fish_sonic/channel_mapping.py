"""Hydrophone directions above the physical ADC/calibration boundary.

Layout names describe ADC0, ADC1, ADC2 respectively. Sample messages and raw
diagnostics always retain physical channel IDs; only direction consumers map
them to head/left/right.
"""
from itertools import permutations

DEFAULT_CHANNEL_LAYOUT = 'left_right_head'
LEGACY_CHANNEL_LAYOUT = 'head_left_right'
_DIRECTIONS = ('head', 'left', 'right')
_DIRECTION_CHANNELS = {
    '_'.join(order): tuple(order.index(direction) for direction in _DIRECTIONS)
    for order in permutations(_DIRECTIONS)
}
CHANNEL_LAYOUTS = tuple(_DIRECTION_CHANNELS)


def layout_from_pzt_channels(channels):
    """Convert the YAML mapping to a layout name in physical ADC order."""
    if channels is None:
        return DEFAULT_CHANNEL_LAYOUT
    keys = ('FrontPZT', 'LeftPZT', 'RightPZT')
    if not isinstance(channels, dict) or set(channels) != set(keys):
        raise ValueError('hydrophone_channels must contain FrontPZT, LeftPZT and RightPZT')
    ids = tuple(channels[key] for key in keys)
    if any(type(channel) is not int for channel in ids) or set(ids) != {0, 1, 2}:
        raise ValueError('FrontPZT, LeftPZT and RightPZT must each use a distinct integer ADC ID (0, 1, 2)')
    return '_'.join(_DIRECTIONS[ids.index(channel)] for channel in range(3))


def channel_mapping(layout=DEFAULT_CHANNEL_LAYOUT):
    """Return direction -> physical ADC ID, rejecting unknown layouts."""
    if not isinstance(layout, str) or layout not in _DIRECTION_CHANNELS:
        raise ValueError('unknown hydrophone channel layout: ' + str(layout))
    return dict(zip(_DIRECTIONS, _DIRECTION_CHANNELS[layout]))


def head_left_right(adc_values, layout=DEFAULT_CHANNEL_LAYOUT):
    """Convert exactly three physical ADC0/1/2 values to directional order."""
    channels = channel_mapping(layout)
    if len(adc_values) != 3:
        raise ValueError('expected three values in physical ADC0, ADC1, ADC2 order')
    return tuple(adc_values[channels[direction]] for direction in _DIRECTIONS)
