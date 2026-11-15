"""Hydrophone directions above the physical ADC/calibration boundary.

Layout names describe ADC0, ADC1, ADC2 respectively. Sample messages and raw
diagnostics always retain physical channel IDs; only direction consumers map
them to head/left/right. Legacy wiring is selectable for offline replay only.
"""

DEFAULT_CHANNEL_LAYOUT = 'left_right_head'
LEGACY_CHANNEL_LAYOUT = 'head_left_right'
CHANNEL_LAYOUTS = (DEFAULT_CHANNEL_LAYOUT, LEGACY_CHANNEL_LAYOUT)
_DIRECTION_CHANNELS = {
    DEFAULT_CHANNEL_LAYOUT: (2, 0, 1),
    LEGACY_CHANNEL_LAYOUT: (0, 1, 2),
}


def channel_mapping(layout=DEFAULT_CHANNEL_LAYOUT):
    """Return direction -> physical ADC ID, rejecting unknown layouts."""
    if not isinstance(layout, str) or layout not in _DIRECTION_CHANNELS:
        raise ValueError('unknown hydrophone channel layout: ' + str(layout))
    return dict(zip(('head', 'left', 'right'), _DIRECTION_CHANNELS[layout]))


def head_left_right(adc_values, layout=DEFAULT_CHANNEL_LAYOUT):
    """Convert exactly three physical ADC0/1/2 values to directional order."""
    channels = channel_mapping(layout)
    if len(adc_values) != 3:
        raise ValueError('expected three values in physical ADC0, ADC1, ADC2 order')
    return tuple(adc_values[channels[direction]] for direction in ('head', 'left', 'right'))
