std_msgs/Header header
# IDs restart for each motion process; correlate with the latched sonic/config.
uint64 request_id
# accepted, rejected, superseded, applied, or cancelled (shutdown).
string event
string detail
# Requested values; rejected non-finite values are retained as NaN/Inf.
# Full threshold-frame width: 0.15 = 15%, or +/-7.5% of the bar.
float64 threshold
string difference_reference
float64 turn_bias
# Volts: add once to L-R; positive favors left, negative favors right.
float64 lr_voltage_offset_v
# Resolved polarity: an accepted keep-current (0) request records -1 or +1.
# Invalid requested values are retained on rejected events.
int8 left_bias_sign
# Nonzero only for an applied request: the cycle where it took effect.
uint64 cycle_index
# Requested bar-share offset; +0.05 shifts the frame left 5%.
float64 lr_ratio_offset
