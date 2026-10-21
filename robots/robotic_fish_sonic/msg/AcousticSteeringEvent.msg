std_msgs/Header header
# IDs restart for each motion process; correlate with the latched sonic/config.
uint64 request_id
# accepted, rejected, superseded, applied, or cancelled (shutdown).
string event
string detail
# Requested values; rejected non-finite values are retained as NaN/Inf.
# Inclusive fraction of the channel arithmetic mean: 0.15 = 15%.
float64 threshold
string difference_reference
float64 turn_bias
# Volts: add once to L-R; positive favors left, negative favors right.
float64 lr_voltage_offset_v
# Nonzero only for an applied request: the cycle where it took effect.
uint64 cycle_index
