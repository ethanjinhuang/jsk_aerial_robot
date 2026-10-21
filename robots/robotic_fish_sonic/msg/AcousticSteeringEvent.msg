std_msgs/Header header
# IDs restart for each motion process; correlate with the latched sonic/config.
uint64 request_id
# accepted, rejected, superseded, applied, or cancelled (shutdown).
string event
string detail
# Requested values; rejected non-finite values are retained as NaN/Inf.
float64 threshold
float64 turn_bias
float64 lr_offset
# Nonzero only for an applied request: the cycle where it took effect.
uint64 cycle_index
