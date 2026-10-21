std_msgs/Header header
bool enabled
string motion_mode
string status
bool decision_updated
bool parameters_pending
uint64 cycle_index
float64 frequency
float64 window_s
float64 threshold
float64 turn_bias
int8 left_bias_sign
# These stamps describe the last decision, retained between updates.
time window_start
time window_end
# Latest accepted acquisition stamp; inspect status before using held decisions.
time sample_stamp
uint32 sample_count
float64 left_mean
float64 right_mean
float64 normalized_difference
float64 command_bias
