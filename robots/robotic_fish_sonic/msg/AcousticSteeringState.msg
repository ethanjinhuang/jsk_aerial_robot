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
float64 lr_offset
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
# normalized_difference - lr_offset used by the retained decision.
float64 compensated_difference
float64 command_bias
# Retained previous-cycle values, not instantaneous voltages.
float64 head_mean
float64 voltage_difference
bool has_decision
# False while stopped, outside tracking, or holding / warming up.
bool decision_current
string decision_direction
# Current action: left/right/straight, holding, warmup, stopped, not_tracking, idle.
string direction
