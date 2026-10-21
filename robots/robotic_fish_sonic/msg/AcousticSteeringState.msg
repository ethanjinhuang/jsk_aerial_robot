std_msgs/Header header
bool enabled
string motion_mode
string status
bool decision_updated
bool parameters_pending
uint64 cycle_index
float64 frequency
float64 window_s
# Difference divided by channel arithmetic mean; 0.15 means 15%, inclusive.
# Range [0,2]; equal channels always select straight, including threshold=0.
float64 threshold
string difference_reference
float64 turn_bias
# Volts: add once to left_mean-right_mean, keeping the denominator unchanged.
float64 lr_voltage_offset_v
int8 left_bias_sign
# These stamps describe the last decision, retained between updates.
time window_start
time window_end
# Latest accepted acquisition stamp; inspect status before using held decisions.
time sample_stamp
uint32 sample_count
float64 left_mean
float64 right_mean
# (left_mean-right_mean)/((left_mean+right_mean)/2), a fraction, not percent.
float64 normalized_difference
# Normalized difference after voltage compensation, from the retained decision.
float64 compensated_difference
float64 command_bias
# Retained previous-cycle values, not instantaneous voltages.
float64 head_mean
float64 voltage_difference
# Diagnostic channels: left_mean+offset/2 and right_mean-offset/2.
# These do not replace the measured/calibrated means above.
float64 compensated_left_mean
float64 compensated_right_mean
float64 compensated_voltage_difference
bool has_decision
# False while stopped, outside tracking, or holding / warming up.
bool decision_current
string decision_direction
# Current action: left/right/straight, holding, warmup, stopped, not_tracking, idle.
string direction
