std_msgs/Header header
bool enabled
string motion_mode
string status
bool decision_updated
bool parameters_pending
uint64 cycle_index
float64 frequency
float64 window_s
# Full threshold-frame width on L/(L+R), [0,1]: 0.15 means +/-7.5%, inclusive.
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
# (L-R+lr_voltage_offset_v)/AVG + 4*lr_ratio_offset, from the retained decision.
# Diagnostic normalization stays channel_mean; compare this to +/-2*threshold.
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
# Left share += offset, right share -= offset; +0.05 shifts the frame left 5%.
float64 lr_ratio_offset
