# One read-only snapshot per control tick, including before the first Joy input.
std_msgs/Header header
# Local monotonic seconds; compare only within this motion process / host boot.
float64 monotonic_s
float64 control_interval_s
bool command_published
string motion_mode
string reason
string input_error
# Servo command units. NaN target and zero integer command when no command exists.
float64 target_position
int16 target_command
float64 requested_bias
float64 applied_bias
float64 requested_amplitude
float64 effective_amplitude
float64 frequency
# NaN outside ordinary auto / tracking; phase is in [0, 2*pi).
float64 gait_elapsed_s
float64 phase_rad
# Local receipt ages, NaN before the corresponding input.
float64 joy_receipt_age_s
float64 valid_joy_age_s
bool acoustic_enabled
string acoustic_status
bool has_adc
bool adc_structurally_valid
# Latest received group, even if rejected by the steering controller.
time adc_sample_stamp
time adc_receipt_stamp
float64 adc_source_age_s
float64 adc_source_age_at_receipt_s
float64 adc_receipt_age_s
float64 adc_receipt_interval_s
# Sample-quality bits from acoustic_core.Status; timing also appears in adc_quality.
uint32 adc_quality_flags
string adc_quality
string acoustic_input_error
# Trailing one-period ADC voltage statistics, in every motion mode.
# The window ends at adc_sample_stamp when adc_cycle_ready is true.
bool adc_cycle_ready
float64 adc_cycle_window_s
uint32 adc_cycle_sample_count
uint32 adc_cycle_quality_flags
# Head, left, right order. Effective means use raw values on calibration fallback.
float64[3] adc_cycle_raw_means
float64[3] adc_cycle_means
float64 adc_cycle_side_mean
# Difference: (left-right)/side_mean.
# Compensated: (left-right+voltage_offset)/side_mean + 4*lr_ratio_offset.
# Diagnostics retain channel_mean normalization; the threshold below is full bar width.
float64 adc_cycle_difference
float64 adc_cycle_compensated_difference
float64 adc_cycle_threshold
float64 adc_cycle_lr_voltage_offset_v
float64 adc_cycle_lr_ratio_offset
