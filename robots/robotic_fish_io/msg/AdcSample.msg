# One ADS1115 conversion. Each channel conversion has its own timing window.
std_msgs/Header header
uint8 channel
int32 raw
float64 voltage
float64 calibrated_voltage
float64 calibration_gain
bool calibration_applied
string calibration_id
# Last confirmed DAC gain-control voltage when this conversion started.
float64 gain_control_voltage
bool gain_control_voltage_valid
time conversion_start
time conversion_end
