# One sequential ADS1115 conversion; timestamp is host result-read completion.
uint32 serial_num
time timestamp
string device
uint8 channel_id
int32 adc_code
float64 volt_raw
# Falls back to volt_raw when status_cali is false.
float64 volt_cali
# volt_cali - volt_raw; NaN when calibration was not applied.
float64 diff_cali
bool status_cali
string cali_id
# Latest confirmed DAC state captured before starting this conversion.
# NaN when status_dac_feedback is false; not independent analog feedback.
float64 dac_volt
bool status_dac_feedback
