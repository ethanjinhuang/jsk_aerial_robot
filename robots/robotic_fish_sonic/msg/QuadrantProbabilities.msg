# Header stamp is publication time; estimate_stamp is never refreshed on a hold.
# FRD: x forward, y right, z down. Probabilities point from fish toward source.
std_msgs/Header header
time estimate_stamp
time window_start
time window_end
float64 probability_front_right
float64 probability_rear_right
float64 probability_rear_left
float64 probability_front_left
float64 probability_left
float64 probability_right
bool has_estimate
bool updated
# Quality annotations never erase an existing estimate.
uint32 WARMUP=1
uint32 LOW_SIGNAL=2
uint32 CALIBRATION_FALLBACK=4
uint32 GAIN_TRANSITION=8
uint32 STALE=16
uint32 INVALID_INPUT=32
uint32 MODEL_UNAVAILABLE=64
uint32 TIME_RESET=128
uint32 GAIN_UNKNOWN=256
uint32 SATURATED=512
uint32 CALIBRATION_CHANGE=1024
uint32 status
uint32 estimate_status
uint32 valid_sample_count
string model_id
string timestamp_semantics
