# Model-free normalized voltage differences, not direction probabilities.
# Assumed mapping: ADC0=head, ADC1=left, ADC2=right; verify physical wiring.
std_msgs/Header header
# Acquisition reference time; held results retain their original stamp.
time sample_stamp
bool has_comparison
bool updated
# False for warmup, stale, calibration fallback, gain changes or invalid input.
bool reliable
# Same quality bits as QuadrantProbabilities, excluding MODEL_UNAVAILABLE.
uint32 status
# (left-right)/(left+right); positive means left is stronger.
float64 left_right_normalized_difference
# (head-sides)/(head+sides), sides=(left+right)/2; positive means head is stronger.
# Both metrics use window means, range [-1,1]. Zero denominator yields 0, unreliable.
float64 head_sides_normalized_difference
