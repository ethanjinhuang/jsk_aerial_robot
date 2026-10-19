"""ADS1115 constants for the configured +/-4.096 V range."""
FULL_SCALE_V = 4.096
LSB_V = FULL_SCALE_V / 32768.0
MAX_RAW = 32767
MAX_VOLTAGE_V = MAX_RAW * LSB_V
