uint8 SCALING_MODE_FIXED = 0
uint8 SCALING_MODE_AUTO = 1
uint8 SCALING_MODE_SMART = 2
int8 scalingMode #the value will be either 0, 1 or 2 (above)

float32 crest #5-15
float32 distance # approx. distance of sound source (<=3.5m)
float32 maximum  # maximum threshold (fixed scaling)
float32 dynamic
int16 maxFrequency #maximum configured frequency
int16 minFrequency #minimum configured frequency