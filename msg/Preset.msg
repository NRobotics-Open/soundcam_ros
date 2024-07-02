uint8 SCALING_MODE_FIXED = 0
uint8 SCALING_MODE_AUTO = 1
uint8 SCALING_MODE_SMART = 2
uint8 scalingMode #the value will be either 0, 1 or 2 (above)

float32 crest #5-15
int32 distance # approx. distance of sound source (<=3500mm)
float32 maximum  # maximum threshold (fixed scaling)
float32 dynamic
int32 maxFrequency #maximum configured frequency
int32 minFrequency #minimum configured frequency