# C-order inflated free mask in continuous odom. Zero includes unknown space.
std_msgs/Header header
float64[3] lower
float64 resolution
uint32[3] shape
float64[3] envelope
uint64 version
uint8[<=2000000] free
