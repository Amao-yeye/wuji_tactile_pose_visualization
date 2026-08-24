# Raw pressure from the paired Wuji Hand tactile glove.
# `pressure` is row-major and preserves NaN values reported by the SDK.
# `rows * cols` must equal `pressure.size()`.

std_msgs/Header header
uint8 handedness
uint32 sequence
uint32 device_timestamp_ms
uint32 rows
uint32 cols
float32[] pressure
