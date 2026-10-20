# Host ROS receive/parse time, not the sensor's physical sampling time.
std_msgs/Header header
# Manufacturer side identifier: 1=left, 2=right.
uint8 foot_id
# Points 1..18 in vendor order; nominal grams, not calibrated newtons.
uint16[18] raw_g
# Accepted sample count per foot since this reader process started.
# The manufacturer protocol has no sequence number; this cannot measure sensor losses.
uint64 sample_index
