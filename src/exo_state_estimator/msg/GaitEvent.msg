# Observation time in the same ROS clock as IMU, joints and VIO.
std_msgs/Header header
uint8 LEFT=1
uint8 RIGHT=2
uint8 foot_id
uint8 HEEL_CONTACT=1
uint8 HEEL_OFF=2
uint8 TOE_CONTACT=3
uint8 TOE_OFF=4
uint8 FULL_FOOT_CONTACT=5
uint8 INITIAL_CONTACT=6
uint8 FINAL_CONTACT_OFF=7
uint8 event
# Strictly increasing per foot. Reset estimator when the detector restarts.
uint64 event_id
float64 confidence
bool valid
