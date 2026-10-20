# A fresh observation, not a periodically re-stamped cached classification.
std_msgs/Header header
uint8 LEFT=1
uint8 RIGHT=2
uint8 foot_id
bool any_contact
# Confirmed stable full-sole support; pressure contact alone is insufficient.
bool full_foot_stable
float64 confidence
bool valid
