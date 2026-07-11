# Navigation Mode Settings
woosh_nav_msgs/ArrType type
woosh_nav_msgs/Mode mode
# Timeout duration (seconds), effective when nav_mode is kTimeout
uint32 wait_timeout
# Maximum speed for navigation, 0 defaults to default speed
float32 max_speed
# Whether passage is allowed
bool permitted_passage
# Passage Vehicle Count
int32 capacity
