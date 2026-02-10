# Planned Path

uint8 PATH_FIELD_SET=1
uint8 TARGET_FIELD_SET=16

# Navigation path
woosh_nav_msgs/Path path
# Map ID of the path
uint32 map_id
# Destination wormhole ID; 0 means no wormhole
uint32 wormhole_id
# Map ID reached by the wormhole
uint32 dest_map_id
# Segment target points
woosh_common_msgs/Pose2D target
# Path optimization
woosh_nav_msgs/PlanPathOptimal optimal
