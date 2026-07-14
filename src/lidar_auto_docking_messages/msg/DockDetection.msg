# Timestamp of the LaserScan used for this detection and frame of pose.
# Consumers must reject stale detections by checking header.stamp.
std_msgs/Header header

bool detected

# Geometry and ICP data from the selected, accepted candidate.
float64 width
uint32 point_count
float64 fitness

geometry_msgs/Pose pose

# Planar norm of pose.position in header.frame_id.
float64 lidar_distance
