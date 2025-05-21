# ArUco marker detection results
# Message containing all detected ArUco markers in an image

# Header with timestamp and frame information
std_msgs/Header header

# Whether any markers were found
bool markers_found

# Number of detected markers
uint32 marker_count

# Array of detected markers
ArucoMarker[] markers

# Processing time in milliseconds
float64 processing_time_ms