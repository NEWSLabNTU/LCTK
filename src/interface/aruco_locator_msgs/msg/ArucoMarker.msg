# Single ArUco marker detection
# Represents a detected ArUco marker with its ID, 2D corners, and estimated 3D pose

# Marker ID
int32 id

# 2D image corners of the marker (4 corners in pixel coordinates)
# Corners are ordered: top-left, top-right, bottom-right, bottom-left
Point2D[4] corners

# Whether pose estimation was successful
bool pose_valid

# 3D pose of the marker relative to the camera frame
# Only valid if pose_valid is true
geometry_msgs/PoseWithCovariance pose