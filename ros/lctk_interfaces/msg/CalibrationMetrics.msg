# Calibration quality metrics computed from IoU analysis
# Used to evaluate the accuracy of extrinsic calibration between LiDAR and camera

# Header with timestamp
std_msgs/Header header

# Intersection over Union score (0.0 to 1.0)
# Measures overlap between ground truth board region and projected pointcloud
float64 iou

# Coverage: fraction of ground truth region covered by projection (0.0 to 1.0)
float64 coverage

# Precision: fraction of projected region within ground truth (0.0 to 1.0)
float64 precision

# Number of projected points within image bounds
uint32 projected_point_count

# Number of projected points within ground truth region
uint32 inlier_point_count

# Ground truth region area in pixels
float64 ground_truth_area

# Projected region area in pixels
float64 projected_area

# Intersection area in pixels
float64 intersection_area

# Union area in pixels
float64 union_area

# Status message (e.g., "OK", "No board detected", "Insufficient points")
string status
