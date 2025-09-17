use builtin_interfaces::msg::Time;
use geometry_msgs::msg::Transform;
use hollow_board_detector::Detection;
use nalgebra as na;
use sensor_msgs::msg::PointCloud2;
use std::time::{Duration, SystemTime};
use std_msgs::msg::Header;
use vision_msgs::msg::{Detection3D, Detection3DArray, ObjectHypothesisWithPose};
use visualization_msgs::msg::{Marker, MarkerArray};

/// Point with 3D position and intensity
#[derive(Debug, Clone)]
pub struct LidarPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl Default for LidarPoint {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            intensity: 0.0,
        }
    }
}

/// Board detection result
#[derive(Debug, Clone)]
pub struct BoardDetection {
    pub pose: nalgebra::Isometry3<f64>,
    pub confidence: f64,
    pub inlier_count: usize,
    pub timestamp: SystemTime,
}

/// Detection with timestamp
#[derive(Debug, Clone)]
pub struct TimestampedDetection {
    pub timestamp: Time,
    pub detection: BoardDetection,
    pub lidar_id: u8,
}

/// Calibration result
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    pub transform: Transform,
    pub timestamp1: u64,
    pub timestamp2: u64,
}

/// ROI bounding box definition
#[derive(Debug, Clone)]
pub struct RoiBounds {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub min_z: f64,
    pub max_z: f64,
}

impl RoiBounds {
    pub fn new(
        center_x: f64,
        center_y: f64,
        center_z: f64,
        size_x: f64,
        size_y: f64,
        size_z: f64,
    ) -> Self {
        let half_x = size_x / 2.0;
        let half_y = size_y / 2.0;
        let half_z = size_z / 2.0;

        Self {
            min_x: center_x - half_x,
            max_x: center_x + half_x,
            min_y: center_y - half_y,
            max_y: center_y + half_y,
            min_z: center_z - half_z,
            max_z: center_z + half_z,
        }
    }

    pub fn center(&self) -> (f64, f64, f64) {
        (
            (self.min_x + self.max_x) / 2.0,
            (self.min_y + self.max_y) / 2.0,
            (self.min_z + self.max_z) / 2.0,
        )
    }

    pub fn size(&self) -> (f64, f64, f64) {
        (
            self.max_x - self.min_x,
            self.max_y - self.min_y,
            self.max_z - self.min_z,
        )
    }
}

/// Parse PointCloud2 message into vector of points
pub fn parse_pointcloud2(msg: &PointCloud2) -> Result<Vec<LidarPoint>, String> {
    // Check if the point cloud has the expected fields
    let x_field = msg
        .fields
        .iter()
        .find(|f| &f.name == "x")
        .ok_or("Missing 'x' field in PointCloud2")?;
    let y_field = msg
        .fields
        .iter()
        .find(|f| &f.name == "y")
        .ok_or("Missing 'y' field in PointCloud2")?;
    let z_field = msg
        .fields
        .iter()
        .find(|f| &f.name == "z")
        .ok_or("Missing 'z' field in PointCloud2")?;
    let intensity_field = msg.fields.iter().find(|f| &f.name == "intensity");

    let point_step = msg.point_step as usize;
    let num_points = (msg.width * msg.height) as usize;
    let mut points = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let base_offset = i * point_step;

        // Extract x, y, z coordinates
        let x = read_f32(&msg.data, base_offset + x_field.offset as usize)?;
        let y = read_f32(&msg.data, base_offset + y_field.offset as usize)?;
        let z = read_f32(&msg.data, base_offset + z_field.offset as usize)?;

        // Extract intensity if available
        let intensity = if let Some(field) = intensity_field {
            read_f32(&msg.data, base_offset + field.offset as usize)?
        } else {
            0.0
        };

        points.push(LidarPoint { x, y, z, intensity });
    }

    Ok(points)
}

/// Read f32 from byte array
fn read_f32(data: &[u8], offset: usize) -> Result<f32, String> {
    if offset + 4 > data.len() {
        return Err("Buffer overflow when reading f32".to_string());
    }
    let bytes: [u8; 4] = data[offset..offset + 4]
        .try_into()
        .map_err(|_| "Failed to convert bytes to f32")?;
    Ok(f32::from_le_bytes(bytes))
}

/// Create Detection3DArray message from board detection
pub fn create_detection_message(detection: &Detection, header: &Header) -> Detection3DArray {
    // Create header
    let header = header.clone();

    // Create bounding box center position
    let center = detection.board_model.pose.translation.vector;
    let position = geometry_msgs::msg::Point {
        x: center.x,
        y: center.y,
        z: center.z,
    };

    // Create orientation
    let q = detection.board_model.pose.rotation;
    let orientation = geometry_msgs::msg::Quaternion {
        x: q.i,
        y: q.j,
        z: q.k,
        w: q.w,
    };

    // Create pose
    let center_pose = geometry_msgs::msg::Pose {
        position,
        orientation,
    };

    // Create bounding box size
    let size = geometry_msgs::msg::Vector3 {
        x: 0.5, // TODO: Get actual board dimensions
        y: 0.5,
        z: 0.1,
    };

    // Create bounding box
    let bbox = vision_msgs::msg::BoundingBox3D {
        center: center_pose,
        size,
    };

    // Create hypothesis
    let class_id = "hollow_board".into();
    let score = 1.0; // Detection doesn't have a score field, using default
    let hypothesis = vision_msgs::msg::ObjectHypothesis { class_id, score };

    let hyp_with_pose = ObjectHypothesisWithPose {
        hypothesis,
        pose: Default::default(),
    };

    // Create detection
    let results = vec![hyp_with_pose];
    let det3d = Detection3D {
        header: Default::default(),
        bbox,
        results,
        id: Default::default(),
    };

    // Create final message
    let detections = vec![det3d];
    Detection3DArray { header, detections }
}

/// Create visualization markers for board
pub fn create_board_markers(detection: &Detection, lidar_id: u8, header: &Header) -> MarkerArray {
    let pos = detection.board_model.pose.translation.vector;
    let q = detection.board_model.pose.rotation;

    // Create position and orientation for reuse
    let position = geometry_msgs::msg::Point {
        x: pos.x,
        y: pos.y,
        z: pos.z,
    };

    let orientation = geometry_msgs::msg::Quaternion {
        x: q.i,
        y: q.j,
        z: q.k,
        w: q.w,
    };

    // Create pose for frame and board markers
    let pose = geometry_msgs::msg::Pose {
        position: position.clone(),
        orientation: orientation.clone(),
    };

    // Create frame marker color based on lidar_id
    let (r, g, b) = if lidar_id == 1 {
        (1.0, 0.0, 0.0)
    } else {
        (0.0, 0.0, 1.0)
    };
    let frame_color = std_msgs::msg::ColorRGBA { r, g, b, a: 0.8 };

    // Create frame marker scale
    let frame_scale = geometry_msgs::msg::Vector3 {
        x: 0.3,  // Arrow length
        y: 0.05, // Arrow width
        z: 0.05, // Arrow height
    };

    // Create lifetime
    let lifetime = builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 };

    // Create frame marker
    let header = header.clone();
    let ns = format!("board_frame_lidar{lidar_id}");
    let id = 0;
    let type_ = 0; // ARROW
    let action = 0; // ADD

    let frame_marker = Marker {
        header: header.clone(),
        ns,
        id,
        type_,
        action,
        pose: pose.clone(),
        scale: frame_scale,
        color: frame_color.clone(),
        lifetime: lifetime.clone(),
        ..Default::default()
    };

    // Create board marker color (same as frame but transparent)
    let mut board_color = frame_color;
    board_color.a = 0.3;

    // Create board marker scale
    let board_scale = geometry_msgs::msg::Vector3 {
        x: 1.0,  // Board width
        y: 1.0,  // Board height
        z: 0.02, // Board thickness
    };

    // Create board marker
    let ns = format!("board_outline_lidar{lidar_id}");
    let id = 1;
    let type_ = 1; // CUBE

    let board_marker = Marker {
        header: header.clone(),
        ns,
        id,
        type_,
        action,
        pose: pose.clone(),
        scale: board_scale,
        color: board_color,
        lifetime: lifetime.clone(),
        ..Default::default()
    };

    // Create text marker position (slightly above board)
    let text_position = geometry_msgs::msg::Point {
        x: position.x,
        y: position.y,
        z: position.z + 0.5,
    };

    let text_orientation = geometry_msgs::msg::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    let text_pose = geometry_msgs::msg::Pose {
        position: text_position,
        orientation: text_orientation,
    };

    // Create text marker scale
    let text_scale = geometry_msgs::msg::Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.2, // Text height
    };

    // Create text color (white)
    let text_color = std_msgs::msg::ColorRGBA {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };

    // Create text marker
    let ns = format!("board_text_lidar{lidar_id}");
    let id = 2;
    let type_ = 9; // TEXT_VIEW_FACING
    let text = format!("LiDAR {lidar_id} Board");

    let text_marker = Marker {
        header,
        ns,
        id,
        type_,
        action,
        pose: text_pose,
        scale: text_scale,
        color: text_color,
        lifetime,
        text,
        ..Default::default()
    };

    // Create marker array
    let markers = vec![frame_marker, board_marker, text_marker];
    MarkerArray { markers }
}

/// Find synchronized detection pair
pub fn find_synchronized_pair(
    detections1: &mut std::collections::VecDeque<TimestampedDetection>,
    detections2: &mut std::collections::VecDeque<TimestampedDetection>,
    tolerance: Duration,
) -> Option<(TimestampedDetection, TimestampedDetection)> {
    let tolerance_ns = tolerance.as_nanos() as u64;

    // Look for synchronized pairs
    for (i, det1) in detections1.iter().enumerate() {
        for (j, det2) in detections2.iter().enumerate() {
            let time1_ns =
                det1.timestamp.sec as u64 * 1_000_000_000 + det1.timestamp.nanosec as u64;
            let time2_ns =
                det2.timestamp.sec as u64 * 1_000_000_000 + det2.timestamp.nanosec as u64;

            let time_diff = time1_ns.abs_diff(time2_ns);

            if time_diff <= tolerance_ns {
                // Found synchronized pair, remove from queues and return
                let det1 = detections1.remove(i).unwrap();
                let det2 = detections2.remove(j).unwrap();
                return Some((det1, det2));
            }
        }
    }

    None
}

/// Compute transformation between two board detections
pub fn compute_transform(
    detection1: &Detection,
    detection2: &Detection,
    same_face: bool,
    apply_bug_fix: bool,
) -> Result<Transform, eyre::Error> {
    // Get board poses
    let pose1 = &detection1.board_model.pose;
    let pose2 = &detection2.board_model.pose;

    // Compute relative transformation
    let transform = if same_face {
        // Both LiDARs see the same face of the board
        pose2.inverse() * pose1
    } else {
        // LiDARs see opposite faces - need to flip
        let flip = na::Isometry3::rotation(na::Vector3::z() * std::f64::consts::PI);
        pose2.inverse() * flip * pose1
    };

    // Apply bug fix if needed (VLP16 coordinate system correction)
    let final_transform = if apply_bug_fix {
        let bug_fix = na::Isometry3::from_parts(
            na::Translation3::new(0.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(
                std::f64::consts::PI,
                0.0,
                std::f64::consts::PI / 2.0,
            ),
        );
        bug_fix * transform * bug_fix.inverse()
    } else {
        transform
    };

    // Convert to ROS Transform message
    let translation = geometry_msgs::msg::Vector3 {
        x: final_transform.translation.x,
        y: final_transform.translation.y,
        z: final_transform.translation.z,
    };

    let q = final_transform.rotation;
    let rotation = geometry_msgs::msg::Quaternion {
        x: q.i,
        y: q.j,
        z: q.k,
        w: q.w,
    };

    let msg = Transform {
        translation,
        rotation,
    };

    Ok(msg)
}

/// Apply ROI cropping to point cloud (simple bounding box filter)
pub fn apply_roi_crop(
    points: &[nalgebra::Point3<f64>],
    roi: &RoiBounds,
) -> Vec<nalgebra::Point3<f64>> {
    points
        .iter()
        .filter(|point| {
            point.x >= roi.min_x
                && point.x <= roi.max_x
                && point.y >= roi.min_y
                && point.y <= roi.max_y
                && point.z >= roi.min_z
                && point.z <= roi.max_z
        })
        .cloned()
        .collect()
}
