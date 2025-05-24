use anyhow::{anyhow, ensure, Result};
use builtin_interfaces::msg::Time;
use cv_convert::{prelude::*, OpenCvPose};
use geometry_msgs::msg::TransformStamped;
use itertools::izip;
use nalgebra as na;
use opencv::{
    calib3d,
    core::{no_array, Point2d, Point2i, Point3d, Scalar, Vector},
    imgproc,
    imgproc::LINE_8,
    prelude::*,
};
use palette::{Hsv, IntoColor, RgbHue, Srgb};
use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, InitOptions, Node, Publisher,
    RclrsErrorFilter, SpinOptions, Subscription, ToLogParams,
};
use sensor_msgs::msg::{CameraInfo, Image, PointCloud2};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{SystemTime, UNIX_EPOCH},
};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

#[derive(Clone)]
struct TimestampedMessage<T> {
    message: T,
    timestamp: u64,
}

#[derive(Clone)]
struct SynchronizedMessages {
    pointcloud: Option<TimestampedMessage<PointCloud2>>,
    image: Option<TimestampedMessage<Image>>,
    transform: Option<TimestampedMessage<na::Isometry3<f64>>>,
    created_at: u64,
}

impl SynchronizedMessages {
    pub fn empty(created_at: u64) -> Self {
        Self {
            pointcloud: None,
            image: None,
            transform: None,
            created_at,
        }
    }
}

struct PointcloudImageOverlayState {
    camera_info: Mutex<Option<CameraInfo>>,
    max_distance: f64,
    min_distance: f64,
    camera_depth_thresh: f64,
    keep_back_points: bool,
    // Store synchronized messages by correlation ID
    synchronized_messages: Mutex<HashMap<String, SynchronizedMessages>>,
    // Timeout for message matching (in seconds)
    message_timeout: u64,
}

pub struct PointcloudImageOverlayNode {
    _state: Arc<PointcloudImageOverlayState>,
    _node: Node,
    _pointcloud_subscription: Subscription<PointCloud2>,
    _image_subscription: Subscription<Image>,
    _camera_info_subscription: Subscription<CameraInfo>,
    _transform_subscription: Subscription<TransformStamped>,
    _overlay_publisher: Publisher<Image>,
}

impl PointcloudImageOverlayNode {
    pub fn new(node: Node) -> Result<Self> {
        // Declare parameters with defaults
        let max_distance: f64 = node
            .declare_parameter("max_distance")
            .default(10.0)
            .mandatory()?
            .get();

        let min_distance: f64 = node
            .declare_parameter("min_distance")
            .default(1.0)
            .mandatory()?
            .get();

        let camera_depth_thresh: f64 = node
            .declare_parameter("camera_depth_thresh")
            .default(0.0)
            .mandatory()?
            .get();

        let keep_back_points: bool = node
            .declare_parameter("keep_back_points")
            .default(false)
            .mandatory()?
            .get();

        ensure!(min_distance < max_distance && min_distance >= 0.0 && max_distance >= 0.0);

        let message_timeout: i64 = node
            .declare_parameter("message_timeout_seconds")
            .default(5i64)
            .mandatory()?
            .get();

        // Create state
        let state = Arc::new(PointcloudImageOverlayState {
            camera_info: Mutex::new(None),
            max_distance,
            min_distance,
            camera_depth_thresh,
            keep_back_points,
            synchronized_messages: Mutex::new(HashMap::new()),
            message_timeout: message_timeout as u64,
        });

        // Create publisher for overlay images
        let overlay_publisher = node.create_publisher::<Image>("pointcloud_image_overlay")?;

        // Create subscribers for synchronized messages from synchronizer
        let pointcloud_subscription = {
            let state = Arc::clone(&state);
            let overlay_publisher = overlay_publisher.clone();

            node.create_subscription::<PointCloud2, _>(
                "input_pointcloud",
                move |msg: PointCloud2| {
                    Self::pointcloud_callback(msg, &state, &overlay_publisher);
                },
            )?
        };

        let image_subscription = {
            let state = Arc::clone(&state);
            let overlay_publisher = overlay_publisher.clone();

            node.create_subscription::<Image, _>("input_image", move |msg: Image| {
                Self::image_callback(msg, &state, &overlay_publisher);
            })?
        };

        let camera_info_subscription = {
            let state = Arc::clone(&state);

            node.create_subscription::<CameraInfo, _>("camera_info", move |msg: CameraInfo| {
                Self::camera_info_callback(msg, &state);
            })?
        };

        let transform_subscription = {
            let state = Arc::clone(&state);
            let overlay_publisher = overlay_publisher.clone();

            node.create_subscription::<TransformStamped, _>(
                "extrinsic_transform",
                move |msg: TransformStamped| {
                    Self::transform_callback(msg, &state, &overlay_publisher);
                },
            )?
        };

        log_info!(
            LOGGER_NAME,
            "Pointcloud image overlay node initialized. Distance range: {min_distance}-{max_distance}m. Message timeout: {}s. Subscribing to synchronized topics.",
            message_timeout
        );

        Ok(Self {
            _state: state,
            _node: node,
            _pointcloud_subscription: pointcloud_subscription,
            _image_subscription: image_subscription,
            _camera_info_subscription: camera_info_subscription,
            _transform_subscription: transform_subscription,
            _overlay_publisher: overlay_publisher,
        })
    }

    fn pointcloud_callback(
        msg: PointCloud2,
        state: &Arc<PointcloudImageOverlayState>,
        overlay_publisher: &Publisher<Image>,
    ) {
        // Extract correlation ID from frame_id
        let correlation_id = msg.header.frame_id.clone();
        let timestamp = Self::ros_time_to_timestamp(&msg.header.stamp);
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Create timestamped pointcloud message
        let timestamped_msg = TimestampedMessage {
            message: msg,
            timestamp,
        };

        // Update or create synchronized message entry
        {
            let mut messages = state.synchronized_messages.lock().unwrap();
            let entry = messages
                .entry(correlation_id.clone())
                .or_insert_with(|| SynchronizedMessages::empty(current_time));
            entry.pointcloud = Some(timestamped_msg);
        }

        // Clean up old messages
        Self::cleanup_old_messages(state);

        // Try to process overlay with matching correlation ID
        Self::try_process_overlay_with_correlation(&correlation_id, state, overlay_publisher);
    }

    fn image_callback(
        msg: Image,
        state: &Arc<PointcloudImageOverlayState>,
        overlay_publisher: &Publisher<Image>,
    ) {
        // Extract correlation ID from frame_id
        let correlation_id = msg.header.frame_id.clone();
        let timestamp = Self::ros_time_to_timestamp(&msg.header.stamp);
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Create timestamped image message
        let timestamped_msg = TimestampedMessage {
            message: msg,
            timestamp,
        };

        // Update or create synchronized message entry
        {
            let mut messages = state.synchronized_messages.lock().unwrap();
            let entry = messages
                .entry(correlation_id.clone())
                .or_insert_with(|| SynchronizedMessages::empty(current_time));
            entry.image = Some(timestamped_msg);
        }

        // Clean up old messages
        Self::cleanup_old_messages(state);

        // Try to process overlay with matching correlation ID
        Self::try_process_overlay_with_correlation(&correlation_id, state, overlay_publisher);
    }

    fn camera_info_callback(msg: CameraInfo, state: &Arc<PointcloudImageOverlayState>) {
        *state.camera_info.lock().unwrap() = Some(msg);
        log_info!(
            LOGGER_NAME,
            "Updated camera intrinsics from CameraInfo topic"
        );
    }

    fn transform_callback(
        msg: TransformStamped,
        state: &Arc<PointcloudImageOverlayState>,
        overlay_publisher: &Publisher<Image>,
    ) {
        // Convert ROS transform to nalgebra isometry
        let translation = na::Vector3::new(
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        );

        let rotation = na::UnitQuaternion::new_normalize(na::Quaternion::new(
            msg.transform.rotation.w,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
        ));

        let transform = na::Isometry3::from_parts(translation.into(), rotation);

        // Extract correlation ID from header frame_id or use child_frame_id
        let correlation_id = msg.child_frame_id.clone();
        let timestamp = Self::ros_time_to_timestamp(&msg.header.stamp);
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Create timestamped transform message
        let timestamped_msg = TimestampedMessage {
            message: transform,
            timestamp,
        };

        // Update or create synchronized message entry
        {
            let mut messages = state.synchronized_messages.lock().unwrap();
            let entry = messages
                .entry(correlation_id.clone())
                .or_insert_with(|| SynchronizedMessages::empty(current_time));
            entry.transform = Some(timestamped_msg);
        }

        // Clean up old messages
        Self::cleanup_old_messages(state);

        // Try to process overlay with matching correlation ID
        Self::try_process_overlay_with_correlation(&correlation_id, state, overlay_publisher);

        log_info!(
            LOGGER_NAME,
            "Updated extrinsic transform from topic with correlation ID: {}",
            correlation_id
        );
    }

    fn try_process_overlay_with_correlation(
        correlation_id: &str,
        state: &Arc<PointcloudImageOverlayState>,
        overlay_publisher: &Publisher<Image>,
    ) {
        // Try to find synchronized messages for this correlation ID
        let synchronized_msg = state
            .synchronized_messages
            .lock()
            .unwrap()
            .get(correlation_id)
            .cloned();
        let camera_info = state.camera_info.lock().unwrap().clone();

        if let (Some(sync_msg), Some(cam_info)) = (synchronized_msg, camera_info) {
            // Check if we have all three required messages
            if let (Some(pc_msg), Some(img_msg), Some(tf_msg)) =
                (&sync_msg.pointcloud, &sync_msg.image, &sync_msg.transform)
            {
                // Check if timestamps are close enough (within 100ms)
                let max_time_diff = 100_000_000; // 100ms in nanoseconds
                let pc_time = pc_msg.timestamp;
                let img_time = img_msg.timestamp;
                let tf_time = tf_msg.timestamp;

                let time_diff_pc_img = if pc_time > img_time {
                    pc_time - img_time
                } else {
                    img_time - pc_time
                };
                let time_diff_pc_tf = if pc_time > tf_time {
                    pc_time - tf_time
                } else {
                    tf_time - pc_time
                };

                if time_diff_pc_img <= max_time_diff && time_diff_pc_tf <= max_time_diff {
                    if let Err(e) = Self::process_overlay(
                        pc_msg.message.clone(),
                        img_msg.message.clone(),
                        cam_info,
                        tf_msg.message,
                        state,
                        overlay_publisher,
                    ) {
                        log_warn!(
                            LOGGER_NAME,
                            "Failed to process overlay for correlation {}: {e}",
                            correlation_id
                        );
                    } else {
                        log_info!(
                            LOGGER_NAME,
                            "Successfully processed overlay for correlation: {}",
                            correlation_id
                        );

                        // Remove processed messages to avoid reprocessing
                        state
                            .synchronized_messages
                            .lock()
                            .unwrap()
                            .remove(correlation_id);
                    }
                } else {
                    log_warn!(
                        LOGGER_NAME,
                        "Timestamp mismatch for correlation {}: pc-img={}ns, pc-tf={}ns",
                        correlation_id,
                        time_diff_pc_img,
                        time_diff_pc_tf
                    );
                }
            }
        }
    }

    fn cleanup_old_messages(state: &Arc<PointcloudImageOverlayState>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let timeout_ns = state.message_timeout * 1_000_000_000; // Convert seconds to nanoseconds

        // Clean up old synchronized message entries
        state
            .synchronized_messages
            .lock()
            .unwrap()
            .retain(|_, sync_msg| current_time.saturating_sub(sync_msg.created_at) < timeout_ns);
    }

    fn ros_time_to_timestamp(stamp: &Time) -> u64 {
        (stamp.sec as u64) * 1_000_000_000 + (stamp.nanosec as u64)
    }

    fn process_overlay(
        pointcloud: PointCloud2,
        image: Image,
        camera_info: CameraInfo,
        transform: na::Isometry3<f64>,
        state: &PointcloudImageOverlayState,
        overlay_publisher: &Publisher<Image>,
    ) -> Result<()> {
        // Convert CameraInfo to OpenCV format
        let camera_matrix = Mat::from_slice_2d(&[
            [camera_info.k[0], camera_info.k[1], camera_info.k[2]],
            [camera_info.k[3], camera_info.k[4], camera_info.k[5]],
            [camera_info.k[6], camera_info.k[7], camera_info.k[8]],
        ])?;

        let dist_coefs = if camera_info.d.is_empty() {
            Mat::zeros(1, 5, opencv::core::CV_64F)?.to_mat()?
        } else {
            Mat::from_slice(&camera_info.d)?
        };

        // Convert ROS Image to OpenCV Mat
        let mut cv_image = Self::ros_image_to_opencv(&image)?;
        let image_h = cv_image.rows();
        let image_w = cv_image.cols();

        // Extract points from PointCloud2
        let input_points = Self::extract_points_from_pointcloud(&pointcloud)?;

        let distance_range = state.min_distance..=state.max_distance;
        let width_range = 0.0..=(image_w as f64);
        let height_range = 0.0..=(image_h as f64);

        let (pcd_points, image_points) = {
            let (pcd_points, opencv_points): (Vec<_>, Vector<Point3d>) = input_points
                .iter()
                .filter(|&pcd_point| {
                    let distance = na::distance(&na::Point3::origin(), pcd_point);
                    distance_range.contains(&distance)
                })
                .filter(|&pcd_point| {
                    state.keep_back_points || {
                        let camera_point = transform * pcd_point;
                        camera_point.z >= state.camera_depth_thresh
                    }
                })
                .map(|point| {
                    let cv_point: Point3d = point.to_cv();
                    (point, cv_point)
                })
                .unzip();

            let mut image_points = Vector::<Point2d>::new();

            if opencv_points.is_empty() {
                (pcd_points, image_points)
            } else {
                let OpenCvPose::<Mat> { rvec, tvec } = transform.try_to_cv()?;

                calib3d::project_points(
                    &opencv_points,
                    &rvec,
                    &tvec,
                    &camera_matrix,
                    &dist_coefs,
                    &mut image_points,
                    &mut no_array(),
                    0.0,
                )?;

                (pcd_points, image_points)
            }
        };

        // Draw points on image
        izip!(pcd_points, image_points)
            .filter(|(_pt3, pt2)| width_range.contains(&pt2.x) && height_range.contains(&pt2.y))
            .for_each(|(pt3, pt2)| {
                let distance = na::distance(&na::Point3::origin(), &pt3);
                let color = {
                    let ratio =
                        (distance - state.min_distance) / (state.max_distance - state.min_distance);
                    if (0.0..=1.0).contains(&ratio) {
                        let hue = RgbHue::from_degrees(ratio as f32 * 270.0);
                        let hsv = Hsv::new(hue, 0.8, 1.0);
                        let srgb: Srgb = hsv.into_color();
                        let (r, g, b) = srgb.into_components();
                        Scalar::new(
                            (b * 255.0) as f64,
                            (g * 255.0) as f64,
                            (r * 255.0) as f64,
                            0.0,
                        )
                    } else {
                        Scalar::new(100.0, 100.0, 100.0, 0.0)
                    }
                };

                let position: Point2i = pt2.to().unwrap();
                imgproc::circle(&mut cv_image, position, 1, color, 1, LINE_8, 0).unwrap();
            });

        // Convert back to ROS Image and publish
        let output_image = Self::opencv_to_ros_image(&cv_image, &image)?;
        if let Err(e) = overlay_publisher.publish(output_image) {
            log_warn!(LOGGER_NAME, "Failed to publish overlay image: {e}");
        }

        Ok(())
    }

    fn ros_image_to_opencv(image: &Image) -> Result<Mat> {
        // Convert ROS Image to OpenCV Mat
        // This is a simplified conversion - you may need cv_bridge for proper conversion
        let rows = image.height as i32;
        let cols = image.width as i32;

        // Assuming BGR8 encoding for now
        if image.encoding == "bgr8" {
            let mat = unsafe {
                Mat::new_rows_cols_with_data(
                    rows,
                    cols,
                    opencv::core::CV_8UC3,
                    image.data.as_ptr() as *mut std::ffi::c_void,
                    opencv::core::Mat_AUTO_STEP,
                )?
            };
            Ok(mat.clone())
        } else {
            Err(anyhow!("Unsupported image encoding: {}", image.encoding))
        }
    }

    fn opencv_to_ros_image(cv_image: &Mat, template: &Image) -> Result<Image> {
        // Convert OpenCV Mat back to ROS Image
        let mut output = template.clone();

        // Get image data as bytes
        let data_size = (cv_image.rows() * cv_image.cols() * cv_image.channels()) as usize;
        let mut data_vec = vec![0u8; data_size];

        unsafe {
            let src_ptr = cv_image.ptr(0)? as *const u8;
            std::ptr::copy_nonoverlapping(src_ptr, data_vec.as_mut_ptr(), data_size);
        }

        output.data = data_vec;
        Ok(output)
    }

    fn extract_points_from_pointcloud(pointcloud: &PointCloud2) -> Result<Vec<na::Point3<f64>>> {
        // This is a simplified point extraction
        // In practice, you'd need to parse the PointCloud2 format properly
        let mut points = Vec::new();

        // Find x, y, z field offsets
        let mut x_offset = None;
        let mut y_offset = None;
        let mut z_offset = None;

        for field in &pointcloud.fields {
            match field.name.as_str() {
                "x" => x_offset = Some(field.offset as usize),
                "y" => y_offset = Some(field.offset as usize),
                "z" => z_offset = Some(field.offset as usize),
                _ => {}
            }
        }

        if let (Some(x_off), Some(y_off), Some(z_off)) = (x_offset, y_offset, z_offset) {
            let point_step = pointcloud.point_step as usize;
            let num_points = pointcloud.data.len() / point_step;

            for i in 0..num_points {
                let base_offset = i * point_step;

                // Extract x, y, z as f32 and convert to f64
                let x = f32::from_le_bytes([
                    pointcloud.data[base_offset + x_off],
                    pointcloud.data[base_offset + x_off + 1],
                    pointcloud.data[base_offset + x_off + 2],
                    pointcloud.data[base_offset + x_off + 3],
                ]) as f64;

                let y = f32::from_le_bytes([
                    pointcloud.data[base_offset + y_off],
                    pointcloud.data[base_offset + y_off + 1],
                    pointcloud.data[base_offset + y_off + 2],
                    pointcloud.data[base_offset + y_off + 3],
                ]) as f64;

                let z = f32::from_le_bytes([
                    pointcloud.data[base_offset + z_off],
                    pointcloud.data[base_offset + z_off + 1],
                    pointcloud.data[base_offset + z_off + 2],
                    pointcloud.data[base_offset + z_off + 3],
                ]) as f64;

                points.push(na::Point3::new(x, y, z));
            }
        }

        Ok(points)
    }
}

fn main() -> Result<()> {
    let context = Context::new(std::env::args(), InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("pointcloud_image_overlay")?;
    let _overlay_node = PointcloudImageOverlayNode::new(node)?;

    log_info!(LOGGER_NAME, "Pointcloud image overlay node started");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
