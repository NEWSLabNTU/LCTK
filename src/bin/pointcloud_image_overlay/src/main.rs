use anyhow::{anyhow, ensure, Result};
use builtin_interfaces::msg::Time;
use geometry_msgs::msg::TransformStamped;
use nalgebra as na;
use palette::{Hsv, IntoColor, RgbHue, Srgb};
use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, InitOptions, Node, RclrsErrorFilter,
    SpinOptions, Subscription, ToLogParams,
};
use rerun::{ChannelDatatype, ColorModel, Image as RerunImage, Pinhole, Points3D, RecordingStream};
use sensor_msgs::msg::{CameraInfo, Image, PointCloud2};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex, OnceLock},
    time::{SystemTime, UNIX_EPOCH},
};
use unzip_n::unzip_n;

unzip_n!(3);

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
    // Rerun recording stream for visualization
    rerun_rec: OnceLock<RecordingStream>,
}

pub struct PointcloudImageOverlayNode {
    state: Arc<PointcloudImageOverlayState>,
    _node: Node,
    _pointcloud_subscription: Subscription<PointCloud2>,
    _image_subscription: Subscription<Image>,
    _camera_info_subscription: Subscription<CameraInfo>,
    _transform_subscription: Subscription<TransformStamped>,
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
            rerun_rec: OnceLock::new(),
        });

        // Create subscribers for synchronized messages from synchronizer
        let pointcloud_subscription = {
            let state = Arc::clone(&state);

            node.create_subscription::<PointCloud2, _>(
                "input_pointcloud",
                move |msg: PointCloud2| {
                    Self::pointcloud_callback(msg, &state);
                },
            )?
        };

        let image_subscription = {
            let state = Arc::clone(&state);

            node.create_subscription::<Image, _>("input_image", move |msg: Image| {
                Self::image_callback(msg, &state);
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

            node.create_subscription::<TransformStamped, _>(
                "extrinsic_transform",
                move |msg: TransformStamped| {
                    Self::transform_callback(msg, &state);
                },
            )?
        };

        log_info!(
            LOGGER_NAME,
            "Pointcloud image overlay node initialized. Distance range: {min_distance}-{max_distance}m. Message timeout: {}s. Subscribing to synchronized topics.",
            message_timeout
        );

        Ok(Self {
            state,
            _node: node,
            _pointcloud_subscription: pointcloud_subscription,
            _image_subscription: image_subscription,
            _camera_info_subscription: camera_info_subscription,
            _transform_subscription: transform_subscription,
        })
    }

    fn pointcloud_callback(msg: PointCloud2, state: &Arc<PointcloudImageOverlayState>) {
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
        Self::try_process_overlay_with_correlation(&correlation_id, state);
    }

    fn image_callback(msg: Image, state: &Arc<PointcloudImageOverlayState>) {
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
        Self::try_process_overlay_with_correlation(&correlation_id, state);
    }

    fn camera_info_callback(msg: CameraInfo, state: &Arc<PointcloudImageOverlayState>) {
        *state.camera_info.lock().unwrap() = Some(msg);
        log_info!(
            LOGGER_NAME,
            "Updated camera intrinsics from CameraInfo topic"
        );
    }

    fn transform_callback(msg: TransformStamped, state: &Arc<PointcloudImageOverlayState>) {
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
        Self::try_process_overlay_with_correlation(&correlation_id, state);

        log_info!(
            LOGGER_NAME,
            "Updated extrinsic transform from topic with correlation ID: {}",
            correlation_id
        );
    }

    fn try_process_overlay_with_correlation(
        correlation_id: &str,
        state: &Arc<PointcloudImageOverlayState>,
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
                    if let Err(e) = Self::process_overlay_with_rerun(
                        pc_msg.message.clone(),
                        img_msg.message.clone(),
                        cam_info,
                        tf_msg.message,
                        state,
                        correlation_id,
                    ) {
                        log_warn!(
                            LOGGER_NAME,
                            "Failed to process Rerun overlay for correlation {}: {e}",
                            correlation_id
                        );
                    } else {
                        log_info!(
                            LOGGER_NAME,
                            "Successfully processed Rerun overlay for correlation: {}",
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

    fn process_overlay_with_rerun(
        pointcloud: PointCloud2,
        image: Image,
        camera_info: CameraInfo,
        transform: na::Isometry3<f64>,
        state: &PointcloudImageOverlayState,
        correlation_id: &str,
    ) -> Result<()> {
        // Get the Rerun recording stream
        let rec = state
            .rerun_rec
            .get()
            .ok_or_else(|| anyhow!("Rerun recording stream not initialized"))?;

        // Log the camera image as background
        let (width, height, tensor_data) = Self::ros_image_to_rerun_data(&image)?;
        rec.log(
            "camera/image",
            &RerunImage::from_color_model_and_bytes(
                tensor_data,
                [width, height],
                ColorModel::RGB,
                ChannelDatatype::U8,
            ),
        )?;

        // Log camera intrinsics for proper projection
        let focal_length = [camera_info.k[0] as f32, camera_info.k[4] as f32]; // fx, fy
        let resolution = [image.width as f32, image.height as f32];
        let principal_point = [camera_info.k[2] as f32, camera_info.k[5] as f32]; // cx, cy

        rec.log(
            "camera",
            &Pinhole::from_focal_length_and_resolution(focal_length, resolution)
                .with_principal_point(principal_point),
        )?;

        // Extract and transform points
        let input_points = Self::extract_points_from_pointcloud(&pointcloud)?;
        let distance_range = state.min_distance..=state.max_distance;

        // Filter and collect points with colors
        let (positions, colors, radii): (Vec<[f32; 3]>, Vec<u32>, Vec<f32>) = input_points
            .iter()
            .filter(|&point| {
                let distance = na::distance(&na::Point3::origin(), point);
                distance_range.contains(&distance)
            })
            .filter(|&point| {
                state.keep_back_points || {
                    let camera_point = transform * point;
                    camera_point.z >= state.camera_depth_thresh
                }
            })
            .map(|point| {
                let camera_point = transform * point;
                let distance = na::distance(&na::Point3::origin(), point);

                // Distance-based color (same HSV mapping as before)
                let ratio =
                    (distance - state.min_distance) / (state.max_distance - state.min_distance);
                let color = if (0.0..=1.0).contains(&ratio) {
                    let hue = ratio as f32 * 270.0;
                    let hsv = Hsv::new(RgbHue::from_degrees(hue), 0.8, 1.0);
                    let srgb: Srgb = hsv.into_color();
                    let (r, g, b) = srgb.into_components();
                    let r = (r * 255.0) as u8;
                    let g = (g * 255.0) as u8;
                    let b = (b * 255.0) as u8;
                    ((r as u32) << 16) | ((g as u32) << 8) | (b as u32) | 0xFF000000
                } else {
                    0xFF646464 // Gray color
                };

                (
                    [
                        camera_point.x as f32,
                        camera_point.y as f32,
                        camera_point.z as f32,
                    ],
                    color,
                    2.0f32, // Point size
                )
            })
            .unzip_n();

        // Log the 3D points (Rerun automatically projects them onto camera view)
        if !positions.is_empty() {
            rec.log(
                format!("camera/points/{}", correlation_id),
                &Points3D::new(positions)
                    .with_colors(colors)
                    .with_radii(radii),
            )?;
        }

        Ok(())
    }

    fn ros_image_to_rerun_data(image: &Image) -> Result<(u32, u32, Vec<u8>)> {
        let height = image.height;
        let width = image.width;

        // Convert image data based on encoding
        match image.encoding.as_str() {
            "bgr8" => {
                // BGR8 format - convert to RGB for Rerun
                let mut rgb_data = Vec::with_capacity(image.data.len());
                for chunk in image.data.chunks_exact(3) {
                    rgb_data.push(chunk[2]); // R
                    rgb_data.push(chunk[1]); // G
                    rgb_data.push(chunk[0]); // B
                }
                Ok((width, height, rgb_data))
            }
            "rgb8" => Ok((width, height, image.data.clone())),
            "mono8" => {
                // Convert mono to RGB by replicating the single channel
                let mut rgb_data = Vec::with_capacity(image.data.len() * 3);
                for &pixel in &image.data {
                    rgb_data.push(pixel); // R
                    rgb_data.push(pixel); // G
                    rgb_data.push(pixel); // B
                }
                Ok((width, height, rgb_data))
            }
            _ => Err(anyhow!("Unsupported image encoding: {}", image.encoding)),
        }
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
    let overlay_node = PointcloudImageOverlayNode::new(node)?;

    // Initialize Rerun recording stream
    let rec = RecordingStream::global(rerun::StoreKind::Recording)
        .ok_or_else(|| anyhow!("Failed to create Rerun recording stream"))?;
    overlay_node
        .state
        .rerun_rec
        .set(rec)
        .map_err(|_| anyhow!("Failed to set Rerun recording stream"))?;

    log_info!(
        LOGGER_NAME,
        "Pointcloud image overlay node started with Rerun visualization"
    );

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
