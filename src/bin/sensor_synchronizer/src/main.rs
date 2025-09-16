use anyhow::{anyhow, Result};
use builtin_interfaces::msg::Time;
use futures::{stream, StreamExt};
use indexmap::IndexMap;
use multi_stream_synchronizer::{sync, Config, StalenessConfig, WithTimestamp};
use rclrs::*;
use sensor_msgs::msg::{Image, PointCloud2};
use std::{
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};
use tokio::sync::mpsc;

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

// Stream key types for synchronization
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum StreamKey {
    Camera,
    Lidar,
}

// Wrapper for Image to implement WithTimestamp
#[derive(Debug, Clone)]
pub struct ImageWrapper {
    pub image: Image,
}

impl WithTimestamp for ImageWrapper {
    fn timestamp(&self) -> Duration {
        ros_time_to_duration(&self.image.header.stamp)
    }
}

// Wrapper for PointCloud2 to implement WithTimestamp
#[derive(Debug, Clone)]
pub struct PointCloudWrapper {
    pub pointcloud: PointCloud2,
}

impl WithTimestamp for PointCloudWrapper {
    fn timestamp(&self) -> Duration {
        ros_time_to_duration(&self.pointcloud.header.stamp)
    }
}

// Combined sensor data type for the synchronizer
#[derive(Debug, Clone)]
pub enum SensorData {
    Image(ImageWrapper),
    PointCloud(PointCloudWrapper),
}

impl WithTimestamp for SensorData {
    fn timestamp(&self) -> Duration {
        match self {
            SensorData::Image(img) => img.timestamp(),
            SensorData::PointCloud(pc) => pc.timestamp(),
        }
    }
}

// State for the synchronizer
struct SyncState {
    config: Config,
    enable_debug: bool,
    sequence_counter: AtomicU32,
    sensor_sender: Mutex<Option<mpsc::UnboundedSender<(StreamKey, SensorData)>>>,
}

pub struct SensorSynchronizer {
    _node: Node,
    _image_subscription: Subscription<Image>,
    _pointcloud_subscription: Subscription<PointCloud2>,
    _sync_image_publisher: Publisher<Image>,
    _sync_pointcloud_publisher: Publisher<PointCloud2>,
}

impl SensorSynchronizer {
    pub fn new(node: Node) -> Result<Self> {
        // Get synchronization parameters
        let sync_window_ms: i64 = node
            .declare_parameter("sync_window_ms")
            .default(50)
            .mandatory()?
            .get();

        let buffer_size: i64 = node
            .declare_parameter("buffer_size")
            .default(20) // Smaller buffer for real-time sensor synchronization
            .mandatory()?
            .get();

        let enable_debug: bool = node
            .declare_parameter("enable_debug")
            .default(false)
            .mandatory()?
            .get();

        let staleness_timeout_ms: i64 = node
            .declare_parameter("staleness_timeout_ms")
            .default(100i64)
            .mandatory()?
            .get();

        log_info!(
            LOGGER_NAME,
            "Real-time sensor synchronizer started with {}ms sync window, buffer size {}, staleness timeout {}ms",
            sync_window_ms,
            buffer_size,
            staleness_timeout_ms
        );

        // Create synchronizer config without staleness detection for debugging
        let config = Config {
            window_size: Duration::from_millis(sync_window_ms as u64),
            start_time: None,
            buf_size: buffer_size as usize,
            staleness_config: None,
        };

        // Create message channel for feeding the synchronizer
        let (sensor_sender, sensor_receiver) = mpsc::unbounded_channel();

        // Create state
        let state = Arc::new(SyncState {
            config,
            enable_debug,
            sequence_counter: AtomicU32::new(0),
            sensor_sender: Mutex::new(Some(sensor_sender)),
        });

        // Create publishers for synchronized data
        let sync_image_publisher = node.create_publisher("synchronized_image")?;
        let sync_pointcloud_publisher = node.create_publisher("synchronized_pointcloud")?;

        // Start the synchronizer task
        {
            let sync_image_publisher = sync_image_publisher.clone();
            let sync_pointcloud_publisher = sync_pointcloud_publisher.clone();
            let state = Arc::clone(&state);

            tokio::spawn(async move {
                Self::run_synchronizer(
                    sensor_receiver,
                    sync_image_publisher,
                    sync_pointcloud_publisher,
                    state,
                )
                .await;
            });
        }

        // Create subscribers
        let image_subscription = {
            let state = Arc::clone(&state);
            node.create_subscription("input_image", move |msg: Image| {
                Self::image_callback(msg, &state);
            })?
        };

        let pointcloud_subscription = {
            let state = Arc::clone(&state);
            node.create_subscription("input_pointcloud", move |msg: PointCloud2| {
                Self::pointcloud_callback(msg, &state);
            })?
        };

        if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Debug mode enabled - will log synchronization statistics"
            );
        }

        Ok(Self {
            _node: node,
            _image_subscription: image_subscription,
            _pointcloud_subscription: pointcloud_subscription,
            _sync_image_publisher: sync_image_publisher,
            _sync_pointcloud_publisher: sync_pointcloud_publisher,
        })
    }

    fn image_callback(msg: Image, state: &Arc<SyncState>) {
        if state.enable_debug {
            log_info!(
                LOGGER_NAME,
                "Received image {}x{} at timestamp {}.{:09}",
                msg.width,
                msg.height,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec
            );
        }

        let wrapped = ImageWrapper { image: msg };
        let sensor_data = SensorData::Image(wrapped);

        if let Some(sender) = state.sensor_sender.lock().unwrap().as_ref() {
            if let Err(e) = sender.send((StreamKey::Camera, sensor_data)) {
                log_warn!(LOGGER_NAME, "Failed to send image to synchronizer: {e}");
            }
        }
    }

    fn pointcloud_callback(msg: PointCloud2, state: &Arc<SyncState>) {
        if state.enable_debug {
            log_info!(
                LOGGER_NAME,
                "Received pointcloud {} points at timestamp {}.{:09}",
                msg.width * msg.height,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec
            );
        }

        let wrapped = PointCloudWrapper { pointcloud: msg };
        let sensor_data = SensorData::PointCloud(wrapped);

        if let Some(sender) = state.sensor_sender.lock().unwrap().as_ref() {
            if let Err(e) = sender.send((StreamKey::Lidar, sensor_data)) {
                log_warn!(
                    LOGGER_NAME,
                    "Failed to send pointcloud to synchronizer: {e}"
                );
            }
        }
    }

    async fn run_synchronizer(
        mut sensor_receiver: mpsc::UnboundedReceiver<(StreamKey, SensorData)>,
        sync_image_publisher: Publisher<Image>,
        sync_pointcloud_publisher: Publisher<PointCloud2>,
        state: Arc<SyncState>,
    ) {
        let sensor_stream =
            stream::poll_fn(move |cx| sensor_receiver.poll_recv(cx)).map(Ok::<_, eyre::Report>);

        let (sync_stream, _feedback_stream) = match sync(
            sensor_stream,
            [StreamKey::Camera, StreamKey::Lidar],
            state.config.clone(),
        ) {
            Ok((stream, feedback)) => (stream, feedback),
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to create synchronizer: {e}");
                return;
            }
        };

        let mut sync_stream = sync_stream;
        while let Some(result) = sync_stream.next().await {
            match result {
                Ok(synchronized_data) => {
                    Self::handle_synchronized_data(
                        synchronized_data,
                        &sync_image_publisher,
                        &sync_pointcloud_publisher,
                        &state,
                    )
                    .await;
                }
                Err(e) => {
                    log_warn!(LOGGER_NAME, "Synchronizer error: {e}");
                }
            }
        }
    }

    async fn handle_synchronized_data(
        synchronized_data: IndexMap<StreamKey, SensorData>,
        sync_image_publisher: &Publisher<Image>,
        sync_pointcloud_publisher: &Publisher<PointCloud2>,
        state: &Arc<SyncState>,
    ) {
        // We need both camera and lidar data to proceed
        let image_data = synchronized_data.get(&StreamKey::Camera);
        let pointcloud_data = synchronized_data.get(&StreamKey::Lidar);

        if let (Some(SensorData::Image(img)), Some(SensorData::PointCloud(pc))) =
            (image_data, pointcloud_data)
        {
            let sequence = state.sequence_counter.fetch_add(1, Ordering::Relaxed);

            if state.enable_debug {
                log_info!(
                    LOGGER_NAME,
                    "Synchronized pair #{}: image {}x{} with pointcloud {} points (time diff: {:?})",
                    sequence,
                    img.image.width,
                    img.image.height,
                    pc.pointcloud.width * pc.pointcloud.height,
                    img.timestamp().saturating_sub(pc.timestamp())
                );
            }

            // Publish synchronized data
            if let Err(e) = sync_image_publisher.publish(img.image.clone()) {
                log_warn!(LOGGER_NAME, "Failed to publish synchronized image: {e}");
            }

            if let Err(e) = sync_pointcloud_publisher.publish(pc.pointcloud.clone()) {
                log_warn!(
                    LOGGER_NAME,
                    "Failed to publish synchronized pointcloud: {e}"
                );
            }
        } else {
            log_warn!(
                LOGGER_NAME,
                "Received incomplete synchronized data (missing camera or lidar)"
            );
        }
    }
}

// Helper function to convert ROS time to Duration
fn ros_time_to_duration(time: &Time) -> Duration {
    Duration::from_nanos(time.sec as u64 * 1_000_000_000 + time.nanosec as u64)
}

#[tokio::main]
async fn main() -> Result<()> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("sensor_synchronizer")?;
    let _sensor_synchronizer = SensorSynchronizer::new(node)?;

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
