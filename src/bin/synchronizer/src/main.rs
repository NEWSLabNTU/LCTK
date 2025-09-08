use anyhow::{anyhow, Result};
use builtin_interfaces::msg::Time;
use futures::{stream, StreamExt};
use indexmap::IndexMap;
use multi_stream_synchronizer::{sync, Config, WithTimestamp};
use rclrs::*;
use std::{
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};
use tokio::sync::mpsc;
use vision_msgs::msg::{Detection2DArray, Detection3DArray};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

// Stream key types for synchronization
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum StreamKey {
    ArUco,
    Board,
}

// Wrapper for Detection2DArray to implement WithTimestamp
#[derive(Debug, Clone)]
pub struct ArUcoDetectionWrapper {
    pub detection: Detection2DArray,
}

impl WithTimestamp for ArUcoDetectionWrapper {
    fn timestamp(&self) -> Duration {
        ros_time_to_duration(&self.detection.header.stamp)
    }
}

// Wrapper for Detection3DArray to implement WithTimestamp
#[derive(Debug, Clone)]
pub struct BoardDetectionWrapper {
    pub detection: Detection3DArray,
}

impl WithTimestamp for BoardDetectionWrapper {
    fn timestamp(&self) -> Duration {
        ros_time_to_duration(&self.detection.header.stamp)
    }
}

// Combined detection type for the synchronizer
#[derive(Debug, Clone)]
pub enum DetectionMessage {
    ArUco(ArUcoDetectionWrapper),
    Board(BoardDetectionWrapper),
}

impl WithTimestamp for DetectionMessage {
    fn timestamp(&self) -> Duration {
        match self {
            DetectionMessage::ArUco(wrapper) => wrapper.timestamp(),
            DetectionMessage::Board(wrapper) => wrapper.timestamp(),
        }
    }
}

struct SynchronizerState {
    config: Config,
    quality_threshold: u8,
    enable_debug: bool,
    sequence_counter: AtomicU32,
    detection_sender: Mutex<Option<mpsc::UnboundedSender<(StreamKey, DetectionMessage)>>>,
}

pub struct SynchronizerNode {
    _state: Arc<SynchronizerState>,
    _node: Node,
}

impl SynchronizerNode {
    pub fn new(node: Node) -> Result<Self> {
        // Declare parameters with defaults
        let window_size_ms: i64 = node
            .declare_parameter("window_size_ms")
            .default(50i64)
            .mandatory()?
            .get();

        let buffer_size: i64 = node
            .declare_parameter("buffer_size")
            .default(100i64)
            .mandatory()?
            .get();

        let quality_threshold: i64 = node
            .declare_parameter("quality_threshold")
            .default(128i64)
            .mandatory()?
            .get();

        let enable_debug: bool = node
            .declare_parameter("enable_debug")
            .default(false)
            .mandatory()?
            .get();

        // Create synchronizer config
        let config = Config {
            window_size: Duration::from_millis(window_size_ms as u64),
            start_time: None,
            buf_size: buffer_size as usize,
        };

        // Create message channel for feeding the synchronizer
        let (detection_sender, detection_receiver) = mpsc::unbounded_channel();

        // Create state
        let state = Arc::new(SynchronizerState {
            config,
            quality_threshold: quality_threshold as u8,
            enable_debug,
            sequence_counter: AtomicU32::new(0),
            detection_sender: Mutex::new(Some(detection_sender)),
        });

        // Create publishers for synchronized detections
        let sync_2d_publisher = node.create_publisher("synchronized_aruco_detections")?;
        let sync_3d_publisher = node.create_publisher("synchronized_board_detections")?;

        // Start the synchronizer task
        {
            let sync_2d_publisher = sync_2d_publisher.clone();
            let sync_3d_publisher = sync_3d_publisher.clone();
            let state = Arc::clone(&state);

            tokio::spawn(async move {
                Self::run_synchronizer(
                    detection_receiver,
                    sync_2d_publisher,
                    sync_3d_publisher,
                    state,
                )
                .await;
            });
        }

        // Create subscribers
        let _aruco_subscription = {
            let state = Arc::clone(&state);

            node.create_subscription("aruco_detections", move |msg: Detection2DArray| {
                Self::aruco_callback(msg, &state);
            })?
        };

        let _board_subscription = {
            let state = Arc::clone(&state);

            node.create_subscription(
                "calibration_board_detections",
                move |msg: Detection3DArray| {
                    Self::board_callback(msg, &state);
                },
            )?
        };

        log_info!(
            LOGGER_NAME,
            "Synchronizer node initialized. Window size: {window_size_ms}ms, Buffer size: {buffer_size}, Quality threshold: {quality_threshold}"
        );

        Ok(Self {
            _state: state,
            _node: node,
        })
    }

    fn aruco_callback(msg: Detection2DArray, state: &Arc<SynchronizerState>) {
        if msg.detections.is_empty() {
            return; // Skip empty detections
        }

        let wrapper = ArUcoDetectionWrapper { detection: msg };
        let detection_msg = DetectionMessage::ArUco(wrapper);

        if let Some(sender) = state.detection_sender.lock().unwrap().as_ref() {
            if let Err(e) = sender.send((StreamKey::ArUco, detection_msg)) {
                log_warn!(
                    LOGGER_NAME,
                    "Failed to send ArUco detection to synchronizer: {e}"
                );
            }
        }
    }

    fn board_callback(msg: Detection3DArray, state: &Arc<SynchronizerState>) {
        if msg.detections.is_empty() {
            return; // Skip empty detections
        }

        let wrapper = BoardDetectionWrapper { detection: msg };
        let detection_msg = DetectionMessage::Board(wrapper);

        if let Some(sender) = state.detection_sender.lock().unwrap().as_ref() {
            if let Err(e) = sender.send((StreamKey::Board, detection_msg)) {
                log_warn!(
                    LOGGER_NAME,
                    "Failed to send board detection to synchronizer: {e}"
                );
            }
        }
    }

    async fn run_synchronizer(
        mut receiver: mpsc::UnboundedReceiver<(StreamKey, DetectionMessage)>,
        sync_2d_publisher: Publisher<Detection2DArray>,
        sync_3d_publisher: Publisher<Detection3DArray>,
        state: Arc<SynchronizerState>,
    ) {
        // Convert mpsc receiver to stream
        let input_stream =
            stream::poll_fn(move |cx| receiver.poll_recv(cx)).map(Ok::<_, eyre::Report>);

        // Run the synchronizer
        let (sync_stream, _feedback_stream) = match sync(
            input_stream,
            [StreamKey::ArUco, StreamKey::Board],
            state.config.clone(),
        ) {
            Ok((stream, feedback)) => (stream, feedback),
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to create synchronizer: {e}");
                return;
            }
        };

        // Process synchronized groups
        let mut sync_stream = sync_stream;
        while let Some(result) = sync_stream.next().await {
            match result {
                Ok(group) => {
                    if let Err(e) = Self::process_synchronized_group(
                        group,
                        &sync_2d_publisher,
                        &sync_3d_publisher,
                        &state,
                    )
                    .await
                    {
                        log_warn!(LOGGER_NAME, "Failed to process synchronized group: {e}");
                    }
                }
                Err(e) => {
                    log_warn!(LOGGER_NAME, "Synchronizer error: {e}");
                }
            }
        }
    }

    async fn process_synchronized_group(
        group: IndexMap<StreamKey, DetectionMessage>,
        sync_2d_publisher: &Publisher<Detection2DArray>,
        sync_3d_publisher: &Publisher<Detection3DArray>,
        state: &SynchronizerState,
    ) -> Result<()> {
        // Extract ArUco and board detections from the group
        let aruco_detection = group.get(&StreamKey::ArUco);
        let board_detection = group.get(&StreamKey::Board);

        let (aruco_msg, board_msg) = match (aruco_detection, board_detection) {
            (Some(DetectionMessage::ArUco(aruco)), Some(DetectionMessage::Board(board))) => {
                (&aruco.detection, &board.detection)
            }
            _ => {
                log_warn!(LOGGER_NAME, "Incomplete detection group received");
                return Ok(());
            }
        };

        // Calculate synchronization metrics
        let aruco_time = ros_time_to_duration(&aruco_msg.header.stamp);
        let board_time = ros_time_to_duration(&board_msg.header.stamp);
        let time_diff_ns = if aruco_time > board_time {
            aruco_time.saturating_sub(board_time).as_nanos() as f64
        } else {
            board_time.saturating_sub(aruco_time).as_nanos() as f64
        };

        let sync_quality = calculate_sync_quality(time_diff_ns as u64);

        // Check quality threshold
        if sync_quality < state.quality_threshold {
            if state.enable_debug {
                log_warn!(
                    LOGGER_NAME,
                    "Sync quality {sync_quality} below threshold {}, skipping",
                    state.quality_threshold
                );
            }
            return Ok(());
        }

        // Generate correlation ID
        let correlation_id = state.sequence_counter.fetch_add(1, Ordering::Relaxed) + 1;
        let correlation_frame = format!("sync_pair_{correlation_id}");

        // Create synchronized messages with correlation info
        let mut sync_aruco = aruco_msg.clone();
        let mut sync_board = board_msg.clone();

        // Set correlation frame_id and synchronized timestamp
        let sync_time = average_ros_time(&aruco_msg.header.stamp, &board_msg.header.stamp);
        sync_aruco.header.frame_id = correlation_frame.clone();
        sync_aruco.header.stamp = sync_time.clone();
        sync_board.header.frame_id = correlation_frame.clone();
        sync_board.header.stamp = sync_time;

        // Publish synchronized detection pair
        if let Err(e) = sync_2d_publisher.publish(sync_aruco) {
            log_warn!(
                LOGGER_NAME,
                "Failed to publish synchronized ArUco detections: {e}"
            );
            return Ok(());
        }

        if let Err(e) = sync_3d_publisher.publish(sync_board) {
            log_warn!(
                LOGGER_NAME,
                "Failed to publish synchronized board detections: {e}"
            );
            return Ok(());
        }

        if state.enable_debug {
            log_info!(
                LOGGER_NAME,
                "Published synchronized detection pair: {correlation_frame}, quality: {sync_quality}, time diff: {time_diff_ns:.0}ns"
            );
        } else {
            log_info!(
                LOGGER_NAME,
                "Published synchronized detection pair: {correlation_frame}"
            );
        }

        Ok(())
    }
}

// Utility functions

fn ros_time_to_duration(stamp: &Time) -> Duration {
    let sec = stamp.sec as u64;
    let nanosec = stamp.nanosec as u64;
    Duration::from_nanos(sec * 1_000_000_000 + nanosec)
}

fn average_ros_time(time1: &Time, time2: &Time) -> Time {
    let dur1 = ros_time_to_duration(time1);
    let dur2 = ros_time_to_duration(time2);
    let avg_dur = Duration::from_nanos((dur1.as_nanos() + dur2.as_nanos()) as u64 / 2);

    Time {
        sec: (avg_dur.as_secs()) as i32,
        nanosec: (avg_dur.subsec_nanos()) as u32,
    }
}

fn calculate_sync_quality(time_diff_ns: u64) -> u8 {
    // Quality decreases with time difference
    // Perfect sync (0ns) = 255, 100ms diff = 0
    let max_diff_for_quality = 100_000_000u64; // 100ms
    if time_diff_ns >= max_diff_for_quality {
        0
    } else {
        (255 * (max_diff_for_quality - time_diff_ns) / max_diff_for_quality) as u8
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("synchronizer")?;
    let _synchronizer_node = SynchronizerNode::new(node)?;

    log_info!(LOGGER_NAME, "Synchronizer node started");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
