use crate::types::{BoardDetection, TimestampedDetection};
use builtin_interfaces::msg::Time;
use std::{
    collections::VecDeque,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

/// Trait for synchronizing detections between LiDARs
pub trait DetectionSynchronizer: Send + Sync {
    fn add_detection(&mut self, detection: BoardDetection, timestamp: Time, lidar_id: u8);
    fn find_synchronized_pair(&mut self) -> Option<(TimestampedDetection, TimestampedDetection)>;
    fn get_queue_sizes(&self) -> (usize, usize);
    fn clear_old_detections(&mut self, max_age: Duration);
}

/// Default implementation of DetectionSynchronizer
pub struct DefaultDetectionSynchronizer {
    detections_lidar1: VecDeque<TimestampedDetection>,
    detections_lidar2: VecDeque<TimestampedDetection>,
    max_queue_size: usize,
    sync_tolerance: Duration,
}

impl DefaultDetectionSynchronizer {
    pub fn new(max_queue_size: usize, sync_tolerance_ms: u64) -> Self {
        Self {
            detections_lidar1: VecDeque::new(),
            detections_lidar2: VecDeque::new(),
            max_queue_size,
            sync_tolerance: Duration::from_millis(sync_tolerance_ms),
        }
    }
}

impl DetectionSynchronizer for DefaultDetectionSynchronizer {
    fn add_detection(&mut self, detection: BoardDetection, timestamp: Time, lidar_id: u8) {
        let timestamped = TimestampedDetection {
            detection,
            timestamp,
            lidar_id,
        };

        match lidar_id {
            1 => {
                self.detections_lidar1.push_back(timestamped);
                if self.detections_lidar1.len() > self.max_queue_size {
                    self.detections_lidar1.pop_front();
                }
            }
            2 => {
                self.detections_lidar2.push_back(timestamped);
                if self.detections_lidar2.len() > self.max_queue_size {
                    self.detections_lidar2.pop_front();
                }
            }
            _ => {
                // Invalid lidar_id, ignore
            }
        }
    }

    fn find_synchronized_pair(&mut self) -> Option<(TimestampedDetection, TimestampedDetection)> {
        crate::types::find_synchronized_pair(
            &mut self.detections_lidar1,
            &mut self.detections_lidar2,
            self.sync_tolerance,
        )
    }

    fn get_queue_sizes(&self) -> (usize, usize) {
        (self.detections_lidar1.len(), self.detections_lidar2.len())
    }

    fn clear_old_detections(&mut self, max_age: Duration) {
        let now = SystemTime::now();
        let cutoff_time = now - max_age;

        // Convert cutoff time to ROS time
        let cutoff_duration = cutoff_time.duration_since(UNIX_EPOCH).unwrap_or_default();
        let cutoff_ros_time = Time {
            sec: cutoff_duration.as_secs() as i32,
            nanosec: cutoff_duration.subsec_nanos(),
        };

        // Remove old detections from lidar1
        while let Some(front) = self.detections_lidar1.front() {
            if ros_time_to_duration(&front.timestamp) < ros_time_to_duration(&cutoff_ros_time) {
                self.detections_lidar1.pop_front();
            } else {
                break;
            }
        }

        // Remove old detections from lidar2
        while let Some(front) = self.detections_lidar2.front() {
            if ros_time_to_duration(&front.timestamp) < ros_time_to_duration(&cutoff_ros_time) {
                self.detections_lidar2.pop_front();
            } else {
                break;
            }
        }
    }
}

/// Convert ROS Time to Duration for comparison
fn ros_time_to_duration(time: &Time) -> Duration {
    Duration::from_secs(time.sec as u64) + Duration::from_nanos(time.nanosec as u64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use std::time::UNIX_EPOCH;

    fn create_test_detection() -> BoardDetection {
        BoardDetection {
            pose: Isometry3::identity(),
            confidence: 0.8,
            inlier_count: 100,
            timestamp: SystemTime::now(),
        }
    }

    fn create_test_time(sec: i32, nanosec: u32) -> Time {
        Time { sec, nanosec }
    }

    #[test]
    fn test_add_detection() {
        let mut sync = DefaultDetectionSynchronizer::new(10, 100);

        let detection = create_test_detection();
        let timestamp = create_test_time(1000, 0);

        sync.add_detection(detection, timestamp, 1);

        let (q1_size, q2_size) = sync.get_queue_sizes();
        assert_eq!(q1_size, 1);
        assert_eq!(q2_size, 0);
    }

    #[test]
    fn test_queue_size_limit() {
        let mut sync = DefaultDetectionSynchronizer::new(2, 100);

        // Add 3 detections to exceed limit
        for i in 0..3 {
            let detection = create_test_detection();
            let timestamp = create_test_time(1000 + i, 0);
            sync.add_detection(detection, timestamp, 1);
        }

        let (q1_size, _) = sync.get_queue_sizes();
        assert_eq!(q1_size, 2); // Should be limited to max_queue_size
    }

    #[test]
    fn test_synchronized_pair_found() {
        let mut sync = DefaultDetectionSynchronizer::new(10, 100); // 100ms tolerance

        // Add detections within sync tolerance
        let detection1 = create_test_detection();
        let timestamp1 = create_test_time(1000, 0);
        sync.add_detection(detection1, timestamp1, 1);

        let detection2 = create_test_detection();
        let timestamp2 = create_test_time(1000, 50_000_000); // 50ms later
        sync.add_detection(detection2, timestamp2, 2);

        let pair = sync.find_synchronized_pair();
        assert!(pair.is_some());

        let (det1, det2) = pair.unwrap();
        assert_eq!(det1.lidar_id, 1);
        assert_eq!(det2.lidar_id, 2);
    }

    #[test]
    fn test_synchronized_pair_not_found() {
        let mut sync = DefaultDetectionSynchronizer::new(10, 50); // 50ms tolerance

        // Add detections outside sync tolerance
        let detection1 = create_test_detection();
        let timestamp1 = create_test_time(1000, 0);
        sync.add_detection(detection1, timestamp1, 1);

        let detection2 = create_test_detection();
        let timestamp2 = create_test_time(1000, 100_000_000); // 100ms later
        sync.add_detection(detection2, timestamp2, 2);

        let pair = sync.find_synchronized_pair();
        assert!(pair.is_none());
    }

    #[test]
    fn test_clear_old_detections() {
        let mut sync = DefaultDetectionSynchronizer::new(10, 100);

        // Get current time and create timestamps relative to it
        let now = SystemTime::now();
        let now_duration = now.duration_since(UNIX_EPOCH).unwrap();

        // Old detection: 600 seconds ago
        let old_duration = now_duration - Duration::from_secs(600);
        let old_timestamp = Time {
            sec: old_duration.as_secs() as i32,
            nanosec: old_duration.subsec_nanos(),
        };
        let old_detection = create_test_detection();
        sync.add_detection(old_detection, old_timestamp, 1);

        // Recent detection: 300 seconds ago
        let recent_duration = now_duration - Duration::from_secs(300);
        let recent_timestamp = Time {
            sec: recent_duration.as_secs() as i32,
            nanosec: recent_duration.subsec_nanos(),
        };
        let recent_detection = create_test_detection();
        sync.add_detection(recent_detection, recent_timestamp, 1);

        // Clear detections older than 500 seconds
        sync.clear_old_detections(Duration::from_secs(500));

        let (q1_size, _) = sync.get_queue_sizes();
        assert_eq!(q1_size, 1); // Only recent detection should remain
    }
}
