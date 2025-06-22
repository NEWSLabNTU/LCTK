use builtin_interfaces::msg::Time;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Convert SystemTime to ROS Time
pub fn system_time_to_ros_time(time: SystemTime) -> Time {
    let duration = time.duration_since(UNIX_EPOCH).unwrap_or_default();
    Time {
        sec: duration.as_secs() as i32,
        nanosec: duration.subsec_nanos(),
    }
}

/// Convert ROS Time to SystemTime
pub fn ros_time_to_system_time(time: &Time) -> SystemTime {
    let duration = Duration::from_secs(time.sec as u64) + Duration::from_nanos(time.nanosec as u64);
    UNIX_EPOCH + duration
}

/// Get current time as ROS Time
pub fn now_ros_time() -> Time {
    system_time_to_ros_time(SystemTime::now())
}

/// Calculate duration between two ROS timestamps
pub fn ros_time_diff(time1: &Time, time2: &Time) -> Duration {
    let duration1 =
        Duration::from_secs(time1.sec as u64) + Duration::from_nanos(time1.nanosec as u64);
    let duration2 =
        Duration::from_secs(time2.sec as u64) + Duration::from_nanos(time2.nanosec as u64);

    if duration1 > duration2 {
        duration1 - duration2
    } else {
        duration2 - duration1
    }
}

/// Check if two ROS timestamps are within tolerance
pub fn ros_times_within_tolerance(time1: &Time, time2: &Time, tolerance: Duration) -> bool {
    ros_time_diff(time1, time2) <= tolerance
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_system_time_conversion() {
        let now = SystemTime::now();
        let ros_time = system_time_to_ros_time(now);
        let back = ros_time_to_system_time(&ros_time);

        // Should be very close (within 1ms due to precision)
        let diff = if now > back {
            now.duration_since(back)
        } else {
            back.duration_since(now)
        };
        assert!(diff.unwrap() < Duration::from_millis(1));
    }

    #[test]
    fn test_ros_time_diff() {
        let time1 = Time {
            sec: 1000,
            nanosec: 0,
        };
        let time2 = Time {
            sec: 1000,
            nanosec: 500_000_000,
        }; // 0.5 seconds later

        let diff = ros_time_diff(&time1, &time2);
        assert_eq!(diff, Duration::from_millis(500));

        // Should be symmetric
        let diff_reverse = ros_time_diff(&time2, &time1);
        assert_eq!(diff, diff_reverse);
    }

    #[test]
    fn test_ros_times_within_tolerance() {
        let time1 = Time {
            sec: 1000,
            nanosec: 0,
        };
        let time2 = Time {
            sec: 1000,
            nanosec: 100_000_000,
        }; // 0.1 seconds later

        // Within 200ms tolerance
        assert!(ros_times_within_tolerance(
            &time1,
            &time2,
            Duration::from_millis(200)
        ));

        // Not within 50ms tolerance
        assert!(!ros_times_within_tolerance(
            &time1,
            &time2,
            Duration::from_millis(50)
        ));
    }

    #[test]
    fn test_now_ros_time() {
        let ros_now = now_ros_time();
        let sys_now = SystemTime::now();

        // Should be very close to current time
        let sys_from_ros = ros_time_to_system_time(&ros_now);
        let diff = if sys_now > sys_from_ros {
            sys_now.duration_since(sys_from_ros)
        } else {
            sys_from_ros.duration_since(sys_now)
        };
        assert!(diff.unwrap() < Duration::from_millis(10));
    }
}
