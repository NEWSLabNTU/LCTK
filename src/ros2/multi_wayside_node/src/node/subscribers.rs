use eyre::Result;
use geometry_msgs::msg::PoseStamped;
use rclrs::{Node, Subscription};
use sensor_msgs::msg::PointCloud2;
use std::sync::Arc;

/// Trait for managing ROS 2 subscribers
pub trait SubscriberManager: Send + Sync {
    // This trait would define the interface for subscriber management
    // In practice, subscribers are created with callbacks, so this is more
    // of a factory pattern for creating subscribers with appropriate callbacks
}

/// Factory for creating ROS 2 subscribers with callbacks
pub struct SubscriberFactory;

impl SubscriberFactory {
    /// Create point cloud subscriber with callback
    pub fn create_pointcloud_subscriber<F>(
        node: &Node,
        topic: &str,
        callback: F,
    ) -> Result<Arc<Subscription<PointCloud2>>>
    where
        F: Fn(PointCloud2) + Send + Sync + 'static,
    {
        let subscription = Arc::new(node.create_subscription(topic, callback)?);
        Ok(subscription)
    }

    /// Create pose adjustment subscriber with callback
    pub fn create_pose_subscriber<F>(
        node: &Node,
        topic: &str,
        callback: F,
    ) -> Result<Arc<Subscription<PoseStamped>>>
    where
        F: Fn(PoseStamped) + Send + Sync + 'static,
    {
        let subscription = Arc::new(node.create_subscription(topic, callback)?);
        Ok(subscription)
    }
}

/// Container for all subscriptions to keep them alive
pub struct SubscriptionContainer {
    _lidar1_sub: Arc<Subscription<PointCloud2>>,
    _lidar2_sub: Arc<Subscription<PointCloud2>>,
    _lidar1_pose_sub: Arc<Subscription<PoseStamped>>,
    _lidar2_pose_sub: Arc<Subscription<PoseStamped>>,
}

impl SubscriptionContainer {
    pub fn new(
        lidar1_sub: Arc<Subscription<PointCloud2>>,
        lidar2_sub: Arc<Subscription<PointCloud2>>,
        lidar1_pose_sub: Arc<Subscription<PoseStamped>>,
        lidar2_pose_sub: Arc<Subscription<PoseStamped>>,
    ) -> Self {
        Self {
            _lidar1_sub: lidar1_sub,
            _lidar2_sub: lidar2_sub,
            _lidar1_pose_sub: lidar1_pose_sub,
            _lidar2_pose_sub: lidar2_pose_sub,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rclrs::{Context, CreateBasicExecutor, InitOptions};

    #[test]
    fn test_subscriber_factory() {
        let context = Context::new(std::env::args(), InitOptions::default()).unwrap();
        let executor = context.create_basic_executor();
        let node = executor.create_node("test_node").unwrap();

        let callback = |_msg: PointCloud2| {
            // Test callback
        };

        let result =
            SubscriberFactory::create_pointcloud_subscriber(&node, "/test_topic", callback);
        assert!(result.is_ok());
    }
}
