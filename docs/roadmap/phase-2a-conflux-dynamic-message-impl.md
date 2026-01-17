# Phase 2a: Conflux DynamicMessage Implementation Plan

## Overview

Restructure conflux to handle `DynamicMessage` ownership properly, enabling the synchronizer to receive, buffer, and republish ROS2 messages without losing content.

## Problem Statement

Current conflux implementation loses message content:
```rust
// subscriber.rs:171-177 - CURRENT (BROKEN)
let timestamped = TimestampedMessage::new(
    topic_owned.clone(),
    timestamp,
    Vec::new(),  // <-- Message content is LOST
    (sec, nanosec),
);
```

Root cause: `DynamicMessage` doesn't implement `Clone` and has no public byte access.

## Solution Architecture

### Key Insight

conflux-core requires `T: WithTimestamp + Clone` for its generic buffer. Since `DynamicMessage` isn't `Clone`, we need a **specialized ROS2 synchronization layer** that:

1. Owns `DynamicMessage` instances directly
2. Uses move semantics instead of cloning
3. Publishes via `DynamicPublisher` when synchronized

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          conflux-ros2                                    │
│                                                                          │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐ │
│  │ DynamicSubscriber │────►│  ROS2SyncState   │────►│ DynamicPublisher │ │
│  │ (per input topic) │     │ (owns messages)  │     │ (per output topic)│ │
│  └──────────────────┘     └──────────────────┘     └──────────────────┘ │
│           │                        │                        │            │
│           │ DynamicMessage         │ Timestamp-based        │ DynamicMsg │
│           │ (move ownership)       │ synchronization        │ (publish)  │
│           ▼                        ▼                        ▼            │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                          Uses algorithms from
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          conflux-core                                    │
│                                                                          │
│  - Time-window matching logic (reuse)                                   │
│  - Staleness detection (reuse)                                          │
│  - Generic types NOT used for DynamicMessage path                       │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## Implementation Tasks

### Task 1: Create ROS2-Specific Message Wrapper

**File**: `crates/conflux-ros2/src/ros2_message.rs` (NEW)

```rust
use rclrs::DynamicMessage;
use std::time::Duration;

/// A ROS2 message with extracted timestamp, owning the DynamicMessage.
///
/// Unlike TimestampedMessage which stores bytes, this owns the actual
/// DynamicMessage for later republishing.
pub struct Ros2Message {
    /// The topic this message came from.
    pub topic: String,

    /// Timestamp extracted from header.stamp.
    pub timestamp: Duration,

    /// The actual DynamicMessage (owned).
    pub message: DynamicMessage,

    /// Original ROS stamp for reconstruction.
    pub ros_stamp: (i32, u32),
}

impl Ros2Message {
    pub fn new(
        topic: String,
        timestamp: Duration,
        message: DynamicMessage,
        ros_stamp: (i32, u32),
    ) -> Self {
        Self { topic, timestamp, message, ros_stamp }
    }
}
```

### Task 2: Create ROS2-Specific Synchronization State

**File**: `crates/conflux-ros2/src/ros2_sync_state.rs` (NEW)

This is a specialized version of conflux-core's `State` that:
- Uses `Option<Ros2Message>` instead of requiring `Clone`
- Implements the same time-window matching algorithm
- Uses take semantics for message extraction

```rust
use indexmap::IndexMap;
use std::collections::VecDeque;
use std::time::Duration;

use crate::ros2_message::Ros2Message;

/// Per-topic message buffer using Option for move semantics.
pub struct Ros2Buffer {
    messages: VecDeque<Option<Ros2Message>>,
    capacity: usize,
}

impl Ros2Buffer {
    pub fn new(capacity: usize) -> Self {
        Self {
            messages: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    /// Push a message, returning oldest if at capacity.
    pub fn push(&mut self, msg: Ros2Message) -> Option<Ros2Message> {
        let dropped = if self.messages.len() >= self.capacity {
            self.messages.pop_front().flatten()
        } else {
            None
        };
        self.messages.push_back(Some(msg));
        dropped
    }

    /// Peek at front message timestamp without taking ownership.
    pub fn front_timestamp(&self) -> Option<Duration> {
        self.messages.front()?.as_ref().map(|m| m.timestamp)
    }

    /// Take the front message (moves ownership out).
    pub fn take_front(&mut self) -> Option<Ros2Message> {
        self.messages.pop_front().flatten()
    }

    /// Number of messages currently buffered.
    pub fn len(&self) -> usize {
        self.messages.iter().filter(|m| m.is_some()).count()
    }

    /// Drop messages with timestamp before the given threshold.
    pub fn drop_before(&mut self, threshold: Duration) {
        while let Some(Some(front)) = self.messages.front() {
            if front.timestamp < threshold {
                self.messages.pop_front();
            } else {
                break;
            }
        }
    }
}

/// Synchronization state for ROS2 DynamicMessages.
pub struct Ros2SyncState {
    /// Per-topic buffers.
    buffers: IndexMap<String, Ros2Buffer>,

    /// Time window for grouping.
    window_size: Duration,

    /// Commit timestamp (messages before this are rejected).
    commit_ts: Duration,
}

impl Ros2SyncState {
    pub fn new(topics: Vec<String>, window_size: Duration, buffer_size: usize) -> Self {
        let buffers = topics
            .into_iter()
            .map(|t| (t, Ros2Buffer::new(buffer_size)))
            .collect();

        Self {
            buffers,
            window_size,
            commit_ts: Duration::ZERO,
        }
    }

    /// Push a message to the appropriate buffer.
    /// Returns Err if topic unknown or message is late.
    pub fn push(&mut self, msg: Ros2Message) -> Result<(), Ros2Message> {
        // Reject late messages
        if msg.timestamp < self.commit_ts {
            return Err(msg);
        }

        let Some(buffer) = self.buffers.get_mut(&msg.topic) else {
            return Err(msg);
        };

        buffer.push(msg);
        Ok(())
    }

    /// Check if all buffers have at least one message.
    pub fn is_ready(&self) -> bool {
        self.buffers.values().all(|b| b.len() >= 1)
    }

    /// Attempt to match and extract a synchronized group.
    /// Returns None if matching not possible.
    pub fn try_match(&mut self) -> Option<IndexMap<String, Ros2Message>> {
        if !self.is_ready() {
            return None;
        }

        // Find inf_ts = max of front timestamps
        let inf_ts = self.buffers
            .values()
            .filter_map(|b| b.front_timestamp())
            .max()?;

        // Check if all fronts are within window of inf_ts
        let all_within_window = self.buffers.values().all(|b| {
            b.front_timestamp()
                .map(|ts| inf_ts.saturating_sub(ts) <= self.window_size)
                .unwrap_or(false)
        });

        if !all_within_window {
            // Drop oldest message from the buffer with smallest timestamp
            let min_topic = self.buffers
                .iter()
                .filter_map(|(t, b)| b.front_timestamp().map(|ts| (t.clone(), ts)))
                .min_by_key(|(_, ts)| *ts)?
                .0;

            if let Some(buffer) = self.buffers.get_mut(&min_topic) {
                buffer.take_front(); // Drop
            }
            return None;
        }

        // Extract synchronized group
        let mut group = IndexMap::new();
        let mut min_ts = Duration::MAX;

        for (topic, buffer) in self.buffers.iter_mut() {
            if let Some(msg) = buffer.take_front() {
                min_ts = min_ts.min(msg.timestamp);
                group.insert(topic.clone(), msg);
            }
        }

        // Update commit timestamp
        self.commit_ts = min_ts;

        Some(group)
    }
}
```

### Task 3: Create Dynamic Publisher Manager

**File**: `crates/conflux-ros2/src/ros2_publisher.rs` (NEW)

```rust
use eyre::{Result, WrapErr};
use indexmap::IndexMap;
use rclrs::{DynamicPublisher, MessageTypeName, Node, PublisherOptions, QoSProfile};

/// Manages dynamic publishers for synchronized output topics.
pub struct Ros2PublisherManager {
    publishers: IndexMap<String, (DynamicPublisher, String)>, // (publisher, msg_type)
}

impl Ros2PublisherManager {
    /// Create publishers for each input topic with the given suffix.
    pub fn new(
        node: &Node,
        topics_and_types: &[(String, String)], // (input_topic, msg_type)
        output_suffix: &str,
        qos: QoSProfile,
    ) -> Result<Self> {
        let mut publishers = IndexMap::new();

        for (input_topic, msg_type) in topics_and_types {
            let output_topic = format!("{}{}", input_topic, output_suffix);
            let message_type: MessageTypeName = msg_type
                .as_str()
                .try_into()
                .wrap_err_with(|| format!("Invalid message type: {}", msg_type))?;

            let options = PublisherOptions::new(&output_topic).qos(qos);
            let publisher = node
                .create_dynamic_publisher(message_type, options)
                .wrap_err_with(|| format!("Failed to create publisher for {}", output_topic))?;

            publishers.insert(input_topic.clone(), (publisher, msg_type.clone()));
        }

        Ok(Self { publishers })
    }

    /// Publish a synchronized message to its corresponding output topic.
    pub fn publish(&self, input_topic: &str, message: rclrs::DynamicMessage) -> Result<()> {
        let (publisher, _) = self.publishers
            .get(input_topic)
            .ok_or_else(|| eyre::eyre!("No publisher for topic: {}", input_topic))?;

        publisher.publish(message)
            .wrap_err_with(|| format!("Failed to publish to {}_sync", input_topic))?;

        Ok(())
    }
}
```

### Task 4: Update Subscriber to Pass DynamicMessage

**File**: `crates/conflux-ros2/src/subscriber.rs` (MODIFY)

```rust
// Change the callback to send Ros2Message instead of TimestampedMessage

pub fn create_dynamic_subscription_v2(
    node: &Node,
    topic: &str,
    msg_type: &str,
    qos: QoSProfile,
    tx: mpsc::UnboundedSender<Ros2Message>,
) -> Result<DynamicSubscriptionHandle> {
    let normalized_type = normalize_msg_type(msg_type);
    let message_type: MessageTypeName = normalized_type
        .as_str()
        .try_into()
        .wrap_err_with(|| format!("Invalid message type: {}", msg_type))?;

    let topic_owned = topic.to_string();

    let mut options = SubscriptionOptions::new(topic);
    options.qos = qos;

    let subscription = node
        .create_dynamic_subscription(
            message_type,
            options,
            move |msg: DynamicMessage, _info: MessageInfo| {
                let (sec, nanosec) = extract_header_stamp(&msg).unwrap_or((0, 0));
                let timestamp = ros_time_to_duration(sec, nanosec);

                let ros2_msg = Ros2Message::new(
                    topic_owned.clone(),
                    timestamp,
                    msg,  // Move ownership of DynamicMessage
                    (sec, nanosec),
                );

                if let Err(e) = tx.send(ros2_msg) {
                    error!("Failed to send message: {}", e);
                }
            },
        )
        .wrap_err("Failed to create subscription")?;

    Ok(DynamicSubscriptionHandle {
        _subscription: subscription,
        msg_type: normalized_type,
        topic: topic.to_string(),
    })
}
```

### Task 5: Create Synchronizer Node Runner

**File**: `crates/conflux-ros2/src/ros2_sync_node.rs` (NEW)

```rust
use eyre::Result;
use indexmap::IndexMap;
use rclrs::{Node, QoSProfile};
use tokio::sync::mpsc;
use tracing::{info, warn};

use crate::{
    ros2_message::Ros2Message,
    ros2_publisher::Ros2PublisherManager,
    ros2_sync_state::Ros2SyncState,
    create_dynamic_subscription_v2,
    DynamicSubscriptionHandle,
};

pub struct Ros2SyncConfig {
    pub inputs: Vec<(String, String)>, // (topic, msg_type)
    pub output_suffix: String,
    pub window_size: std::time::Duration,
    pub buffer_size: usize,
    pub qos: QoSProfile,
}

pub struct Ros2SyncNode {
    _subscriptions: Vec<DynamicSubscriptionHandle>,
    publishers: Ros2PublisherManager,
    state: Ros2SyncState,
    rx: mpsc::UnboundedReceiver<Ros2Message>,
}

impl Ros2SyncNode {
    pub fn new(node: &Node, config: Ros2SyncConfig) -> Result<Self> {
        let (tx, rx) = mpsc::unbounded_channel();

        // Create subscriptions
        let mut subscriptions = Vec::new();
        for (topic, msg_type) in &config.inputs {
            let sub = create_dynamic_subscription_v2(
                node,
                topic,
                msg_type,
                config.qos,
                tx.clone(),
            )?;
            subscriptions.push(sub);
        }

        // Create publishers
        let publishers = Ros2PublisherManager::new(
            node,
            &config.inputs,
            &config.output_suffix,
            config.qos,
        )?;

        // Create sync state
        let topics: Vec<String> = config.inputs.iter().map(|(t, _)| t.clone()).collect();
        let state = Ros2SyncState::new(topics, config.window_size, config.buffer_size);

        Ok(Self {
            _subscriptions: subscriptions,
            publishers,
            state,
            rx,
        })
    }

    pub async fn run(mut self) -> Result<()> {
        info!("Starting ROS2 synchronization node");

        while let Some(msg) = self.rx.recv().await {
            let topic = msg.topic.clone();

            // Push to state
            if let Err(rejected) = self.state.push(msg) {
                warn!(
                    topic = %rejected.topic,
                    timestamp = ?rejected.timestamp,
                    "Rejected late message"
                );
                continue;
            }

            // Try to match and publish
            while let Some(group) = self.state.try_match() {
                info!(
                    num_messages = group.len(),
                    "Publishing synchronized group"
                );

                for (input_topic, ros2_msg) in group {
                    if let Err(e) = self.publishers.publish(&input_topic, ros2_msg.message) {
                        warn!(
                            topic = %input_topic,
                            error = %e,
                            "Failed to publish synchronized message"
                        );
                    }
                }
            }
        }

        Ok(())
    }
}
```

### Task 6: Update conflux_node to Use New API

**File**: `conflux_node/src/node.rs` (MODIFY)

Replace the existing implementation with the new `Ros2SyncNode`.

### Task 7: Update Config for Output Suffix

**File**: `conflux_node/src/config.rs` (MODIFY)

```rust
/// Configuration for the output.
#[derive(Debug, Clone, Deserialize)]
pub struct OutputConfig {
    /// Suffix to append to input topics for output (e.g., "_sync").
    #[serde(default = "default_output_suffix")]
    pub suffix: String,
}

fn default_output_suffix() -> String {
    "_sync".to_string()
}
```

## File Summary

| File | Action | Description |
|------|--------|-------------|
| `ros2_message.rs` | NEW | Message wrapper owning DynamicMessage |
| `ros2_sync_state.rs` | NEW | Specialized sync state with move semantics |
| `ros2_publisher.rs` | NEW | Dynamic publisher manager |
| `subscriber.rs` | MODIFY | Add `create_dynamic_subscription_v2` |
| `ros2_sync_node.rs` | NEW | Complete sync node runner |
| `lib.rs` | MODIFY | Export new modules |
| `node.rs` | MODIFY | Use new Ros2SyncNode |
| `config.rs` | MODIFY | Add output suffix config |

## Testing Plan

### Unit Tests
1. `Ros2Buffer` push/take/drop_before operations
2. `Ros2SyncState` matching logic with various timestamp patterns
3. Config parsing with new output suffix

### Integration Tests
1. Two-topic synchronization with test_msgs
2. Verify output topics have correct message content
3. Verify timestamps preserved correctly

### End-to-End Test with LCTK
1. Run `aruco_locator_node` + `lidar_board_detector`
2. Run `conflux_node` with calibration config
3. Verify `*_sync` topics publish correctly
4. Verify `advanced_extrinsic_solver` receives synchronized data

## Migration Path

1. Implement new modules alongside existing code
2. Add feature flag `ros2-sync-v2` to switch implementations
3. Test thoroughly before removing old code
4. Update documentation and examples

## Dependencies

No new external dependencies required. Uses existing:
- `rclrs` (git commit 562e815)
- `tokio` for async
- `indexmap` for ordered maps
- `eyre` for error handling
