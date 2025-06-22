#!/usr/bin/env python3

"""
ROI Interactive Node for Multi-Wayside Calibration System

This Python node provides interactive ROI (Region of Interest) manipulation
capabilities for the multi_wayside_node using RViz2 interactive markers.

Features:
- Interactive 3D ROI box manipulation in RViz2
- Real-time ROI updates via service calls to Rust node
- Visual feedback during manipulation
- Support for multiple LiDAR ROI boxes

Author: LCTK Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray
)
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from std_msgs.msg import Header, ColorRGBA

from rosbag_deck_interface.srv import SetROIBounds

import threading
import time
from typing import Dict, Tuple


class ROIInteractiveNode(Node):
    """Interactive ROI manipulation node using RViz2 interactive markers."""

    def __init__(self):
        super().__init__('roi_interactive_node')

        # Declare parameters
        self.declare_parameter('roi_box_size_x', 4.0)
        self.declare_parameter('roi_box_size_y', 4.0)
        self.declare_parameter('roi_box_size_z', 2.0)
        self.declare_parameter('roi_box_position_x', 2.0)
        self.declare_parameter('roi_box_position_y', 0.0)
        self.declare_parameter('roi_box_position_z', 0.0)
        self.declare_parameter('roi_marker_scale', 0.2)
        self.declare_parameter('roi_update_rate', 10.0)
        self.declare_parameter('frame_id', 'map')

        # Get parameters
        self.default_size_x = self.get_parameter('roi_box_size_x').value
        self.default_size_y = self.get_parameter('roi_box_size_y').value
        self.default_size_z = self.get_parameter('roi_box_size_z').value
        self.default_position_x = self.get_parameter('roi_box_position_x').value
        self.default_position_y = self.get_parameter('roi_box_position_y').value
        self.default_position_z = self.get_parameter('roi_box_position_z').value
        self.marker_scale = self.get_parameter('roi_marker_scale').value
        self.update_rate = self.get_parameter('roi_update_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Create callback group for multithreading
        self.callback_group = ReentrantCallbackGroup()

        # Initialize interactive marker server
        self.marker_server = InteractiveMarkerServer(
            self,
            'roi_interactive_markers',
            callback_group=self.callback_group
        )

        # Create service client for ROI updates
        self.roi_service_client = self.create_client(
            SetROIBounds,
            '/set_roi_bounds',
            callback_group=self.callback_group
        )

        # ROI state tracking
        self.roi_states: Dict[int, Dict] = {}
        self.roi_lock = threading.Lock()

        # Initialize ROI markers for both LiDARs
        self.initialize_roi_markers()

        # Apply all marker changes
        self.marker_server.applyChanges()

        self.get_logger().info('ROI Interactive Node initialized successfully')
        self.get_logger().info(f'Use RViz2 to manipulate ROI boxes for LiDAR 1 and LiDAR 2')
        self.get_logger().info(f'ROI updates will be sent to multi_wayside_node via /set_roi_bounds service')

    def initialize_roi_markers(self):
        """Initialize interactive markers for both LiDAR ROI boxes."""
        # LiDAR 1 - Red ROI box
        self.create_roi_marker(
            lidar_id=1,
            color=(1.0, 0.0, 0.0, 0.3),  # Red with transparency
            initial_position=(self.default_position_x, self.default_position_y, self.default_position_z),
            initial_size=(self.default_size_x, self.default_size_y, self.default_size_z)
        )

        # LiDAR 2 - Blue ROI box
        self.create_roi_marker(
            lidar_id=2,
            color=(0.0, 0.0, 1.0, 0.3),  # Blue with transparency
            initial_position=(self.default_position_x, self.default_position_y, self.default_position_z),
            initial_size=(self.default_size_x, self.default_size_y, self.default_size_z)
        )

    def create_roi_marker(self, lidar_id: int, color: Tuple[float, float, float, float],
                         initial_position: Tuple[float, float, float],
                         initial_size: Tuple[float, float, float]):
        """Create an interactive ROI marker for the specified LiDAR."""
        marker_name = f"roi_lidar_{lidar_id}"

        # Store initial ROI state
        with self.roi_lock:
            self.roi_states[lidar_id] = {
                'position': initial_position,
                'size': initial_size,
                'last_update': time.time()
            }

        # Create the interactive marker
        int_marker = InteractiveMarker()
        int_marker.header = Header()
        int_marker.header.frame_id = self.frame_id
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = marker_name
        int_marker.description = f"LiDAR {lidar_id} ROI Box"
        int_marker.scale = self.marker_scale

        # Set initial pose
        int_marker.pose = Pose()
        int_marker.pose.position = Point(
            x=initial_position[0],
            y=initial_position[1],
            z=initial_position[2]
        )
        int_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Create visual representation (box outline)
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.interaction_mode = InteractiveMarkerControl.NONE

        # Create box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale = Vector3(
            x=initial_size[0],
            y=initial_size[1],
            z=initial_size[2]
        )
        box_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # Add movement controls
        self.add_movement_controls(int_marker)

        # Add scale controls
        self.add_scale_controls(int_marker, initial_size)

        # Insert marker with callback
        self.marker_server.insert(int_marker, self.marker_feedback_callback)

    def add_movement_controls(self, int_marker: InteractiveMarker):
        """Add 6-DOF movement controls to the interactive marker."""
        # X-axis movement
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=1.0)
        control.always_visible = True
        int_marker.controls.append(control)

        # Y-axis movement
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        control.always_visible = True
        int_marker.controls.append(control)

        # Z-axis movement
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)
        control.always_visible = True
        int_marker.controls.append(control)

        # 3D movement
        control = InteractiveMarkerControl()
        control.name = "move_3d"
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True
        int_marker.controls.append(control)

    def add_scale_controls(self, int_marker: InteractiveMarker, initial_size: Tuple[float, float, float]):
        """Add scale controls for resizing the ROI box."""
        # Note: Interactive markers don't directly support scaling
        # This is a simplified approach - real scaling would require
        # custom control implementation or separate scale handles

        # For now, we'll rely on the movement controls and
        # implement scaling through pose changes and size tracking
        pass

    def marker_feedback_callback(self, feedback: InteractiveMarkerFeedback):
        """Handle interactive marker feedback (user manipulation)."""
        try:
            # Extract LiDAR ID from marker name
            lidar_id = int(feedback.marker_name.split('_')[-1])

            if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                self.handle_pose_update(lidar_id, feedback)
            elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
                self.get_logger().debug(f'Started manipulating LiDAR {lidar_id} ROI')
            elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                self.get_logger().debug(f'Finished manipulating LiDAR {lidar_id} ROI')
                self.send_roi_update(lidar_id, feedback.pose)

        except Exception as e:
            self.get_logger().error(f'Error in marker feedback callback: {e}')

    def handle_pose_update(self, lidar_id: int, feedback: InteractiveMarkerFeedback):
        """Handle real-time pose updates during manipulation."""
        # Update internal state
        with self.roi_lock:
            if lidar_id in self.roi_states:
                self.roi_states[lidar_id]['position'] = (
                    feedback.pose.position.x,
                    feedback.pose.position.y,
                    feedback.pose.position.z
                )
                self.roi_states[lidar_id]['last_update'] = time.time()

        # Update marker visualization
        self.update_marker_visualization(lidar_id, feedback.pose)

    def update_marker_visualization(self, lidar_id: int, pose: Pose):
        """Update the visual representation of the ROI marker."""
        # Get current marker
        marker_name = f"roi_lidar_{lidar_id}"

        # Update the marker pose
        self.marker_server.setPose(marker_name, pose)
        self.marker_server.applyChanges()

    def send_roi_update(self, lidar_id: int, pose: Pose):
        """Send ROI bounds update to the Rust multi_wayside_node."""
        if not self.roi_service_client.service_is_ready():
            self.get_logger().warn('ROI service not available, skipping update')
            return

        # Get current size from stored state
        with self.roi_lock:
            if lidar_id not in self.roi_states:
                self.get_logger().error(f'No ROI state found for LiDAR {lidar_id}')
                return
            current_size = self.roi_states[lidar_id]['size']

        # Create service request
        request = SetROIBounds.Request()
        request.lidar_id = lidar_id
        request.center_x = pose.position.x
        request.center_y = pose.position.y
        request.center_z = pose.position.z
        request.size_x = current_size[0]
        request.size_y = current_size[1]
        request.size_z = current_size[2]

        # Send asynchronous service call
        future = self.roi_service_client.call_async(request)
        future.add_done_callback(
            lambda f: self.handle_roi_service_response(f, lidar_id)
        )

    def handle_roi_service_response(self, future, lidar_id: int):
        """Handle response from ROI bounds service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully updated ROI for LiDAR {lidar_id}')
            else:
                self.get_logger().error(f'Failed to update ROI for LiDAR {lidar_id}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for LiDAR {lidar_id}: {e}')

    def get_roi_bounds(self, lidar_id: int) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Get current ROI bounds for the specified LiDAR."""
        with self.roi_lock:
            if lidar_id in self.roi_states:
                state = self.roi_states[lidar_id]
                return state['position'], state['size']
            else:
                # Return defaults
                return (
                    (self.default_position_x, self.default_position_y, self.default_position_z),
                    (self.default_size_x, self.default_size_y, self.default_size_z)
                )


def main(args=None):
    """Main entry point for the ROI interactive node."""
    rclpy.init(args=args)

    try:
        # Create node
        node = ROIInteractiveNode()

        # Use MultiThreadedExecutor for handling callbacks concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        node.get_logger().info('ROI Interactive Node is running...')
        node.get_logger().info('Open RViz2 and add Interactive Markers display')
        node.get_logger().info('Topic: /roi_interactive_markers/update')

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down ROI Interactive Node...')

    except Exception as e:
        print(f'Failed to start ROI Interactive Node: {e}')

    finally:
        # Cleanup
        rclpy.shutdown()


if __name__ == '__main__':
    main()
