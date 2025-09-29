#!/usr/bin/env python3

import sys
import signal
import curses
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from lctk_interfaces.srv import GetBBoxParams, SetBBoxParams, SaveBBoxParams
import numpy as np


def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
    x, y, z, w = q.x, q.y, q.z, q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) in radians to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


class BBoxState:
    def __init__(self):
        self.position = np.array([2.5, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.size_xyz = np.array([1.0, 3.0, 2.0])

    def to_pose(self):
        pose = Pose()
        pose.position = Point(
            x=self.position[0], y=self.position[1], z=self.position[2]
        )
        pose.orientation = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        return pose

    def from_pose(self, pose, size_xyz):
        self.position = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.roll, self.pitch, self.yaw = quaternion_to_euler(pose.orientation)
        self.size_xyz = np.array(size_xyz)


class FilterBoxTunerNode(Node):
    def __init__(self):
        super().__init__("filter_box_tuner")

        self.get_bbox_client = self.create_client(
            GetBBoxParams, "/calibration/lidar_board_detector/get_bbox_params"
        )
        self.set_bbox_client = self.create_client(
            SetBBoxParams, "/calibration/lidar_board_detector/set_bbox_params"
        )
        self.save_bbox_client = self.create_client(
            SaveBBoxParams, "/calibration/lidar_board_detector/save_bbox_params"
        )

        while not self.get_bbox_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_bbox_params service...")
        while not self.set_bbox_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_bbox_params service...")
        while not self.save_bbox_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for save_bbox_params service...")

        self.get_logger().info("All services available")

    def get_bbox_params(self):
        request = GetBBoxParams.Request()
        future = self.get_bbox_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            return future.result()
        return None

    def set_bbox_params(self, pose, size_xyz):
        request = SetBBoxParams.Request()
        request.pose = pose
        request.size_xyz = list(size_xyz)
        future = self.set_bbox_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.done():
            return future.result()
        return None

    def save_bbox_params(self):
        request = SaveBBoxParams.Request()
        request.file_path = ""
        future = self.save_bbox_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            return future.result()
        return None


class TunerApp:
    def __init__(self, node):
        self.node = node
        self.bbox_state = BBoxState()
        self.status_message = "Ready"
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.running = False

    def refresh_from_node(self):
        self.status_message = "Fetching params from node..."
        response = self.node.get_bbox_params()
        if response:
            self.bbox_state.from_pose(response.pose, response.size_xyz)
            self.status_message = "Refreshed from node"
        else:
            self.status_message = "Error: Timeout waiting for response"

    def send_to_node(self):
        pose = self.bbox_state.to_pose()
        response = self.node.set_bbox_params(pose, self.bbox_state.size_xyz)
        if response and response.success:
            self.status_message = "Updated"
        else:
            self.status_message = (
                f"Error: {response.message if response else 'Timeout'}"
            )

    def save_to_file(self):
        self.status_message = "Saving to file..."
        response = self.node.save_bbox_params()
        if response and response.success:
            self.status_message = f"Saved to {response.saved_file_path}"
        else:
            self.status_message = (
                f"Error: {response.message if response else 'Timeout'}"
            )

    def handle_key(self, key, shift_pressed):
        small_step = 0.1
        large_step = 0.5
        translation_step = large_step if shift_pressed else small_step

        small_angle = math.radians(5.0)
        large_angle = math.radians(15.0)
        rotation_step = large_angle if shift_pressed else small_angle

        if key == curses.KEY_UP:
            self.bbox_state.position[0] += translation_step
            self.send_to_node()
        elif key == curses.KEY_DOWN:
            self.bbox_state.position[0] -= translation_step
            self.send_to_node()
        elif key == curses.KEY_LEFT:
            self.bbox_state.position[1] += translation_step
            self.send_to_node()
        elif key == curses.KEY_RIGHT:
            self.bbox_state.position[1] -= translation_step
            self.send_to_node()
        elif key == curses.KEY_PPAGE:
            self.bbox_state.position[2] += translation_step
            self.send_to_node()
        elif key == curses.KEY_NPAGE:
            self.bbox_state.position[2] -= translation_step
            self.send_to_node()
        elif key in [ord("r"), ord("R")]:
            self.bbox_state.yaw += rotation_step
            self.send_to_node()
        elif key in [ord("f"), ord("F")]:
            self.bbox_state.yaw -= rotation_step
            self.send_to_node()
        elif key in [ord("t"), ord("T")]:
            self.bbox_state.pitch += rotation_step
            self.send_to_node()
        elif key in [ord("g"), ord("G")]:
            self.bbox_state.pitch -= rotation_step
            self.send_to_node()
        elif key in [ord("y"), ord("Y")]:
            self.bbox_state.roll += rotation_step
            self.send_to_node()
        elif key in [ord("h"), ord("H")]:
            self.bbox_state.roll -= rotation_step
            self.send_to_node()
        elif key == ord(" "):
            self.refresh_from_node()
        elif key in [ord("s"), ord("S")]:
            self.save_to_file()
        elif key in [ord("q"), ord("Q"), 27]:
            self.running = False
            self.status_message = "Quitting..."
        elif key == ord("?"):
            self.status_message = "See controls below"

    def draw_ui(self, stdscr):
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        roll_deg = math.degrees(self.bbox_state.roll)
        pitch_deg = math.degrees(self.bbox_state.pitch)
        yaw_deg = math.degrees(self.bbox_state.yaw)

        try:
            row = 1
            stdscr.addstr(
                row,
                2,
                "╔════════════════════════════════════════════════════════════╗",
                curses.color_pair(1),
            )
            row += 1
            stdscr.addstr(
                row,
                2,
                "║          Filter Box Tuner - Interactive Tool              ║",
                curses.color_pair(1),
            )
            row += 1
            stdscr.addstr(
                row,
                2,
                "╚════════════════════════════════════════════════════════════╝",
                curses.color_pair(1),
            )
            row += 2

            stdscr.addstr(row, 2, "Current Parameters:", curses.color_pair(2))
            row += 1
            stdscr.addstr(
                row,
                2,
                f"  Position:  X: {self.bbox_state.position[0]:.3f} m,  "
                f"Y: {self.bbox_state.position[1]:.3f} m,  "
                f"Z: {self.bbox_state.position[2]:.3f} m",
            )
            row += 1
            stdscr.addstr(
                row,
                2,
                f"  Rotation:  Roll: {roll_deg:.1f}°,  "
                f"Pitch: {pitch_deg:.1f}°,  Yaw: {yaw_deg:.1f}°",
            )
            row += 1
            stdscr.addstr(
                row,
                2,
                f"  Size:      X: {self.bbox_state.size_xyz[0]:.3f} m,  "
                f"Y: {self.bbox_state.size_xyz[1]:.3f} m,  "
                f"Z: {self.bbox_state.size_xyz[2]:.3f} m",
            )
            row += 2

            stdscr.addstr(row, 2, "Keyboard Controls:", curses.color_pair(3))
            row += 1
            stdscr.addstr(row, 2, "  Translation (0.1m steps, Shift for 0.5m):")
            row += 1
            stdscr.addstr(row, 4, "↑/↓         - Move forward/backward (X)")
            row += 1
            stdscr.addstr(row, 4, "←/→         - Move left/right (Y)")
            row += 1
            stdscr.addstr(row, 4, "PgUp/PgDn   - Move up/down (Z)")
            row += 2
            stdscr.addstr(row, 2, "  Rotation (5° steps, Shift for 15°):")
            row += 1
            stdscr.addstr(row, 4, "R/F         - Yaw +/- (rotate around Z)")
            row += 1
            stdscr.addstr(row, 4, "T/G         - Pitch +/- (rotate around Y)")
            row += 1
            stdscr.addstr(row, 4, "Y/H         - Roll +/- (rotate around X)")
            row += 2
            stdscr.addstr(row, 2, "  Commands:")
            row += 1
            stdscr.addstr(row, 4, "S           - Save current params to file")
            row += 1
            stdscr.addstr(row, 4, "Space       - Refresh params from node")
            row += 1
            stdscr.addstr(row, 4, "?           - Show this help")
            row += 1
            stdscr.addstr(row, 4, "Q/ESC       - Quit")
            row += 1
            stdscr.addstr(row, 4, "Ctrl-C      - Quit (graceful)")
            row += 2

            stdscr.addstr(
                row, 2, f"Status: {self.status_message}", curses.color_pair(4)
            )

        except curses.error:
            pass

        stdscr.refresh()

    def run(self, stdscr):
        curses.curs_set(0)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
        stdscr.timeout(100)

        self.refresh_from_node()
        self.draw_ui(stdscr)

        while self.running:
            key = stdscr.getch()
            if key != -1:
                shift_pressed = False
                if 65 <= key <= 90:
                    shift_pressed = True

                self.handle_key(key, shift_pressed)
                self.draw_ui(stdscr)


def main():
    rclpy.init()
    node = FilterBoxTunerNode()

    app = TunerApp(node)

    try:
        curses.wrapper(app.run)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Filter Box Tuner exited.")


if __name__ == "__main__":
    main()
