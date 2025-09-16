#!/usr/bin/env python3
"""
Monitor ROS2 topic timestamps with human-readable datetime conversion.
Shows the most recent timestamps from camera and LiDAR topics.
"""

import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from typing import Dict, Optional


class TopicMonitor:
    def __init__(self):
        self.running = True
        self.latest_timestamps = {
            "camera": None,
            "lidar": None,
            "sync_camera": None,
            "sync_lidar": None,
        }
        self.latest_datetimes = {
            "camera": None,
            "lidar": None,
            "sync_camera": None,
            "sync_lidar": None,
        }
        self.topics = {
            "camera": "/sensing/camera/front_center/image_raw",
            "lidar": "/sensing/lidar/top/pointcloud_raw",
            "sync_camera": "/sensing/camera/front_center/synchronized_image",
            "sync_lidar": "/sensing/lidar/top/synchronized_pointcloud",
        }
        self.processes = []
        self.debug_output = {}
        self.debug_error = {}
        self.debug_exception = {}

        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print("\n\nShutting down monitor...")
        self.running = False
        self.cleanup_processes()
        sys.exit(0)

    def cleanup_processes(self):
        """Clean up all spawned processes to prevent orphans"""
        for proc in self.processes:
            if proc.poll() is None:  # Process is still running
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                except Exception:
                    pass
        self.processes.clear()

    def ros_timestamp_to_datetime(self, sec, nanosec):
        """Convert ROS timestamp to human-readable datetime"""
        try:
            # ROS timestamps are typically in UNIX time
            timestamp = sec + nanosec / 1e9
            return datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[
                :-3
            ]
        except Exception as e:
            return f"Invalid timestamp: {e}"

    def monitor_topic(self, topic_name, topic_key):
        """Monitor a single topic in a separate thread"""
        cmd = ["ros2", "topic", "echo", "--once", topic_name]

        while self.running:
            try:
                # Use --once to get just one message, then repeat
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    preexec_fn=os.setsid,  # Create new process group for clean termination
                )
                self.processes.append(proc)

                stdout, stderr = proc.communicate(timeout=10)

                if proc in self.processes:
                    self.processes.remove(proc)

                if stdout and self.running:
                    # Parse the YAML-like output to find timestamp
                    lines = stdout.split("\n")
                    sec = None
                    nanosec = None

                    # Debug: print first few lines to understand structure
                    if topic_key == "camera":  # Only debug camera to avoid spam
                        debug_lines = lines[:10]
                        # Store debug info but don't print here to avoid interfering with display

                    # Look for header.stamp pattern
                    for i, line in enumerate(lines):
                        line = line.strip()
                        if "stamp:" in line:
                            # Look for sec and nanosec in the following lines
                            for j in range(i + 1, min(i + 10, len(lines))):
                                next_line = lines[j].strip()
                                if next_line.startswith("sec:"):
                                    try:
                                        sec = int(next_line.split(":")[1].strip())
                                    except (ValueError, IndexError):
                                        pass
                                elif next_line.startswith("nanosec:"):
                                    try:
                                        nanosec = int(next_line.split(":")[1].strip())
                                    except (ValueError, IndexError):
                                        pass
                            break
                        # Also try direct pattern matching
                        elif line.startswith("sec:") and sec is None:
                            try:
                                sec = int(line.split(":")[1].strip())
                            except (ValueError, IndexError):
                                pass
                        elif line.startswith("nanosec:") and nanosec is None:
                            try:
                                nanosec = int(line.split(":")[1].strip())
                            except (ValueError, IndexError):
                                pass

                    if sec is not None and nanosec is not None:
                        self.latest_timestamps[topic_key] = (sec, nanosec)
                        self.latest_datetimes[topic_key] = (
                            self.ros_timestamp_to_datetime(sec, nanosec)
                        )
                    else:
                        # Store debug info for troubleshooting
                        self.debug_output = {topic_key: lines[:15]}

                elif stderr and self.running:
                    # Store error for debugging
                    self.debug_error = {topic_key: stderr}

                # Small delay before next check
                time.sleep(0.5)

            except subprocess.TimeoutExpired:
                if proc in self.processes:
                    proc.kill()
                    self.processes.remove(proc)
                continue
            except Exception as e:
                if not self.running:
                    break
                # Store error for debugging
                self.debug_exception = {topic_key: str(e)}
                time.sleep(1)

    def display_loop(self):
        """Main display loop that updates the console output"""
        print("ROS2 Topic Timestamp Monitor")
        print("=" * 50)
        print("Topics:")
        print(f"  Camera: {self.topics['camera']}")
        print(f"  LiDAR:  {self.topics['lidar']}")
        print("\nPress Ctrl+C to exit\n")

        while self.running:
            try:
                # Clear screen and move cursor to top
                os.system("clear")
                print("ROS2 Topic Timestamp Monitor")
                print("=" * 50)
                print(f"Topics:")
                print(f"  Camera: {self.topics['camera']}")
                print(f"  LiDAR:  {self.topics['lidar']}")
                print(f"\nLast updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                print("-" * 50)

                # Display raw topics
                print("RAW TOPICS:")
                if self.latest_timestamps["camera"]:
                    sec, nanosec = self.latest_timestamps["camera"]
                    print(f"Camera: {self.latest_datetimes['camera']}")
                    print(f"        Raw: {sec}.{nanosec:09d}")
                else:
                    print("Camera: No data received")
                    if "camera" in self.debug_error:
                        print(f"        Error: {self.debug_error['camera'][:50]}...")

                if self.latest_timestamps["lidar"]:
                    sec, nanosec = self.latest_timestamps["lidar"]
                    print(f"LiDAR:  {self.latest_datetimes['lidar']}")
                    print(f"        Raw: {sec}.{nanosec:09d}")
                else:
                    print("LiDAR:  No data received")
                    if "lidar" in self.debug_error:
                        print(f"        Error: {self.debug_error['lidar'][:50]}...")

                print("\nSYNCHRONIZED TOPICS:")
                if self.latest_timestamps["sync_camera"]:
                    sec, nanosec = self.latest_timestamps["sync_camera"]
                    print(f"Sync Camera: {self.latest_datetimes['sync_camera']}")
                    print(f"             Raw: {sec}.{nanosec:09d}")
                else:
                    print("Sync Camera: No data received")

                if self.latest_timestamps["sync_lidar"]:
                    sec, nanosec = self.latest_timestamps["sync_lidar"]
                    print(f"Sync LiDAR:  {self.latest_datetimes['sync_lidar']}")
                    print(f"             Raw: {sec}.{nanosec:09d}")
                else:
                    print("Sync LiDAR:  No data received")

                # Calculate time differences
                print("\nTIME DIFFERENCES:")
                if self.latest_timestamps["camera"] and self.latest_timestamps["lidar"]:
                    cam_sec, cam_nanosec = self.latest_timestamps["camera"]
                    lid_sec, lid_nanosec = self.latest_timestamps["lidar"]

                    cam_time = cam_sec + cam_nanosec / 1e9
                    lid_time = lid_sec + lid_nanosec / 1e9

                    diff_ms = abs(cam_time - lid_time) * 1000
                    print(f"Raw topics:    {diff_ms:.3f} ms")

                if (
                    self.latest_timestamps["sync_camera"]
                    and self.latest_timestamps["sync_lidar"]
                ):
                    cam_sec, cam_nanosec = self.latest_timestamps["sync_camera"]
                    lid_sec, lid_nanosec = self.latest_timestamps["sync_lidar"]

                    cam_time = cam_sec + cam_nanosec / 1e9
                    lid_time = lid_sec + lid_nanosec / 1e9

                    diff_ms = abs(cam_time - lid_time) * 1000
                    print(f"Sync topics:   {diff_ms:.3f} ms")

                print("\nPress Ctrl+C to exit")

                time.sleep(1)

            except KeyboardInterrupt:
                break
            except Exception as e:
                if self.running:
                    print(f"Display error: {e}")
                    time.sleep(1)

    def run(self):
        """Start monitoring topics"""
        print("Starting topic monitors...")

        # Start monitoring threads for each topic
        threads = []
        for topic_key, topic_name in self.topics.items():
            thread = threading.Thread(
                target=self.monitor_topic,
                args=(topic_name, topic_key),
                daemon=True,
                name=f"monitor_{topic_key}",
            )
            threads.append(thread)
            thread.start()

        # Start display loop (this will block until Ctrl+C)
        try:
            self.display_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.cleanup_processes()


def main():
    """Main function"""
    # Check if ROS2 is available
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=5
        )
        if result.returncode != 0:
            print(
                "Error: ROS2 topics not accessible. Please source your ROS2 environment:"
            )
            print("  source /opt/ros/humble/setup.bash")
            print("  source install/setup.bash")
            print(f"Error: {result.stderr}")
            sys.exit(1)

        # Check if our topics exist
        topics_list = result.stdout
        camera_topic = "/sensing/camera/front_center/image_raw"
        lidar_topic = "/sensing/lidar/top/pointcloud_raw"

        if camera_topic not in topics_list:
            print(f"Warning: Camera topic {camera_topic} not found")
            print("Available camera topics:")
            for line in topics_list.split("\n"):
                if "camera" in line and line.strip():
                    print(f"  {line.strip()}")

        if lidar_topic not in topics_list:
            print(f"Warning: LiDAR topic {lidar_topic} not found")
            print("Available lidar topics:")
            for line in topics_list.split("\n"):
                if "lidar" in line and line.strip():
                    print(f"  {line.strip()}")

    except (
        subprocess.CalledProcessError,
        FileNotFoundError,
        subprocess.TimeoutExpired,
    ):
        print(
            "Error: ROS2 not found or not responsive. Please source your ROS2 environment:"
        )
        print("  source /opt/ros/humble/setup.bash")
        print("  source install/setup.bash")
        sys.exit(1)

    monitor = TopicMonitor()
    monitor.run()


if __name__ == "__main__":
    main()
