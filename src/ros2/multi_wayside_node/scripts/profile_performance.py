#!/usr/bin/env python3
"""
Performance profiling script for multi_wayside_node.

This script monitors and analyzes the performance of the multi_wayside_node,
including CPU usage, memory consumption, message rates, and processing latencies.
"""

import argparse
import json
import os
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray


class PerformanceProfiler(Node):
    def __init__(self, duration_sec=60, output_dir="performance_results"):
        super().__init__("performance_profiler")

        self.duration_sec = duration_sec
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # Process monitoring
        self.process_name = "multi_wayside_node"
        self.target_process = None

        # Metrics storage
        self.metrics = {
            "timestamps": [],
            "cpu_percent": [],
            "memory_mb": [],
            "lidar1_rate": deque(maxlen=100),
            "lidar2_rate": deque(maxlen=100),
            "detection1_rate": deque(maxlen=100),
            "detection2_rate": deque(maxlen=100),
            "calibration_count": 0,
            "processing_latencies": [],
            "message_sizes": {
                "lidar1_points": [],
                "lidar2_points": [],
            },
            "warnings": [],
            "errors": [],
        }

        # Time tracking
        self.last_msgs = {
            "lidar1_points": None,
            "lidar2_points": None,
            "lidar1_detection": None,
            "lidar2_detection": None,
        }

        # Create subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subs = {
            "lidar1_points": self.create_subscription(
                PointCloud2,
                "/lidar1/points",
                lambda msg: self.on_pointcloud(msg, "lidar1"),
                qos,
            ),
            "lidar2_points": self.create_subscription(
                PointCloud2,
                "/lidar2/points",
                lambda msg: self.on_pointcloud(msg, "lidar2"),
                qos,
            ),
            "lidar1_detection": self.create_subscription(
                Detection3DArray,
                "/lidar1/board_detection",
                lambda msg: self.on_detection(msg, "lidar1"),
                qos,
            ),
            "lidar2_detection": self.create_subscription(
                Detection3DArray,
                "/lidar2/board_detection",
                lambda msg: self.on_detection(msg, "lidar2"),
                qos,
            ),
            "calibration": self.create_subscription(
                TransformStamped, "/calibration_transform", self.on_calibration, qos
            ),
            "rosout": self.create_subscription(Log, "/rosout", self.on_log, qos),
        }

        # Start monitoring
        self.start_time = time.time()
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self.monitor_process)
        self.monitor_thread.start()

        # Schedule stop
        self.timer = self.create_timer(duration_sec, self.stop_profiling)

        self.get_logger().info(
            f"Started performance profiling for {duration_sec} seconds"
        )

    def find_process(self):
        """Find the multi_wayside_node process"""
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                cmdline = " ".join(proc.info["cmdline"] or [])
                if self.process_name in cmdline:
                    self.target_process = proc
                    self.get_logger().info(
                        f"Found {self.process_name} process (PID: {proc.pid})"
                    )
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return False

    def monitor_process(self):
        """Monitor CPU and memory usage"""
        if not self.find_process():
            self.get_logger().warn(f"Could not find {self.process_name} process")
            return

        while self.monitoring:
            try:
                if self.target_process and self.target_process.is_running():
                    cpu = self.target_process.cpu_percent(interval=0.1)
                    memory = self.target_process.memory_info().rss / 1024 / 1024  # MB

                    self.metrics["timestamps"].append(time.time() - self.start_time)
                    self.metrics["cpu_percent"].append(cpu)
                    self.metrics["memory_mb"].append(memory)
                else:
                    # Try to find process again
                    self.find_process()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self.find_process()

            time.sleep(1.0)

    def on_pointcloud(self, msg: PointCloud2, lidar_id: str):
        """Track point cloud messages"""
        now = time.time()
        key = f"{lidar_id}_points"

        # Calculate rate
        if self.last_msgs[key] is not None:
            dt = now - self.last_msgs[key]
            rate = 1.0 / dt if dt > 0 else 0
            self.metrics[f"{lidar_id}_rate"].append(rate)

        self.last_msgs[key] = now

        # Track message size
        msg_size = len(msg.data) / 1024 / 1024  # MB
        self.metrics["message_sizes"][key].append(msg_size)

    def on_detection(self, msg: Detection3DArray, lidar_id: str):
        """Track detection messages"""
        now = time.time()
        key = f"{lidar_id}_detection"

        # Calculate rate and latency
        if self.last_msgs[key] is not None:
            dt = now - self.last_msgs[key]
            rate = 1.0 / dt if dt > 0 else 0
            self.metrics[f"detection{lidar_id[-1]}_rate"].append(rate)

        self.last_msgs[key] = now

        # Estimate processing latency (time since last point cloud)
        pc_key = f"{lidar_id}_points"
        if self.last_msgs[pc_key] is not None:
            latency = now - self.last_msgs[pc_key]
            self.metrics["processing_latencies"].append(latency * 1000)  # ms

    def on_calibration(self, msg: TransformStamped):
        """Track calibration events"""
        self.metrics["calibration_count"] += 1
        self.get_logger().info(
            f"Calibration #{self.metrics['calibration_count']} received"
        )

    def on_log(self, msg: Log):
        """Track warnings and errors"""
        if msg.level >= Log.WARN:
            entry = {
                "time": time.time() - self.start_time,
                "level": msg.level,
                "msg": msg.msg,
            }

            if msg.level == Log.WARN:
                self.metrics["warnings"].append(entry)
            elif msg.level >= Log.ERROR:
                self.metrics["errors"].append(entry)

    def stop_profiling(self):
        """Stop profiling and generate report"""
        self.monitoring = False
        self.monitor_thread.join()

        self.get_logger().info("Profiling complete, generating report...")
        self.generate_report()

        # Shutdown
        self.destroy_node()
        rclpy.shutdown()

    def generate_report(self):
        """Generate performance analysis report"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Save raw data
        json_file = os.path.join(
            self.output_dir, f"performance_metrics_{timestamp}.json"
        )
        with open(json_file, "w") as f:
            # Convert deques to lists for JSON serialization
            data = {}
            for key, value in self.metrics.items():
                if isinstance(value, deque):
                    data[key] = list(value)
                else:
                    data[key] = value
            json.dump(data, f, indent=2)

        # Generate plots
        self.generate_plots(timestamp)

        # Generate summary report
        self.generate_summary(timestamp)

    def generate_plots(self, timestamp):
        """Generate performance visualization plots"""
        fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        fig.suptitle(f"Multi-Wayside Node Performance Profile - {timestamp}")

        # CPU Usage
        ax = axes[0, 0]
        if self.metrics["timestamps"] and self.metrics["cpu_percent"]:
            ax.plot(self.metrics["timestamps"], self.metrics["cpu_percent"])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("CPU %")
            ax.set_title("CPU Usage")
            ax.grid(True)

        # Memory Usage
        ax = axes[0, 1]
        if self.metrics["timestamps"] and self.metrics["memory_mb"]:
            ax.plot(self.metrics["timestamps"], self.metrics["memory_mb"])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Memory (MB)")
            ax.set_title("Memory Usage")
            ax.grid(True)

        # Message Rates
        ax = axes[1, 0]
        for key in ["lidar1_rate", "lidar2_rate"]:
            if self.metrics[key]:
                rates = list(self.metrics[key])
                times = np.linspace(0, self.duration_sec, len(rates))
                ax.plot(times, rates, label=key.replace("_rate", ""))
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Rate (Hz)")
        ax.set_title("Point Cloud Message Rates")
        ax.legend()
        ax.grid(True)

        # Detection Rates
        ax = axes[1, 1]
        for key in ["detection1_rate", "detection2_rate"]:
            if self.metrics[key]:
                rates = list(self.metrics[key])
                times = np.linspace(0, self.duration_sec, len(rates))
                ax.plot(times, rates, label=f"LiDAR {key[9]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Rate (Hz)")
        ax.set_title("Detection Message Rates")
        ax.legend()
        ax.grid(True)

        # Processing Latencies
        ax = axes[2, 0]
        if self.metrics["processing_latencies"]:
            ax.hist(self.metrics["processing_latencies"], bins=50, alpha=0.7)
            ax.set_xlabel("Latency (ms)")
            ax.set_ylabel("Count")
            ax.set_title("Detection Processing Latency Distribution")
            ax.grid(True, alpha=0.3)

        # Message Sizes
        ax = axes[2, 1]
        for key, sizes in self.metrics["message_sizes"].items():
            if sizes:
                ax.hist(sizes, bins=30, alpha=0.5, label=key.replace("_points", ""))
        ax.set_xlabel("Size (MB)")
        ax.set_ylabel("Count")
        ax.set_title("Point Cloud Message Size Distribution")
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_file = os.path.join(self.output_dir, f"performance_plots_{timestamp}.png")
        plt.savefig(plot_file, dpi=150)
        self.get_logger().info(f"Saved performance plots to {plot_file}")

    def generate_summary(self, timestamp):
        """Generate text summary report"""
        report_file = os.path.join(
            self.output_dir, f"performance_summary_{timestamp}.txt"
        )

        with open(report_file, "w") as f:
            f.write(f"Multi-Wayside Node Performance Summary\n")
            f.write(f"=====================================\n")
            f.write(f"Timestamp: {timestamp}\n")
            f.write(f"Duration: {self.duration_sec} seconds\n\n")

            # CPU and Memory
            if self.metrics["cpu_percent"]:
                f.write(f"CPU Usage:\n")
                f.write(f"  Average: {np.mean(self.metrics['cpu_percent']):.1f}%\n")
                f.write(f"  Max: {np.max(self.metrics['cpu_percent']):.1f}%\n")
                f.write(f"  Std Dev: {np.std(self.metrics['cpu_percent']):.1f}%\n\n")

            if self.metrics["memory_mb"]:
                f.write(f"Memory Usage:\n")
                f.write(f"  Average: {np.mean(self.metrics['memory_mb']):.1f} MB\n")
                f.write(f"  Max: {np.max(self.metrics['memory_mb']):.1f} MB\n")
                f.write(f"  Final: {self.metrics['memory_mb'][-1]:.1f} MB\n\n")

            # Message Rates
            f.write(f"Message Rates (Hz):\n")
            for key in [
                "lidar1_rate",
                "lidar2_rate",
                "detection1_rate",
                "detection2_rate",
            ]:
                if self.metrics[key]:
                    rates = list(self.metrics[key])
                    f.write(f"  {key}:\n")
                    f.write(f"    Average: {np.mean(rates):.1f} Hz\n")
                    f.write(f"    Std Dev: {np.std(rates):.2f} Hz\n")
            f.write("\n")

            # Processing Latency
            if self.metrics["processing_latencies"]:
                f.write(f"Detection Processing Latency:\n")
                f.write(
                    f"  Average: {np.mean(self.metrics['processing_latencies']):.1f} ms\n"
                )
                f.write(
                    f"  Median: {np.median(self.metrics['processing_latencies']):.1f} ms\n"
                )
                f.write(
                    f"  95th percentile: {np.percentile(self.metrics['processing_latencies'], 95):.1f} ms\n"
                )
                f.write(
                    f"  Max: {np.max(self.metrics['processing_latencies']):.1f} ms\n\n"
                )

            # Calibrations
            f.write(f"Calibrations:\n")
            f.write(f"  Total: {self.metrics['calibration_count']}\n")
            if self.duration_sec > 0:
                f.write(
                    f"  Rate: {self.metrics['calibration_count'] / self.duration_sec * 60:.2f} per minute\n\n"
                )

            # Warnings and Errors
            f.write(f"Log Analysis:\n")
            f.write(f"  Warnings: {len(self.metrics['warnings'])}\n")
            f.write(f"  Errors: {len(self.metrics['errors'])}\n\n")

            if self.metrics["warnings"]:
                f.write("Recent Warnings:\n")
                for w in self.metrics["warnings"][-5:]:
                    f.write(f"  [{w['time']:.1f}s] {w['msg']}\n")
                f.write("\n")

            if self.metrics["errors"]:
                f.write("Recent Errors:\n")
                for e in self.metrics["errors"][-5:]:
                    f.write(f"  [{e['time']:.1f}s] {e['msg']}\n")

        self.get_logger().info(f"Saved performance summary to {report_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Profile multi_wayside_node performance"
    )
    parser.add_argument(
        "--duration", type=int, default=60, help="Profiling duration in seconds"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="performance_results",
        help="Output directory for results",
    )

    args = parser.parse_args()

    rclpy.init()

    try:
        profiler = PerformanceProfiler(
            duration_sec=args.duration, output_dir=args.output_dir
        )
        rclpy.spin(profiler)
    except KeyboardInterrupt:
        pass
    finally:
        if "profiler" in locals():
            profiler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
