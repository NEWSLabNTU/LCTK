#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
import rcl_interfaces.msg
import sys
import termios
import tty
import threading
import math


class BBoxAdjuster(Node):
    """Interactive bounding box parameter adjustment for calibration_board_locator node."""

    def __init__(self):
        super().__init__("bbox_adjuster")

        # Parameters
        self.declare_parameter(
            "target_node",
            "/calibration/calibration_board_locator/calibration_board_locator",
        )
        self.declare_parameter("bbox_config_file", "")

        self.target_node = (
            self.get_parameter("target_node").get_parameter_value().string_value
        )
        self.bbox_config_file = (
            self.get_parameter("bbox_config_file").get_parameter_value().string_value
        )

        if not self.bbox_config_file:
            self.get_logger().error("bbox_config_file parameter is required")
            sys.exit(1)

        # Current bbox parameters
        self.bbox_x = 2.5
        self.bbox_y = 0.0
        self.bbox_z = 0.0
        self.bbox_size_x = 1.0
        self.bbox_size_y = 3.0
        self.bbox_size_z = 2.0
        self.bbox_roll = 0.0
        self.bbox_pitch = 0.0
        self.bbox_yaw = 0.0

        # Load current parameters from target node
        self.load_current_parameters()

        # Key handling
        self.step_size = 0.1
        self.running = True

        self.get_logger().info(f"BBox Adjuster connected to {self.target_node}")
        self.get_logger().info(f"Config file: {self.bbox_config_file}")
        self.print_help()
        self.print_current_params()

        # Start keyboard listener in separate thread
        self.keyboard_thread = threading.Thread(
            target=self.keyboard_listener, daemon=True
        )
        self.keyboard_thread.start()

        # If not running in interactive mode, exit gracefully
        if not sys.stdin.isatty():
            self.running = False

    def load_current_parameters(self):
        """Load current parameters from the target node."""
        try:
            # Get parameters from target node
            client = self.create_client(
                GetParameters, f"{self.target_node}/get_parameters"
            )
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"Could not connect to {self.target_node} parameter service"
                )
                return

            request = GetParameters.Request()
            request.names = [
                "bbox_x",
                "bbox_y",
                "bbox_z",
                "bbox_size_x",
                "bbox_size_y",
                "bbox_size_z",
                "bbox_roll",
                "bbox_pitch",
                "bbox_yaw",
            ]

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                response = future.result()
                if len(response.values) >= 9:
                    self.bbox_x = response.values[0].double_value
                    self.bbox_y = response.values[1].double_value
                    self.bbox_z = response.values[2].double_value
                    self.bbox_size_x = response.values[3].double_value
                    self.bbox_size_y = response.values[4].double_value
                    self.bbox_size_z = response.values[5].double_value
                    self.bbox_roll = response.values[6].double_value
                    self.bbox_pitch = response.values[7].double_value
                    self.bbox_yaw = response.values[8].double_value
                    self.get_logger().info("Loaded current parameters from target node")

        except Exception as e:
            self.get_logger().warn(f"Failed to load parameters from target node: {e}")

    def update_target_parameters(self):
        """Update parameters on the target node."""
        try:
            client = self.create_client(
                SetParameters, f"{self.target_node}/set_parameters"
            )
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"Could not connect to {self.target_node} parameter service"
                )
                return False

            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    "bbox_x", Parameter.Type.DOUBLE, self.bbox_x
                ).to_parameter_msg(),
                Parameter(
                    "bbox_y", Parameter.Type.DOUBLE, self.bbox_y
                ).to_parameter_msg(),
                Parameter(
                    "bbox_z", Parameter.Type.DOUBLE, self.bbox_z
                ).to_parameter_msg(),
                Parameter(
                    "bbox_size_x", Parameter.Type.DOUBLE, self.bbox_size_x
                ).to_parameter_msg(),
                Parameter(
                    "bbox_size_y", Parameter.Type.DOUBLE, self.bbox_size_y
                ).to_parameter_msg(),
                Parameter(
                    "bbox_size_z", Parameter.Type.DOUBLE, self.bbox_size_z
                ).to_parameter_msg(),
                Parameter(
                    "bbox_roll", Parameter.Type.DOUBLE, self.bbox_roll
                ).to_parameter_msg(),
                Parameter(
                    "bbox_pitch", Parameter.Type.DOUBLE, self.bbox_pitch
                ).to_parameter_msg(),
                Parameter(
                    "bbox_yaw", Parameter.Type.DOUBLE, self.bbox_yaw
                ).to_parameter_msg(),
            ]

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                response = future.result()
                success = all(result.successful for result in response.results)
                if success:
                    self.get_logger().info("Updated target node parameters")
                else:
                    self.get_logger().warn("Some parameters failed to update")
                return success
            return False

        except Exception as e:
            self.get_logger().error(f"Failed to update target parameters: {e}")
            return False

    def save_to_config_file(self):
        """Save current parameters to the bbox config file."""
        try:
            config = {
                "pose": {
                    "translation": [self.bbox_x, self.bbox_y, self.bbox_z],
                    "rotation": [1.0, 0.0, 0.0, 0.0],  # Identity quaternion
                    "euler_angles": [self.bbox_roll, self.bbox_pitch, self.bbox_yaw],
                },
                "size_xyz": [self.bbox_size_x, self.bbox_size_y, self.bbox_size_z],
            }

            with open(self.bbox_config_file, "w") as f:
                f.write("{\n")
                f.write(
                    "    // Bounding box configuration for filtering point cloud data\n"
                )
                f.write(
                    "    // The pose defines the position and orientation of the box center\n"
                )
                f.write('    "pose": {\n')
                f.write("        // Translation from origin (x, y, z in meters)\n")
                f.write(
                    f'        "translation": [{self.bbox_x}, {self.bbox_y}, {self.bbox_z}],\n'
                )
                f.write("        // Rotation as quaternion (w, x, y, z)\n")
                f.write('        "rotation": [1.0, 0.0, 0.0, 0.0],\n')
                f.write("        // Euler angles (roll, pitch, yaw in radians)\n")
                f.write(
                    f'        "euler_angles": [{self.bbox_roll}, {self.bbox_pitch}, {self.bbox_yaw}]\n'
                )
                f.write("    },\n")
                f.write("    // Size of the bounding box in meters (x, y, z)\n")
                f.write(
                    f'    "size_xyz": [{self.bbox_size_x}, {self.bbox_size_y}, {self.bbox_size_z}]\n'
                )
                f.write("}\n")

            self.get_logger().info(f"Saved configuration to {self.bbox_config_file}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to save config file: {e}")
            return False

    def print_help(self):
        """Print help message."""
        help_text = """
╔══════════════════════════════════════════════════════════════════════════════╗
║                         BBOX INTERACTIVE ADJUSTER                           ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ TRANSLATION:                                                                 ║
║   w/s: Move X (forward/backward)     a/d: Move Y (left/right)               ║
║   r/f: Move Z (up/down)                                                     ║
║                                                                              ║
║ SIZE:                                                                        ║
║   t/g: Size X    y/h: Size Y    u/j: Size Z                                ║
║                                                                              ║
║ ROTATION (Euler angles):                                                     ║
║   i/k: Roll      o/l: Pitch     p/;: Yaw                                   ║
║                                                                              ║
║ STEP SIZE:                                                                   ║
║   +/-: Increase/decrease step size                                          ║
║                                                                              ║
║ ACTIONS:                                                                     ║
║   ENTER: Update target node parameters                                      ║
║   c: Save to config file                                                    ║
║   q: Quit                                                                   ║
║   ?: Show this help                                                         ║
╚══════════════════════════════════════════════════════════════════════════════╝
        """
        print(help_text)

    def print_current_params(self):
        """Print current bbox parameters."""
        print(f"\n┌─ Current BBox Parameters (step: {self.step_size:.3f}) ─┐")
        print(
            f"│ Position: X={self.bbox_x:6.3f}  Y={self.bbox_y:6.3f}  Z={self.bbox_z:6.3f} │"
        )
        print(
            f"│ Size:     X={self.bbox_size_x:6.3f}  Y={self.bbox_size_y:6.3f}  Z={self.bbox_size_z:6.3f} │"
        )
        print(
            f"│ Rotation: R={math.degrees(self.bbox_roll):6.1f}°  P={math.degrees(self.bbox_pitch):6.1f}°  Y={math.degrees(self.bbox_yaw):6.1f}° │"
        )
        print("└─────────────────────────────────────────────────────────┘")

    def keyboard_listener(self):
        """Listen for keyboard input."""
        # Check if stdin is a proper terminal
        if not sys.stdin.isatty():
            self.get_logger().error("This program requires an interactive terminal.")
            self.get_logger().error(
                "Please run directly: ros2 run bbox_interactive_adjuster bbox_adjuster --ros-args -p target_node:=/calibration/calibration_board_locator/calibration_board_locator -p bbox_config_file:/path/to/bbox.json5"
            )
            self.running = False
            return

        # Save original terminal settings
        try:
            old_settings = termios.tcgetattr(sys.stdin)
        except termios.error as e:
            self.get_logger().error(f"Cannot access terminal: {e}")
            self.get_logger().error("Please run in an interactive terminal session.")
            self.running = False
            return

        try:
            tty.setcbreak(sys.stdin.fileno())

            while self.running:
                key = sys.stdin.read(1)

                # Translation
                if key == "w":
                    self.bbox_x += self.step_size
                elif key == "s":
                    self.bbox_x -= self.step_size
                elif key == "a":
                    self.bbox_y -= self.step_size
                elif key == "d":
                    self.bbox_y += self.step_size
                elif key == "r":
                    self.bbox_z += self.step_size
                elif key == "f":
                    self.bbox_z -= self.step_size

                # Size
                elif key == "t":
                    self.bbox_size_x += self.step_size
                elif key == "g":
                    self.bbox_size_x = max(0.1, self.bbox_size_x - self.step_size)
                elif key == "y":
                    self.bbox_size_y += self.step_size
                elif key == "h":
                    self.bbox_size_y = max(0.1, self.bbox_size_y - self.step_size)
                elif key == "u":
                    self.bbox_size_z += self.step_size
                elif key == "j":
                    self.bbox_size_z = max(0.1, self.bbox_size_z - self.step_size)

                # Rotation
                elif key == "i":
                    self.bbox_roll += math.radians(5.0)
                elif key == "k":
                    self.bbox_roll -= math.radians(5.0)
                elif key == "o":
                    self.bbox_pitch += math.radians(5.0)
                elif key == "l":
                    self.bbox_pitch -= math.radians(5.0)
                elif key == "p":
                    self.bbox_yaw += math.radians(5.0)
                elif key == ";":
                    self.bbox_yaw -= math.radians(5.0)

                # Step size
                elif key == "+" or key == "=":
                    self.step_size = min(1.0, self.step_size * 1.5)
                elif key == "-":
                    self.step_size = max(0.001, self.step_size / 1.5)

                # Actions
                elif key == "\r" or key == "\n":  # Enter
                    self.update_target_parameters()
                elif key == "c":
                    self.save_to_config_file()
                elif key == "?":
                    self.print_help()
                elif key == "q":
                    self.running = False
                    break

                # Update display after any parameter change
                if key in "wsadrftgyhujikol;p+-=\r\nc?":
                    self.print_current_params()

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)

    try:
        adjuster = BBoxAdjuster()

        # Check if we're in interactive mode
        if not sys.stdin.isatty():
            print("\n" + "=" * 80)
            print("ERROR: This program requires an interactive terminal!")
            print("=" * 80)
            print("To use the bbox adjuster, run it in an interactive terminal:")
            print()
            print("Option 1: Use GNU screen or tmux:")
            print("  screen -S bbox_adjuster")
            print("  # or")
            print("  tmux new-session -s bbox_adjuster")
            print("  # then run:")
            print(
                "  . install/setup.sh && ./install/bbox_interactive_adjuster/bin/bbox_adjuster \\"
            )
            print("      --ros-args \\")
            print(
                "      -p target_node:=/calibration/calibration_board_locator/calibration_board_locator \\"
            )
            print(f"      -p bbox_config_file:{adjuster.bbox_config_file}")
            print()
            print("Option 2: Run directly in your terminal (single line):")
            print(
                f". install/setup.sh && ./install/bbox_interactive_adjuster/bin/bbox_adjuster --ros-args -p target_node:=/calibration/calibration_board_locator/calibration_board_locator -p bbox_config_file:{adjuster.bbox_config_file}"
            )
            print("=" * 80)
            return

        # Spin in a separate thread to handle ROS services
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(adjuster)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # Wait for keyboard listener to finish
        adjuster.keyboard_thread.join()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
