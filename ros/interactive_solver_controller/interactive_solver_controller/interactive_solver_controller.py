#!/usr/bin/env python3
"""
Interactive Solver Controller - Rich TUI

A text-based user interface for controlling the advanced extrinsic solver.
Uses the Rich library for beautiful console output.

Author: LCTK Team
License: MIT
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass

import rclpy
from lctk_interfaces.srv import (
    AddDetectionToBuffer,
    AdjustTransform,
    ClearDetectionBuffer,
    DumpDetections,
    GetBufferStatus,
    GetPoseInfo,
    ListDetectionBuffer,
    LoadDetections,
    RemoveDetectionFromBuffer,
    ResetTransform,
)
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# Check for keyboard input support
try:
    import termios
    import tty

    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False


@dataclass
class PoseData:
    """Pose data with translation and rotation."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class DisplayState:
    """State for the TUI display."""

    num_detections: int = 0
    total_correspondences: int = 0
    is_publishing: bool = False
    solve_status: str = "No calibration"
    has_pose: bool = False
    solved_pose: PoseData = None
    current_pose: PoseData = None
    adjustment: PoseData = None
    translation_step: float = 0.01
    rotation_step: float = 0.01
    last_message: str = ""
    last_message_success: bool = True

    def __post_init__(self):
        if self.solved_pose is None:
            self.solved_pose = PoseData()
        if self.current_pose is None:
            self.current_pose = PoseData()
        if self.adjustment is None:
            self.adjustment = PoseData()


class InteractiveSolverController(Node):
    """Interactive TUI controller using Rich."""

    # Service path layout: <namespace>/lidar_to_camera_solver/<service>
    NODE_NAME = "lidar_to_camera_solver"
    DISCOVERY_SUFFIX = "/get_pose_info"  # unique service used to locate a solver

    def __init__(self):
        super().__init__("interactive_solver_controller")
        self.state = DisplayState()
        self.default_save_path = os.path.expanduser("~/detections.json")
        self.SERVICE_BASE = None

    def discover_service_bases(self, timeout: float = 5.0):
        """Scan the ROS graph for lidar_to_camera_solver service bases.

        Returns a sorted list of service-base prefixes (one per running solver),
        e.g. "/calibration/seyond_lidar_left_camera/lidar_to_camera_solver".
        """
        match_suffix = f"/{self.NODE_NAME}{self.DISCOVERY_SUFFIX}"
        deadline = time.time() + timeout
        found = set()
        while time.time() < deadline:
            for name, types in self.get_service_names_and_types():
                if name.endswith(match_suffix) and any(
                    "GetPoseInfo" in t for t in types
                ):
                    found.add(name[: -len(self.DISCOVERY_SUFFIX)])
            if found:
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        return sorted(found)

    def configure(self, service_base: str):
        """Bind this controller to a discovered/specified service base."""
        self.SERVICE_BASE = service_base
        self._create_service_clients()

    def _create_service_clients(self):
        """Create all service clients."""
        self.add_detection_client = self.create_client(
            AddDetectionToBuffer, f"{self.SERVICE_BASE}/add_detection"
        )
        self.clear_buffer_client = self.create_client(
            ClearDetectionBuffer, f"{self.SERVICE_BASE}/clear_buffer"
        )
        self.get_status_client = self.create_client(
            GetBufferStatus, f"{self.SERVICE_BASE}/get_status"
        )
        self.list_buffer_client = self.create_client(
            ListDetectionBuffer, f"{self.SERVICE_BASE}/list_buffer"
        )
        self.remove_detection_client = self.create_client(
            RemoveDetectionFromBuffer, f"{self.SERVICE_BASE}/remove_detection"
        )
        self.dump_detections_client = self.create_client(
            DumpDetections, f"{self.SERVICE_BASE}/dump_detections"
        )
        self.load_detections_client = self.create_client(
            LoadDetections, f"{self.SERVICE_BASE}/load_detections"
        )
        self.adjust_transform_client = self.create_client(
            AdjustTransform, f"{self.SERVICE_BASE}/adjust_transform"
        )
        self.reset_transform_client = self.create_client(
            ResetTransform, f"{self.SERVICE_BASE}/reset_transform"
        )
        self.get_pose_info_client = self.create_client(
            GetPoseInfo, f"{self.SERVICE_BASE}/get_pose_info"
        )

    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """Wait for essential services."""
        essential = [
            self.add_detection_client,
            self.get_status_client,
            self.get_pose_info_client,
        ]
        for client in essential:
            if not client.wait_for_service(timeout_sec=timeout):
                return False
        return True

    def _call(self, client, request, timeout: float = 5.0):
        """Generic service call."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def refresh_state(self):
        """Refresh display state from services."""
        # Get buffer status
        resp = self._call(self.get_status_client, GetBufferStatus.Request())
        if resp:
            self.state.num_detections = resp.buffer_size
            self.state.total_correspondences = resp.total_correspondences
            self.state.is_publishing = resp.is_publishing
            self.state.solve_status = resp.last_solve_status

        # Get pose info
        resp = self._call(self.get_pose_info_client, GetPoseInfo.Request())
        if resp and resp.has_pose:
            self.state.has_pose = True
            self.state.solved_pose = PoseData(
                resp.solved_x,
                resp.solved_y,
                resp.solved_z,
                resp.solved_roll,
                resp.solved_pitch,
                resp.solved_yaw,
            )
            self.state.current_pose = PoseData(
                resp.current_x,
                resp.current_y,
                resp.current_z,
                resp.current_roll,
                resp.current_pitch,
                resp.current_yaw,
            )
            self.state.adjustment = PoseData(
                resp.adjust_x,
                resp.adjust_y,
                resp.adjust_z,
                resp.adjust_roll,
                resp.adjust_pitch,
                resp.adjust_yaw,
            )
        else:
            self.state.has_pose = False

    def set_message(self, msg: str, success: bool = True):
        """Set status message."""
        self.state.last_message = msg
        self.state.last_message_success = success

    def add_detection(self) -> bool:
        resp = self._call(self.add_detection_client, AddDetectionToBuffer.Request())
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False

    def clear_buffer(self) -> bool:
        resp = self._call(self.clear_buffer_client, ClearDetectionBuffer.Request())
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False

    def remove_last(self) -> bool:
        if self.state.num_detections == 0:
            self.set_message("Buffer is empty", False)
            return False
        req = RemoveDetectionFromBuffer.Request()
        req.index = self.state.num_detections - 1
        resp = self._call(self.remove_detection_client, req)
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False

    def dump_detections(self) -> bool:
        req = DumpDetections.Request()
        req.file_path = self.default_save_path
        resp = self._call(self.dump_detections_client, req)
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False

    def load_detections(self) -> bool:
        req = LoadDetections.Request()
        req.file_path = self.default_save_path
        req.append = False
        resp = self._call(self.load_detections_client, req)
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False

    def adjust_transform(self, **kwargs) -> bool:
        req = AdjustTransform.Request()
        req.delta_x = kwargs.get("delta_x", 0.0)
        req.delta_y = kwargs.get("delta_y", 0.0)
        req.delta_z = kwargs.get("delta_z", 0.0)
        req.delta_roll = kwargs.get("delta_roll", 0.0)
        req.delta_pitch = kwargs.get("delta_pitch", 0.0)
        req.delta_yaw = kwargs.get("delta_yaw", 0.0)
        resp = self._call(self.adjust_transform_client, req)
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False

    def reset_transform(self) -> bool:
        resp = self._call(self.reset_transform_client, ResetTransform.Request())
        if resp:
            self.set_message(resp.message, resp.success)
            return resp.success
        self.set_message("Service timeout", False)
        return False


def create_status_panel(state: DisplayState) -> Panel:
    """Create status panel."""
    table = Table(show_header=False, box=None, padding=(0, 1))
    table.add_column(style="bold")
    table.add_column()

    table.add_row("Detections:", f"[cyan]{state.num_detections}[/]")
    table.add_row("Correspondences:", f"[cyan]{state.total_correspondences}[/]")
    pub_str = "[green]Yes[/]" if state.is_publishing else "[red]No[/]"
    table.add_row("Publishing:", pub_str)
    table.add_row("Status:", state.solve_status[:40])

    return Panel(table, title="[bold]Buffer Status[/]", border_style="blue")


def create_pose_table(title: str, pose: PoseData, style: str = "cyan") -> Table:
    """Create a pose display table."""
    table = Table(show_header=False, box=None, padding=(0, 1))
    table.add_column(width=8)
    table.add_column(width=12, justify="right")

    table.add_row("X:", f"[{style}]{pose.x * 1000:+.2f} mm[/]")
    table.add_row("Y:", f"[{style}]{pose.y * 1000:+.2f} mm[/]")
    table.add_row("Z:", f"[{style}]{pose.z * 1000:+.2f} mm[/]")
    table.add_row("Roll:", f"[{style}]{math.degrees(pose.roll):+.2f}°[/]")
    table.add_row("Pitch:", f"[{style}]{math.degrees(pose.pitch):+.2f}°[/]")
    table.add_row("Yaw:", f"[{style}]{math.degrees(pose.yaw):+.2f}°[/]")

    return Panel(
        table,
        title=f"[bold]{title}[/]",
        border_style=style.split()[0] if " " in style else style,
    )


def create_poses_panel(state: DisplayState) -> Panel:
    """Create poses panel with all three poses."""
    if not state.has_pose:
        return Panel(
            "[dim]No pose computed yet[/]",
            title="[bold]Poses[/]",
            border_style="yellow",
        )

    # Create three columns
    layout = Layout()
    layout.split_row(
        Layout(name="solved"),
        Layout(name="adjust"),
        Layout(name="current"),
    )

    layout["solved"].update(
        create_pose_table("Solved (PnP)", state.solved_pose, "blue")
    )
    layout["adjust"].update(create_pose_table("Adjustment", state.adjustment, "yellow"))
    layout["current"].update(create_pose_table("Current", state.current_pose, "green"))

    return Panel(layout, title="[bold]Pose Information[/]", border_style="cyan")


def create_step_panel(state: DisplayState) -> Panel:
    """Create step size panel."""
    text = Text()
    text.append("Translation: ", style="bold")
    text.append(f"{state.translation_step * 1000:.1f} mm", style="cyan")
    text.append("  |  ", style="dim")
    text.append("Rotation: ", style="bold")
    text.append(f"{math.degrees(state.rotation_step):.2f}°", style="cyan")
    return Panel(text, title="[bold]Step Size[/]", border_style="magenta")


def create_keybindings_panel() -> Panel:
    """Create key bindings panel."""
    table = Table(show_header=False, box=None, padding=(0, 1), expand=True)
    table.add_column(width=12)
    table.add_column()
    table.add_column(width=12)
    table.add_column()

    # Row 1
    table.add_row(
        "[yellow]Space[/]", "Add detection", "[yellow]Backspace[/]", "Delete last"
    )
    # Row 2
    table.add_row("[yellow]c[/]", "Clear buffer", "[yellow]0[/]", "Reset adjustments")
    # Row 3
    table.add_row("[yellow]p[/]", "Save to file", "[yellow]o[/]", "Load from file")
    # Row 4 - separator
    table.add_row("", "", "", "")
    # Row 5
    table.add_row("[cyan]q/a[/]", "+/- X", "[cyan]r/f[/]", "+/- Roll")
    # Row 6
    table.add_row("[cyan]w/s[/]", "+/- Y", "[cyan]t/g[/]", "+/- Pitch")
    # Row 7
    table.add_row("[cyan]e/d[/]", "+/- Z", "[cyan]y/b[/]", "+/- Yaw")
    # Row 8 - separator
    table.add_row("", "", "", "")
    # Row 9
    table.add_row("[magenta]][/]", "Step +", "[magenta][[][/]", "Step -")
    # Row 10
    table.add_row("[red]ESC[/]", "Exit", "", "")

    return Panel(table, title="[bold]Key Bindings[/]", border_style="green")


def create_message_panel(state: DisplayState) -> Panel:
    """Create message panel."""
    if not state.last_message:
        return Panel("[dim]Ready[/]", border_style="dim")

    style = "green" if state.last_message_success else "red"
    prefix = "[OK]" if state.last_message_success else "[FAIL]"
    return Panel(f"[{style}]{prefix}[/] {state.last_message}", border_style=style)


def create_layout(state: DisplayState) -> Layout:
    """Create the main layout."""
    layout = Layout()

    # Main vertical split
    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="main"),
        Layout(name="footer", size=3),
    )

    # Header
    layout["header"].update(
        Panel("[bold cyan]Extrinsic Calibration Controller[/]", border_style="cyan")
    )

    # Main area split
    layout["main"].split_row(
        Layout(name="left", ratio=2),
        Layout(name="right", ratio=1),
    )

    # Left side: poses and status
    layout["left"].split_column(
        Layout(name="status", size=8),
        Layout(name="poses"),
        Layout(name="step", size=3),
    )

    layout["status"].update(create_status_panel(state))
    layout["poses"].update(create_poses_panel(state))
    layout["step"].update(create_step_panel(state))

    # Right side: key bindings
    layout["right"].update(create_keybindings_panel())

    # Footer: message
    layout["footer"].update(create_message_panel(state))

    return layout


def read_key() -> str:
    """Read a single key press."""
    if not HAS_TERMIOS:
        return ""

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)

        # Handle escape sequences
        if ch == "\x1b":
            tty.setraw(fd)
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                sys.stdin.read(1)  # Consume arrow key
                return ""
            elif ch2 == "" or ch2 == "\x1b":
                return "ESC"
            return ""
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _resolve_service_base(node, console, override):
    """Determine the solver service base via CLI override or graph discovery.

    Returns the service-base string, or None if the user should abort.
    """
    if override:
        base = override.rstrip("/")
        # Accept either a namespace or a full service base.
        if not base.endswith(f"/{node.NODE_NAME}"):
            base = f"{base}/{node.NODE_NAME}"
        console.print(f"Using service base: [cyan]{base}[/]")
        return base

    console.print("Discovering solver services...", end=" ")
    bases = node.discover_service_bases(timeout=5.0)
    if not bases:
        console.print("[red]none found[/]")
        console.print(
            "[red]No lidar_to_camera_solver services on the ROS graph.[/]\n"
            "[dim]Is the solver running with use_advanced_solver=true?[/]"
        )
        return None
    if len(bases) == 1:
        console.print(f"[green]found[/] [cyan]{bases[0]}[/]")
        return bases[0]

    console.print(f"[green]found {len(bases)}[/]")
    console.print("[yellow]Multiple solvers found:[/]")
    for i, b in enumerate(bases):
        console.print(f"  [{i}] {b}")
    while True:
        choice = console.input("Select index ([dim]q to quit[/]): ").strip()
        if choice.lower() == "q":
            return None
        if choice.isdigit() and int(choice) < len(bases):
            return bases[int(choice)]
        console.print("[red]Invalid selection[/]")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(prog="interactive_solver_controller")
    parser.add_argument(
        "--service-base",
        "--namespace",
        dest="service_base",
        default=None,
        help="Solver namespace or full service base. Default: auto-discover from graph.",
    )
    cli_args = remove_ros_args(args=sys.argv)[1:]
    parsed, _ = parser.parse_known_args(cli_args)

    console = Console()
    node = InteractiveSolverController()

    service_base = _resolve_service_base(node, console, parsed.service_base)
    if service_base is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    node.configure(service_base)

    console.print("[bold cyan]Extrinsic Calibration Controller[/]")
    console.print("Connecting to services...", end=" ")

    if not node.wait_for_services(timeout=10.0):
        console.print("[red]FAILED[/]")
        console.print(
            f"[red]Could not connect to solver services at {service_base}.[/]"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    console.print("[green]OK[/]")

    # Initial refresh
    node.refresh_state()

    running = True
    try:
        with Live(
            create_layout(node.state),
            console=console,
            refresh_per_second=4,
            screen=True,
        ) as live:
            while running and rclpy.ok():
                key = read_key()

                if not key:
                    node.refresh_state()
                    live.update(create_layout(node.state))
                    time.sleep(0.05)
                    continue

                # Handle keys
                if key == "ESC" or key == "\x03":  # ESC or Ctrl+C
                    running = False

                elif key == " ":  # Space - add detection
                    node.add_detection()

                elif key in ("\x7f", "\x08"):  # Backspace
                    node.remove_last()

                elif key == "c":
                    node.clear_buffer()

                elif key == "0":
                    node.reset_transform()

                elif key == "p":
                    node.dump_detections()

                elif key == "o":
                    node.load_detections()

                elif key == "]":
                    node.state.translation_step *= 2.0
                    node.state.rotation_step *= 2.0
                    node.set_message(
                        f"Step: {node.state.translation_step * 1000:.1f}mm / {math.degrees(node.state.rotation_step):.2f}°"
                    )

                elif key == "[":
                    node.state.translation_step /= 2.0
                    node.state.rotation_step /= 2.0
                    node.set_message(
                        f"Step: {node.state.translation_step * 1000:.1f}mm / {math.degrees(node.state.rotation_step):.2f}°"
                    )

                # Translation adjustments
                elif key == "q":
                    node.adjust_transform(delta_x=node.state.translation_step)
                elif key == "a":
                    node.adjust_transform(delta_x=-node.state.translation_step)
                elif key == "w":
                    node.adjust_transform(delta_y=node.state.translation_step)
                elif key == "s":
                    node.adjust_transform(delta_y=-node.state.translation_step)
                elif key == "e":
                    node.adjust_transform(delta_z=node.state.translation_step)
                elif key == "d":
                    node.adjust_transform(delta_z=-node.state.translation_step)

                # Rotation adjustments
                elif key == "r":
                    node.adjust_transform(delta_roll=node.state.rotation_step)
                elif key == "f":
                    node.adjust_transform(delta_roll=-node.state.rotation_step)
                elif key == "t":
                    node.adjust_transform(delta_pitch=node.state.rotation_step)
                elif key == "g":
                    node.adjust_transform(delta_pitch=-node.state.rotation_step)
                elif key == "y":
                    node.adjust_transform(delta_yaw=node.state.rotation_step)
                elif key == "b":
                    node.adjust_transform(delta_yaw=-node.state.rotation_step)

                # Refresh and update display
                node.refresh_state()
                live.update(create_layout(node.state))

    except KeyboardInterrupt:
        pass
    finally:
        console.print("\n[dim]Exiting...[/]")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
