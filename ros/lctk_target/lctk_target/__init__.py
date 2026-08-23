"""ROS-free Target Definition loader and board-local ArUco geometry."""

from .target import TargetIdentity, ValidatedTarget, load_target

__all__ = ["TargetIdentity", "ValidatedTarget", "load_target"]
