"""Frame algebra for exporting LCTK extrinsics to Autoware calibration YAML.

Conventions (see docs/superpowers/specs/2026-07-16-autoware-export-design.md):

- An Autoware calibration entry ``parent: {child: {x,y,z,roll,pitch,yaw}}`` is the
  pose of the child frame in the parent frame — the homogeneous matrix mapping
  child-frame coordinates into the parent frame.
- RPY is URDF fixed-axis XYZ: ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``, radians.
- The solver's rvec/tvec map LiDAR-frame points into the camera optical frame
  (``T_optical<-lidar``); the exported entry needs the opposite direction.
"""

import math

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis RPY to rotation matrix: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def matrix_to_rpy(R: np.ndarray) -> tuple:
    """Inverse of rpy_to_matrix. Near pitch = ±π/2 the roll/yaw split is
    degenerate; roll is pinned to 0 there (any split reproduces R)."""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-12:
        return (0.0, pitch, math.atan2(-R[0, 1], R[1, 1]))
    return (
        math.atan2(R[2, 1], R[2, 2]),
        pitch,
        math.atan2(R[1, 0], R[0, 0]),
    )


def rodrigues(rvec: np.ndarray) -> np.ndarray:
    """Axis-angle vector to rotation matrix (no cv2 dependency)."""
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-12:
        return np.eye(3)
    kx, ky, kz = rvec / angle
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def inv_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    return make_transform(R.T, -R.T @ T[:3, 3])


def entry_to_transform(entry: dict) -> np.ndarray:
    """Autoware calibration entry (pose of child in parent) to 4x4 matrix."""
    return make_transform(
        rpy_to_matrix(entry["roll"], entry["pitch"], entry["yaw"]),
        [entry["x"], entry["y"], entry["z"]],
    )


def transform_to_entry(T: np.ndarray) -> dict:
    roll, pitch, yaw = matrix_to_rpy(T[:3, :3])
    x, y, z = T[:3, 3]
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
    }


# Pose of the camera optical frame in the camera_link frame (ROS REP-103):
# optical z (forward) -> link x, optical x (right) -> link -y, optical y (down) -> link -z.
OPTICAL_IN_CAMERA_LINK = make_transform(
    rpy_to_matrix(-math.pi / 2, 0.0, -math.pi / 2), np.zeros(3)
)


def kit_to_camera_link(
    T_kit_lidar: np.ndarray, rvec: np.ndarray, tvec: np.ndarray
) -> np.ndarray:
    """Compose the exported entry:

    T(kit -> camera_link) = T(kit -> lidar)              [existing YAML entry]
                          @ inv(T_optical<-lidar)        [solver rvec/tvec, inverted]
                          @ inv(OPTICAL_IN_CAMERA_LINK)  [optical -> camera_link]
    """
    T_optical_from_lidar = make_transform(rodrigues(rvec), tvec)
    return (
        T_kit_lidar
        @ inv_transform(T_optical_from_lidar)
        @ inv_transform(OPTICAL_IN_CAMERA_LINK)
    )
