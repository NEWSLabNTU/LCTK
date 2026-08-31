"""The camera frame a queued pair was measured in.

No solver in this tree subscribes to an image, so when a capture turns out bad --
motion blur, an occluded marker, glare across the plate -- nothing downstream can
say why. This module keeps the latest frame, and snapshots it when a pair is
queued, with the detected corners drawn on.

``decode_image`` takes the ``sensor_msgs/Image`` **fields** rather than the
message, so this module imports no ROS and its tests need no graph. It also means
``cv_bridge`` is not a dependency for what amounts to a reshape.

**A preview must never be able to break a capture.** Every failure path here
returns falsy rather than raising; calibration correctness does not depend on a
picture being available.
"""

from __future__ import annotations

import threading
from collections import OrderedDict
from collections.abc import Sequence

import cv2
import numpy as np

_CORNER_COLOR = (0, 255, 0)
_REPROJECTED_COLOR = (0, 128, 255)


def decode_image(
    *, height: int, width: int, encoding: str, step: int, data: bytes
) -> np.ndarray:
    """One ``sensor_msgs/Image`` as an HxWx3 BGR array.

    ``step`` is honoured rather than assumed: a padded row stride silently
    shears the image if you reshape by ``width`` alone.
    """
    encoding = encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        channels = 3
    elif encoding == "mono8":
        channels = 1
    else:
        raise ValueError(
            f"unsupported image encoding '{encoding}'; "
            "preview supports bgr8, rgb8 and mono8"
        )

    buffer = np.frombuffer(data, dtype=np.uint8)
    expected = step * height
    if buffer.size < expected:
        raise ValueError(
            f"image data is {buffer.size} bytes; {expected} required for "
            f"{height} rows of stride {step}"
        )
    rows = buffer[:expected].reshape(height, step)
    frame = rows[:, : width * channels].reshape(height, width, channels)

    if encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == "mono8":
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


def annotate(
    frame: np.ndarray,
    corners: Sequence[np.ndarray],
    reprojected: Sequence[np.ndarray] | None,
) -> np.ndarray:
    """A copy of ``frame`` with detected corners and reprojected points drawn.

    Never mutates the input: the latest-frame buffer is shared, and drawing into
    it would corrupt the next capture.
    """
    canvas = frame.copy()
    for quad in corners:
        points = np.asarray(quad, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [points], isClosed=True, color=_CORNER_COLOR, thickness=1)
        # Mark corner 0 so a quarter-turn in the correspondence order is visible.
        first = tuple(int(v) for v in np.asarray(quad, dtype=np.float64)[0])
        cv2.circle(canvas, first, 3, _CORNER_COLOR, -1)
    for point in reprojected or ():
        x, y = (int(v) for v in np.asarray(point, dtype=np.float64).ravel()[:2])
        cv2.drawMarker(
            canvas, (x, y), _REPROJECTED_COLOR, cv2.MARKER_CROSS, markerSize=6
        )
    return canvas


def encode_jpeg(frame: np.ndarray, quality: int) -> bytes:
    ok, buffer = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise ValueError("cv2.imencode refused the frame")
    return buffer.tobytes()


class PreviewStore:
    """Latest camera frame, plus a bounded cache of per-pair JPEG snapshots."""

    def __init__(self, *, max_previews: int, jpeg_quality: int):
        self._max_previews = max_previews
        self._jpeg_quality = jpeg_quality
        self._latest: np.ndarray | None = None
        self._previews: OrderedDict[int, bytes] = OrderedDict()
        self._lock = threading.Lock()

    def set_latest(self, frame: np.ndarray | None) -> None:
        """Store the newest frame.

        Called from the subscription callback, so it does no work beyond the
        assignment.
        """
        with self._lock:
            self._latest = frame

    def capture(self, pair_id: int, corners, reprojected) -> bool:
        """Snapshot the latest frame against ``pair_id``. False if there is none."""
        with self._lock:
            frame = self._latest
        if frame is None:
            return False
        try:
            data = encode_jpeg(
                annotate(frame, corners, reprojected), self._jpeg_quality
            )
        except (ValueError, cv2.error):
            return False
        with self._lock:
            self._previews[pair_id] = data
            self._previews.move_to_end(pair_id)
            while len(self._previews) > self._max_previews:
                self._previews.popitem(last=False)
        return True

    def get(self, pair_id: int) -> bytes | None:
        with self._lock:
            return self._previews.get(pair_id)

    def drop(self, pair_id: int) -> None:
        with self._lock:
            self._previews.pop(pair_id, None)

    def clear(self) -> None:
        with self._lock:
            self._previews.clear()
