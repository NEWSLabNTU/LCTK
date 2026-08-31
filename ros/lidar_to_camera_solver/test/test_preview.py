"""PreviewStore: the picture a queued pair was measured in.

No solver in this tree subscribes to an image, which is why a bad capture is
currently undiagnosable. These tests pin the two properties that matter: a
preview must never be able to break a capture, and the bytes must be a real JPEG.
"""

import numpy as np
import pytest
from lidar_to_camera_solver.preview import (
    PreviewStore,
    annotate,
    decode_image,
    encode_jpeg,
)


def bgr_frame(height=16, width=24):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 2] = 255  # red in BGR
    return frame


def test_decode_bgr8_round_trips():
    frame = bgr_frame()
    decoded = decode_image(
        height=16, width=24, encoding="bgr8", step=24 * 3, data=frame.tobytes()
    )
    assert decoded.shape == (16, 24, 3)
    assert np.array_equal(decoded, frame)


def test_decode_rgb8_swaps_channels():
    rgb = np.zeros((4, 4, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255  # red in RGB
    decoded = decode_image(
        height=4, width=4, encoding="rgb8", step=12, data=rgb.tobytes()
    )
    assert decoded[0, 0, 2] == 255, "red must land in the BGR red channel"
    assert decoded[0, 0, 0] == 0


def test_decode_mono8_expands_to_three_channels():
    mono = np.full((4, 4), 128, dtype=np.uint8)
    decoded = decode_image(
        height=4, width=4, encoding="mono8", step=4, data=mono.tobytes()
    )
    assert decoded.shape == (4, 4, 3)
    assert np.all(decoded == 128)


def test_decode_honours_row_padding():
    # step larger than width*channels: rows are padded, which naive reshaping ignores.
    padded = np.zeros((4, 10 * 3), dtype=np.uint8)
    padded[:, : 4 * 3] = 7
    decoded = decode_image(
        height=4, width=4, encoding="bgr8", step=30, data=padded.tobytes()
    )
    assert decoded.shape == (4, 4, 3)
    assert np.all(decoded == 7)


def test_decode_rejects_an_unsupported_encoding():
    with pytest.raises(ValueError, match="bayer_rggb8"):
        decode_image(
            height=2, width=2, encoding="bayer_rggb8", step=2, data=b"\x00" * 4
        )


def test_encode_jpeg_produces_jpeg_magic_bytes():
    data = encode_jpeg(bgr_frame(), quality=80)
    assert data[:2] == b"\xff\xd8", "JPEG SOI marker"
    assert data[-2:] == b"\xff\xd9", "JPEG EOI marker"


def test_annotate_does_not_mutate_the_input_frame():
    frame = bgr_frame()
    original = frame.copy()
    annotate(frame, [np.array([[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]])], None)
    assert np.array_equal(frame, original)


def test_annotate_draws_something():
    frame = bgr_frame()
    marked = annotate(
        frame, [np.array([[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]])], None
    )
    assert not np.array_equal(marked, frame)


def test_capture_without_a_frame_reports_failure_and_does_not_raise():
    store = PreviewStore(max_previews=4, jpeg_quality=80)
    assert store.capture(1, corners=[], reprojected=None) is False
    assert store.get(1) is None


def test_capture_stores_retrievable_jpeg_bytes():
    store = PreviewStore(max_previews=4, jpeg_quality=80)
    store.set_latest(bgr_frame())
    assert store.capture(7, corners=[], reprojected=None) is True
    assert store.get(7)[:2] == b"\xff\xd8"


def test_store_evicts_the_oldest_beyond_the_bound():
    store = PreviewStore(max_previews=2, jpeg_quality=80)
    store.set_latest(bgr_frame())
    for pair_id in (1, 2, 3):
        store.capture(pair_id, corners=[], reprojected=None)
    assert store.get(1) is None, "oldest must be evicted"
    assert store.get(2) is not None
    assert store.get(3) is not None


def test_drop_and_clear():
    store = PreviewStore(max_previews=4, jpeg_quality=80)
    store.set_latest(bgr_frame())
    store.capture(1, corners=[], reprojected=None)
    store.drop(1)
    assert store.get(1) is None
    store.capture(2, corners=[], reprojected=None)
    store.clear()
    assert store.get(2) is None
