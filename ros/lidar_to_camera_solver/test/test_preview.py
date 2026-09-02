"""PreviewStore: the picture a queued pair was measured in.

No solver in this tree subscribes to an image, which is why a bad capture is
currently undiagnosable. These tests pin the two properties that matter: a
preview must never be able to break a capture, and the bytes must be a real JPEG.
"""

import cv2
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


def test_annotate_survives_the_thumbnail_downscale():
    """A hairline stroke is invisible on the review page, so it may as well not exist.

    The page shows previews at ~220 px. A 960-px-wide frame is shrunk 4x and a
    1080-px one nearly 9x, so a 1 px stroke lands at well under a pixel and
    disappears -- which is what shipped, and it hid exactly the corner-order
    evidence the preview is there to show. Rather than assert a constant nobody
    can interpret, shrink the annotated frame the way the browser does and
    require the marks to still be there.
    """
    frame = np.zeros((600, 960, 3), dtype=np.uint8)
    quad = np.array([[300.0, 200.0], [420.0, 200.0], [420.0, 320.0], [300.0, 320.0]])

    marked = annotate(frame, [quad], None)
    thumbnail = cv2.resize(marked, (220, 138), interpolation=cv2.INTER_AREA)

    # Count pixels that stay clearly green rather than peak brightness: a
    # hairline still leaves one bright pixel where the corners meet, so a `max`
    # check passes on exactly the code this test exists to reject. Measured on
    # this frame, a 1 px stroke leaves 1 such pixel and a 3 px stroke leaves 116.
    survived = int((thumbnail[:, :, 1] > 128).sum())
    assert survived >= 40, (
        f"only {survived} pixel(s) of the corner outline survived the downscale "
        "to review-page size; the stroke is too thin to see"
    )


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


@pytest.mark.parametrize("encoding", ["bgra8", "rgba8"])
def test_decode_image_accepts_the_four_channel_encodings_a_zed_publishes(encoding):
    """A ZED publishes bgra8. Refusing it left the review page with no picture at
    all on the one rig assisted mode was built for, and the node said so only in
    a per-frame warning nobody was reading."""
    height, width = 2, 3
    # Distinct per-channel values, so a wrong channel order cannot pass.
    pixel = [10, 20, 30, 255] if encoding == "bgra8" else [30, 20, 10, 255]
    data = bytes(pixel * width * height)

    frame = decode_image(
        height=height,
        width=width,
        encoding=encoding,
        step=width * 4,
        data=data,
    )

    assert frame.shape == (height, width, 3), "alpha must be dropped, not kept"
    assert list(frame[0, 0]) == [10, 20, 30], "BGR order"
