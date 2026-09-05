"""Camera preview helpers and the stamped-evidence distortion golden.

No solver in this tree subscribes to an image, which is why a bad capture is
currently undiagnosable. These tests pin the two properties that matter: a
preview must never be able to break a capture, and the bytes must be a real JPEG.
"""

import cv2
import numpy as np
import pytest
from lidar_to_camera_solver.evidence_store import EvidenceStore
from lidar_to_camera_solver.preview import (
    annotate,
    decode_image,
    encode_jpeg,
)


def bgr_frame(height=16, width=24):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 2] = 255  # red in BGR
    return frame


def _store_with_frame(
    image: np.ndarray,
    *,
    max_previews: int = 4,
    jpeg_quality: int = 80,
    stamp: float = 1.0,
    camera_matrix: np.ndarray | None = None,
    distortion: np.ndarray | None = None,
) -> EvidenceStore:
    store = EvidenceStore(
        max_previews=max_previews,
        jpeg_quality=jpeg_quality,
        review_evidence_seconds=1.0,
        stamp_tolerance_s=0.001,
    )
    store.observe_intrinsics(
        np.eye(3) if camera_matrix is None else camera_matrix,
        np.zeros(5) if distortion is None else distortion,
    )
    height, width = image.shape[:2]
    store.observe_frame(
        stamp,
        height,
        width,
        "bgr8",
        width * 3,
        image.tobytes(),
    )
    return store


_SAMPLE3_CAMERA_MATRIX = np.array(
    [
        [1164.6233338297, 0.0, 950.1242940800],
        [0.0, 1161.1018211652, 538.5516554830],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
_SAMPLE3_DISTORTION = np.array(
    [0.1007777625, -0.3423565275, -0.0067918667, 0.0039835784, 0.4138983068],
    dtype=np.float64,
)


def _distorted_marker_frame(
    rectified_quad: np.ndarray, distortion: np.ndarray
) -> np.ndarray:
    """Render a red marker where ``rectified_quad`` lands in a raw image."""
    frame = np.full((1080, 1920, 3), 255, dtype=np.uint8)
    normalized = np.column_stack(
        (
            (rectified_quad[:, 0] - _SAMPLE3_CAMERA_MATRIX[0, 2])
            / _SAMPLE3_CAMERA_MATRIX[0, 0],
            (rectified_quad[:, 1] - _SAMPLE3_CAMERA_MATRIX[1, 2])
            / _SAMPLE3_CAMERA_MATRIX[1, 1],
            np.ones(len(rectified_quad)),
        )
    )
    raw_quad, _ = cv2.projectPoints(
        normalized,
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        _SAMPLE3_CAMERA_MATRIX,
        distortion,
    )
    raw_quad = raw_quad.reshape(-1, 2)
    cv2.fillPoly(frame, [np.rint(raw_quad).astype(np.int32)], (0, 0, 255))
    return frame


def _assert_green_outline_lands_on_marker(jpeg: bytes) -> None:
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    green = (
        (image[:, :, 1] > 120)
        & (image[:, :, 1] > image[:, :, 0] * 1.5)
        & (image[:, :, 1] > image[:, :, 2] * 1.5)
    ).astype(np.uint8)
    assert green.any(), "the golden marker outline is missing"

    # Derive the marker boundary from the returned image itself.  In particular,
    # do not compare against ``raw_quad``: after the fix the frame is rectified,
    # so the source-frame coordinates are intentionally no longer the output
    # coordinates.  The red fill is just a synthetic marker with a colour that
    # remains distinct from the green annotation through JPEG encoding.
    red = (
        (image[:, :, 2] > 120)
        & (image[:, :, 2] > image[:, :, 0] * 1.5)
        & (image[:, :, 2] > image[:, :, 1] * 1.5)
    ).astype(np.uint8)
    assert red.any(), "the synthetic marker is missing from the returned image"
    marker_points = np.column_stack(np.where(red))[:, ::-1].astype(np.int32)
    hull = cv2.convexHull(marker_points.reshape(-1, 1, 2))
    contour = np.zeros(green.shape, dtype=np.uint8)
    cv2.polylines(
        contour,
        [hull],
        isClosed=True,
        color=255,
        thickness=5,
    )
    distance = cv2.distanceTransform(255 - contour, cv2.DIST_L2, 3)
    # Corner 0 carries a filled direction marker, so its interior pixels are
    # intentionally not on the boundary.  Identify that corner from the
    # returned pixels (the densest green neighbourhood of the image-derived
    # hull vertices), rather than relying on the synthetic quad's coordinates.
    vertices = cv2.approxPolyDP(hull, epsilon=3.0, closed=True)
    assert len(vertices) >= 3, "the synthetic marker boundary is degenerate"
    yy, xx = np.ogrid[: green.shape[0], : green.shape[1]]
    green_density = []
    for x, y in vertices[:, 0, :]:
        neighbourhood = (xx - x) ** 2 + (yy - y) ** 2 <= 15**2
        green_density.append(int(green[neighbourhood].sum()))
    corner0 = vertices[int(np.argmax(green_density)), 0, :]
    vertex_mask = np.zeros(green.shape, dtype=np.uint8)
    cv2.circle(vertex_mask, tuple(int(value) for value in corner0), 12, 1, -1)
    score_pixels = green.astype(bool) & ~vertex_mask.astype(bool)
    nearby = distance[score_pixels] <= 5.0
    assert nearby.any(), "the golden marker outline has no measurable edge pixels"
    assert float(np.mean(nearby)) >= 0.9, (
        "the drawn corners do not land on the marker in the source frame"
    )


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
    store = EvidenceStore(max_previews=4, jpeg_quality=80)
    evidence = store.capture(1, 1.0, corners=[])
    assert evidence.preview_jpeg is None
    assert store.get(1) == evidence


def test_capture_stores_retrievable_jpeg_bytes():
    store = _store_with_frame(bgr_frame(), jpeg_quality=80)
    evidence = store.capture(7, 1.0, corners=[])
    assert evidence.preview_jpeg is not None
    assert store.get(7).preview_jpeg[:2] == b"\xff\xd8"


def test_sample3_distorted_edge_marker_is_aligned_in_its_source_frame():
    """The raw sample3 marker is not at the published (rectified) pixels.

    ArUco publishes corners after ``undistortPoints``.  The old store draws those
    coordinates straight onto raw pixels, which is only correct at D=0 (or near
    the principal point).  This deliberately uses a marker near the image edge,
    where sample3's real coefficients move it by tens of pixels.
    """
    rectified_quad = np.array(
        [[1800.0, 850.0], [1860.0, 850.0], [1860.0, 910.0], [1800.0, 910.0]],
        dtype=np.float64,
    )
    frame = _distorted_marker_frame(rectified_quad, _SAMPLE3_DISTORTION)
    store = _store_with_frame(
        frame,
        max_previews=1,
        jpeg_quality=100,
        camera_matrix=_SAMPLE3_CAMERA_MATRIX,
        distortion=_SAMPLE3_DISTORTION,
    )
    evidence = store.capture(1, 1.0, corners=[rectified_quad])
    assert evidence.preview_jpeg is not None
    _assert_green_outline_lands_on_marker(evidence.preview_jpeg)


def test_zero_distortion_edge_marker_is_aligned_by_the_existing_store():
    """The inert D=0 case must pass even before the rectification fix."""
    rectified_quad = np.array(
        [[1800.0, 850.0], [1860.0, 850.0], [1860.0, 910.0], [1800.0, 910.0]],
        dtype=np.float64,
    )
    frame = _distorted_marker_frame(rectified_quad, np.zeros(5, dtype=np.float64))
    store = _store_with_frame(
        frame,
        max_previews=1,
        jpeg_quality=100,
        camera_matrix=_SAMPLE3_CAMERA_MATRIX,
        distortion=np.zeros(5, dtype=np.float64),
    )
    evidence = store.capture(1, 1.0, corners=[rectified_quad])
    assert evidence.preview_jpeg is not None
    _assert_green_outline_lands_on_marker(evidence.preview_jpeg)


def test_store_evicts_the_oldest_beyond_the_bound():
    store = _store_with_frame(bgr_frame(), max_previews=2, jpeg_quality=80)
    for pair_id in (1, 2, 3):
        store.capture(pair_id, 1.0, corners=[])
    assert store.get(1) is None, "oldest must be evicted"
    assert store.get(2) is not None
    assert store.get(3) is not None


def test_drop_and_clear():
    store = _store_with_frame(bgr_frame(), jpeg_quality=80)
    store.capture(1, 1.0, corners=[])
    store.drop(1)
    assert store.get(1) is None
    store.capture(2, 1.0, corners=[])
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
