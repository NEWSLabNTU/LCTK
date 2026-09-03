"""Every shipped session parses, and the sample-data one keeps its topics.

These read the sessions as they actually sit on disk. The manifests under
`sessions/` are the shipped product of this repo -- what `just demo` runs and
what `lctk_session new` is copied from -- so a manifest that no longer parses
is a shipping defect, not a test fixture problem.
"""

from pathlib import Path

import pytest
import yaml
from lctk_launch.config_parser import parse_config
from lctk_launch.session import MANIFEST_NAME, parse_data

SESSIONS = Path(__file__).resolve().parents[3] / "sessions"

NAMES = [
    "sample1",
    "sample2",
    "sample3-hollow-velodyne",
    "sample4",
    "sample5",
    "seyond-left",
    "seyond-right",
    "solid600-handheld-seyond",
    "solid600-handheld-zed",
    "twolidar-vlp32-falcon",
    "vehicle-multisensor",
    "vlp32-zed-hollow",
]

# The four sampleN sessions besides dataset 3 have never been run: their target,
# detector preset, frames and sync window are all copied from sample3 and none of
# them is verified against the recording. They are bbox-free on purpose -- a crop
# box is per-recording geometry, and inventing one is how M-29 silenced the demo.
UNVERIFIED_SAMPLES = ["sample1", "sample2", "sample4", "sample5"]

# The TWO_LIDAR_* recordings are gitignored (~2.4 GB), so this session's bag is
# a symlink an operator places by hand. A `kind: bag` manifest is verified
# against its recording's metadata.yaml at parse time -- that verification is
# the entire point of M-26 -- so where the bag is absent the session genuinely
# cannot be parsed, and skipping is the honest outcome rather than weakening
# the check to make it pass everywhere.
# The same holds for solid600-handheld-zed, whose recording is a field capture
# that has never been small enough to ship.
BAG_SESSIONS = [
    # Both solid600-* sessions are `kind: bag` against the same gitignored
    # field capture. solid600-handheld-zed was missing from this list, so on
    # any machine without the symlink its parse test failed rather than
    # skipping -- it only passed here because the symlink happened to exist.
    "solid600-handheld-seyond",
    "solid600-handheld-zed",
    "twolidar-vlp32-falcon",
    "vlp32-zed-hollow",
]


def _needs_bag(name: str):
    bag = SESSIONS / name / "bag"
    return pytest.mark.skipif(
        not bag.is_dir(),
        reason=(
            f"no recording at {bag}; see {SESSIONS / name / 'README.md'} for "
            "where to obtain it and symlink it there"
        ),
    )


def _manifest(name: str) -> str:
    return str(SESSIONS / name / MANIFEST_NAME)


def test_the_shipped_session_list_is_exactly_what_is_on_disk():
    """A session added or renamed without updating NAMES would otherwise leave
    the parametrization below silently covering the old set."""
    on_disk = sorted(path.parent.name for path in SESSIONS.glob(f"*/{MANIFEST_NAME}"))
    assert on_disk == sorted(NAMES)


@pytest.mark.parametrize(
    "name",
    [
        pytest.param(name, marks=[_needs_bag(name)] if name in BAG_SESSIONS else [])
        for name in NAMES
    ],
)
def test_every_shipped_session_parses(name):
    parse_config(_manifest(name))


def test_the_sample_session_keeps_the_topics_the_playback_publishes():
    """The migration must not move dataset 3's topics; the playback defaults
    and the calibration graph have to keep meeting at the same names.

    This is why the lidar device is named `top` and not `top_lidar`: under
    `kind: pcap_avi` the topic is derived from the device name, and `top` is
    what reproduces lidar_camera.launch.xml's long-standing defaults.
    """
    pipeline = parse_config(_manifest("sample3-hollow-velodyne"))
    assert (
        pipeline.lidars["top"].pointcloud_topic == "/sensing/lidar/top/pointcloud_raw"
    )
    assert (
        pipeline.cameras["front_center"].image_topic
        == "/sensing/camera/front_center/image_raw"
    )


def test_the_sample_session_carries_its_own_crop_box():
    """The bbox-mode preset needs a crop box, and it must be the session's own.

    The shared config/board/bbox.json5 this used to be adjacent to had been
    retuned for a Seyond rosbag, which put the box where dataset 3's board
    never is; the detector then found zero points in it on every frame and
    published nothing (M-29). A session-local box cannot be retuned by another
    rig's operator without noticing whose file they are editing.
    """
    pipeline = parse_config(_manifest("sample3-hollow-velodyne"))
    bbox = Path(pipeline.lidar_board_detectors[0].bbox_config)
    assert bbox == SESSIONS / "sample3-hollow-velodyne" / "bbox.json5"
    assert bbox.is_file()


@_needs_bag("twolidar-vlp32-falcon")
def test_the_two_lidar_session_names_the_topics_the_bag_records():
    """M-26: the old config named /velodyne_points; the bag records
    /lidar/vlp32/velodyne_points. Parsing verifies against metadata.yaml, so this
    test failing means the manifest is wrong, not the test."""
    pipeline = parse_config(_manifest("twolidar-vlp32-falcon"))
    topics = {lidar.pointcloud_topic for lidar in pipeline.lidars.values()}
    assert topics == {"/lidar/vlp32/velodyne_points", "/lidar/falcon/iv_points"}


@_needs_bag("twolidar-vlp32-falcon")
def test_the_two_lidar_session_keeps_its_per_device_detector_override():
    """The per-LiDAR detector_config override is the feature this session exists
    to demonstrate: a spinning VLP-32C and a solid-state Falcon share one target
    while each keeps its own sensor tuning. Losing it in the migration would
    leave both LiDARs on the Velodyne preset and look like a tuning problem."""
    pipeline = parse_config(_manifest("twolidar-vlp32-falcon"))
    presets = {
        detector.lidar_name: Path(detector.detector_config).name
        for detector in pipeline.lidar_board_detectors
    }
    assert presets == {"top_lidar": "velodyne.json5", "front_lidar": "seyond.json5"}


def test_the_solid_session_keeps_its_tighter_sync_window():
    """50 ms, not the 100 ms every hollow session uses: the solid board is
    hand-held and moving, so a mis-paired frame is wrong rather than merely
    noisy. A migration that quietly normalised this to 100 should fail here."""
    pipeline = parse_config(_manifest("solid600-handheld-zed"))
    assert pipeline.sync.tolerance_ms == 50.0


def test_the_right_camera_session_names_its_camera_right():
    """The example this replaces called the device `left_camera` while giving it
    the right camera's topic and frame -- a copy-paste from the left example.
    The device name reaches generated node names and namespaces, so the wrong
    one made a right-camera calibration report itself as left."""
    pipeline = parse_config(_manifest("seyond-right"))
    assert list(pipeline.cameras) == ["right_camera"]
    camera = pipeline.cameras["right_camera"]
    assert camera.image_topic == "/camera/right/image_raw"
    assert camera.frame_id == "camera_right"


def test_every_shipped_session_documents_itself():
    """A session is meant to be read before it is run -- especially the four
    whose data does not ship, where the README is the only place that says so."""
    for name in NAMES:
        readme = SESSIONS / name / "README.md"
        assert readme.is_file(), f"{name} has no README.md"
        assert readme.read_text(encoding="utf-8").strip()


@pytest.mark.parametrize("name", UNVERIFIED_SAMPLES)
def test_every_sample_session_owns_its_crop_box_and_intrinsics(name):
    """Self-contained means the files live here, not in a sibling session.

    These four originally shipped bbox-free with no crop box, on the reasoning
    that borrowing another recording's box is what silenced the demo in M-29.
    Run that way they detected nothing -- background subtraction absorbed a board
    that barely moves in these recordings. They now carry their own copies, which
    were verified to detect on 2026-09-01. Naming sample3's file by path would
    reintroduce exactly the sharing M-29 was about.
    """
    session_dir = SESSIONS / name
    assert (session_dir / "bbox.json5").is_file()
    assert (session_dir / "camera_info.yaml").is_file()

    pipeline = parse_config(_manifest(name))
    for detector in pipeline.lidar_board_detectors:
        assert detector.bbox_config is not None, f"{name} needs a crop box to detect"
        assert Path(detector.bbox_config).parent == session_dir, (
            f"{name} points at a crop box outside its own directory: "
            f"{detector.bbox_config}"
        )
        assert Path(detector.detector_config).name == "velodyne_bbox.json5", (
            "the bbox-free preset finds nothing on these recordings"
        )


@pytest.mark.parametrize("name", UNVERIFIED_SAMPLES)
def test_every_sample_session_records_that_it_was_verified(name):
    """The README is where a reader learns whether these values were checked.

    They began as assumptions and are now measured; the file has to say which,
    because the manifest alone looks identical either way.
    """
    text = (SESSIONS / name / "README.md").read_text(encoding="utf-8").lower()
    assert "verified" in text
    assert "zero detector rejections" in text
    assert "still not known" in text, (
        "the README must keep saying what remains unchecked -- the extrinsic "
        "itself has not been validated against the physical rig"
    )


@pytest.mark.parametrize("name", ["sample3-hollow-velodyne"] + UNVERIFIED_SAMPLES)
def test_every_sample_session_owns_its_recording(name):
    """A session is self-contained: its data sits inside the session directory.

    The recordings used to live in lctk_sample_data and be reached with
    $(find-pkg-share lctk_sample_data)/data/<N>, which meant copying a session
    elsewhere left its data behind.
    """
    directory = SESSIONS / name
    manifest = yaml.safe_load((directory / MANIFEST_NAME).read_text(encoding="utf-8"))
    assert manifest["data"]["dir"] == "$(session-dir)/data"
    source = parse_data(manifest["data"], directory)
    assert source.directory == directory / "data"
    assert (source.directory / "lidar.pcap").is_file()
    assert (source.directory / "video.avi").is_file()
