"""Real-pcap regression floor for the Method E operating point.

Env-gated: skips (with a reason) only when the sample pcaps or
velodyne_decoder are absent. When data is present it MUST run — it is the
only automated guard on the finalize headline number.
"""
import pytest

pytestmark = pytest.mark.realdata


def _have_pcaps():
    try:
        from boarddet.ingest import DATA_DIR
    except Exception:
        return False
    return all((DATA_DIR / n / "lidar.pcap").exists() for n in "12345")


@pytest.mark.skipif(not _have_pcaps(),
                    reason="sample pcaps ros/lctk_sample_data/data/{1..5} absent")
def test_methode_loo_recall_floor(tmp_path):
    pytest.importorskip("velodyne_decoder",
                        reason="velodyne_decoder not installed")
    from boarddet.benchmark_e_loo import DEFAULT_BBOX_PATH, load_sources, run_loo
    from boarddet.bbox_ref import load_bbox
    from boarddet.presets import production_config

    sources = load_sources("pcap", ["1", "2", "3", "4", "5"],
                           sensor="vlp32", max_frames=40)
    board = production_config()
    summary = run_loo(sources, board, tmp_path, box=load_bbox(DEFAULT_BBOX_PATH),
                      min_sources=3)

    folds = summary["folds"]
    recalls = {k: v["recall"] for k, v in folds.items()}
    # No fold may collapse to near-zero (the ds5-overlap failure mode).
    assert min(recalls.values()) >= 0.35, recalls
    # Pooled recall over all frames must hold the operating-point level.
    total_true = sum(v["n_true_board"] for v in folds.values())
    total_frames = sum(v["n_frames"] for v in folds.values())
    assert total_true / total_frames >= 0.80, recalls
    # Precision: accepted detections should overwhelmingly be true-board.
    total_dets = sum(v["n_detections"] for v in folds.values())
    assert total_dets > 0
    assert total_true / total_dets >= 0.95, {
        "true": total_true, "dets": total_dets}
