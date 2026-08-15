"""M-17: a reference for the wrong rig must announce itself, not score 0/15 in silence.

The shipped ground truth describes a different rig than the sample data: its baseline is 0.213 m
where the solve gives 0.889 m. `just demo enable_judge=true` therefore reports 0.0/15.0 no matter
how good the calibration is -- the same shape as C-04's unreachable ICP gate, and it teaches
operators to ignore the one instrument that reports quality.

The load-bearing observation is that **||t|| is invariant under inversion**. A direction mistake
-- the M-01 class of error -- cannot change the baseline magnitude. So a large gap in ||t|| is
positive evidence of different *geometry*, not of a convention mix-up, and it is safe to say so
without risking a confident wrong diagnosis.
"""

import numpy as np

from calibration_judge.judge_node import CalibrationJudgeNode as J

MAX_ERROR_M = 0.10


def transform(translation, rot=None):
    T = np.eye(4)
    T[:3, :3] = rot if rot is not None else np.eye(3)
    T[:3, 3] = translation
    return T


def test_flags_a_reference_from_a_different_rig():
    """The real case: 0.213 m reference against a 0.889 m solve."""
    gt = transform([0.151103, -0.021758, -0.147006])
    est = transform([0.887778, -0.028121, -0.038840])

    msg = J.check_reference_plausibility(gt, est, MAX_ERROR_M)

    assert msg is not None, "a 0.68 m baseline gap was not flagged"
    assert "0.21" in msg and "0.89" in msg, f"message should name both baselines: {msg}"


def test_silent_when_the_reference_matches():
    """A merely-imperfect calibration must not be accused of using the wrong reference."""
    gt = transform([0.15, -0.02, -0.15])
    est = transform([0.17, -0.01, -0.16])

    assert J.check_reference_plausibility(gt, est, MAX_ERROR_M) is None


def test_does_not_fire_on_a_pure_direction_error():
    """The M-01 failure mode must NOT be reported as a wrong rig.

    Inverting a transform preserves ||t||, so a reference recorded in the opposite convention has
    exactly the same baseline. If this check fired there it would send the reader hunting for a
    hardware discrepancy that does not exist.
    """
    rot = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    est = transform([0.07, -0.02, -0.89], rot)
    inverted = np.linalg.inv(est)

    assert np.isclose(np.linalg.norm(est[:3, 3]), np.linalg.norm(inverted[:3, 3])), (
        "premise: inversion preserves the baseline magnitude"
    )
    assert J.check_reference_plausibility(inverted, est, MAX_ERROR_M) is None


def test_stays_quiet_just_below_the_threshold():
    """Right at the edge, prefer silence: a false accusation is worse than none."""
    gt = transform([0.20, 0.0, 0.0])
    est = transform([0.20 + 2.9 * MAX_ERROR_M, 0.0, 0.0])

    assert J.check_reference_plausibility(gt, est, MAX_ERROR_M) is None


def test_scales_with_the_configured_tolerance():
    """A rig with loose thresholds should tolerate a proportionally larger gap."""
    gt = transform([0.20, 0.0, 0.0])
    est = transform([0.20 + 0.5, 0.0, 0.0])

    assert J.check_reference_plausibility(gt, est, 0.10) is not None
    assert J.check_reference_plausibility(gt, est, 1.00) is None
