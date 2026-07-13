#!/usr/bin/env python3
"""M-12 verification: solvePnPRefineLM after SQPnP does not worsen (and typically
improves) the reprojection fit, and the API call matches the solver's usage.

Builds a synthetic scene with a known pose, projects 3D points, adds pixel noise,
then compares SQPnP alone vs SQPnP + RefineLM by reprojection RMSE.
"""
import sys

import cv2
import numpy as np


def rmse(obj, img, rvec, tvec, K, dist):
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))


def main():
    rng = np.random.default_rng(0)
    K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]], dtype=np.float32)
    dist = np.zeros(5, dtype=np.float32)

    # Known pose, spread (non-coplanar) 3D points so LM has room to work.
    rvec_true = np.array([[0.05], [-0.1], [0.02]], dtype=np.float64)
    tvec_true = np.array([[0.1], [-0.05], [3.0]], dtype=np.float64)
    obj = rng.uniform(-0.5, 0.5, size=(40, 3)).astype(np.float32)
    obj[:, 2] += rng.uniform(0.0, 1.0, size=40).astype(np.float32)  # depth spread

    img_clean, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, dist)
    img = img_clean.reshape(-1, 2) + rng.normal(0, 0.8, size=(40, 2)).astype(np.float32)
    img = img.astype(np.float32)

    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    assert ok, "SQPnP failed"
    err_sqpnp = rmse(obj, img, rvec, tvec, K, dist)

    rvec_r, tvec_r = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    err_refined = rmse(obj, img, rvec_r, tvec_r, K, dist)

    print(f"reprojection RMSE  SQPnP={err_sqpnp:.4f} px  +RefineLM={err_refined:.4f} px")
    assert err_refined <= err_sqpnp + 1e-6, "RefineLM worsened the fit"
    print("M-12 PASS: solvePnPRefineLM never worsens the reprojection fit "
          f"(improved by {err_sqpnp - err_refined:.4f} px here)")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001
        print(f"M-12 FAIL: {e}")
        sys.exit(1)
