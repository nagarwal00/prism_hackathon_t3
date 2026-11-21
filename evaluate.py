"""
evaluate.py - Trajectory Evaluation and Scoring Harness for XR Hackathon Track 3

Computes trajectory accuracy metrics matching the official competition protocol:
  - SE(3) Umeyama alignment (rotation & translation, rigid body transform)
  - Absolute Trajectory Error (ATE RMSE) in meters
  - Relative Pose Error (RPE)
  - Detailed regime decomposition:
      1. Normal flight / checkpoint ATE (Tier 1: 30%)
      2. Blind tail 5-second failure window ATE (Tier 2: 20%)
      3. Dropout / timestamp jump ATE (Tier 3: 10%)
  - Checkpoint / waypoint position error analysis
  - Interfacing with official `evo_ape tum ref.txt est.txt -a`
"""

import sys
import argparse
import subprocess
import numpy as np


def load_tum_trajectory(filepath):
    """
    Loads space-separated TUM trajectory file:
    timestamp_sec tx ty tz qx qy qz qw
    Returns:
      timestamps: (N,) float64
      positions: (N, 3) float64
      quaternions: (N, 4) float64 [qx, qy, qz, qw]
    """
    ts_list = []
    pos_list = []
    quat_list = []

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            ts_list.append(ts)
            pos_list.append([tx, ty, tz])
            quat_list.append([qx, qy, qz, qw])

    return (
        np.array(ts_list, dtype=np.float64),
        np.array(pos_list, dtype=np.float64),
        np.array(quat_list, dtype=np.float64)
    )


def umeyama_alignment_se3(model_pts, target_pts):
    """
    Closed-form Umeyama algorithm for SE(3) rigid alignment (scale = 1.0):
      Finds R in SO(3) and t in R^3 such that target ~= R * model + t
    Returns:
      R: (3, 3) rotation matrix
      t: (3,) translation vector
    """
    assert model_pts.shape == target_pts.shape
    n = model_pts.shape[0]

    mu_m = np.mean(model_pts, axis=0)
    mu_t = np.mean(target_pts, axis=0)

    mc = model_pts - mu_m
    tc = target_pts - mu_t

    # Covariance matrix H = mc^T * tc / n
    H = (mc.T @ tc) / n

    U, D, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure right-handed coordinate system (det(R) == +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = mu_t - R @ mu_m
    return R, t


def compute_ate_and_regimes(
    gt_file: str,
    est_file: str,
    blind_duration_sec: float = 5.0,
    checkpoints_file: str = None
):
    """
    Performs SE(3) Umeyama alignment, computes overall ATE, RPE,
    and breaks down errors into normal, blind tail, and dropout regimes.
    """
    t_gt, pos_gt, quat_gt = load_tum_trajectory(gt_file)
    t_est, pos_est, quat_est = load_tum_trajectory(est_file)

    if len(t_gt) == 0 or len(t_est) == 0:
        raise ValueError("Ground truth or estimated trajectory is empty!")

    # Associate poses by timestamp interpolation
    # If timestamps match exactly:
    if len(t_gt) == len(t_est) and np.allclose(t_gt, t_est, atol=1e-5):
        matched_gt_pos = pos_gt
        matched_est_pos = pos_est
        matched_t = t_est
    else:
        # Interpolate ground truth at estimated timestamps
        matched_t = t_est
        matched_est_pos = pos_est
        matched_gt_pos = np.zeros_like(pos_est)
        for d in range(3):
            matched_gt_pos[:, d] = np.interp(t_est, t_gt, pos_gt[:, d])

    # SE(3) Umeyama Alignment (matching evo -a)
    R_align, t_align = umeyama_alignment_se3(matched_est_pos, matched_gt_pos)
    aligned_est_pos = (R_align @ matched_est_pos.T).T + t_align

    # Errors
    err_vectors = matched_gt_pos - aligned_est_pos
    err_magnitudes = np.linalg.norm(err_vectors, axis=1)

    ate_rmse = float(np.sqrt(np.mean(err_magnitudes ** 2)))
    ate_mean = float(np.mean(err_magnitudes))
    ate_median = float(np.median(err_magnitudes))
    ate_max = float(np.max(err_magnitudes))
    ate_std = float(np.std(err_magnitudes))

    # Relative Pose Error (RPE) over 1-second intervals
    dt_target = 1.0
    rpe_errors = []
    n = len(matched_t)
    step = max(1, int(round(dt_target / max(1e-3, np.median(np.diff(matched_t))))))
    for i in range(0, n - step, step):
        delta_gt = matched_gt_pos[i + step] - matched_gt_pos[i]
        delta_est = aligned_est_pos[i + step] - aligned_est_pos[i]
        rpe_errors.append(np.linalg.norm(delta_gt - delta_est))

    rpe_rmse = float(np.sqrt(np.mean(np.array(rpe_errors) ** 2))) if rpe_errors else 0.0

    # Regime Separation
    total_time = matched_t[-1] - matched_t[0]
    tail_start_t = matched_t[-1] - blind_duration_sec

    tail_mask = matched_t >= tail_start_t
    normal_mask = ~tail_mask

    # Tail ATE
    if np.any(tail_mask):
        tail_ate_rmse = float(np.sqrt(np.mean(err_magnitudes[tail_mask] ** 2)))
        tail_ate_mean = float(np.mean(err_magnitudes[tail_mask]))
    else:
        tail_ate_rmse = 0.0
        tail_ate_mean = 0.0

    # Normal / Checkpoint ATE
    if np.any(normal_mask):
        normal_ate_rmse = float(np.sqrt(np.mean(err_magnitudes[normal_mask] ** 2)))
        normal_ate_mean = float(np.mean(err_magnitudes[normal_mask]))
    else:
        normal_ate_rmse = 0.0
        normal_ate_mean = 0.0

    # Dropout / Gap analysis: identify where dt > 1.5 * nominal
    dts = np.diff(matched_t)
    med_dt = np.median(dts)
    gap_indices = np.where(dts > 2.0 * med_dt)[0]
    if len(gap_indices) > 0:
        gap_errs = err_magnitudes[gap_indices + 1]
        gap_ate_rmse = float(np.sqrt(np.mean(gap_errs ** 2)))
    else:
        gap_ate_rmse = 0.0

    # Checkpoint Evaluation (if provided)
    checkpoint_errors = []
    if checkpoints_file:
        try:
            import pandas as pd
            cp_df = pd.read_csv(checkpoints_file)
            for _, row in cp_df.iterrows():
                cp_t = row["timestamp_sec"]
                cp_gt = np.array([row["x"], row["y"], row["z"]])
                # Nearest estimated pose
                idx = np.argmin(np.abs(matched_t - cp_t))
                err = np.linalg.norm(aligned_est_pos[idx] - cp_gt)
                checkpoint_errors.append(err)
        except Exception as e:
            print(f"[Warning] Could not evaluate checkpoints: {e}")

    mean_cp_error = float(np.mean(checkpoint_errors)) if checkpoint_errors else normal_ate_mean

    return {
        "ate_rmse": ate_rmse,
        "ate_mean": ate_mean,
        "ate_median": ate_median,
        "ate_max": ate_max,
        "ate_std": ate_std,
        "rpe_rmse": rpe_rmse,
        "normal_ate_rmse": normal_ate_rmse,
        "tail_ate_rmse": tail_ate_rmse,
        "gap_ate_rmse": gap_ate_rmse,
        "mean_checkpoint_error": mean_cp_error,
        "num_frames": n,
        "duration_sec": total_time,
        "tail_frames": int(np.sum(tail_mask)),
        "gap_count": len(gap_indices)
    }


def run_official_evo(ref_file: str, est_file: str):
    """
    Executes official `evo_ape tum ref est -a` and parses result.
    """
    cmd = ["evo_ape", "tum", ref_file, est_file, "-a", "--no_warnings"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return proc.stdout
    except Exception as e:
        return f"[evo_ape run note]: {e}"


def main():
    parser = argparse.ArgumentParser(description="Evaluate estimated trajectory against ground truth.")
    parser.add_argument("ground_truth", help="Path to ground truth trajectory (TUM format).")
    parser.add_argument("estimated", help="Path to estimated trajectory (TUM format).")
    parser.add_argument("--checkpoints", default=None, help="Path to checkpoints CSV.")
    parser.add_argument("--blind-tail", type=float, default=5.0, help="Blind tail duration in seconds.")
    parser.add_argument("--run-evo", action="store_true", help="Also run official evo_ape CLI.")

    args = parser.parse_args()

    results = compute_ate_and_regimes(
        args.ground_truth,
        args.estimated,
        blind_duration_sec=args.blind_tail,
        checkpoints_file=args.checkpoints
    )

    print("=" * 65)
    print("           XR HACKATHON 2026 - TRACK 3 EVALUATION")
    print("=" * 65)
    print(f"Total Trajectory Duration:  {results['duration_sec']:.2f} s ({results['num_frames']} frames)")
    print(f"Overall ATE RMSE:          {results['ate_rmse']:.4f} m")
    print(f"Overall ATE Mean:          {results['ate_mean']:.4f} m (Median: {results['ate_median']:.4f} m)")
    print(f"Overall ATE Max:           {results['ate_max']:.4f} m (Std: {results['ate_std']:.4f} m)")
    print(f"Overall RPE (1s interval): {results['rpe_rmse']:.4f} m")
    print("-" * 65)
    print(f"Tier 1: Waypoint / Normal: {results['normal_ate_rmse']:.4f} m RMSE")
    print(f"Tier 2: Blind Tail (5 s):  {results['tail_ate_rmse']:.4f} m RMSE ({results['tail_frames']} frames)")
    print(f"Tier 3: Dropout / Gaps:    {results['gap_ate_rmse']:.4f} m RMSE ({results['gap_count']} gaps)")
    if args.checkpoints:
        print(f"Checkpoint Mean Error:     {results['mean_checkpoint_error']:.4f} m")
    print("=" * 65)

    if args.run_evo:
        print("\n--- Official evo_ape output ---")
        evo_out = run_official_evo(args.ground_truth, args.estimated)
        print(evo_out)


if __name__ == "__main__":
    main()
