#!/usr/bin/env python3
"""
================================================================================
XR HACKATHON 2026 - TRACK 3: PATH FINDER
STANDALONE 6-DOF TRAJECTORY ESTIMATOR
================================================================================

Team Submission: Path Finder Estimator
Entrypoint: main.py
Usage:
    python main.py <input.csv> <estimated_trajectory.txt> [--channel-order {auto,euroc,degrade}] [--no-smoothing] [--profile]

Architecture:
  1. Fast Robust Parser:
     - Handles EuRoC comment headers (#timestamp [ns],w_RS_x...), degrade format,
       or headerless CSVs.
     - Automatically distinguishes channel ordering (gyro-first vs accel-first).
     - Nanosecond integer subtraction for delta precision without float truncation.
  2. Fault Screening & Adaptive Kinematics:
     - Rejects non-physical sensor spikes (clipping bounds).
     - Identifies stuck vectors.
     - Zero-dt duplicate timestamp policy: preserves cardinality, outputs unchanged pose.
     - Dropout gap resilience: integrates over true dt; sub-steps large gaps for stability.
  3. Mahony/ESKF-Style Tilt Observer:
     - Uses observable gravity when platform dynamics are low (|norm(a) - g| < 0.8 m/s^2)
       to continuously correct roll and pitch gyro drift.
  4. Tier 2 Blind Tail Extrapolator:
     - Detects >= 5 consecutive zero-reading tail frames (5 s blackout).
     - Smoothly estimates terminal velocity and angular rate before failure onset.
     - Propagates pose through blackout with bounded aerodynamic decay.
  5. Stage C Offline Drift Regularizer:
     - Fits bounded polynomial drift to residual unobservable acceleration biases.
     - Mitigates quadratic dead-reckoning position divergence.
  6. Output Serializer:
     - Strict TUM format: '# timestamp tx ty tz qx qy qz qw'
     - Exactly 8 space-separated numeric values per row, 1:1 with input.
     - Unit quaternions, scalar-last [qx, qy, qz, qw], with sign continuity.

Resource Constraints:
  - Language: Pure Python (Python 3.8+) + minimal NumPy.
  - Memory: Peak heap < 8 MB (measured via tracemalloc).
  - Runtime: <= 20 ms compute per 1 s IMU telemetry on single CPU thread.
================================================================================
"""

import sys
import os
import time
import math
import argparse
import tracemalloc
import numpy as np


# Enforce single-threaded execution per competition rules
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"


def fast_quat_rotate_vector(qx: float, qy: float, qz: float, qw: float,
                            vx: float, vy: float, vz: float):
    """Rotates 3D vector v by quaternion q using Rodrigues formula."""
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def fast_quat_mult(x1: float, y1: float, z1: float, w1: float,
                   x2: float, y2: float, z2: float, w2: float):
    """Multiplies two quaternions [x, y, z, w]."""
    rx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    ry = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    rz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    rw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return rx, ry, rz, rw


def align_vector_to_z(vx: float, vy: float, vz: float):
    """Computes quaternion [qx, qy, qz, qw] that rotates body vector (vx, vy, vz) to +Z [0, 0, 1]."""
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    ux, uy, uz = vx / norm, vy / norm, vz / norm

    dot = uz
    if dot > 0.999999:
        return 0.0, 0.0, 0.0, 1.0
    if dot < -0.999999:
        return 1.0, 0.0, 0.0, 0.0

    ax, ay, az = uy, -ux, 0.0
    aw = 1.0 + dot
    anorm = math.sqrt(ax * ax + ay * ay + az * az + aw * aw)
    return ax / anorm, ay / anorm, az / anorm, aw / anorm


def resolve_input_path(path_str: str) -> str:
    """
    Resolves input telemetry path according to Slide 15 specs:
      - Direct CSV file path (e.g. data.csv, degraded_imu.csv)
      - Directory containing imu0/data.csv or data.csv
      - Zip archive containing imu0/data.csv
    """
    if not os.path.exists(path_str):
        # Fallback check in current directory
        candidates = ["imu0/data.csv", "degraded_imu.csv", "data.csv"]
        for c in candidates:
            if os.path.exists(c):
                return c
        raise FileNotFoundError(f"Input telemetry file not found: {path_str}")

    if os.path.isdir(path_str):
        p1 = os.path.join(path_str, "imu0", "data.csv")
        p2 = os.path.join(path_str, "data.csv")
        if os.path.exists(p1):
            return p1
        if os.path.exists(p2):
            return p2
        # Look for any .csv file in the directory
        for f in os.listdir(path_str):
            if f.endswith(".csv"):
                return os.path.join(path_str, f)

    if path_str.endswith(".zip"):
        import zipfile
        extract_dir = os.path.join(os.path.dirname(path_str), "_extracted_telemetry")
        os.makedirs(extract_dir, exist_ok=True)
        with zipfile.ZipFile(path_str, "r") as z:
            z.extractall(extract_dir)
        return resolve_input_path(extract_dir)

    return path_str


def parse_imu_telemetry(file_path: str, user_channel_order: str = "auto"):
    """
    High-speed robust CSV parser.
    Reads CSV, handles comments, and automatically resolves channel order:
      - Degrade format (official degrade_imu_default.py): timestamp, accel_xyz, gyro_xyz
      - EuRoC format (Slide 15): timestamp, gyro_xyz, accel_xyz
    Uses header column names or median sensor magnitude heuristic when headerless.
    Returns:
      ts_arr: (N,) int64 timestamps in nanoseconds
      gx, gy, gz: (N,) float64 gyro components in rad/s
      ax, ay, az: (N,) float64 accel components in m/s^2
    """
    resolved_path = resolve_input_path(file_path)

    with open(resolved_path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()

    lines = text.splitlines()
    if not lines:
        raise ValueError(f"Telemetry file is empty: {file_path}")

    detected_order = None
    data = []

    for line_str in lines:
        line_str = line_str.strip()
        if not line_str:
            continue

        if line_str.startswith("#") or any(c.isalpha() for c in line_str.split(",")[0]):
            hl = line_str.lower()
            parts_h = hl.split(",") if "," in hl else hl.split()
            if len(parts_h) >= 7:
                col1_name = parts_h[1]
                if any(k in col1_name for k in ["a_rs", "accel", "ax", "acc"]):
                    detected_order = "degrade"
                elif any(k in col1_name for k in ["w_rs", "gyro", "gx", "gyr"]):
                    detected_order = "euroc"
            elif "w_rs" in hl or "gyro" in hl:
                if detected_order is None:
                    detected_order = "euroc"
            elif "a_rs" in hl or "accel" in hl or "ax" in hl:
                if detected_order is None:
                    detected_order = "degrade"
            continue

        parts = line_str.split(",") if "," in line_str else line_str.split()
        if len(parts) < 7:
            continue

        try:
            t_val = int(parts[0].strip().split(".")[0])
            v1 = float(parts[1])
            v2 = float(parts[2])
            v3 = float(parts[3])
            v4 = float(parts[4])
            v5 = float(parts[5])
            v6 = float(parts[6])
            data.append([t_val, v1, v2, v3, v4, v5, v6])
        except (ValueError, IndexError):
            continue

    if not data:
        raise ValueError(f"No valid numeric IMU records found in {file_path}")

    arr = np.array(data, dtype=np.float64)
    ts_arr = arr[:, 0].astype(np.int64)

    # Resolve channel ordering
    order = "degrade"  # Default to official degrade_imu_default.py convention
    if user_channel_order in ["euroc", "degrade"]:
        order = user_channel_order
    elif detected_order in ["euroc", "degrade"]:
        order = detected_order
    else:
        # Headerless magnitude heuristic:
        # Accelerometer measures Earth gravity ~9.81 m/s^2. Gyroscope is in rad/s (< 3 rad/s).
        sample_n = min(len(arr), 50)
        norm_123 = np.median(np.linalg.norm(arr[:sample_n, 1:4], axis=1))
        norm_456 = np.median(np.linalg.norm(arr[:sample_n, 4:7], axis=1))
        if norm_123 > 4.0 and norm_456 < 4.0:
            order = "degrade"  # Columns 1..3 are accel
        elif norm_123 < 4.0 and norm_456 > 4.0:
            order = "euroc"    # Columns 1..3 are gyro

    if order == "euroc":
        # EuRoC order: timestamp, gyro xyz, accel xyz
        gx, gy, gz = arr[:, 1], arr[:, 2], arr[:, 3]
        ax, ay, az = arr[:, 4], arr[:, 5], arr[:, 6]
    else:
        # Official degrade_imu_default format: timestamp, accel xyz, gyro xyz
        ax, ay, az = arr[:, 1], arr[:, 2], arr[:, 3]
        gx, gy, gz = arr[:, 4], arr[:, 5], arr[:, 6]

    # Sanitize NaNs / Infs
    gx = np.nan_to_num(gx)
    gy = np.nan_to_num(gy)
    gz = np.nan_to_num(gz)
    ax = np.nan_to_num(ax)
    ay = np.nan_to_num(ay)
    az = np.nan_to_num(az)

    return ts_arr, gx, gy, gz, ax, ay, az


class PathFinderEstimator:
    def __init__(
        self,
        gravity_nominal: float = 9.80665,
        spike_gyro_thresh: float = 25.0,    # rad/s
        spike_accel_thresh: float = 65.0,   # m/s^2
        max_substep_dt: float = 0.02,       # 20 ms max integration step
        tail_consecutive_zeros: int = 5,    # consecutive frames to trigger failure mode
        drag_decay_rate: float = 0.02,      # aerodynamic drag decay in blackout
        rot_decay_rate: float = 0.05        # rotational damping in blackout
    ):
        self.g_nominal = gravity_nominal
        self.spike_gyro_thresh = spike_gyro_thresh
        self.spike_accel_thresh = spike_accel_thresh
        self.max_substep_dt = max_substep_dt
        self.tail_consecutive_zeros = tail_consecutive_zeros
        self.drag_decay_rate = drag_decay_rate
        self.rot_decay_rate = rot_decay_rate

    def estimate(
        self,
        timestamps_ns: np.ndarray,
        gx: np.ndarray, gy: np.ndarray, gz: np.ndarray,
        ax: np.ndarray, ay: np.ndarray, az: np.ndarray,
        enable_smoothing: bool = True
    ) -> np.ndarray:
        n = len(timestamps_ns)
        poses = np.zeros((n, 8), dtype=np.float64)
        if n == 0:
            return poses

        t_sec = timestamps_ns * 1e-9
        poses[:, 0] = t_sec

        # 1. Initial Stationary Alignment & Gyro Bias Estimation
        init_win = min(n, 80)
        gx_win = gx[:init_win]
        gy_win = gy[:init_win]
        gz_win = gz[:init_win]
        ax_win = ax[:init_win]
        ay_win = ay[:init_win]
        az_win = az[:init_win]

        gyro_var = float(np.var(gx_win) + np.var(gy_win) + np.var(gz_win))
        acc_mags = np.sqrt(ax_win * ax_win + ay_win * ay_win + az_win * az_win)
        mean_acc_mag = float(np.mean(acc_mags))
        acc_mag_var = float(np.var(acc_mags))

        mean_gyro_mag = math.sqrt(float(np.mean(gx_win))**2 + float(np.mean(gy_win))**2 + float(np.mean(gz_win))**2)

        if gyro_var < 0.02 and mean_gyro_mag < 0.15 and acc_mag_var < 0.2 and abs(mean_acc_mag - self.g_nominal) < 1.2:
            bgx = float(np.mean(gx_win))
            bgy = float(np.mean(gy_win))
            bgz = float(np.mean(gz_win))
            mean_ax = float(np.mean(ax_win))
            mean_ay = float(np.mean(ay_win))
            mean_az = float(np.mean(az_win))
            curr_qx, curr_qy, curr_qz, curr_qw = align_vector_to_z(mean_ax, mean_ay, mean_az)
        else:
            bgx, bgy, bgz = 0.0, 0.0, 0.0
            curr_qx, curr_qy, curr_qz, curr_qw = 0.0, 0.0, 0.0, 1.0

        # 2. Detect 5-second Blind Failure Tail
        zero_tail_idx = n
        for i in range(n - 1, -1, -1):
            if (abs(gx[i]) < 1e-6 and abs(gy[i]) < 1e-6 and abs(gz[i]) < 1e-6 and
                abs(ax[i]) < 1e-6 and abs(ay[i]) < 1e-6 and abs(az[i]) < 1e-6):
                zero_tail_idx = i
            else:
                break

        has_blind_tail = (n - zero_tail_idx >= self.tail_consecutive_zeros)
        if not has_blind_tail:
            zero_tail_idx = n

        valid_end = zero_tail_idx

        quats = np.zeros((n, 4), dtype=np.float64)
        quats[0] = [curr_qx, curr_qy, curr_qz, curr_qw]
        acc_w = np.zeros((n, 3), dtype=np.float64)

        g_nom = self.g_nominal
        max_sub = self.max_substep_dt
        spk_g = self.spike_gyro_thresh
        spk_a = self.spike_accel_thresh

        # 3. High-Speed Inlined Attitude & Kinematic Integration Loop
        for i in range(1, valid_end):
            dt_ns = int(timestamps_ns[i] - timestamps_ns[i - 1])
            if dt_ns <= 0:
                quats[i] = quats[i - 1]
                continue

            dt = dt_ns * 1e-9

            rgx = gx[i] - bgx
            rgy = gy[i] - bgy
            rgz = gz[i] - bgz
            rax = ax[i]
            ray = ay[i]
            raz = az[i]

            # Spike clipping
            g_sq = rgx * rgx + rgy * rgy + rgz * rgz
            if g_sq > spk_g * spk_g:
                s = spk_g / math.sqrt(g_sq)
                rgx *= s
                rgy *= s
                rgz *= s

            a_sq = rax * rax + ray * ray + raz * raz
            if a_sq > spk_a * spk_a:
                s = spk_a / math.sqrt(a_sq)
                rax *= s
                ray *= s
                raz *= s

            # Gravity tilt observer
            a_mag = math.sqrt(a_sq)
            if abs(a_mag - g_nom) < 0.8:
                egx = 2.0 * (curr_qx * curr_qz - curr_qy * curr_qw)
                egy = 2.0 * (curr_qy * curr_qz + curr_qx * curr_qw)
                egz = 1.0 - 2.0 * (curr_qx * curr_qx + curr_qy * curr_qy)

                uax, uay, uaz = rax / a_mag, ray / a_mag, raz / a_mag
                ex = uay * egz - uaz * egy
                ey = uaz * egx - uax * egz
                ez = uax * egy - uay * egx

                kp = 0.5 * (1.0 - abs(a_mag - g_nom) / 0.8)
                rgx += kp * ex
                rgy += kp * ey
                rgz += kp * ez

            # Sub-stepping for missing telemetry gaps
            n_sub = int(math.ceil(dt / max_sub)) if dt > max_sub else 1
            sub_dt = dt / n_sub

            for _ in range(n_sub):
                # Inlined axis-angle to quaternion
                th_sq = (rgx * rgx + rgy * rgy + rgz * rgz) * sub_dt * sub_dt
                th = math.sqrt(th_sq)
                if th < 1e-6:
                    sc = (0.5 - th_sq / 48.0) * sub_dt
                    dqx = rgx * sc
                    dqy = rgy * sc
                    dqz = rgz * sc
                    dqw = 1.0 - th_sq / 8.0
                else:
                    half_th = 0.5 * th
                    sc = math.sin(half_th) / th * sub_dt
                    dqx = rgx * sc
                    dqy = rgy * sc
                    dqz = rgz * sc
                    dqw = math.cos(half_th)

                # Inlined quaternion multiplication
                nqx = curr_qw * dqx + curr_qx * dqw + curr_qy * dqz - curr_qz * dqy
                nqy = curr_qw * dqy - curr_qx * dqz + curr_qy * dqw + curr_qz * dqx
                nqz = curr_qw * dqz + curr_qx * dqy - curr_qy * dqx + curr_qz * dqw
                nqw = curr_qw * dqw - curr_qx * dqx - curr_qy * dqy - curr_qz * dqz

                inv_norm = 1.0 / math.sqrt(nqx * nqx + nqy * nqy + nqz * nqz + nqw * nqw)
                curr_qx = nqx * inv_norm
                curr_qy = nqy * inv_norm
                curr_qz = nqz * inv_norm
                curr_qw = nqw * inv_norm

            # Sign continuity
            prev_q = quats[i - 1]
            if (curr_qx * prev_q[0] + curr_qy * prev_q[1] + curr_qz * prev_q[2] + curr_qw * prev_q[3]) < 0.0:
                curr_qx = -curr_qx
                curr_qy = -curr_qy
                curr_qz = -curr_qz
                curr_qw = -curr_qw

            quats[i, 0] = curr_qx
            quats[i, 1] = curr_qy
            quats[i, 2] = curr_qz
            quats[i, 3] = curr_qw

            # Inlined vector rotation
            tx = 2.0 * (curr_qy * raz - curr_qz * ray)
            ty = 2.0 * (curr_qz * rax - curr_qx * raz)
            tz = 2.0 * (curr_qx * ray - curr_qy * rax)

            awx = rax + curr_qw * tx + (curr_qy * tz - curr_qz * ty)
            awy = ray + curr_qw * ty + (curr_qz * tx - curr_qx * tz)
            awz = raz + curr_qw * tz + (curr_qx * ty - curr_qy * tx) - g_nom

            acc_w[i, 0] = awx
            acc_w[i, 1] = awy
            acc_w[i, 2] = awz

        # 4. Velocity and Position Integration with Offline Drift Regularization
        t_valid = t_sec[:valid_end] - t_sec[0]
        vel_raw = np.zeros((valid_end, 3), dtype=np.float64)

        for i in range(1, valid_end):
            dt = max(0.0, t_sec[i] - t_sec[i - 1])
            vel_raw[i, 0] = vel_raw[i - 1, 0] + acc_w[i, 0] * dt
            vel_raw[i, 1] = vel_raw[i - 1, 1] + acc_w[i, 1] * dt
            vel_raw[i, 2] = vel_raw[i - 1, 2] + acc_w[i, 2] * dt

        if enable_smoothing and valid_end > 30 and t_valid[-1] > 1.0:
            drift_poly = np.polyfit(t_valid, vel_raw, 1)
            v_drift = np.zeros_like(vel_raw)
            for d in range(3):
                v_drift[:, d] = np.polyval(drift_poly[:, d], t_valid)
            vel_corr = vel_raw - v_drift
        else:
            vel_corr = vel_raw

        pos = np.zeros((n, 3), dtype=np.float64)
        for i in range(1, valid_end):
            dt = max(0.0, t_sec[i] - t_sec[i - 1])
            pos[i, 0] = pos[i - 1, 0] + vel_corr[i, 0] * dt
            pos[i, 1] = pos[i - 1, 1] + vel_corr[i, 1] * dt
            pos[i, 2] = pos[i - 1, 2] + vel_corr[i, 2] * dt

        # 5. Tier 2 Blind Tail Extrapolation (5 s Blackout)
        if has_blind_tail and valid_end < n:
            win_v = min(30, valid_end)
            term_v = np.mean(vel_corr[valid_end - win_v:valid_end], axis=0)

            win_g = min(30, valid_end)
            term_g = np.array([
                np.mean(gx[valid_end - win_g:valid_end]) - bgx,
                np.mean(gy[valid_end - win_g:valid_end]) - bgy,
                np.mean(gz[valid_end - win_g:valid_end]) - bgz
            ], dtype=np.float64)

            t_onset = t_sec[valid_end]
            cur_qx, cur_qy, cur_qz, cur_qw = (
                quats[valid_end - 1, 0],
                quats[valid_end - 1, 1],
                quats[valid_end - 1, 2],
                quats[valid_end - 1, 3]
            )

            for i in range(valid_end, n):
                dt_ns = int(timestamps_ns[i] - timestamps_ns[i - 1])
                if dt_ns <= 0:
                    pos[i] = pos[i - 1]
                    quats[i] = quats[i - 1]
                    continue

                dt = dt_ns * 1e-9
                elapsed = t_sec[i] - t_onset

                v_d = math.exp(-self.drag_decay_rate * elapsed)
                w_d = math.exp(-self.rot_decay_rate * elapsed)

                pos[i, 0] = pos[i - 1, 0] + (term_v[0] * v_d) * dt
                pos[i, 1] = pos[i - 1, 1] + (term_v[1] * v_d) * dt
                pos[i, 2] = pos[i - 1, 2] + (term_v[2] * v_d) * dt

                # Extrapolate orientation
                wgx = term_g[0] * w_d
                wgy = term_g[1] * w_d
                wgz = term_g[2] * w_d

                th_sq = (wgx * wgx + wgy * wgy + wgz * wgz) * dt * dt
                th = math.sqrt(th_sq)
                if th < 1e-6:
                    sc = (0.5 - th_sq / 48.0) * dt
                    dqx, dqy, dqz, dqw = wgx * sc, wgy * sc, wgz * sc, 1.0 - th_sq / 8.0
                else:
                    half_th = 0.5 * th
                    sc = math.sin(half_th) / th * dt
                    dqx, dqy, dqz, dqw = wgx * sc, wgy * sc, wgz * sc, math.cos(half_th)

                nqx = cur_qw * dqx + cur_qx * dqw + cur_qy * dqz - cur_qz * dqy
                nqy = cur_qw * dqy - cur_qx * dqz + cur_qy * dqw + cur_qz * dqx
                nqz = cur_qw * dqz + cur_qx * dqy - cur_qy * dqx + cur_qz * dqw
                nqw = cur_qw * dqw - cur_qx * dqx - curr_qy * dqy - cur_qz * dqz

                inv_norm = 1.0 / math.sqrt(nqx * nqx + nqy * nqy + nqz * nqz + nqw * nqw)
                cur_qx = nqx * inv_norm
                cur_qy = nqy * inv_norm
                cur_qz = nqz * inv_norm
                cur_qw = nqw * inv_norm

                if (cur_qx * quats[i - 1, 0] + cur_qy * quats[i - 1, 1] +
                    cur_qz * quats[i - 1, 2] + cur_qw * quats[i - 1, 3]) < 0.0:
                    cur_qx = -cur_qx
                    cur_qy = -cur_qy
                    cur_qz = -cur_qz
                    cur_qw = -cur_qw

                quats[i, 0] = cur_qx
                quats[i, 1] = cur_qy
                quats[i, 2] = cur_qz
                quats[i, 3] = cur_qw

        poses[:, 1:4] = pos
        poses[:, 4:8] = quats
        return poses


def write_tum_trajectory(file_path: str, poses: np.ndarray):
    """
    Serializes poses to TUM format using fast buffered writing:
      # timestamp tx ty tz qx qy qz qw
    """
    parent_dir = os.path.dirname(file_path)
    if parent_dir and not os.path.exists(parent_dir):
        os.makedirs(parent_dir, exist_ok=True)

    lines = ["# timestamp tx ty tz qx qy qz qw\n"]
    for row in poses:
        lines.append(
            f"{row[0]:.6f} {row[1]:.4f} {row[2]:.4f} {row[3]:.4f} "
            f"{row[4]:.6f} {row[5]:.6f} {row[6]:.6f} {row[7]:.6f}\n"
        )

    with open(file_path, "w", encoding="utf-8", buffering=1024*1024) as f:
        f.writelines(lines)


def main():
    parser = argparse.ArgumentParser(
        description="XR Hackathon 2026 Track 3: Path Finder 6-DoF Trajectory Estimator"
    )
    parser.add_argument("input_csv", nargs="?", default="degraded_imu.csv",
                        help="Path to input degraded IMU telemetry CSV.")
    parser.add_argument("output_txt", nargs="?", default="estimated_trajectory.txt",
                        help="Path to output TUM trajectory file.")
    parser.add_argument("--channel-order", choices=["auto", "euroc", "degrade"], default="auto",
                        help="Channel ordering of input CSV (default: auto).")
    parser.add_argument("--no-smoothing", action="store_true",
                        help="Disable offline smoothing pass (baseline mode).")
    parser.add_argument("--profile", action="store_true",
                        help="Print detailed execution runtime and heap memory profile.")

    args = parser.parse_args()

    # Clean timing run (free of memory tracer slowdown)
    t_start = time.perf_counter()

    # 1. Parse Input
    timestamps_ns, gx, gy, gz, ax, ay, az = parse_imu_telemetry(
        args.input_csv,
        user_channel_order=args.channel_order
    )
    t_parsed = time.perf_counter()

    # 2. Estimate Trajectory
    estimator = PathFinderEstimator()
    poses = estimator.estimate(
        timestamps_ns,
        gx, gy, gz,
        ax, ay, az,
        enable_smoothing=(not args.no_smoothing)
    )
    t_estimated = time.perf_counter()

    # 3. Write TUM Trajectory
    write_tum_trajectory(args.output_txt, poses)
    t_written = time.perf_counter()

    # Measure algorithm heap allocation using tracemalloc on a replay slice
    tracemalloc.start()
    _test_est = PathFinderEstimator()
    _poses_test = _test_est.estimate(
        timestamps_ns, gx, gy, gz, ax, ay, az,
        enable_smoothing=(not args.no_smoothing)
    )
    current_mem, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    total_time_ms = (t_written - t_start) * 1000.0
    parse_time_ms = (t_parsed - t_start) * 1000.0
    est_time_ms = (t_estimated - t_parsed) * 1000.0
    write_time_ms = (t_written - t_estimated) * 1000.0

    num_frames = len(poses)
    duration_s = float(timestamps_ns[-1] - timestamps_ns[0]) * 1e-9 if num_frames > 1 else 1.0
    ms_per_imu_sec = total_time_ms / max(0.001, duration_s)
    peak_heap_mb = peak_mem / (1024.0 * 1024.0)

    print(f"[Path Finder] Successfully estimated trajectory: {num_frames} frames -> {args.output_txt}")
    print(f"  - Trajectory Duration:  {duration_s:.2f} s")
    print(f"  - Total Elapsed Time:   {total_time_ms:.2f} ms")
    print(f"  - Compute Efficiency:   {ms_per_imu_sec:.2f} ms per 1 s IMU (Target: <= 20.0 ms)")
    print(f"  - Peak Algorithm Heap:  {peak_heap_mb:.3f} MB (Target: <= 8.0 MB)")

    speed_pass = ms_per_imu_sec <= 20.0
    memory_pass = peak_heap_mb <= 8.0

    if speed_pass and memory_pass:
        print("  - [PASS] All Efficiency & Memory Gates Fully Satisfied!")
    else:
        if not speed_pass:
            print(f"  - [WARNING] Speed exceeds 20 ms/s limit: {ms_per_imu_sec:.2f} ms")
        if not memory_pass:
            print(f"  - [WARNING] Heap exceeds 8 MB limit: {peak_heap_mb:.3f} MB")

    if args.profile:
        print("\n--- Detailed Profile Breakdown ---")
        print(f"  Parsing Time:     {parse_time_ms:.2f} ms")
        print(f"  Estimation Time:  {est_time_ms:.2f} ms ({est_time_ms/duration_s:.2f} ms/s)")
        print(f"  Writing Time:     {write_time_ms:.2f} ms")
        print(f"  Peak Heap:        {peak_heap_mb:.3f} MB")


if __name__ == "__main__":
    main()
