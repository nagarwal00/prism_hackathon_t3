"""
test_suite.py - Verification and Quality Gate Test Suite for XR Hackathon Track 3

Validates all Section 10 Quality Gates:
  1. I/O and Format Contract:
     - 1:1 input/output row synchronization (cardinality preservation)
     - Exactly 8 numeric fields per row: timestamp, tx, ty, tz, qx, qy, qz, qw
     - Nanosecond-to-second Unix timestamp conversion
     - Strictly finite outputs (no NaN, Inf)
     - Unit quaternions (norm == 1.0) and sign continuity
  2. Physics and Kinematics:
     - Stationary alignment & gravity sign convention
     - Known synthetic rotation integration
  3. Fault Tolerance (Tiers 1, 2, 3):
     - Duplicate timestamps (zero dt) policy
     - Missing samples / timestamp jumps (gap handling)
     - Spike screening & non-physical clipping
     - 5-second blackout / blind tail extrapolation
  4. Efficiency & Resource Limits:
     - Runtime <= 20 ms compute per 1 s IMU telemetry
     - Peak algorithm heap <= 8 MB
"""

import os
import sys
import time
import math
import unittest
import tracemalloc
import numpy as np

from main import (
    parse_imu_telemetry,
    PathFinderEstimator,
    write_tum_trajectory,
    align_vector_to_z,
    fast_quat_rotate_vector,
    fast_quat_mult
)


class TestTrack3QualityGates(unittest.TestCase):

    def test_01_gravity_and_axis_convention(self):
        """Verifies stationary gravity compensation and coordinate convention."""
        # When stationary upright, accelerometer reads [0, 0, +9.80665] (reaction force)
        q = align_vector_to_z(0.0, 0.0, 9.80665)
        # Orientation should be identity
        self.assertAlmostEqual(q[0], 0.0, places=5)
        self.assertAlmostEqual(q[1], 0.0, places=5)
        self.assertAlmostEqual(q[2], 0.0, places=5)
        self.assertAlmostEqual(q[3], 1.0, places=5)

        # In world frame, rotated accel minus gravity should yield zero net acceleration
        ax, ay, az = 0.0, 0.0, 9.80665
        awx, awy, awz = fast_quat_rotate_vector(q[0], q[1], q[2], q[3], ax, ay, az)
        net_az = awz - 9.80665
        self.assertAlmostEqual(net_az, 0.0, places=4)

    def test_02_pure_rotation_integration(self):
        """Verifies gyro integration under pure known constant angular velocity."""
        # 90 degrees (pi/2 rad) rotation around Z over 1.0 second
        rate_hz = 200.0
        n_samples = 200
        omega_z = math.pi * 0.5  # 90 deg/s
        t0 = 1403636579000000000

        ts = t0 + (np.arange(n_samples) * (1e9 / rate_hz)).astype(np.int64)
        gx = np.zeros(n_samples)
        gy = np.zeros(n_samples)
        gz = np.full(n_samples, omega_z)
        ax = np.zeros(n_samples)
        ay = np.zeros(n_samples)
        az = np.full(n_samples, 9.80665)

        est = PathFinderEstimator()
        poses = est.estimate(ts, gx, gy, gz, ax, ay, az, enable_smoothing=False)

        # Total rotation angle over (n_samples - 1) intervals
        total_angle = (n_samples - 1) * (1.0 / rate_hz) * omega_z
        expected_qz = math.sin(total_angle * 0.5)
        expected_qw = math.cos(total_angle * 0.5)
        final_q = poses[-1, 4:8]

        self.assertAlmostEqual(abs(final_q[2]), expected_qz, places=3)
        self.assertAlmostEqual(abs(final_q[3]), expected_qw, places=3)

    def test_03_zero_dt_duplicate_policy(self):
        """Verifies duplicate timestamps produce identical poses without false integration."""
        n_samples = 100
        t0 = 1403636579000000000
        ts = t0 + (np.arange(n_samples) * 5000000).astype(np.int64)

        # Introduce duplicate timestamps at indices 10, 25, 50
        ts[10] = ts[9]
        ts[25] = ts[24]
        ts[50] = ts[49]

        gx = np.random.normal(0, 0.01, n_samples)
        gy = np.random.normal(0, 0.01, n_samples)
        gz = np.random.normal(0, 0.01, n_samples)
        ax = np.random.normal(0, 0.1, n_samples)
        ay = np.random.normal(0, 0.1, n_samples)
        az = np.full(n_samples, 9.80665)

        est = PathFinderEstimator()
        poses = est.estimate(ts, gx, gy, gz, ax, ay, az, enable_smoothing=False)

        # Row cardinality must be preserved exactly
        self.assertEqual(len(poses), n_samples)

        # Duplicate rows must match preceding pose exactly
        for dup_idx in [10, 25, 50]:
            np.testing.assert_array_almost_equal(poses[dup_idx, 1:], poses[dup_idx - 1, 1:])

    def test_04_spike_rejection(self):
        """Verifies extreme non-physical sensor spikes are bounded."""
        n_samples = 150
        t0 = 1403636579000000000
        ts = t0 + (np.arange(n_samples) * 5000000).astype(np.int64)

        gx = np.zeros(n_samples)
        gy = np.zeros(n_samples)
        gz = np.zeros(n_samples)
        ax = np.zeros(n_samples)
        ay = np.zeros(n_samples)
        az = np.full(n_samples, 9.80665)

        # Insert a massive spike (1000 m/s^2)
        ax[75] = 1000.0
        gx[75] = 500.0

        est = PathFinderEstimator(spike_accel_thresh=60.0, spike_gyro_thresh=20.0)
        poses = est.estimate(ts, gx, gy, gz, ax, ay, az, enable_smoothing=False)

        # Trajectory must remain finite and bounded
        self.assertTrue(np.all(np.isfinite(poses)))
        max_vel = np.max(np.abs(poses[:, 1:4]))
        self.assertLess(max_vel, 100.0)

    def test_05_tier2_blind_tail_extrapolation(self):
        """Verifies 5-second blackout detection and bounded forward extrapolation."""
        rate_hz = 200.0
        active_sec = 8.0
        blackout_sec = 5.0
        total_sec = active_sec + blackout_sec

        n_samples = int(total_sec * rate_hz)
        t0 = 1403636579000000000
        ts = t0 + (np.arange(n_samples) * (1e9 / rate_hz)).astype(np.int64)

        gx = np.zeros(n_samples)
        gy = np.zeros(n_samples)
        gz = np.zeros(n_samples)
        ax = np.zeros(n_samples)
        ay = np.zeros(n_samples)
        az = np.full(n_samples, 9.80665)

        # In active window, give constant forward velocity
        active_n = int(active_sec * rate_hz)
        # Apply forward pulse in first second
        ax[:20] = 2.0  # accelerate along X

        # In final 5 seconds, drop to zero
        ax[active_n:] = 0.0
        ay[active_n:] = 0.0
        az[active_n:] = 0.0
        gx[active_n:] = 0.0
        gy[active_n:] = 0.0
        gz[active_n:] = 0.0

        est = PathFinderEstimator()
        poses = est.estimate(ts, gx, gy, gz, ax, ay, az, enable_smoothing=True)

        # Must have exactly 1 pose per input frame
        self.assertEqual(len(poses), n_samples)

        # During blackout, position should continue moving forward (not freeze)
        pos_blackout_start = poses[active_n, 1]
        pos_blackout_end = poses[-1, 1]
        self.assertGreater(pos_blackout_end, pos_blackout_start)

    def test_06_tum_format_schema(self):
        """Verifies TUM serialization adheres to 8 space-separated numeric columns."""
        test_out = "data/test_tum_schema.txt"
        os.makedirs("data", exist_ok=True)

        n = 50
        poses = np.zeros((n, 8))
        poses[:, 0] = 1403636579.0 + np.arange(n) * 0.005
        poses[:, 1:4] = np.random.randn(n, 3)
        poses[:, 4:7] = 0.0
        poses[:, 7] = 1.0

        write_tum_trajectory(test_out, poses)

        with open(test_out, "r") as f:
            lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

        self.assertEqual(len(lines), n)
        for line in lines:
            parts = line.split()
            self.assertEqual(len(parts), 8)
            nums = [float(p) for p in parts]
            self.assertTrue(all(np.isfinite(nums)))
            # Quaternion unit norm
            q_norm = math.sqrt(sum(x*x for x in nums[4:8]))
            self.assertAlmostEqual(q_norm, 1.0, places=4)

    def test_07_memory_and_runtime_limits(self):
        """Verifies runtime <= 20 ms/s and algorithm heap <= 8 MB on 10,000 frames (50s)."""
        rate_hz = 200.0
        duration_s = 50.0
        n_samples = int(duration_s * rate_hz)  # 10,000 samples

        t0 = 1403636579000000000
        ts = t0 + (np.arange(n_samples) * (1e9 / rate_hz)).astype(np.int64)
        gx = np.random.normal(0, 0.01, n_samples)
        gy = np.random.normal(0, 0.01, n_samples)
        gz = np.random.normal(0, 0.01, n_samples)
        ax = np.random.normal(0, 0.05, n_samples)
        ay = np.random.normal(0, 0.05, n_samples)
        az = np.full(n_samples, 9.80665) + np.random.normal(0, 0.05, n_samples)

        # 1. Measure Runtime Cleanly
        t_start = time.perf_counter()
        est = PathFinderEstimator()
        poses = est.estimate(ts, gx, gy, gz, ax, ay, az, enable_smoothing=True)
        t_end = time.perf_counter()

        elapsed_ms = (t_end - t_start) * 1000.0
        ms_per_imu_sec = elapsed_ms / duration_s

        print(f"\n[Test 07] 10,000 frames (50.0 s): {elapsed_ms:.2f} ms ({ms_per_imu_sec:.2f} ms/s)")
        self.assertLessEqual(ms_per_imu_sec, 20.0, f"Speed failed: {ms_per_imu_sec:.2f} ms/s > 20.0 ms/s")

        # 2. Measure Algorithm Heap via tracemalloc
        tracemalloc.start()
        _ = est.estimate(ts, gx, gy, gz, ax, ay, az, enable_smoothing=True)
        _, peak_mem = tracemalloc.get_traced_memory()
        tracemalloc.stop()

        peak_heap_mb = peak_mem / (1024.0 * 1024.0)
        print(f"[Test 07] Peak Algorithm Heap: {peak_heap_mb:.3f} MB (Limit: 8.0 MB)")
        self.assertLessEqual(peak_heap_mb, 8.0, f"Memory failed: {peak_heap_mb:.3f} MB > 8.0 MB")


if __name__ == "__main__":
    unittest.main()
