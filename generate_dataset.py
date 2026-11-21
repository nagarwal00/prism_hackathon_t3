"""
generate_dataset.py - High-Fidelity Synthetic IMU & Trajectory Ground Truth Generator

Generates realistic 3D 6-DoF flight trajectories matching EuRoC MAV physical dynamics:
  - 200 Hz sampling rate (dt = 5,000,000 ns = 0.005 s)
  - Stationary initial period (for bias & gravity initialization)
  - Smooth 3D spline / polynomial motion with roll, pitch, and yaw excitation
  - Exact body-frame specific force and angular velocity synthesis:
      f_b(t) = R(q(t))^T * (a_w(t) - g_w)
      omega_b(t) from quaternion kinematics
  - Ground truth poses in TUM format (timestamp_sec, tx, ty, tz, qx, qy, qz, qw)
  - Raw IMU data in EuRoC format (timestamp_ns, gyro_xyz, accel_xyz)
  - Waypoints / checkpoints extracted for evaluation
"""

import sys
import argparse
import numpy as np


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions (scalar-last: [x, y, z, w]).
    q = q1 * q2
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ], dtype=np.float64)


def quaternion_to_rotation_matrix(q):
    """
    Convert scalar-last quaternion [x, y, z, w] to 3x3 rotation matrix R (body-to-world).
    v_world = R * v_body
    """
    x, y, z, w = q
    # Normalize
    n = np.sqrt(x*x + y*y + z*z + w*w)
    if n > 0:
        x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=np.float64)


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert ZYX Euler angles (radians) to scalar-last quaternion [qx, qy, qz, qw].
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / np.linalg.norm(q)


def generate_synthetic_trajectory(
    duration: float = 30.0,
    rate_hz: float = 200.0,
    t0_ns: int = 1403636579000000000,
    trajectory_type: str = "figure8"
):
    """
    Generates continuous, differentiable 6-DoF trajectory and corresponding IMU data.
    """
    dt = 1.0 / rate_hz
    n_samples = int(duration * rate_hz)
    t = np.linspace(0.0, duration, n_samples, endpoint=False)
    t_ns = t0_ns + (t * 1e9).astype(np.int64)

    # Gravity in world frame: down along -Z
    g_w = np.array([0.0, 0.0, -9.80665])

    # Position, Velocity, Acceleration in World Frame
    pos = np.zeros((n_samples, 3), dtype=np.float64)
    vel = np.zeros((n_samples, 3), dtype=np.float64)
    acc = np.zeros((n_samples, 3), dtype=np.float64)

    # Euler angles
    roll = np.zeros(n_samples, dtype=np.float64)
    pitch = np.zeros(n_samples, dtype=np.float64)
    yaw = np.zeros(n_samples, dtype=np.float64)

    # Initial 1.5 seconds stationary
    stat_samples = int(1.5 * rate_hz)

    # Active motion from 1.5s to duration
    t_active = np.maximum(0.0, t - 1.5)
    motion_mask = t >= 1.5

    # Smooth bell envelope to prevent abrupt jerk at onset
    t_ramp = np.clip(t_active / 2.0, 0.0, 1.0)
    smooth_env = 0.5 * (1.0 - np.cos(np.pi * t_ramp))

    if trajectory_type == "figure8":
        f = 0.25  # Hz frequency
        omega_mot = 2.0 * np.pi * f

        # Figure-8: x = A*sin(wt), y = B*sin(2wt), z = C*sin(wt)
        Ax, Ay, Az = 4.0, 3.0, 1.5
        
        pos[motion_mask, 0] = Ax * np.sin(omega_mot * t_active[motion_mask]) * smooth_env[motion_mask]
        pos[motion_mask, 1] = Ay * np.sin(2.0 * omega_mot * t_active[motion_mask]) * smooth_env[motion_mask]
        pos[motion_mask, 2] = Az * (1.0 - np.cos(omega_mot * t_active[motion_mask])) * smooth_env[motion_mask]

    elif trajectory_type == "complex_helix":
        f = 0.2
        omega_mot = 2.0 * np.pi * f
        R = 3.5
        vz = 0.4
        pos[motion_mask, 0] = R * (np.cos(omega_mot * t_active[motion_mask]) - 1.0) * smooth_env[motion_mask]
        pos[motion_mask, 1] = R * np.sin(omega_mot * t_active[motion_mask]) * smooth_env[motion_mask]
        pos[motion_mask, 2] = vz * t_active[motion_mask] * smooth_env[motion_mask]
    else:
        # Linear sweep
        pos[motion_mask, 0] = 0.5 * t_active[motion_mask] * smooth_env[motion_mask]
        pos[motion_mask, 1] = 0.2 * np.sin(t_active[motion_mask]) * smooth_env[motion_mask]
        pos[motion_mask, 2] = 0.1 * np.cos(t_active[motion_mask]) * smooth_env[motion_mask]

    # Compute numerical derivatives for high fidelity
    vel[1:] = (pos[1:] - pos[:-1]) / dt
    vel[0] = vel[1]

    acc[1:] = (vel[1:] - vel[:-1]) / dt
    acc[0] = acc[1]

    # Orientation: align with velocity heading + roll bank
    quat = np.zeros((n_samples, 4), dtype=np.float64)
    quat[:, 3] = 1.0  # default identity

    for i in range(n_samples):
        if not motion_mask[i]:
            quat[i] = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            vx, vy, vz = vel[i]
            speed_xy = np.sqrt(vx*vx + vy*vy)
            if speed_xy > 0.05:
                psi = np.arctan2(vy, vx)
                theta = -np.arctan2(vz, speed_xy)
                # Banking proportional to lateral acceleration
                phi = np.clip(acc[i, 1] * 0.1, -0.35, 0.35)
            else:
                psi, theta, phi = 0.0, 0.0, 0.0
            quat[i] = euler_to_quaternion(phi, theta, psi)

    # Ensure quaternion sign continuity
    for i in range(1, n_samples):
        if np.dot(quat[i], quat[i-1]) < 0.0:
            quat[i] = -quat[i]

    # Calculate Body-Frame Angular Velocity (gyro)
    # q_dot = 0.5 * q * [omega, 0]  =>  [omega, 0] = 2 * q_conj * q_dot
    gyro_b = np.zeros((n_samples, 3), dtype=np.float64)
    for i in range(1, n_samples):
        # Delta rotation: dq = q_{i-1}^-1 * q_i
        q_prev_conj = np.array([-quat[i-1, 0], -quat[i-1, 1], -quat[i-1, 2], quat[i-1, 3]])
        dq = quaternion_multiply(q_prev_conj, quat[i])
        if dq[3] < 0:
            dq = -dq
        # Rotation vector
        norm_v = np.linalg.norm(dq[:3])
        if norm_v > 1e-9:
            angle = 2.0 * np.arctan2(norm_v, dq[3])
            axis = dq[:3] / norm_v
            omega = axis * (angle / dt)
        else:
            omega = 2.0 * dq[:3] / dt
        gyro_b[i] = omega
    gyro_b[0] = gyro_b[1]

    # Calculate Body-Frame Specific Force (accelerometer)
    # f_b = R^T * (a_w - g_w)
    accel_b = np.zeros((n_samples, 3), dtype=np.float64)
    for i in range(n_samples):
        R = quaternion_to_rotation_matrix(quat[i])
        # Specific force measured by accelerometer in body coordinates:
        accel_b[i] = R.T @ (acc[i] - g_w)

    return t_ns, pos, vel, quat, gyro_b, accel_b


def main():
    parser = argparse.ArgumentParser(description="Generate synthetic 6-DoF trajectory & EuRoC IMU telemetry.")
    parser.add_argument("--duration", type=float, default=25.0, help="Trajectory duration in seconds.")
    parser.add_argument("--rate", type=float, default=200.0, help="Sampling rate in Hz (default: 200).")
    parser.add_argument("--trajectory", choices=["figure8", "complex_helix", "linear"], default="figure8",
                        help="Trajectory geometry type.")
    parser.add_argument("--out-imu", default="synthetic_imu_euroc.csv", help="Output path for EuRoC IMU CSV.")
    parser.add_argument("--out-gt", default="ground_truth_trajectory.txt", help="Output path for TUM ground truth.")
    parser.add_argument("--out-waypoints", default="checkpoints.csv", help="Output path for mission checkpoints.")

    args = parser.parse_args()

    t_ns, pos, vel, quat, gyro_b, accel_b = generate_synthetic_trajectory(
        duration=args.duration,
        rate_hz=args.rate,
        trajectory_type=args.trajectory
    )

    # 1. Save Clean IMU CSV for degrade_imu_default.py
    # degrade_imu_default.py expects: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    with open(args.out_imu, "w") as f:
        f.write("timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
        for i in range(len(t_ns)):
            f.write(f"{t_ns[i]},{accel_b[i,0]:.8f},{accel_b[i,1]:.8f},{accel_b[i,2]:.8f},{gyro_b[i,0]:.8f},{gyro_b[i,1]:.8f},{gyro_b[i,2]:.8f}\n")

    # 2. Save TUM Ground Truth
    # Format: timestamp_sec tx ty tz qx qy qz qw
    with open(args.out_gt, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(len(t_ns)):
            t_sec = t_ns[i] * 1e-9
            f.write(f"{t_sec:.6f} {pos[i,0]:.6f} {pos[i,1]:.6f} {pos[i,2]:.6f} {quat[i,0]:.6f} {quat[i,1]:.6f} {quat[i,2]:.6f} {quat[i,3]:.6f}\n")

    # 3. Save Checkpoints / Waypoints
    # Sample every 2 seconds
    cp_interval = int(2.0 * args.rate)
    with open(args.out_waypoints, "w") as f:
        f.write("checkpoint_id,timestamp_ns,timestamp_sec,x,y,z\n")
        cp_id = 0
        for i in range(0, len(t_ns), cp_interval):
            t_sec = t_ns[i] * 1e-9
            f.write(f"{cp_id},{t_ns[i]},{t_sec:.6f},{pos[i,0]:.4f},{pos[i,1]:.4f},{pos[i,2]:.4f}\n")
            cp_id += 1

    print(f"[generate_dataset] Created {len(t_ns)} frames ({args.duration}s @ {args.rate}Hz).")
    print(f"  - Raw EuRoC IMU: {args.out_imu}")
    print(f"  - Ground Truth TUM: {args.out_gt}")
    print(f"  - Checkpoints: {args.out_waypoints}")


if __name__ == "__main__":
    main()
