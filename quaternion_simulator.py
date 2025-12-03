"""
This script simulates an IMU by generating synthetic quaternion, gyro, and
accelerometer data, and streams it over a serial port in the same format as the stm32.

A virtual com port connection NEEDS to be setup for this to be
[ quaternion_simulator.py ]  ->  COM[number]  ===virtual cable===  COM[number]  ->  [ your viewer ]
          writes                                                                        reads

The script can be started with various arguments.
Examples:

python imu_sim.py --port COM[number] --mode rot --rate-hz 100 --yaw-rate 30 --roll-rate 10

python imu_sim.py --port COM[number] --mode trans --a-amp-g 0.05 --a-freq 0.5 --a-dir x

python imu_sim.py --port COM[number] --mode both --yaw-rate 20 --roll-rate 10 --a-amp-g 0.03 --a-freq 0.7 --a-dir y

"""

import argparse, time, math, sys
import numpy as np
import serial

# ----------------- math helpers -----------------
def quat_from_euler_zyx(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    # intrinsic Z-Y-X (yaw, pitch, roll)
    w = cr*cp*cy + sr*cp*sy - cr*sp*sy + sr*sp*cy
    x = sr*cp*cy - cr*cp*sy + cr*sp*cy + sr*sp*sy
    yq= cr*sp*cy + sr*sp*sy + sr*cp*sy - cr*cp*cy
    z = cr*cp*sy + sr*cp*cy - sr*sp*cy + cr*sp*sy
    q = np.array([w, x, yq, z], float)
    return q / np.linalg.norm(q)

def rotmat_from_quat(q):
    w, x, y, z = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)]
    ], float)

def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z], float)

def quat_mul(q1, q2):
    w1,x1,y1,z1 = q1; w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], float)

def delta_to_body_rates(q_prev, q_curr, dt):
    """Approximate body rates [deg/s] from quaternion delta."""
    if dt <= 0: return (0.0, 0.0, 0.0)
    dq = quat_mul(quat_conj(q_prev), q_curr)  # rotation from prev->curr
    dq /= np.linalg.norm(dq) + 1e-12
    angle = 2*math.acos(max(-1.0, min(1.0, dq[0])))
    if angle < 1e-12:
        axis_world = np.array([1.0, 0.0, 0.0])
    else:
        s = math.sin(angle/2.0)
        axis_world = dq[1:]/(s + 1e-12)
        axis_world /= (np.linalg.norm(axis_world) + 1e-12)
    # express axis in the PREVIOUS body frame
    R_prev = rotmat_from_quat(q_prev)
    axis_body = R_prev.T @ axis_world
    w_body_rad = axis_body * (angle / dt)  # rad/s
    return tuple(np.degrees(w_body_rad))

# ----------------- signal generators -----------------
def accel_world_g(t, a_amp_g, a_freq_hz, a_dir):
    """Commanded linear accel in WORLD coords, in g (not including gravity)."""
    if a_amp_g <= 0 or a_freq_hz <= 0:
        return np.zeros(3)
    omega = 2*math.pi*a_freq_hz
    a = a_amp_g * math.sin(omega*t)
    if a_dir == 'x': return np.array([a, 0, 0], float)
    if a_dir == 'y': return np.array([0, a, 0], float)
    if a_dir == 'z': return np.array([0, 0, a], float)
    # custom vector "x,y,z"
    try:
        v = np.array([float(s) for s in a_dir.split(',')], float)
        if v.size == 3:
            v = v / (np.linalg.norm(v) + 1e-12)
            return v * a
    except Exception:
        pass
    return np.array([a, 0, 0], float)

# ----------------- main generator -----------------
def run(port, baud, mode, rate_hz,
        yaw_rate, pitch_rate, roll_rate,
        a_amp_g, a_freq_hz, a_dir,
        noise_gyro_dps, noise_accel_g):
    ser = serial.Serial(port, baud, timeout=0)
    print(f"[sim] writing to {port} @ {baud} ({mode})")
    dt = 1.0 / rate_hz
    t0 = time.perf_counter()
    tick0 = int(time.time() * 1000)
    t_prev = t0
    q_prev = quat_from_euler_zyx(0, 0, 0)

    # state for rotation and accel
    roll0 = pitch0 = yaw0 = 0.0

    try:
        while True:
            t = time.perf_counter() - t0
            tick = tick0 + int(t * 1000)

            # ---- orientation (quaternion) ----
            if mode in ('rot', 'both'):
                roll  = roll0  + roll_rate  * t
                pitch = pitch0 + pitch_rate * t
                yaw   = yaw0   + yaw_rate   * t
            else:
                roll = pitch = yaw = 0.0

            q = quat_from_euler_zyx(roll, pitch, yaw)
            R = rotmat_from_quat(q)

            # ---- angular rates (body) via delta quaternion ----
            dt_est = max(1e-4, min(0.1, t - (t_prev - t0)))
            gx, gy, gz = delta_to_body_rates(q_prev, q, dt_est)
            if noise_gyro_dps > 0:
                gx += np.random.normal(0, noise_gyro_dps)
                gy += np.random.normal(0, noise_gyro_dps)
                gz += np.random.normal(0, noise_gyro_dps)

            # ---- accelerometer (body, in g) ----
            # gravity in world g = [0,0,1]
            a_cmd_world = accel_world_g(t, a_amp_g, a_freq_hz, a_dir) if mode in ('trans','both') else np.zeros(3)
            a_world_total_g = np.array([0, 0, 1], float) + a_cmd_world
            # sensor measures in BODY frame: a_body = R^T * a_world_total
            a_body_g = R.T @ a_world_total_g
            if noise_accel_g > 0:
                a_body_g += np.random.normal(0, noise_accel_g, size=3)

            # ---- stream ----
            line = ("IMU,{tick},{w:.6f},{x:.6f},{y:.6f},{z:.6f},"
                    "{gx:.3f},{gy:.3f},{gz:.3f},"
                    "{ax:.4f},{ay:.4f},{az:.4f}\r\n").format(
                        tick=tick, w=q[0], x=q[1], y=q[2], z=q[3],
                        gx=gx, gy=gy, gz=gz,
                        ax=a_body_g[0], ay=a_body_g[1], az=a_body_g[2]
                    )
            ser.write(line.encode('ascii'))

            # prep next
            q_prev = q
            t_prev = time.perf_counter()
            # pace
            sleep_left = dt - (time.perf_counter() - (t0 + t))
            if sleep_left > 0:
                time.sleep(sleep_left)
    except KeyboardInterrupt:
        print("\n[sim] stopped.")
    finally:
        try: ser.close()
        except: pass

# ----------------- CLI -----------------
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="IMU stream simulator (STM32 format).")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["rot","trans","both"], default="rot",
                    help="rot: rotation only; trans: translation only; both: rotation+translation")
    ap.add_argument("--rate-hz", type=float, default=100.0, help="output rate (Hz)")

    # rotation: constant rates, easier for debugging
    ap.add_argument("--yaw-rate",   type=float, default=20.0, help="deg/s about +Z (yaw)")
    ap.add_argument("--pitch-rate", type=float, default=0.0,  help="deg/s about +Y (pitch)")
    ap.add_argument("--roll-rate",  type=float, default=10.0, help="deg/s about +X (roll)")

    # translation: sinusoidal accel in world frame (in g)
    ap.add_argument("--a-amp-g",  type=float, default=0.00, help="accel amplitude (g), e.g. 0.05 for ~0.5 m/s^2")
    ap.add_argument("--a-freq",    type=float, default=0.5,  help="accel frequency (Hz)")
    ap.add_argument("--a-dir",     type=str,   default="x",  help="direction: x|y|z or 'x,y,z' unit vector")

    # noise
    ap.add_argument("--noise-gyro-dps",  type=float, default=0.0)
    ap.add_argument("--noise-accel-g",   type=float, default=0.0)

    args = ap.parse_args()
    run(args.port, args.baud, args.mode, args.rate_hz,
        args.yaw_rate, args.pitch_rate, args.roll_rate,
        args.a_amp_g, args.a_freq, args.a_dir,
        args.noise_gyro_dps, args.noise_accel_g)
