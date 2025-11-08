#!/usr/bin/env python3
"""
Fly a world-frame (x, y, z, yaw_deg) trajectory on Crazyflie using low-level position setpoints.

- Uses cf.commander.send_position_setpoint(x, y, z, yaw_deg)  [WORLD FRAME]
- Requires: commander.enHighLevel = 0  and  flightmode.posSet = 1
- Smooth takeoff and landing ramps
- Optional y-velocity feed-forward if CSV has a 'vy' column (meters/second)

CSV format (header required):
time_s,x,y,z,yaw_deg[,vy]

Notes:
- time_s is monotonic seconds since start of trajectory (0.0, 0.02, ...)
- yaw_deg is in degrees
- vy (optional) is meters/second in WORLD frame (positive = left)

Example:
0.00, 0.00, 0.00, 0.50, 0.0
1.00, 0.50, 0.00, 0.50, 0.0
2.00, 0.50, 0.30, 0.50, 0.0
3.00, 0.00, 0.30, 0.50, 0.0
4.00, 0.00, 0.00, 0.50, 0.0
"""

import argparse
import csv
import math
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ---------- Utilities ----------

def set_param(scf, name, value):
    try:
        scf.cf.param.set_value(name, str(value))
    except Exception:
        pass

def ensure_low_level_world_pos_mode(scf):
    # World-frame position setpoints require HighLevel OFF and posSet ON
    set_param(scf, 'commander.enHighLevel', 0)
    set_param(scf, 'flightmode.posSet', 1)

def reset_kalman(scf, settle_s=1.0):
    set_param(scf, 'stabilizer.estimator', 2)           # 2 = Kalman
    set_param(scf, 'kalman.resetEstimation', 1)
    time.sleep(0.1)
    set_param(scf, 'kalman.resetEstimation', 0)
    time.sleep(settle_s)

def arm(cf):
    try:
        cf.platform.send_arming_request(True)
        time.sleep(0.4)
    except Exception:
        pass

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def lerp(a, b, u):
    return a + u * (b - a)

def load_csv(csv_path):
    csv_path = Path("Traj.csv")
    rows = []
    with open(csv_path, 'r', newline='') as f:
        r = csv.DictReader(f)
        need = {'time_s', 'x', 'y', 'z', 'yaw_deg'}
        if not need.issubset(set(r.fieldnames or [])):
            raise ValueError(f"CSV must contain columns: {sorted(need)}")
        has_vy = 'vy' in r.fieldnames
        for d in r:
            rows.append({
                't': float(d['time_s']),
                'x': float(d['x']),
                'y': float(d['y']),
                'z': float(d['z']),
                'yaw': float(d['yaw_deg']),
                'vy': float(d['vy']) if has_vy and d['vy'] != '' else 0.0
            })
    rows.sort(key=lambda k: k['t'])
    return rows

def build_default_traj():
    # 5-second “└─┘” demo at z=0.5
    pts = [
        (0.0,  0.00, 0.00, 0.50, 0.0),
        (1.0,  0.50, 0.00, 0.50, 0.0),
        (2.0,  0.50, 0.30, 0.50, 0.0),
        (3.0,  0.00, 0.30, 0.50, 0.0),
        (4.0,  0.00, 0.00, 0.50, 0.0),
        (5.0,  0.00, 0.00, 0.50, 0.0),
    ]
    return [{'t':t,'x':x,'y':y,'z':z,'yaw':yaw,'vy':0.0} for (t,x,y,z,yaw) in pts]

def interp_sample(pts, t_now):
    """Linear interpolation in time over x,y,z,yaw and step for vy."""
    if t_now <= pts[0]['t']:
        return pts[0]
    if t_now >= pts[-1]['t']:
        return pts[-1]
    # find segment
    for i in range(len(pts)-1):
        a, b = pts[i], pts[i+1]
        if a['t'] <= t_now <= b['t']:
            u = (t_now - a['t']) / max(1e-9, (b['t'] - a['t']))
            yaw = lerp(a['yaw'], b['yaw'], u)
            # unwrap yaw for shortest interpolation
            dyaw = ((b['yaw'] - a['yaw'] + 180) % 360) - 180
            yaw = a['yaw'] + u * dyaw
            return {
                't': t_now,
                'x': lerp(a['x'], b['x'], u),
                'y': lerp(a['y'], b['y'], u),
                'z': lerp(a['z'], b['z'], u),
                'yaw': yaw,
                'vy': a['vy'] if u < 0.5 else b['vy'],  # simple step-hold
            }
    return pts[-1]

# ---------- Flight routines ----------

def ramp_takeoff(cf, z_target, seconds, rate_hz):
    if z_target <= 0.0 or seconds <= 0.0:
        return
    dt = 1.0 / rate_hz
    steps = max(1, int(seconds * rate_hz))
    for k in range(steps):
        u = (k + 1) / steps
        z = u * z_target
        cf.commander.send_position_setpoint(0.0, 0.0, z, 0.0)
        time.sleep(dt)

def ramp_land(cf, z_start, seconds, rate_hz):
    if z_start <= 0.0 or seconds <= 0.0:
        return
    dt = 1.0 / rate_hz
    steps = max(1, int(seconds * rate_hz))
    for k in range(steps):
        u = (k + 1) / steps
        z = (1.0 - u) * z_start
        cf.commander.send_position_setpoint(0.0, 0.0, z, 0.0)
        time.sleep(dt)
    # final settle on ground
    cf.commander.send_stop_setpoint()

def follow_trajectory_lowlevel(cf, traj, rate_hz=50.0, vy_ff=0.0):
    """
    Stream world-frame position setpoints at 'rate_hz'.
    If vy_ff > 0 and traj contains 'vy', apply a small feed-forward on y:
        y_cmd = y + vy * vy_ff
    vy_ff has units of seconds (e.g., 0.08..0.20 works as a “look-ahead”).
    """
    dt = 1.0 / rate_hz
    t0 = time.monotonic()
    T_end = traj[-1]['t']
    while True:
        t = time.monotonic() - t0
        if t > T_end:
            samp = traj[-1]
        else:
            samp = interp_sample(traj, t)

        y_cmd = samp['y'] + samp['vy'] * vy_ff
        cf.commander.send_position_setpoint(
            float(samp['x']),
            float(y_cmd),
            float(samp['z']),
            float(samp['yaw'])
        )
        if t > T_end + 0.05:
            break
        time.sleep(dt)

# ---------- Main ----------

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--uri', default='radio://0/80/2M')
    p.add_argument('--csv', default='')
    p.add_argument('--rate_hz', type=float, default=25.0)
    p.add_argument('--takeoff_z', type=float, default=1.0)
    p.add_argument('--takeoff_s', type=float, default=1.5)
    p.add_argument('--land_s', type=float, default=1.5)
    p.add_argument('--vy_ff', type=float, default=0.0, help='y-velocity feed-forward time [s], e.g., 0.1')
    p.add_argument('--no_reset', action='store_true', help='skip Kalman reset if you prefer')
    args = p.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    if args.csv:
        traj = load_csv(args.csv)
    else:
        traj = build_default_traj()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        ensure_low_level_world_pos_mode(scf)
        if not args.no_reset:
            reset_kalman(scf, settle_s=1.0)

        arm(cf)

        # Takeoff to a safe height
        ramp_takeoff(cf, z_target=max(0.2, args.takeoff_z), seconds=args.takeoff_s, rate_hz=args.rate_hz)

        # Follow trajectory (world frame)
        follow_trajectory_lowlevel(cf, traj, rate_hz=args.rate_hz, vy_ff=args.vy_ff)

        # Land
        z_last = traj[-1]['z'] if traj else args.takeoff_z
        ramp_land(cf, z_start=max(0.0, z_last), seconds=args.land_s, rate_hz=args.rate_hz)

if __name__ == '__main__':
    main()