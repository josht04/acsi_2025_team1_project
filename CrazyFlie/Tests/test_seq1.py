#!/usr/bin/env python3
"""
Fly a world-frame (x, y, z, yaw_deg) trajectory on Crazyflie using low-level position setpoints.

- Uses cf.commander.send_position_setpoint(x, y, z, yaw_deg)  [WORLD FRAME]
- HighLevel OFF (commander.enHighLevel = 0), low-level position ON (flightmode.posSet = 1)
- Smooth takeoff to 1.0 m and smooth landing
- CSV MUST contain columns: time_s, x, y, z, yaw_deg, vy
- Optional y-velocity feed-forward via --vy_ff (seconds of look-ahead)

Example CSV (header required):
time_s,x,y,z,yaw_deg,vy
0.00,0.00,0.00,1.00,0.0,0.00
1.00,0.50,0.00,1.00,0.0,0.00
2.00,0.50,0.30,1.00,0.0,0.10
3.00,0.00,0.30,1.00,0.0,0.00
4.00,0.00,0.00,1.00,0.0,-0.10
"""

import argparse
import csv
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

def lerp(a, b, u):
    return a + u * (b - a)

def load_csv(csv_path):
    csv_path = Path("Traj.csv")
    rows = []
    with open(csv_path, 'r', newline='') as f:
        r = csv.DictReader(f)
        need = {'time_s', 'x', 'y', 'z', 'yaw_deg', 'vy'}
        if not need.issubset(set(r.fieldnames or [])):
            raise ValueError(f"CSV must contain columns: {sorted(need)}")
        for d in r:
            rows.append({
                't': float(d['time_s']),
                'x': float(d['x']),
                'y': float(d['y']),
                'z': float(d['z']),
                'yaw': float(d['yaw_deg']),
                'vy': float(d['vy']),
            })
    rows.sort(key=lambda k: k['t'])
    return rows

def build_default_traj():
    # 5-second demo at z=1.0 with vy present
    pts = [
        (0.0, 0.00, 0.00, 1.00, 0.0, 0.00),
        (1.0, 0.50, 0.00, 1.00, 0.0, 0.00),
        (2.0, 0.50, 0.30, 1.00, 0.0, 0.10),
        (3.0, 0.00, 0.30, 1.00, 0.0, 0.00),
        (4.0, 0.00, 0.00, 1.00, 0.0, -0.10),
        (5.0, 0.00, 0.00, 1.00, 0.0, 0.00),
    ]
    return [{'t':t,'x':x,'y':y,'z':z,'yaw':yaw,'vy':vy} for (t,x,y,z,yaw,vy) in pts]

def interp_sample(pts, t_now):
    """Linear interpolation in time over x,y,z,yaw; step for vy."""
    if t_now <= pts[0]['t']:
        return pts[0]
    if t_now >= pts[-1]['t']:
        return pts[-1]
    for i in range(len(pts)-1):
        a, b = pts[i], pts[i+1]
        if a['t'] <= t_now <= b['t']:
            u = (t_now - a['t']) / max(1e-9, (b['t'] - a['t']))
            # shortest-path yaw interpolation in degrees
            dyaw = ((b['yaw'] - a['yaw'] + 180) % 360) - 180
            yaw = a['yaw'] + u * dyaw
            return {
                't': t_now,
                'x': lerp(a['x'], b['x'], u),
                'y': lerp(a['y'], b['y'], u),
                'z': lerp(a['z'], b['z'], u),
                'yaw': yaw,
                'vy': a['vy'] if u < 0.5 else b['vy'],
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
        cf.commander.send_stop_setpoint()
        return
    dt = 1.0 / rate_hz
    steps = max(1, int(seconds * rate_hz))
    for k in range(steps):
        u = (k + 1) / steps
        z = (1.0 - u) * z_start
        cf.commander.send_position_setpoint(0.0, 0.0, z, 0.0)
        time.sleep(dt)
    cf.commander.send_stop_setpoint()

def follow_trajectory_lowlevel(cf, traj, rate_hz=25.0, vy_ff=0.0):
    """
    Stream world-frame position setpoints at 'rate_hz'.
    y_cmd = y + vy * vy_ff   (vy_ff in seconds; simple look-ahead feed-forward)
    """
    dt = 1.0 / rate_hz
    t0 = time.monotonic()
    T_end = traj[-1]['t']
    while True:
        t = time.monotonic() - t0
        samp = traj[-1] if t > T_end else interp_sample(traj, t)
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
    p.add_argument('--csv', default='Traj.csv')
    p.add_argument('--rate_hz', type=float, default=25.0)   # default 25 Hz
    p.add_argument('--takeoff_z', type=float, default=1.0)  # takeoff to 1 m
    p.add_argument('--takeoff_s', type=float, default=1.5)
    p.add_argument('--land_s', type=float, default=1.5)
    p.add_argument('--vy_ff', type=float, default=0.0, help='y-velocity feed-forward time [s], e.g., 0.1')
    p.add_argument('--no_reset', action='store_true', help='skip Kalman reset')
    args = p.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    traj = load_csv(args.csv) #if args.csv #else build_default_traj()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        ensure_low_level_world_pos_mode(scf)   # HL OFF, posSet ON
        if not args.no_reset:
            reset_kalman(scf, settle_s=1.0)

        # Explicit arming (harmless if redundant)
        arm(cf)

        # Takeoff
        ramp_takeoff(cf, z_target=max(0.2, args.takeoff_z), seconds=args.takeoff_s, rate_hz=args.rate_hz)

        # Follow trajectory (world frame)
        follow_trajectory_lowlevel(cf, traj, rate_hz=args.rate_hz, vy_ff=args.vy_ff)

        # Land
        z_last = traj[-1]['z'] if traj else args.takeoff_z
        ramp_land(cf, z_start=max(0.0, z_last), seconds=args.land_s, rate_hz=args.rate_hz)

if __name__ == '__main__':
    main()