#!/usr/bin/env python3
"""
World-frame trajectory follower (x,y,z,yaw) with Y logging only.
Logs EKF-estimated y-position (observer) and Flowdeck-measured y while flying.
CSV MUST have: time_s, x, y, z, yaw_deg, vy
"""

import argparse, csv, time
from pathlib import Path
from threading import Lock
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ---------- helpers ----------
def set_param(scf, name, value):
    try:
        scf.cf.param.set_value(name, str(value))
    except Exception:
        pass

def ensure_low_level_world_pos_mode(scf):
    set_param(scf, "commander.enHighLevel", 0)
    set_param(scf, "flightmode.posSet", 1)

def reset_kalman(scf, settle_s=1.0):
    set_param(scf, "stabilizer.estimator", 2)
    set_param(scf, "kalman.resetEstimation", 1)
    time.sleep(0.1)
    set_param(scf, "kalman.resetEstimation", 0)
    time.sleep(settle_s)

def arm(cf):
    try:
        cf.platform.send_arming_request(True)
        time.sleep(0.4)
    except Exception:
        pass

def lerp(a,b,u): return a + u*(b-a)

def load_csv(csv_path):
    csv_path = Path("Traj.csv")
    rows = []
    with open(csv_path, 'r', newline='') as f:
        r = csv.DictReader(f)
        need = {'\ufefftime_s', 'x', 'y', 'z', 'yaw_deg', 'vy'}
        if not need.issubset(set(r.fieldnames or [])):
            raise ValueError(f"CSV must contain columns: {sorted(need)}")
        for d in r:
            rows.append({
                't': float(d['\ufefftime_s']),
                'x': float(d['x']),
                'y': float(d['y']),
                'z': float(d['z']),
                'yaw': float(d['yaw_deg']),
                'vy': float(d['vy']),
            })
    rows.sort(key=lambda k: k['t'])
    return rows

def interp_sample(pts,t):
    if t<=pts[0]['t']: return pts[0]
    if t>=pts[-1]['t']: return pts[-1]
    for i in range(len(pts)-1):
        a,b=pts[i],pts[i+1]
        if a['t']<=t<=b['t']:
            u=(t-a['t'])/max(1e-9,(b['t']-a['t']))
            dyaw=((b['yaw']-a['yaw']+180)%360)-180
            return {
                'x':lerp(a['x'],b['x'],u),
                'y':lerp(a['y'],b['y'],u),
                'z':lerp(a['z'],b['z'],u),
                'yaw':a['yaw']+u*dyaw,
                'vy':lerp(a['vy'],b['vy'],u)
            }
    return pts[-1]

# ---------- main ----------
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--uri',default='radio://0/80/2M')
    ap.add_argument('--csv',default="Traj.csv")
    ap.add_argument('--outfile',default='y_log.csv')
    ap.add_argument('--rate_hz',type=float,default=25.0)
    ap.add_argument('--takeoff_z',type=float,default=1.0)
    args=ap.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)
    traj=load_csv(args.csv)

    est={'y':float('nan'),'y_flow':float('nan')}
    lock=Lock()
    rows=[]

    with SyncCrazyflie(args.uri,cf=Crazyflie(rw_cache='./cache')) as scf:
        cf=scf.cf
        ensure_low_level_world_pos_mode(scf)
        reset_kalman(scf)
        arm(cf)

        # EKF + Flow log
        lg=LogConfig(name='ylog',period_in_ms=int(1000/args.rate_hz))
        lg.add_variable('stateEstimate.y','float')
        try:
            lg.add_variable('position.y','float')  # Flowdeck variable (if available)
        except:
            pass

        def on_log(ts,data,name):
            with lock:
                est['y']=data.get('stateEstimate.y',est['y'])
                est['y_flow']=data.get('position.y',est.get('y_flow',float('nan')))

        cf.log.add_config(lg)
        lg.data_received_cb.add_callback(on_log)
        lg.start()

        # Takeoff ramp
        dt=1/args.rate_hz
        steps=int(args.rate_hz*1.5)
        for k in range(steps):
            cf.commander.send_position_setpoint(0,0,(k+1)/steps*args.takeoff_z,0)
            time.sleep(dt)

        # Follow trajectory
        t0=time.monotonic(); T_end=traj[-1]['t']
        while True:
            t=time.monotonic()-t0
            s=interp_sample(traj,t)
            cf.commander.send_position_setpoint(s['x'],s['y'],s['z'],s['yaw'])
            with lock:
                rows.append([f'{t:.3f}',est['y'],est['y_flow']])
            if t>T_end+0.05: break
            time.sleep(dt)

        # Land
        for k in range(steps):
            z=(1-(k+1)/steps)*args.takeoff_z
            cf.commander.send_position_setpoint(0,0,z,0)
            time.sleep(dt)
        cf.commander.send_stop_setpoint()
        lg.stop()

    with open(args.outfile,'w',newline='') as f:
        w=csv.writer(f)
        w.writerow(['t_sec','y_est','y_flow'])
        w.writerows(rows)
    print(f"Saved: {Path(args.outfile).resolve()}")

if __name__=='__main__':
    main()