#!/usr/bin/env python3
import argparse, csv, sys, time
from pathlib import Path
from threading import Event, Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"


# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
    (0.0, 0.0, 0.4, 0),
    (0.0, 0.0, 0.5, 0),
    (0.5, -0.5, 0.5, 0),
    (0.5, 0.5, 0.5, 0),
    (-0.5, 0.5, 0.5, 0),
    (-0.5, -0.5, 0.5, 0),
    (0.0, 0.0, 0.5, 0),
    (0.0, 0.0, 0.4, 0),
    (0.0, 0.0, 0.0, 0),
]

def make_log_configs(period_ms: int):
    lg_est = LogConfig(name='est', period_in_ms=period_ms)
    for v in ['stateEstimate.x','stateEstimate.y','stateEstimate.z',
              'stateEstimate.vx','stateEstimate.vy','stateEstimate.vz']:
        lg_est.add_variable(v, 'float')

    lg_imu = LogConfig(name='imu', period_in_ms=period_ms)
    for v in ['acc.x','acc.y','acc.z']:
        lg_imu.add_variable(v, 'float')

    return lg_est, lg_imu

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default=DEFAULT_URI)
    ap.add_argument('--outfile', default='hover_log_2_3.csv')
    ap.add_argument('--label', default='default_config')
    ap.add_argument('--rate_hz', type=float, default=50.0)
    ap.add_argument('--do_hover', action='store_true')
    ap.add_argument('--height', type=float, default=0.2)
    ap.add_argument('--hover_s', type=float, default=15.0)
    ap.add_argument('--warmup_s', type=float, default=2.0)
    ap.add_argument('--land_extra_s', type=float, default=1.5)
    args = ap.parse_args()

    period_ms = max(10, int(1000.0 / args.rate_hz))  # cap silly values

    cflib.crtp.init_drivers(enable_debug_driver=False)

    headers = ['t_sec','label','x','y','z','vx','vy','vz','ax','ay','az']
    outfile = Path(args.outfile)

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf, \
         open(outfile, 'w', newline='') as f_csv:

        cf = scf.cf
        writer = csv.writer(f_csv); writer.writerow(headers)

        lg_est, lg_imu = make_log_configs(period_ms)

        # Shared state for merging packets from both log blocks
        t0 = time.monotonic()
        latest = {'x': float('nan'),'y': float('nan'),'z': float('nan'),
                  'vx': float('nan'),'vy': float('nan'),'vz': float('nan'),
                  'ax': float('nan'),'ay': float('nan'),'az': float('nan')}
        lock = Lock()

        def write_row():
            t = time.monotonic() - t0
            writer.writerow([f'{t:.6f}', args.label,
                             latest['x'], latest['y'], latest['z'],
                             latest['vx'], latest['vy'], latest['vz'],
                             latest['ax'], latest['ay'], latest['az']])

        def on_est(ts, data, name):
            with lock:
                latest['x']  = data.get('stateEstimate.x', latest['x'])
                latest['y']  = data.get('stateEstimate.y', latest['y'])
                latest['z']  = data.get('stateEstimate.z', latest['z'])
                latest['vx'] = data.get('stateEstimate.vx', latest['vx'])
                latest['vy'] = data.get('stateEstimate.vy', latest['vy'])
                latest['vz'] = data.get('stateEstimate.vz', latest['vz'])
                write_row()

        def on_imu(ts, data, name):
            with lock:
                latest['ax'] = data.get('acc.x', latest['ax'])
                latest['ay'] = data.get('acc.y', latest['ay'])
                latest['az'] = data.get('acc.z', latest['az'])
                write_row()

        def on_err(lc, msg):
            print(f'[log error:{lc.name}] {msg}', file=sys.stderr)

        try:
            cf.log.add_config(lg_est)
            cf.log.add_config(lg_imu)
        except KeyError as e:
            print("A variable wasn’t found. Ensure firmware exposes stateEstimate.* and acc.*", e)
            sys.exit(1)

        lg_est.data_received_cb.add_callback(on_est)
        lg_imu.data_received_cb.add_callback(on_imu)
        lg_est.error_cb.add_callback(on_err)
        lg_imu.error_cb.add_callback(on_err)

        print(f'Start logging at ~{1000/period_ms:.1f} Hz (split across two blocks)…')
        lg_est.start(); lg_imu.start()
        time.sleep(max(0.0, args.warmup_s))

        with MotionCommander(scf, default_height=args.height) as mc:
                # We are already airborne at ~args.height here.
                print(f"Hovering at ~{args.height:.2f} m …")
                time.sleep(1)

                print('Moving up 0.2m')
                mc.up(0.2)
                # Wait a bit
                time.sleep(1)
                
        for position in sequence:
            print('Setting position {}'.format(position))
            for i in range(5):
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2],
                                                    position[3])
                time.sleep(0.1)


        else:
            print('Logging only. Ctrl+C to stop.')
            try:
                while True: time.sleep(0.2)
            except KeyboardInterrupt:
                pass

        print('Stopping logs…')
        lg_imu.stop(); lg_est.stop()

    print(f'Saved: {outfile.resolve()}')

if __name__ == '__main__':
    main()