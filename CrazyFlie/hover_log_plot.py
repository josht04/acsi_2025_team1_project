#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_hover_log(csv_file):
    # Load CSV
    df = pd.read_csv(csv_file)

    # If the CSV has multiple configs & you only want one, you can filter:
    # df = df[df['label'] == 'pidA']

    t = df['t_sec']

    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(f"Crazyflie Hover Log: {csv_file}", fontsize=14)

    # ---- Position ----
    axs[0].plot(t, df['x'], label='x (m)')
    axs[0].plot(t, df['y'], label='y (m)')
    axs[0].plot(t, df['z'], label='z (m)')
    axs[0].set_ylabel("Position [m]")
    axs[0].legend()
    axs[0].grid(True)

    # ---- Velocity ----
    axs[1].plot(t, df['vx'], label='vx (m/s)')
    axs[1].plot(t, df['vy'], label='vy (m/s)')
    axs[1].plot(t, df['vz'], label='vz (m/s)')
    axs[1].set_ylabel("Velocity [m/s]")
    axs[1].legend()
    axs[1].grid(True)

    # ---- Acceleration ----
    axs[2].plot(t, df['ax'], label='ax (m/s^2)')
    axs[2].plot(t, df['ay'], label='ay (m/s^2)')
    axs[2].plot(t, df['az'], label='az (m/s^2)')
    axs[2].set_ylabel("Acceleration [m/s²]")
    axs[2].set_xlabel("Time [s]")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_hover_log.py <logfile.csv>")
        sys.exit(1)
    plot_hover_log(sys.argv[1])