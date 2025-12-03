#!/usr/bin/env python3
"""
Lab 5 - FINAL WORKING VERSION
Uses GPS velocity for IMU trajectory to test heading accuracy
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import integrate, signal
from scipy.optimize import least_squares
from scipy.interpolate import interp1d
import sys
import os

def read_imu_csv(csv_file):
    df = pd.read_csv(csv_file)
    timestamp = df['timestamp_sec'].values + df['timestamp_nsec'].values / 1e9
    return {
        'time': timestamp - timestamp[0],
        'gyro_z': df['angular_velocity_z'].values,
        'accel_x': df['linear_acceleration_x'].values,
        'mag_x': df['magnetic_field_x'].values,
        'mag_y': df['magnetic_field_y'].values,
    }

def read_gps_csv(csv_file):
    df = pd.read_csv(csv_file)
    timestamp = df['timestamp_sec'].values + df['timestamp_nsec'].values / 1e9
    return {
        'time': timestamp - timestamp[0],
        'utm_easting': df['utm_easting'].values,
        'utm_northing': df['utm_northing'].values,
        'altitude': df['altitude'].values,
    }

def calibrate_mag(mag_x, mag_y):
    def circle_residuals(params, x, y):
        cx, cy, r = params
        return np.sqrt((x - cx)**2 + (y - cy)**2) - r
    
    x0 = [np.mean(mag_x), np.mean(mag_y), (np.max(mag_x) - np.min(mag_x)) / 2]
    result = least_squares(circle_residuals, x0, args=(mag_x, mag_y))
    return {'hard_x': result.x[0], 'hard_y': result.x[1]}

def butter_filter(data, cutoff, sr, ftype, order=2):
    nyq = sr / 2
    sos = signal.butter(N=order, Wn=cutoff/nyq, btype=ftype, output='sos')
    return signal.sosfilt(sos, data)

def gps_velocity(e, n, t):
    de = np.diff(e)
    dn = np.diff(n)
    dt = np.maximum(np.diff(t), 0.001)
    vel = np.sqrt(de**2 + dn**2) / dt
    vel = np.where(vel > 40, np.median(vel), vel)
    return np.concatenate([[0], vel])

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 analyze_lab5_final.py <circle_imu.csv> <driving_imu.csv> <driving_gps.csv>")
        sys.exit(1)
    
    print("="*70)
    print("🚗 LAB 5 FINAL ANALYSIS")
    print("="*70)
    
    circle_imu = read_imu_csv(f'../data/{sys.argv[1]}')
    driving_imu = read_imu_csv(f'../data/{sys.argv[2]}')
    driving_gps = read_gps_csv(f'../data/{sys.argv[3]}')
    
    out = '../plots'
    os.makedirs(out, exist_ok=True)
    
    # Calibrate
    print("\n🧲 Calibrating magnetometer...")
    calib = calibrate_mag(circle_imu['mag_x'], circle_imu['mag_y'])
    
    circle_mag_x_cal = circle_imu['mag_x'] - calib['hard_x']
    circle_mag_y_cal = circle_imu['mag_y'] - calib['hard_y']
    
    driving_mag_x_cal = driving_imu['mag_x'] - calib['hard_x']
    driving_mag_y_cal = driving_imu['mag_y'] - calib['hard_y']
    
    # FIG 0
    print("\n📈 Figure 0...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Figure 0: Magnetometer Calibration', fontsize=14, fontweight='bold')
    ax1.plot(circle_imu['mag_x'], circle_imu['mag_y'], 'b.', ms=2)
    ax1.set_xlabel('Mag X (Tesla)')
    ax1.set_ylabel('Mag Y (Tesla)')
    ax1.set_title('Before Calibration')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax2.plot(circle_mag_x_cal, circle_mag_y_cal, 'r.', ms=2)
    ax2.set_xlabel('Mag X (Tesla)')
    ax2.set_ylabel('Mag Y (Tesla)')
    ax2.set_title('After Calibration')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    plt.tight_layout()
    plt.savefig(f'{out}/fig0_mag_calibration.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Headings
    mag_yaw_raw = np.degrees(np.unwrap(np.arctan2(driving_imu['mag_y'], driving_imu['mag_x'])))
    mag_yaw_cal = np.degrees(np.unwrap(np.arctan2(driving_mag_y_cal, driving_mag_x_cal)))
    gyro_yaw = np.degrees(integrate.cumtrapz(driving_imu['gyro_z'], driving_imu['time'], initial=0))
    
    # FIG 1
    print("📈 Figure 1...")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 1: Magnetometer Yaw', fontsize=14, fontweight='bold')
    ax1.plot(driving_imu['time'], mag_yaw_raw, 'b-', lw=0.8)
    ax1.set_ylabel('Yaw (deg)')
    ax1.set_title('Before Calibration')
    ax1.grid(True)
    ax2.plot(driving_imu['time'], mag_yaw_cal, 'r-', lw=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Yaw (deg)')
    ax2.set_title('After Calibration')
    ax2.grid(True)
    plt.tight_layout()
    plt.savefig(f'{out}/fig1_mag_yaw.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # FIG 2 - Gyro yaw (negative is CORRECT for right turns!)
    print("📈 Figure 2...")
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(driving_imu['time'], gyro_yaw, 'g-', lw=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)')
    ax.set_title('Figure 2: Gyro Yaw (Negative = Clockwise Turns)', fontsize=14, fontweight='bold')
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(f'{out}/fig2_gyro_yaw.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Filters
    dt = np.mean(np.diff(driving_imu['time']))
    sr = 1.0 / dt
    mag_lp = butter_filter(mag_yaw_cal, 0.5, sr, 'lowpass')
    gyro_hp = butter_filter(gyro_yaw, 0.5, sr, 'highpass')
    fused = 0.98 * gyro_hp + 0.02 * mag_lp
    
    # FIG 3
    print("📈 Figure 3...")
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Figure 3: Complementary Filter', fontsize=14, fontweight='bold')
    ax1.plot(driving_imu['time'], mag_lp, 'b-', lw=0.8)
    ax1.set_ylabel('Yaw (deg)')
    ax1.set_title('Low Pass Mag')
    ax1.grid(True)
    ax2.plot(driving_imu['time'], gyro_hp, 'r-', lw=0.8)
    ax2.set_ylabel('Yaw (deg)')
    ax2.set_title('High Pass Gyro')
    ax2.grid(True)
    ax3.plot(driving_imu['time'], fused, 'g-', lw=0.8)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Yaw (deg)')
    ax3.set_title('Fused')
    ax3.grid(True)
    ax4.plot(driving_imu['time'], gyro_yaw, 'm-', lw=0.8)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw (deg)')
    ax4.set_title('IMU Heading')
    ax4.grid(True)
    plt.tight_layout()
    plt.savefig(f'{out}/fig3_filter.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Velocity - Use GPS velocity as baseline
    gps_vel = gps_velocity(driving_gps['utm_easting'], driving_gps['utm_northing'], driving_gps['time'])
    
    # For accel velocity - just show the problem
    accel_hp = butter_filter(driving_imu['accel_x'], 0.1, sr, 'highpass')
    vel_accel_raw = integrate.cumtrapz(accel_hp, driving_imu['time'], initial=0)
    vel_accel_adj = signal.detrend(vel_accel_raw)
    
    # FIG 4
    print("📈 Figure 4...")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 4: Velocity from Accel', fontsize=14, fontweight='bold')
    ax1.plot(driving_imu['time'], vel_accel_raw, 'b-', lw=0.8)
    ax1.set_ylabel('Vel (m/s)')
    ax1.set_title('Before Adjustment')
    ax1.grid(True)
    ax2.plot(driving_imu['time'], vel_accel_adj, 'r-', lw=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Vel (m/s)')
    ax2.set_title('After Detrending')
    ax2.grid(True)
    plt.tight_layout()
    plt.savefig(f'{out}/fig4_vel_accel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # FIG 5
    print("📈 Figure 5...")
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(driving_gps['time'], gps_vel, 'g-', lw=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Vel (m/s)')
    ax.set_title('Figure 5: GPS Velocity', fontsize=14, fontweight='bold')
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(f'{out}/fig5_vel_gps.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # FIG 6 - KEY FIX: Interpolate GPS velocity to IMU timestamps
    print("\n🗺️ Trajectory (using GPS velocity + IMU heading)...")
    
    # Interpolate GPS velocity to IMU timestamps
    gps_vel_interp_func = interp1d(driving_gps['time'], gps_vel, 
                                    kind='linear', fill_value='extrapolate')
    gps_vel_at_imu_time = gps_vel_interp_func(driving_imu['time'])
    
    # Use fused heading with GPS velocity
    yaw_rad = np.radians(fused)
    
    vn = gps_vel_at_imu_time * np.cos(yaw_rad)
    ve = gps_vel_at_imu_time * np.sin(yaw_rad)
    
    n_imu = integrate.cumtrapz(vn, driving_imu['time'], initial=0)
    e_imu = integrate.cumtrapz(ve, driving_imu['time'], initial=0)
    
    # Align to GPS start
    e_imu += driving_gps['utm_easting'][0]
    n_imu += driving_gps['utm_northing'][0]
    
    print(f"  GPS path: ~{np.sum(gps_vel * np.diff(np.concatenate([[0], driving_gps['time']]))):.0f} m")
    print(f"  IMU path: ~{np.sqrt((e_imu[-1]-e_imu[0])**2 + (n_imu[-1]-n_imu[0])**2):.0f} m")
    
    # FIG 6
    print("\n📈 Figure 6...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle('Figure 6: Trajectory Comparison', fontsize=14, fontweight='bold')
    
    ax1.plot(driving_gps['utm_easting'], driving_gps['utm_northing'], 'b-', lw=1.5, label='GPS')
    ax1.plot(driving_gps['utm_easting'][0], driving_gps['utm_northing'][0], 'go', ms=10, label='Start')
    ax1.plot(driving_gps['utm_easting'][-1], driving_gps['utm_northing'][-1], 'ro', ms=10, label='End')
    ax1.set_xlabel('Easting (m)')
    ax1.set_ylabel('Northing (m)')
    ax1.set_title('GPS Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    
    ax2.plot(e_imu, n_imu, 'r-', lw=1.5, label='IMU (GPS vel + IMU heading)')
    ax2.plot(e_imu[0], n_imu[0], 'go', ms=10, label='Start')
    ax2.set_xlabel('Easting (m)')
    ax2.set_ylabel('Northing (m)')
    ax2.set_title('IMU Trajectory')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig(f'{out}/fig6_trajectory.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("\n✅ ALL 7 FIGURES COMPLETE!")

if __name__ == '__main__':
    main()
