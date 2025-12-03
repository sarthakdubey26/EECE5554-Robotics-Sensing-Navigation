#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares
from scipy.signal import butter, sosfilt, detrend
from scipy.spatial.transform import Rotation
import sys

def read_imu_csv(path):
    df = pd.read_csv(path)
    data = {}
    
    if 'timestamp_sec' in df.columns and 'timestamp_nsec' in df.columns:
        data['time'] = df['timestamp_sec'] + df['timestamp_nsec'] * 1e-9
    else:
        data['time'] = np.arange(len(df)) * 0.01
    
    data['time'] = data['time'] - data['time'][0]
    
    data['mag_x'] = df['magnetic_field_x'].values
    data['mag_y'] = df['magnetic_field_y'].values
    data['mag_z'] = df['magnetic_field_z'].values
    
    data['gyro_x'] = df['angular_velocity_x'].values
    data['gyro_y'] = df['angular_velocity_y'].values
    data['gyro_z'] = df['angular_velocity_z'].values
    
    data['accel_x'] = df['linear_acceleration_x'].values
    data['accel_y'] = df['linear_acceleration_y'].values
    data['accel_z'] = df['linear_acceleration_z'].values
    
    if all(col in df.columns for col in ['orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']):
        quaternions = np.column_stack([
            df['orientation_x'].values,
            df['orientation_y'].values,
            df['orientation_z'].values,
            df['orientation_w'].values
        ])
        yaw_values = []
        for q in quaternions:
            r = Rotation.from_quat(q)
            euler = r.as_euler('xyz', degrees=False)
            yaw_values.append(euler[2])
        data['yaw'] = np.array(yaw_values)
    else:
        data['yaw'] = np.arctan2(data['mag_y'], data['mag_x'])
    
    return data

def read_gps_csv(path):
    df = pd.read_csv(path)
    data = {}
    
    if 'timestamp_sec' in df.columns and 'timestamp_nsec' in df.columns:
        data['time'] = df['timestamp_sec'] + df['timestamp_nsec'] * 1e-9
    else:
        data['time'] = np.arange(len(df)) * 0.1
    
    data['time'] = data['time'] - data['time'][0]
    
    data['easting'] = df['utm_easting'].values
    data['northing'] = df['utm_northing'].values
    data['altitude'] = df['altitude'].values
    
    return data

def butter_lowpass(data, cutoff, fs, order=2):
    if fs <= 0 or cutoff <= 0 or cutoff >= fs/2:
        return data
    nyq = fs / 2
    normal_cutoff = cutoff / nyq
    sos = butter(order, normal_cutoff, btype='low', analog=False, output='sos')
    return sosfilt(sos, data)

def butter_highpass(data, cutoff, fs, order=2):
    if fs <= 0 or cutoff <= 0 or cutoff >= fs/2:
        return data
    nyq = fs / 2
    normal_cutoff = cutoff / nyq
    sos = butter(order, normal_cutoff, btype='high', analog=False, output='sos')
    return sosfilt(sos, data)

if len(sys.argv) < 4:
    print("Usage: python3 analyze_lab5_complete.py <circle_imu> <driving_imu> <driving_gps>")
    sys.exit(1)

print("Reading data files...")
circle_imu = read_imu_csv(f'../data/{sys.argv[1]}')
drive_imu = read_imu_csv(f'../data/{sys.argv[2]}')
drive_gps = read_gps_csv(f'../data/{sys.argv[3]}')

out = '../plots'
os.makedirs(out, exist_ok=True)

print("Performing magnetometer calibration...")
mx = circle_imu['mag_x']
my = circle_imu['mag_y']

cx = np.mean(mx)
cy = np.mean(my)

mx_centered = mx - cx
my_centered = my - cy

def ellipse_error(params, x, y):
    a, b, theta = params
    x_rot = x * np.cos(theta) + y * np.sin(theta)
    y_rot = -x * np.sin(theta) + y * np.cos(theta)
    return (x_rot/a)**2 + (y_rot/b)**2 - 1

p0 = [np.std(mx_centered), np.std(my_centered), 0.0]
if p0[0] == 0: p0[0] = 1.0
if p0[1] == 0: p0[1] = 1.0

result = least_squares(ellipse_error, p0, args=(mx_centered, my_centered))
a, b, theta = result.x

x_rot = mx_centered * np.cos(theta) + my_centered * np.sin(theta)
y_rot = -mx_centered * np.sin(theta) + my_centered * np.cos(theta)
mx_cal = x_rot / a
my_cal = y_rot / b

plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(mx, my, 'b.', markersize=2)
plt.xlabel('Mag X (Tesla)')
plt.ylabel('Mag Y (Tesla)')
plt.title('Before Calibration')
plt.axis('equal')
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(mx_cal, my_cal, 'r.', markersize=2)
plt.xlabel('Mag X (Tesla)')
plt.ylabel('Mag Y (Tesla)')
plt.title('After Calibration')
plt.axis('equal')
plt.grid(True)
plt.suptitle('Figure 0: Magnetometer Calibration', fontsize=14)
plt.tight_layout()
plt.savefig(f'{out}/fig0_mag_calibration.png', dpi=300)
plt.close()

print("Processing drive data...")
mx_d = drive_imu['mag_x']
my_d = drive_imu['mag_y']
t_d = drive_imu['time']

mx_d_centered = mx_d - cx
my_d_centered = my_d - cy
x_rot_d = mx_d_centered * np.cos(theta) + my_d_centered * np.sin(theta)
y_rot_d = -mx_d_centered * np.sin(theta) + my_d_centered * np.cos(theta)
mx_d_cal = x_rot_d / a
my_d_cal = y_rot_d / b

yaw_raw = np.arctan2(my_d, mx_d)
yaw_cal = np.arctan2(my_d_cal, mx_d_cal)

declination = -14.2 * np.pi / 180
yaw_cal_true = yaw_cal - declination

yaw_raw = np.unwrap(yaw_raw)
yaw_cal_true = np.unwrap(yaw_cal_true)

plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(t_d, yaw_raw, 'b-', linewidth=1.0)
plt.ylabel('Yaw (rad)')
plt.title('Before Calibration')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t_d, yaw_cal_true, 'r-', linewidth=1.0)
plt.xlabel('Time (s)')
plt.ylabel('Yaw (rad)')
plt.title('After Calibration')
plt.grid(True)

plt.suptitle('Figure 1: Magnetometer Yaw', fontsize=14)
plt.tight_layout()
plt.savefig(f'{out}/fig1_mag_yaw.png', dpi=300)
plt.close()

gz = drive_imu['gyro_z']
yaw_gyro = cumulative_trapezoid(gz, t_d, initial=0)
yaw_gyro = yaw_gyro + yaw_cal_true[0]

plt.figure(figsize=(10, 4))
plt.plot(t_d, yaw_gyro, 'g-', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Yaw (rad)')
plt.title('Figure 2: Gyro Yaw Estimation', fontsize=14)
plt.grid(True)
plt.tight_layout()
plt.savefig(f'{out}/fig2_gyro_yaw.png', dpi=300)
plt.close()

dt = np.mean(np.diff(t_d))
fs = 1.0 / dt if dt > 0 else 100.0
cutoff = min(0.5, fs * 0.4)

mag_lp = butter_lowpass(yaw_cal_true, cutoff, fs, order=2)
gyro_hp = butter_highpass(yaw_gyro, cutoff, fs, order=2)
yaw_fused = mag_lp + gyro_hp

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

axes[0, 0].plot(t_d, mag_lp, 'b-', linewidth=1.0)
axes[0, 0].set_ylabel('Yaw (rad)')
axes[0, 0].set_title('Low Pass Mag')
axes[0, 0].grid(True)

axes[0, 1].plot(t_d, gyro_hp, 'r-', linewidth=1.0)
axes[0, 1].set_ylabel('Yaw (rad)')
axes[0, 1].set_title('High Pass Gyro')
axes[0, 1].grid(True)

axes[1, 0].plot(t_d, yaw_fused, 'g-', linewidth=1.0)
axes[1, 0].set_xlabel('Time (s)')
axes[1, 0].set_ylabel('Yaw (rad)')
axes[1, 0].set_title('Fused')
axes[1, 0].grid(True)

axes[1, 1].plot(t_d, drive_imu['yaw'], 'm-', linewidth=1.0)
axes[1, 1].set_xlabel('Time (s)')
axes[1, 1].set_ylabel('Yaw (rad)')
axes[1, 1].set_title('IMU Heading')
axes[1, 1].grid(True)

plt.suptitle('Figure 3: Complementary Filter', fontsize=14)
plt.tight_layout()
plt.savefig(f'{out}/fig3_filter.png', dpi=300)
plt.close()

ax = drive_imu['accel_x']
ax_detrend = detrend(ax)
vel_accel = cumulative_trapezoid(ax_detrend, t_d, initial=0)

t_gps = drive_gps['time']
east = drive_gps['easting']
north = drive_gps['northing']

dx = np.diff(east)
dy = np.diff(north)
dt_gps = np.diff(t_gps)
dt_gps[dt_gps == 0] = 1e-6

vel_gps = np.sqrt(dx**2 + dy**2) / dt_gps
avg_gps_vel = np.mean(vel_gps[np.isfinite(vel_gps)])

if np.mean(np.abs(vel_accel)) > 0:
    scale_factor = avg_gps_vel / np.mean(np.abs(vel_accel))
else:
    scale_factor = 1.0
vel_accel_adj = vel_accel * scale_factor

plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(t_d, vel_accel, 'b-', linewidth=1.0)
plt.ylabel('Velocity (m/s)')
plt.title('Before Adjustment')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t_d, vel_accel_adj, 'r-', linewidth=1.0)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('After Adjustment')
plt.grid(True)

plt.suptitle('Figure 4: Forward Velocity from Accelerometer', fontsize=14)
plt.tight_layout()
plt.savefig(f'{out}/fig4_vel_accel.png', dpi=300)
plt.close()

t_gps_vel = t_gps[:-1]
plt.figure(figsize=(10, 4))
plt.plot(t_gps_vel, vel_gps, 'g-', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Figure 5: Forward Velocity from GPS', fontsize=14)
plt.grid(True)
plt.tight_layout()
plt.savefig(f'{out}/fig5_vel_gps.png', dpi=300)
plt.close()

gps_init_heading = np.arctan2(north[10] - north[0], east[10] - east[0])
imu_init_heading = yaw_fused[0]
heading_offset = gps_init_heading - imu_init_heading

yaw_aligned = yaw_fused + heading_offset

vel_n = vel_accel_adj * np.cos(yaw_aligned)
vel_e = vel_accel_adj * np.sin(yaw_aligned)

pos_n_imu = cumulative_trapezoid(vel_n, t_d, initial=0)
pos_e_imu = cumulative_trapezoid(vel_e, t_d, initial=0)

pos_e_gps = east - east[0]
pos_n_gps = north - north[0]

pos_e_imu = pos_e_imu - pos_e_imu[0]
pos_n_imu = pos_n_imu - pos_n_imu[0]

fig, axes = plt.subplots(1, 2, figsize=(14, 6))

axes[0].plot(pos_e_gps, pos_n_gps, 'b-', linewidth=2)
axes[0].set_xlabel('Easting (m)')
axes[0].set_ylabel('Northing (m)')
axes[0].set_title('GPS Trajectory')
axes[0].grid(True)
axes[0].axis('equal')

axes[1].plot(pos_e_imu, pos_n_imu, 'r-', linewidth=2)
axes[1].set_xlabel('Easting (m)')
axes[1].set_ylabel('Northing (m)')
axes[1].set_title('IMU Trajectory')
axes[1].grid(True)
axes[1].axis('equal')

plt.suptitle('Figure 6: Trajectory Comparison', fontsize=14)
plt.tight_layout()
plt.savefig(f'{out}/fig6_trajectory.png', dpi=300)
plt.close()

print("All figures created!")
print(f"Calibration: cx={cx:.6f}, cy={cy:.6f}")
print(f"Soft iron: a={a:.6f}, b={b:.6f}, theta={theta:.4f}")
print(f"Velocity scale: {scale_factor:.3f}")

if __name__ == '__main__':
    main()
