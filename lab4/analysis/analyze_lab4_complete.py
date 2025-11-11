#!/usr/bin/env python3
"""
EECE 5554 Lab 4: Complete Inertial Odometry Analysis
Creates all 16 figures for circle and square walks
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import least_squares
import sys
import os

def parse_vnymr(vnymr_string):
    """Parse VNYMR string"""
    try:
        parts = vnymr_string.strip().replace('$', '').replace('"', '').split(',')
        if len(parts) < 13 or parts[0] != 'VNYMR':
            return None
        
        return {
            'yaw': float(parts[1]),
            'pitch': float(parts[2]),
            'roll': float(parts[3]),
            'mag_x': float(parts[4]) * 1e-4,
            'mag_y': float(parts[5]) * 1e-4,
            'mag_z': float(parts[6]) * 1e-4,
            'accel_x': float(parts[7]),
            'accel_y': float(parts[8]),
            'accel_z': float(parts[9]),
            'gyro_x': float(parts[10]),
            'gyro_y': float(parts[11]),
            'gyro_z': float(parts[12].split('*')[0])
        }
    except:
        return None

def read_csv(csv_file):
    """Read walking data CSV"""
    print(f"\n📂 Reading {csv_file}...")
    
    data = {
        'time': [], 'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
        'accel_x': [], 'accel_y': [], 'accel_z': [],
        'roll': [], 'pitch': [], 'yaw': [],
        'mag_x': [], 'mag_y': [], 'mag_z': []
    }
    
    with open(csv_file, 'r') as f:
        next(f)
        start_time = None
        count = 0
        
        for line in f:
            try:
                parts = line.strip().split(',', 1)
                timestamp = int(parts[0])
                
                if start_time is None:
                    start_time = timestamp
                
                parsed = parse_vnymr(parts[1])
                
                if parsed:
                    data['time'].append((timestamp - start_time) / 1e9)
                    data['gyro_x'].append(parsed['gyro_x'])
                    data['gyro_y'].append(parsed['gyro_y'])
                    data['gyro_z'].append(parsed['gyro_z'])
                    data['accel_x'].append(parsed['accel_x'])
                    data['accel_y'].append(parsed['accel_y'])
                    data['accel_z'].append(parsed['accel_z'])
                    data['mag_x'].append(parsed['mag_x'])
                    data['mag_y'].append(parsed['mag_y'])
                    data['mag_z'].append(parsed['mag_z'])
                    data['roll'].append(parsed['roll'])
                    data['pitch'].append(parsed['pitch'])
                    data['yaw'].append(parsed['yaw'])
                    count += 1
            except:
                continue
    
    for key in data:
        data[key] = np.array(data[key])
    
    print(f"✅ Loaded {count} samples ({data['time'][-1]:.1f} seconds)")
    return data

def calibrate_magnetometer(mag_x, mag_y):
    """Calibrate magnetometer using circle fit"""
    print("  Calibrating magnetometer...")
    
    def circle_residuals(params, x, y):
        cx, cy, r = params
        return np.sqrt((x - cx)**2 + (y - cy)**2) - r
    
    x0 = [np.mean(mag_x), np.mean(mag_y), 
          (np.max(mag_x) - np.min(mag_x)) / 2]
    
    result = least_squares(circle_residuals, x0, args=(mag_x, mag_y))
    hard_iron_x, hard_iron_y, radius = result.x
    
    mag_x_cal = mag_x - hard_iron_x
    mag_y_cal = mag_y - hard_iron_y
    
    print(f"    Hard iron: X={hard_iron_x:.6f}, Y={hard_iron_y:.6f}")
    
    return mag_x_cal, mag_y_cal, hard_iron_x, hard_iron_y

def create_circle_figures(data, output_dir='../plots'):
    """Create all 8 figures for circle walk"""
    print("\n📈 Creating Circle Walk Figures...")
    os.makedirs(output_dir, exist_ok=True)
    
    # FIRST: Calibrate magnetometer
    mag_x_cal, mag_y_cal, hard_iron_x, hard_iron_y = calibrate_magnetometer(
        data['mag_x'], data['mag_y']
    )
    
    # Figure 1: Magnetometer calibration
    print("  Creating Figure 1...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Figure 1: Magnetometer Calibration (Circle)', fontsize=14, fontweight='bold')
    
    ax1.plot(data['mag_x'], data['mag_y'], 'b.', markersize=2, label='Uncalibrated')
    ax1.set_xlabel('Mag X (Tesla)')
    ax1.set_ylabel('Mag Y (Tesla)')
    ax1.set_title('Before Calibration')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    
    ax2.plot(mag_x_cal, mag_y_cal, 'r.', markersize=2, label='Calibrated')
    ax2.set_xlabel('Mag X (Tesla)')
    ax2.set_ylabel('Mag Y (Tesla)')
    ax2.set_title('After Calibration')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig1_circle_mag_calibration.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Integrate gyro
    rot_x = integrate.cumtrapz(data['gyro_x'], data['time'], initial=0)
    rot_y = integrate.cumtrapz(data['gyro_y'], data['time'], initial=0)
    rot_z = integrate.cumtrapz(data['gyro_z'], data['time'], initial=0)
    
    # Figures 2-4: Gyro analysis
    print("  Creating Figures 2-4...")
    
    # Figure 2: X-axis
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 2: X-axis Gyro (Circle)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], np.degrees(data['gyro_x']), 'b-', linewidth=0.8)
    ax1.set_ylabel('Angular Rate X (deg/s)')
    ax1.set_title('Rotational Rate')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], np.degrees(rot_x), 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Rotation X (degrees)')
    ax2.set_title('Integrated Rotation')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig2_circle_x_rotation.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 3: Y-axis
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 3: Y-axis Gyro (Circle)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], np.degrees(data['gyro_y']), 'b-', linewidth=0.8)
    ax1.set_ylabel('Angular Rate Y (deg/s)')
    ax1.set_title('Rotational Rate')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], np.degrees(rot_y), 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Rotation Y (degrees)')
    ax2.set_title('Integrated Rotation')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig3_circle_y_rotation.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 4: Z-axis with mag heading
    mag_head = np.degrees(np.arctan2(mag_y_cal, mag_x_cal))
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Figure 4: Z-axis Gyro and Mag Heading (Circle)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], np.degrees(data['gyro_z']), 'b-', linewidth=0.8)
    ax1.set_ylabel('Angular Rate Z (deg/s)')
    ax1.set_title('Rotational Rate')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], np.degrees(rot_z), 'r-', linewidth=0.8)
    ax2.set_ylabel('Rotation Z (degrees)')
    ax2.set_title('Integrated Rotation')
    ax2.grid(True, alpha=0.3)
    ax3.plot(data['time'], mag_head, 'g-', linewidth=0.8)
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Heading (degrees)')
    ax3.set_title('Magnetometer Heading')
    ax3.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig4_circle_z_heading.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Integrate accelerations
    vel_x = integrate.cumtrapz(data['accel_x'], data['time'], initial=0)
    vel_y = integrate.cumtrapz(data['accel_y'], data['time'], initial=0)
    vel_z = integrate.cumtrapz(data['accel_z'], data['time'], initial=0)
    
    # Figures 5-7: Acceleration and velocity
    print("  Creating Figures 5-7...")
    
    # Figure 5: X
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 5: X-axis Accel (Circle)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], data['accel_x'], 'b-', linewidth=0.8)
    ax1.set_ylabel('Acceleration X (m/s²)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], vel_x, 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity X (m/s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig5_circle_x_accel_vel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 6: Y
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 6: Y-axis Accel (Circle)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], data['accel_y'], 'b-', linewidth=0.8)
    ax1.set_ylabel('Acceleration Y (m/s²)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], vel_y, 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity Y (m/s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig6_circle_y_accel_vel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 7: Z
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 7: Z-axis Accel (Circle)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], data['accel_z'], 'b-', linewidth=0.8)
    ax1.set_ylabel('Acceleration Z (m/s²)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], vel_z, 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity Z (m/s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig7_circle_z_accel_vel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 8: Position plot
    print("  Creating Figure 8: Position...")
    disp_x = integrate.cumtrapz(vel_x, data['time'], initial=0)
    
    heading_mag_rad = np.radians(mag_head)
    pos_n_mag = integrate.cumtrapz(disp_x * np.cos(heading_mag_rad), data['time'], initial=0)
    pos_e_mag = integrate.cumtrapz(disp_x * np.sin(heading_mag_rad), data['time'], initial=0)
    
    heading_gyro_rad = rot_z
    pos_n_gyro = integrate.cumtrapz(disp_x * np.cos(heading_gyro_rad), data['time'], initial=0)
    pos_e_gyro = integrate.cumtrapz(disp_x * np.sin(heading_gyro_rad), data['time'], initial=0)
    
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(pos_e_mag, pos_n_mag, 'b-', linewidth=1.5, label='Mag Heading')
    ax.plot(pos_e_gyro, pos_n_gyro, 'r--', linewidth=1.5, label='Gyro Heading')
    ax.plot(pos_e_mag[0], pos_n_mag[0], 'go', markersize=10, label='Start')
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_title('Figure 8: Position (Circle)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig8_circle_position.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Return calibration parameters
    return {'hard_iron_x': hard_iron_x, 'hard_iron_y': hard_iron_y}

def create_square_figures(data, calib_params, output_dir='../plots'):
    """Create all 8 figures for square walk"""
    print("\n📈 Creating Square Walk Figures...")
    
    # Apply calibration from circle
    mag_x_cal = data['mag_x'] - calib_params['hard_iron_x']
    mag_y_cal = data['mag_y'] - calib_params['hard_iron_y']
    
    # Figure 9
    print("  Creating Figure 9...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Figure 9: Magnetometer Calibration (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['mag_x'], data['mag_y'], 'b.', markersize=2, label='Uncalibrated')
    ax1.set_xlabel('Mag X (Tesla)')
    ax1.set_ylabel('Mag Y (Tesla)')
    ax1.set_title('Before Calibration')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    ax2.plot(mag_x_cal, mag_y_cal, 'r.', markersize=2, label='Calibrated')
    ax2.set_xlabel('Mag X (Tesla)')
    ax2.set_ylabel('Mag Y (Tesla)')
    ax2.set_title('After Calibration')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.legend()
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig9_square_mag_calibration.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Integrate gyro
    rot_x = integrate.cumtrapz(data['gyro_x'], data['time'], initial=0)
    rot_y = integrate.cumtrapz(data['gyro_y'], data['time'], initial=0)
    rot_z = integrate.cumtrapz(data['gyro_z'], data['time'], initial=0)
    
    # Figures 10-12
    print("  Creating Figures 10-12...")
    
    # Figure 10: X
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 10: X-axis Gyro (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], np.degrees(data['gyro_x']), 'b-', linewidth=0.8)
    ax1.set_ylabel('Angular Rate X (deg/s)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], np.degrees(rot_x), 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Rotation X (degrees)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig10_square_x_rotation.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 11: Y
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 11: Y-axis Gyro (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], np.degrees(data['gyro_y']), 'b-', linewidth=0.8)
    ax1.set_ylabel('Angular Rate Y (deg/s)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], np.degrees(rot_y), 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Rotation Y (degrees)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig11_square_y_rotation.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 12: Z with mag heading
    mag_head = np.degrees(np.arctan2(mag_y_cal, mag_x_cal))
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Figure 12: Z-axis Gyro and Mag Heading (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], np.degrees(data['gyro_z']), 'b-', linewidth=0.8)
    ax1.set_ylabel('Angular Rate Z (deg/s)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], np.degrees(rot_z), 'r-', linewidth=0.8)
    ax2.set_ylabel('Rotation Z (degrees)')
    ax2.grid(True, alpha=0.3)
    ax3.plot(data['time'], mag_head, 'g-', linewidth=0.8)
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Heading (degrees)')
    ax3.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig12_square_z_heading.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Integrate accelerations
    vel_x = integrate.cumtrapz(data['accel_x'], data['time'], initial=0)
    vel_y = integrate.cumtrapz(data['accel_y'], data['time'], initial=0)
    vel_z = integrate.cumtrapz(data['accel_z'], data['time'], initial=0)
    
    # Figures 13-15
    print("  Creating Figures 13-15...")
    
    # Figure 13: X
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 13: X-axis Accel (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], data['accel_x'], 'b-', linewidth=0.8)
    ax1.set_ylabel('Acceleration X (m/s²)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], vel_x, 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity X (m/s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig13_square_x_accel_vel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 14: Y
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 14: Y-axis Accel (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], data['accel_y'], 'b-', linewidth=0.8)
    ax1.set_ylabel('Acceleration Y (m/s²)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], vel_y, 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity Y (m/s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig14_square_y_accel_vel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 15: Z
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Figure 15: Z-axis Accel (Square)', fontsize=14, fontweight='bold')
    ax1.plot(data['time'], data['accel_z'], 'b-', linewidth=0.8)
    ax1.set_ylabel('Acceleration Z (m/s²)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(data['time'], vel_z, 'r-', linewidth=0.8)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity Z (m/s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig15_square_z_accel_vel.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Figure 16: Position
    print("  Creating Figure 16: Position...")
    disp_x = integrate.cumtrapz(vel_x, data['time'], initial=0)
    
    heading_mag_rad = np.radians(mag_head)
    pos_n_mag = integrate.cumtrapz(disp_x * np.cos(heading_mag_rad), data['time'], initial=0)
    pos_e_mag = integrate.cumtrapz(disp_x * np.sin(heading_mag_rad), data['time'], initial=0)
    
    heading_gyro_rad = rot_z
    pos_n_gyro = integrate.cumtrapz(disp_x * np.cos(heading_gyro_rad), data['time'], initial=0)
    pos_e_gyro = integrate.cumtrapz(disp_x * np.sin(heading_gyro_rad), data['time'], initial=0)
    
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(pos_e_mag, pos_n_mag, 'b-', linewidth=1.5, label='Mag Heading')
    ax.plot(pos_e_gyro, pos_n_gyro, 'r--', linewidth=1.5, label='Gyro Heading')
    ax.plot(pos_e_mag[0], pos_n_mag[0], 'go', markersize=10, label='Start')
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_title('Figure 16: Position (Square)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig16_square_position.png', dpi=300, bbox_inches='tight')
    plt.close()

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 analyze_lab4_complete.py circle.csv square.csv")
        sys.exit(1)
    
    circle_csv = sys.argv[1]
    square_csv = sys.argv[2]
    
    print("="*70)
    print("📊 LAB 4: INERTIAL ODOMETRY ANALYSIS")
    print("="*70)
    
    # Circle walk
    circle_data = read_csv(circle_csv)
    calib_params = create_circle_figures(circle_data)
    
    # Square walk
    square_data = read_csv(square_csv)
    create_square_figures(square_data, calib_params)
    
    print("\n" + "="*70)
    print("✅ ALL 16 FIGURES COMPLETE!")
    print("="*70)
    print("\n📁 Check ../plots/ directory")
    print("="*70 + "\n")

if __name__ == '__main__':
    main()
