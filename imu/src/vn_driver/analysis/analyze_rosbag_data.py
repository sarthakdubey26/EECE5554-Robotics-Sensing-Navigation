#!/usr/bin/env python3
"""
EECE 5554 Lab 3: ROS2 Bag IMU Data Analysis
Analyzes ROS2 bag with VectorNav data to create Figures 0-3 and extract Allan variance
"""

import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import allantools
import sys
import os

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
    rotation = R.from_quat([x, y, z, w])
    euler = rotation.as_euler('ZYX', degrees=True)
    return euler[2], euler[1], euler[0]  # roll, pitch, yaw

def read_rosbag(bag_file):
    """Read ROS2 bag file and extract IMU data"""
    print(f"\n📂 Reading ROS2 bag: {bag_file}")
    
    # Connect to SQLite database
    conn = sqlite3.connect(bag_file)
    cursor = conn.cursor()
    
    # Get topic information
    cursor.execute("SELECT id, name, type FROM topics WHERE name='/imu'")
    topic_info = cursor.fetchone()
    
    if not topic_info:
        print("❌ Error: /imu topic not found in bag!")
        conn.close()
        return None
    
    topic_id = topic_info[0]
    print(f"✅ Found topic: {topic_info[1]} ({topic_info[2]})")
    
    # Read messages
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id = ?
        ORDER BY timestamp
    """, (topic_id,))
    
    data = {
        'time': [], 'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
        'accel_x': [], 'accel_y': [], 'accel_z': [],
        'roll': [], 'pitch': [], 'yaw': []
    }
    
    print("📖 Parsing messages...")
    
    message_count = 0
    start_time = None
    
    for row in cursor.fetchall():
        timestamp_ns = row[0]
        msg_data = row[1]
        
        if start_time is None:
            start_time = timestamp_ns
        
        # Convert timestamp to seconds from start
        time_sec = (timestamp_ns - start_time) / 1e9
        
        # Parse the message data (CDR serialization)
        # This is a simplified parser - actual CDR parsing is complex
        # We'll extract the quaternion and sensor data from the binary blob
        
        try:
            # Skip CDR header (4 bytes)
            offset = 4
            
            # Skip header timestamp (8 bytes sec + 4 bytes nanosec)
            offset += 12
            
            # Skip frame_id string (4 bytes length + string + padding)
            frame_id_len = int.from_bytes(msg_data[offset:offset+4], 'little')
            offset += 4 + frame_id_len
            # Align to 4 bytes
            offset = ((offset + 3) // 4) * 4
            
            # IMU message starts here
            # Skip IMU header (timestamp + frame_id)
            offset += 12  # timestamp
            imu_frame_len = int.from_bytes(msg_data[offset:offset+4], 'little')
            offset += 4 + imu_frame_len
            offset = ((offset + 3) // 4) * 4
            
            # Quaternion (4 doubles = 32 bytes)
            qx = np.frombuffer(msg_data[offset:offset+8], dtype=np.float64)[0]
            qy = np.frombuffer(msg_data[offset+8:offset+16], dtype=np.float64)[0]
            qz = np.frombuffer(msg_data[offset+16:offset+24], dtype=np.float64)[0]
            qw = np.frombuffer(msg_data[offset+24:offset+32], dtype=np.float64)[0]
            offset += 32
            
            # Skip quaternion covariance (9 doubles = 72 bytes)
            offset += 72
            
            # Angular velocity (3 doubles = 24 bytes)
            gyro_x = np.frombuffer(msg_data[offset:offset+8], dtype=np.float64)[0]
            gyro_y = np.frombuffer(msg_data[offset+8:offset+16], dtype=np.float64)[0]
            gyro_z = np.frombuffer(msg_data[offset+16:offset+24], dtype=np.float64)[0]
            offset += 24
            
            # Skip angular velocity covariance (9 doubles = 72 bytes)
            offset += 72
            
            # Linear acceleration (3 doubles = 24 bytes)
            accel_x = np.frombuffer(msg_data[offset:offset+8], dtype=np.float64)[0]
            accel_y = np.frombuffer(msg_data[offset+8:offset+16], dtype=np.float64)[0]
            accel_z = np.frombuffer(msg_data[offset+16:offset+24], dtype=np.float64)[0]
            
            # Convert quaternion to Euler
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
            
            # Store data
            data['time'].append(time_sec)
            data['gyro_x'].append(gyro_x)
            data['gyro_y'].append(gyro_y)
            data['gyro_z'].append(gyro_z)
            data['accel_x'].append(accel_x)
            data['accel_y'].append(accel_y)
            data['accel_z'].append(accel_z)
            data['roll'].append(roll)
            data['pitch'].append(pitch)
            data['yaw'].append(yaw)
            
            message_count += 1
            
            # Progress update
            if message_count % 10000 == 0:
                print(f"  Processed {message_count} messages ({time_sec/3600:.2f} hours)...")
                
        except Exception as e:
            # Skip corrupted messages
            continue
    
    conn.close()
    
    # Convert to numpy arrays
    for key in data:
        data[key] = np.array(data[key])
    
    print(f"\n✅ Loaded {len(data['time'])} samples")
    print(f"   Duration: {data['time'][-1]:.2f} seconds ({data['time'][-1]/3600:.2f} hours)")
    if len(data['time']) > 1:
        sample_rate = len(data['time']) / data['time'][-1]
        print(f"   Sample rate: {sample_rate:.2f} Hz")
    
    return data

def plot_figure_0(data, output_dir='plots'):
    """Figure 0: Rotational Rate (Gyro) in deg/s"""
    print("\n📈 Creating Figure 0: Rotational Rates...")
    
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Figure 0: Rotational Rate from Gyroscope', fontsize=16, fontweight='bold')
    
    # Only plot first 5 minutes for visibility
    time_limit = min(300, data['time'][-1])
    mask = data['time'] <= time_limit
    
    ax1.plot(data['time'][mask], np.degrees(data['gyro_x'][mask]), 'r-', linewidth=0.5, label='X-axis')
    ax1.set_ylabel('Angular Rate X (deg/s)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    ax1.set_title(f'Showing first {time_limit/60:.0f} minutes of {data["time"][-1]/3600:.2f} hours', fontsize=10)
    
    ax2.plot(data['time'][mask], np.degrees(data['gyro_y'][mask]), 'g-', linewidth=0.5, label='Y-axis')
    ax2.set_ylabel('Angular Rate Y (deg/s)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    
    ax3.plot(data['time'][mask], np.degrees(data['gyro_z'][mask]), 'b-', linewidth=0.5, label='Z-axis')
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Angular Rate Z (deg/s)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig0_gyro_rates.png'), dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig0_gyro_rates.png")
    plt.close()

def plot_figure_1(data, output_dir='plots'):
    """Figure 1: Linear Acceleration in m/s²"""
    print("📈 Creating Figure 1: Linear Accelerations...")
    
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Figure 1: Linear Acceleration from Accelerometer', fontsize=16, fontweight='bold')
    
    time_limit = min(300, data['time'][-1])
    mask = data['time'] <= time_limit
    
    ax1.plot(data['time'][mask], data['accel_x'][mask], 'r-', linewidth=0.5, label='X-axis')
    ax1.set_ylabel('Acceleration X (m/s²)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    ax1.set_title(f'Showing first {time_limit/60:.0f} minutes of {data["time"][-1]/3600:.2f} hours', fontsize=10)
    
    ax2.plot(data['time'][mask], data['accel_y'][mask], 'g-', linewidth=0.5, label='Y-axis')
    ax2.set_ylabel('Acceleration Y (m/s²)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    
    ax3.plot(data['time'][mask], data['accel_z'][mask], 'b-', linewidth=0.5, label='Z-axis')
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Acceleration Z (m/s²)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig1_accelerations.png'), dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig1_accelerations.png")
    plt.close()

def plot_figure_2(data, output_dir='plots'):
    """Figure 2: Orientation (Euler angles) in degrees"""
    print("📈 Creating Figure 2: Orientations...")
    
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Figure 2: Orientation from VectorNav Estimation', fontsize=16, fontweight='bold')
    
    time_limit = min(300, data['time'][-1])
    mask = data['time'] <= time_limit
    
    ax1.plot(data['time'][mask], data['roll'][mask], 'r-', linewidth=0.5, label='Roll')
    ax1.set_ylabel('Roll (degrees)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    ax1.set_title(f'Showing first {time_limit/60:.0f} minutes of {data["time"][-1]/3600:.2f} hours', fontsize=10)
    
    ax2.plot(data['time'][mask], data['pitch'][mask], 'g-', linewidth=0.5, label='Pitch')
    ax2.set_ylabel('Pitch (degrees)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    
    ax3.plot(data['time'][mask], data['yaw'][mask], 'b-', linewidth=0.5, label='Yaw')
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Yaw (degrees)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig2_orientations.png'), dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig2_orientations.png")
    plt.close()

def extract_allan_params(taus, adevs):
    """Extract Allan variance parameters"""
    idx_1s = np.argmin(np.abs(taus - 1.0))
    N = adevs[idx_1s]
    
    B = np.min(adevs)
    
    log_taus = np.log10(taus)
    log_adevs = np.log10(adevs)
    mid_idx = len(taus) // 2
    if mid_idx < len(taus) - 1:
        slopes = np.diff(log_adevs[mid_idx:]) / np.diff(log_taus[mid_idx:])
        slope_idx = mid_idx + np.argmin(np.abs(slopes - 0.5))
        K = adevs[slope_idx] / np.sqrt(taus[slope_idx])
    else:
        K = 0.0
    
    return N, K, B

def plot_figure_3(data, output_dir='plots', rate=40):
    """Figure 3: Allan Deviation"""
    print("\n📈 Creating Figure 3: Allan Deviation (this may take 10-15 minutes)...")
    
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 16))
    fig.suptitle('Figure 3: Gyroscope Allan Deviation', fontsize=16, fontweight='bold')
    
    axes = [('gyro_x', 'X', 'r', ax1), ('gyro_y', 'Y', 'g', ax2), ('gyro_z', 'Z', 'b', ax3)]
    
    params_summary = []
    
    for axis_key, axis_name, color, ax in axes:
        print(f"  Computing Allan deviation for {axis_name}-axis...")
        
        gyro_data = data[axis_key]
        
        try:
            (taus, adevs, errors, ns) = allantools.oadev(
                gyro_data,
                rate=rate,
                data_type="freq",
                taus='octave'
            )
            
            N, K, B = extract_allan_params(taus, adevs)
            params_summary.append((axis_name, N, K, B))
            
            ax.loglog(taus, adevs, f'{color}o-', linewidth=2, markersize=4)
            ax.set_ylabel(f'Allan Deviation (rad/s)', fontsize=11)
            ax.set_title(f'{axis_name}-axis Gyroscope', fontsize=12)
            ax.grid(True, which='both', alpha=0.3)
            
            textstr = f'N (ARW): {N:.6f} rad/√s\nK (RRW): {K:.6f} rad/s^1.5\nB (Bias): {B:.6f} rad/s'
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)
            
            print(f"    ✅ {axis_name}-axis: N={N:.6f}, K={K:.6f}, B={B:.6f}")
            
        except Exception as e:
            print(f"    ❌ Error: {e}")
            params_summary.append((axis_name, 0, 0, 0))
    
    ax3.set_xlabel('Averaging Time τ (seconds)', fontsize=11)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fig3_allan_deviation.png'), dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig3_allan_deviation.png")
    plt.close()
    
    params_file = os.path.join(output_dir, 'allan_parameters.txt')
    with open(params_file, 'w') as f:
        f.write("ALLAN VARIANCE PARAMETERS (5-Hour Data)\n")
        f.write("="*60 + "\n\n")
        for axis_name, N, K, B in params_summary:
            f.write(f"{axis_name}-axis:\n")
            f.write(f"  Angle Random Walk (N): {N:.6f} rad/√s\n")
            f.write(f"  Rate Random Walk (K):  {K:.6f} rad/s^1.5\n")
            f.write(f"  Bias Stability (B):    {B:.6f} rad/s\n\n")
    
    print(f"✅ Saved: {params_file}")
    return params_summary

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_rosbag_data.py <bag_file.db3>")
        print("\nExample:")
        print("  python3 analyze_rosbag_data.py 5_hour_imu_data_0.db3")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    
    if not os.path.exists(bag_file):
        print(f"❌ Error: File '{bag_file}' not found!")
        sys.exit(1)
    
    print("=" * 70)
    print("📊 ROS2 BAG IMU DATA ANALYSIS - EECE 5554 LAB 3")
    print("=" * 70)
    
    data = read_rosbag(bag_file)
    
    if data is None:
        sys.exit(1)
    
    plot_figure_0(data)
    plot_figure_1(data)
    plot_figure_2(data)
    
    params = plot_figure_3(data)
    
    print("\n" + "=" * 70)
    print("📋 ALLAN VARIANCE PARAMETERS")
    print("=" * 70)
    for axis_name, N, K, B in params:
        print(f"\n{axis_name}-axis:")
        print(f"  Angle Random Walk (N): {N:.6f} rad/√s")
        print(f"  Rate Random Walk (K):  {K:.6f} rad/s^1.5")
        print(f"  Bias Stability (B):    {B:.6f} rad/s")
    
    print("\n" + "=" * 70)
    print("✅ ANALYSIS COMPLETE!")
    print("=" * 70)
    print("\n📁 Generated files:")
    print("  - plots/fig0_gyro_rates.png")
    print("  - plots/fig1_accelerations.png")
    print("  - plots/fig2_orientations.png")
    print("  - plots/fig3_allan_deviation.png")
    print("  - plots/allan_parameters.txt")
    print("\n🎯 Upload these to Canvas!")
    print("=" * 70 + "\n")

if __name__ == '__main__':
    main()
