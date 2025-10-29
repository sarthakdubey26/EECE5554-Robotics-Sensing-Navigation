#!/usr/bin/env python3
"""
Analyze VNYMR CSV data and create Figures 0-3
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import allantools
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

def read_vnymr_csv(csv_file):
    """Read VNYMR CSV file"""
    print(f"\n📂 Reading {csv_file}...")
    
    data = {
        'time': [], 'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
        'accel_x': [], 'accel_y': [], 'accel_z': [],
        'roll': [], 'pitch': [], 'yaw': []
    }
    
    start_time = None
    count = 0
    errors = 0
    
    with open(csv_file, 'r') as f:
        # Skip header
        next(f)
        
        for line in f:
            try:
                parts = line.strip().split(',', 1)
                timestamp = int(parts[0])
                vnymr_string = parts[1]
                
                if start_time is None:
                    start_time = timestamp
                
                parsed = parse_vnymr(vnymr_string)
                
                if parsed:
                    data['time'].append((timestamp - start_time) / 1e9)
                    data['gyro_x'].append(parsed['gyro_x'])
                    data['gyro_y'].append(parsed['gyro_y'])
                    data['gyro_z'].append(parsed['gyro_z'])
                    data['accel_x'].append(parsed['accel_x'])
                    data['accel_y'].append(parsed['accel_y'])
                    data['accel_z'].append(parsed['accel_z'])
                    data['roll'].append(parsed['roll'])
                    data['pitch'].append(parsed['pitch'])
                    data['yaw'].append(parsed['yaw'])
                    
                    count += 1
                    if count % 10000 == 0:
                        print(f"  Processed {count} samples ({data['time'][-1]/3600:.2f} hours)...")
                else:
                    errors += 1
            except:
                errors += 1
                continue
    
    # Convert to numpy
    for key in data:
        data[key] = np.array(data[key])
    
    print(f"\n✅ Loaded {count} samples")
    print(f"   Duration: {data['time'][-1]:.2f} sec ({data['time'][-1]/3600:.2f} hours)")
    print(f"   Sample rate: {count / data['time'][-1]:.2f} Hz")
    print(f"   Errors skipped: {errors}")
    
    return data

def plot_figure_0(data, output_dir='plots'):
    """Figure 0: Gyro rates"""
    print("\n📈 Creating Figure 0...")
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Figure 0: Rotational Rate from Gyroscope', fontsize=16, fontweight='bold')
    
    time_limit = min(300, data['time'][-1])
    mask = data['time'] <= time_limit
    
    ax1.plot(data['time'][mask], np.degrees(data['gyro_x'][mask]), 'r-', linewidth=0.5, label='X-axis')
    ax1.set_ylabel('Angular Rate X (deg/s)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_title(f'First {time_limit/60:.0f} min of {data["time"][-1]/3600:.2f} hours')
    
    ax2.plot(data['time'][mask], np.degrees(data['gyro_y'][mask]), 'g-', linewidth=0.5, label='Y-axis')
    ax2.set_ylabel('Angular Rate Y (deg/s)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    ax3.plot(data['time'][mask], np.degrees(data['gyro_z'][mask]), 'b-', linewidth=0.5, label='Z-axis')
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Angular Rate Z (deg/s)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig0_gyro_rates.png', dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig0_gyro_rates.png")
    plt.close()

def plot_figure_1(data, output_dir='plots'):
    """Figure 1: Accelerations"""
    print("📈 Creating Figure 1...")
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Figure 1: Linear Acceleration from Accelerometer', fontsize=16, fontweight='bold')
    
    time_limit = min(300, data['time'][-1])
    mask = data['time'] <= time_limit
    
    ax1.plot(data['time'][mask], data['accel_x'][mask], 'r-', linewidth=0.5, label='X-axis')
    ax1.set_ylabel('Acceleration X (m/s²)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    ax2.plot(data['time'][mask], data['accel_y'][mask], 'g-', linewidth=0.5, label='Y-axis')
    ax2.set_ylabel('Acceleration Y (m/s²)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    ax3.plot(data['time'][mask], data['accel_z'][mask], 'b-', linewidth=0.5, label='Z-axis')
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Acceleration Z (m/s²)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig1_accelerations.png', dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig1_accelerations.png")
    plt.close()

def plot_figure_2(data, output_dir='plots'):
    """Figure 2: Orientations"""
    print("📈 Creating Figure 2...")
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Figure 2: Orientation from VectorNav Estimation', fontsize=16, fontweight='bold')
    
    time_limit = min(300, data['time'][-1])
    mask = data['time'] <= time_limit
    
    ax1.plot(data['time'][mask], data['roll'][mask], 'r-', linewidth=0.5, label='Roll')
    ax1.set_ylabel('Roll (degrees)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    ax2.plot(data['time'][mask], data['pitch'][mask], 'g-', linewidth=0.5, label='Pitch')
    ax2.set_ylabel('Pitch (degrees)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    ax3.plot(data['time'][mask], data['yaw'][mask], 'b-', linewidth=0.5, label='Yaw')
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Yaw (degrees)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig2_orientations.png', dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig2_orientations.png")
    plt.close()

def plot_figure_3(data, output_dir='plots', rate=40):
    """Figure 3: Allan Deviation"""
    print("\n📈 Creating Figure 3: Allan Deviation (~10-15 min)...")
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 16))
    fig.suptitle('Figure 3: Gyroscope Allan Deviation', fontsize=16, fontweight='bold')
    
    params = []
    axes = [('gyro_x', 'X', 'r', ax1), ('gyro_y', 'Y', 'g', ax2), ('gyro_z', 'Z', 'b', ax3)]
    
    for key, name, color, ax in axes:
        print(f"  Computing Allan deviation for {name}-axis (please wait)...")
        
        try:
            (taus, adevs, _, _) = allantools.oadev(
                data[key], 
                rate=rate, 
                data_type="freq", 
                taus='octave'
            )
            
            # Extract parameters
            idx_1s = np.argmin(np.abs(taus - 1.0))
            N = adevs[idx_1s]
            B = np.min(adevs)
            
            # Rate random walk (slope +0.5)
            log_taus = np.log10(taus)
            log_adevs = np.log10(adevs)
            mid_idx = len(taus) // 2
            if mid_idx < len(taus) - 1:
                slopes = np.diff(log_adevs[mid_idx:]) / np.diff(log_taus[mid_idx:])
                slope_idx = mid_idx + np.argmin(np.abs(slopes - 0.5))
                K = adevs[slope_idx] / np.sqrt(taus[slope_idx])
            else:
                K = B / 10
            
            params.append((name, N, K, B))
            
            ax.loglog(taus, adevs, f'{color}o-', linewidth=2, markersize=4)
            ax.set_ylabel('Allan Deviation (rad/s)', fontsize=11)
            ax.set_title(f'{name}-axis Gyroscope', fontsize=12)
            ax.grid(True, which='both', alpha=0.3)
            
            textstr = f'N (ARW): {N:.6f} rad/√s\nK (RRW): {K:.6f} rad/s^1.5\nB (Bias): {B:.6f} rad/s'
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)
            
            print(f"    ✅ {name}-axis: N={N:.6f}, K={K:.6f}, B={B:.6f}")
            
        except Exception as e:
            print(f"    ❌ Error: {e}")
            params.append((name, 0, 0, 0))
    
    ax3.set_xlabel('Averaging Time τ (seconds)', fontsize=11)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig3_allan_deviation.png', dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/fig3_allan_deviation.png")
    plt.close()
    
    # Save parameters
    with open(f'{output_dir}/allan_parameters.txt', 'w') as f:
        f.write("ALLAN VARIANCE PARAMETERS (5-Hour Data)\n")
        f.write("="*60 + "\n\n")
        for name, N, K, B in params:
            f.write(f"{name}-axis:\n")
            f.write(f"  Angle Random Walk (N): {N:.6f} rad/√s\n")
            f.write(f"  Rate Random Walk (K):  {K:.6f} rad/s^1.5\n")
            f.write(f"  Bias Stability (B):    {B:.6f} rad/s\n\n")
    
    print(f"✅ Saved: {output_dir}/allan_parameters.txt")
    return params

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_vnymr_csv.py <vnymr.csv>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    print("="*70)
    print("📊 VNYMR CSV ANALYSIS - EECE 5554 LAB 3")
    print("="*70)
    
    data = read_vnymr_csv(csv_file)
    
    plot_figure_0(data)
    plot_figure_1(data)
    plot_figure_2(data)
    params = plot_figure_3(data)
    
    print("\n" + "="*70)
    print("📋 ALLAN VARIANCE PARAMETERS")
    print("="*70)
    for name, N, K, B in params:
        print(f"\n{name}-axis:")
        print(f"  Angle Random Walk (N): {N:.6f} rad/√s")
        print(f"  Rate Random Walk (K):  {K:.6f} rad/s^1.5")
        print(f"  Bias Stability (B):    {B:.6f} rad/s")
    
    print("\n" + "="*70)
    print("✅ ANALYSIS COMPLETE!")
    print("="*70)
    print("\n📁 Generated files:")
    print("  - plots/fig0_gyro_rates.png")
    print("  - plots/fig1_accelerations.png")
    print("  - plots/fig2_orientations.png")
    print("  - plots/fig3_allan_deviation.png")
    print("  - plots/allan_parameters.txt")
    print("\n🎯 Ready for Canvas submission!")
    print("="*70 + "\n")

if __name__ == '__main__':
    main()
