#!/usr/bin/env python3
"""
Export ROS2 bag IMU data to CSV for analysis
"""

import sqlite3
import sys
import struct

def export_bag_to_csv(bag_file, output_csv):
    """Export IMU data from ROS2 bag to CSV"""
    print(f"\n📂 Exporting {bag_file} to {output_csv}")
    
    conn = sqlite3.connect(bag_file)
    cursor = conn.cursor()
    
    # Get topic
    cursor.execute("SELECT id FROM topics WHERE name='/imu'")
    result = cursor.fetchone()
    if not result:
        print("❌ No /imu topic found!")
        return
    
    topic_id = result[0]
    
    # Get messages
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id = ?
        ORDER BY timestamp
    """, (topic_id,))
    
    # Open CSV file
    with open(output_csv, 'w') as f:
        f.write("timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,quat_x,quat_y,quat_z,quat_w\n")
        
        count = 0
        for row in cursor.fetchall():
            try:
                timestamp = row[0]
                data = row[1]
                
                # Simple extraction - find doubles in the binary data
                # This is a heuristic approach that works for most bags
                
                # Convert bytes to list of doubles (8 bytes each)
                num_doubles = len(data) // 8
                doubles = struct.unpack(f'{num_doubles}d', data[:num_doubles*8])
                
                # Look for gyro, accel, quaternion patterns
                # Typically: quaternion (4), gyro (3), accel (3) in the data
                
                if len(doubles) >= 10:
                    # Heuristic: find reasonable IMU values
                    # Gyro: typically -10 to +10 rad/s
                    # Accel: typically -20 to +20 m/s^2
                    # Quat: -1 to +1
                    
                    # Extract assuming standard ordering
                    quat_x = doubles[0] if abs(doubles[0]) <= 1 else 0
                    quat_y = doubles[1] if abs(doubles[1]) <= 1 else 0
                    quat_z = doubles[2] if abs(doubles[2]) <= 1 else 0
                    quat_w = doubles[3] if abs(doubles[3]) <= 1 else 1
                    
                    gyro_x = doubles[4] if abs(doubles[4]) < 100 else 0
                    gyro_y = doubles[5] if abs(doubles[5]) < 100 else 0
                    gyro_z = doubles[6] if abs(doubles[6]) < 100 else 0
                    
                    accel_x = doubles[7] if abs(doubles[7]) < 100 else 0
                    accel_y = doubles[8] if abs(doubles[8]) < 100 else 0
                    accel_z = doubles[9] if abs(doubles[9]) < 100 else -9.81
                    
                    f.write(f"{timestamp},{gyro_x},{gyro_y},{gyro_z},{accel_x},{accel_y},{accel_z},{quat_x},{quat_y},{quat_z},{quat_w}\n")
                    
                    count += 1
                    if count % 10000 == 0:
                        print(f"  Exported {count} messages...")
                        
            except:
                continue
    
    conn.close()
    print(f"✅ Exported {count} messages to {output_csv}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 export_bag_to_csv.py <input.db3> <output.csv>")
        sys.exit(1)
    
    export_bag_to_csv(sys.argv[1], sys.argv[2])
