#!/usr/bin/env python3
"""
Robust bag export that handles various database structures
"""

import sqlite3
import sys
import os

def inspect_and_export(bag_file, output_csv):
    """Inspect database structure and export data"""
    
    if not os.path.exists(bag_file):
        print(f"❌ File not found: {bag_file}")
        return
    
    print(f"\n📂 Opening: {bag_file}")
    print(f"   Size: {os.path.getsize(bag_file) / 1024 / 1024:.2f} MB")
    
    try:
        conn = sqlite3.connect(bag_file)
        cursor = conn.cursor()
    except Exception as e:
        print(f"❌ Cannot open database: {e}")
        return
    
    # Get all tables
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
    tables = [t[0] for t in cursor.fetchall()]
    print(f"\n📋 Tables found: {tables}")
    
    # Check each table structure
    for table in tables:
        cursor.execute(f"PRAGMA table_info({table})")
        columns = [col[1] for col in cursor.fetchall()]
        cursor.execute(f"SELECT COUNT(*) FROM {table}")
        count = cursor.fetchone()[0]
        print(f"\n   Table: {table}")
        print(f"     Rows: {count}")
        print(f"     Columns: {columns}")
    
    # Try to find topics
    print(f"\n🔍 Looking for topics...")
    
    if 'topics' in tables:
        cursor.execute("SELECT * FROM topics")
        topics = cursor.fetchall()
        print(f"   Found {len(topics)} topics:")
        for topic in topics:
            print(f"     {topic}")
        
        # Get topic ID for /vectornav or /imu
        topic_id = None
        topic_name = None
        for topic in topics:
            if '/vectornav' in str(topic) or 'vectornav' in str(topic):
                topic_id = topic[0]
                topic_name = topic[1] if len(topic) > 1 else 'vectornav'
                break
            elif '/imu' in str(topic):
                topic_id = topic[0]
                topic_name = topic[1] if len(topic) > 1 else 'imu'
                break
        
        if not topic_id:
            print(f"   ❌ No IMU/vectornav topic found")
            conn.close()
            return
        
        print(f"\n✅ Using topic: {topic_name} (ID: {topic_id})")
        
        # Try to read messages
        cursor.execute(f"SELECT COUNT(*) FROM messages WHERE topic_id={topic_id}")
        msg_count = cursor.fetchone()[0]
        print(f"   Messages: {msg_count}")
        
        # Export messages
        print(f"\n📝 Exporting to {output_csv}...")
        
        cursor.execute(f"""
            SELECT timestamp, data 
            FROM messages 
            WHERE topic_id={topic_id}
            ORDER BY timestamp
            LIMIT 10
        """)
        
        # Check first 10 messages to understand format
        print(f"\n🔍 Inspecting first 10 messages:")
        for i, row in enumerate(cursor.fetchall()):
            timestamp = row[0]
            data = row[1]
            print(f"\n   Message {i+1}:")
            print(f"     Timestamp: {timestamp}")
            print(f"     Data type: {type(data)}")
            print(f"     Data length: {len(data) if data else 0} bytes")
            
            # Try to decode as string
            try:
                if isinstance(data, bytes):
                    decoded = data.decode('utf-8', errors='ignore')
                    print(f"     Decoded (first 100 chars): {decoded[:100]}")
                    
                    if '$VNYMR' in decoded:
                        print(f"     ✅ Contains VNYMR string!")
                else:
                    print(f"     Data: {data}")
            except Exception as e:
                print(f"     Decode error: {e}")
        
        # Now export all data
        print(f"\n📤 Exporting all messages...")
        
        cursor.execute(f"""
            SELECT timestamp, data 
            FROM messages 
            WHERE topic_id={topic_id}
            ORDER BY timestamp
        """)
        
        with open(output_csv, 'w') as f:
            f.write("timestamp,vnymr_string\n")
            
            count = 0
            for row in cursor.fetchall():
                try:
                    timestamp = row[0]
                    data = row[1]
                    
                    if isinstance(data, bytes):
                        decoded = data.decode('utf-8', errors='ignore').strip()
                        if '$VNYMR' in decoded:
                            # Clean the string
                            vnymr = decoded.split('$VNYMR')[1].split('\n')[0]
                            vnymr = '$VNYMR' + vnymr
                            f.write(f'{timestamp},"{vnymr}"\n')
                            count += 1
                            
                            if count % 10000 == 0:
                                print(f"     Exported {count} messages...")
                except:
                    continue
        
        print(f"\n✅ Exported {count} VNYMR strings to {output_csv}")
        
    else:
        print(f"   ❌ No 'topics' table found")
        print(f"   This might not be a ROS2 bag file")
    
    conn.close()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 export_bag_robust.py <input.db3> <output.csv>")
        print("\nExample:")
        print("  python3 export_bag_robust.py 5hour_data.db3 5hour_strings.csv")
        sys.exit(1)
    
    inspect_and_export(sys.argv[1], sys.argv[2])
