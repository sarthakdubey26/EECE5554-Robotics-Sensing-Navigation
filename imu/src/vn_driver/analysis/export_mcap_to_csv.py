#!/usr/bin/env python3
"""
Export MCAP file to CSV
"""

from mcap.reader import make_reader
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
            'accel_x': float(parts[7]),
            'accel_y': float(parts[8]),
            'accel_z': float(parts[9]),
            'gyro_x': float(parts[10]),
            'gyro_y': float(parts[11]),
            'gyro_z': float(parts[12].split('*')[0])
        }
    except Exception as e:
        return None

def export_mcap(mcap_file, output_csv):
    """Export MCAP file to CSV"""
    print(f"\n📂 Reading MCAP: {mcap_file}")
    print(f"   Size: {os.path.getsize(mcap_file) / 1024 / 1024:.2f} MB")
    
    with open(mcap_file, 'rb') as f:
        reader = make_reader(f)
        
        # Get summary
        summary = reader.get_summary()
        print(f"\n📊 MCAP Info:")
        print(f"   Duration: {(summary.statistics.message_end_time - summary.statistics.message_start_time) / 1e9:.2f} seconds")
        print(f"   Messages: {summary.statistics.message_count}")
        print(f"   Channels: {len(summary.channels)}")
        
        # Show channels
        print(f"\n📋 Channels:")
        for channel_id, channel in summary.channels.items():
            print(f"   - {channel.topic} ({channel.message_encoding})")
        
        # Export data
        print(f"\n📝 Exporting to {output_csv}...")
        
        with open(output_csv, 'w') as out:
            out.write("timestamp,vnymr_string\n")
            
            count = 0
            errors = 0
            
            for schema, channel, message in reader.iter_messages():
                try:
                    # Decode message data
                    data_str = message.data.decode('utf-8', errors='ignore')
                    
                    # Look for VNYMR strings
                    if '$VNYMR' in data_str:
                        # Extract just the VNYMR part
                        vnymr = data_str
                        if not vnymr.startswith('$VNYMR'):
                            # Find and extract VNYMR
                            idx = data_str.find('$VNYMR')
                            if idx >= 0:
                                vnymr = data_str[idx:]
                                # Take until end of line
                                vnymr = vnymr.split('\n')[0].split('\r')[0]
                        
                        # Verify it's parseable
                        if parse_vnymr(vnymr):
                            out.write(f'{message.log_time},"{vnymr}"\n')
                            count += 1
                            
                            if count % 1000 == 0:
                                print(f"  Exported {count} messages...")
                    else:
                        errors += 1
                        
                except Exception as e:
                    errors += 1
                    continue
        
        print(f"\n✅ Exported {count} VNYMR strings")
        print(f"⚠️  Skipped {errors} non-VNYMR messages")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 export_mcap_to_csv.py <input.mcap> <output.csv>")
        print("\nExample:")
        print("  python3 export_mcap_to_csv.py freeform_motion.mcap freeform.csv")
        sys.exit(1)
    
    export_mcap(sys.argv[1], sys.argv[2])
