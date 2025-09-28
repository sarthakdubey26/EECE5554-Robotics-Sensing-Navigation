#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from gnss.msg import Customgps
import serial
import utm
import sys
import time
from datetime import datetime

class GNSSDriver(Node):
    def __init__(self, port):
        super().__init__('gnss_driver')
        
        # Create publisher
        self.publisher = self.create_publisher(Customgps, '/gps', 10)
        
        # Setup serial connection
        try:
            self.serial_port = serial.Serial(port, 4800, timeout=1)
            self.get_logger().info(f'Connected to GPS on port: {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to GPS: {e}')
            sys.exit(1)
            
        # Create timer for reading data
        self.timer = self.create_timer(0.1, self.read_gps_data)  # 10 Hz
        
    def parse_gpgga(self, gpgga_string):
        """Parse GPGGA string and extract relevant data"""
        try:
            parts = gpgga_string.split(',')
            
            if len(parts) < 15 or parts[0] != '$GPGGA':
                return None
                
            # Extract UTC time
            utc_time = parts[1]
            if not utc_time:
                return None
                
            # Extract latitude
            lat_deg = parts[2]
            lat_dir = parts[3]
            if not lat_deg or not lat_dir:
                return None
                
            # Convert latitude from DDMM.MMMM to decimal degrees
            lat_degrees = float(lat_deg[:2])
            lat_minutes = float(lat_deg[2:])
            latitude = lat_degrees + lat_minutes / 60.0
            if lat_dir == 'S':
                latitude = -latitude
                
            # Extract longitude
            lon_deg = parts[4]
            lon_dir = parts[5]
            if not lon_deg or not lon_dir:
                return None
                
            # Convert longitude from DDDMM.MMMM to decimal degrees
            lon_degrees = float(lon_deg[:3])
            lon_minutes = float(lon_deg[3:])
            longitude = lon_degrees + lon_minutes / 60.0
            if lon_dir == 'W':
                longitude = -longitude
                
            # Extract altitude
            altitude = float(parts[9]) if parts[9] else 0.0
            
            # Extract HDOP
            hdop = float(parts[8]) if parts[8] else 0.0
            
            return {
                'utc_time': utc_time,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'hdop': hdop
            }
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error parsing GPGGA: {e}')
            return None
            
    def utc_to_epoch(self, utc_time_str):
        """Convert UTC time string to epoch time"""
        try:
            # UTC time is in format HHMMSS.SS
            if len(utc_time_str) < 6:
                return 0, 0
                
            hours = int(utc_time_str[:2])
            minutes = int(utc_time_str[2:4])
            seconds = float(utc_time_str[4:])
            
            # Get current date
            now = datetime.now()
            
            # Create datetime object with current date and GPS time
            gps_time = datetime(now.year, now.month, now.day, hours, minutes, int(seconds))
            
            # Convert to epoch time
            epoch_time = int(gps_time.timestamp())
            nanoseconds = int((seconds % 1) * 1e9)
            
            return epoch_time, nanoseconds
            
        except Exception as e:
            self.get_logger().warn(f'Error converting UTC time: {e}')
            return 0, 0
            
    def read_gps_data(self):
        """Read and process GPS data"""
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                if line.startswith('$GPGGA'):
                    parsed_data = self.parse_gpgga(line)
                    
                    if parsed_data:
                        # Convert to UTM
                        try:
                            utm_easting, utm_northing, zone, letter = utm.from_latlon(
                                parsed_data['latitude'], parsed_data['longitude']
                            )
                        except Exception as e:
                            self.get_logger().warn(f'UTM conversion failed: {e}')
                            return
                            
                        # Convert UTC time to epoch
                        epoch_sec, epoch_nsec = self.utc_to_epoch(parsed_data['utc_time'])
                        
                        # Create custom message
                        msg = Customgps()
                        
                        # Header
                        msg.header = Header()
                        msg.header.frame_id = 'GPS1_Frame'
                        msg.header.stamp.sec = epoch_sec
                        msg.header.stamp.nanosec = epoch_nsec
                        
                        # GPS data
                        msg.latitude = parsed_data['latitude']
                        msg.longitude = parsed_data['longitude']
                        msg.altitude = parsed_data['altitude']
                        msg.utm_easting = utm_easting
                        msg.utm_northing = utm_northing
                        msg.zone = zone
                        msg.letter = letter
                        msg.hdop = parsed_data['hdop']
                        msg.gpgga_read = line
                        
                        # Publish message
                        self.publisher.publish(msg)
                        self.get_logger().info(f'Published GPS data: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}')
                        
        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')
            
    def __del__(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    
    # Get port from command line arguments
    if len(sys.argv) < 2:
        print("Usage: gnss_driver.py <port>")
        print("Example: gnss_driver.py /dev/ttyUSB0")
        sys.exit(1)
        
    port = sys.argv[1]
    
    try:
        driver = GNSSDriver(port)
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
