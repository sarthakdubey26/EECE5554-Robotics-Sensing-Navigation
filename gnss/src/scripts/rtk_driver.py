#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from gnss.msg import Customrtk
import serial
import utm
import sys
from datetime import datetime

class RTKDriver(Node):
    def __init__(self, port):
        super().__init__('rtk_driver')
        self.publisher = self.create_publisher(Customrtk, '/gps', 10)
        
        try:
            self.serial_port = serial.Serial(port, 4800, timeout=1)
            self.get_logger().info(f'RTK GPS connected: {port}')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            sys.exit(1)
            
        self.timer = self.create_timer(0.1, self.read_rtk_data)
        
    def parse_gngga(self, gngga_string):
        try:
            parts = gngga_string.split(',')
            if len(parts) < 15 or parts[0] != '$GNGGA':
                return None
                
            utc_time = parts[1]
            if not utc_time:
                return None
                
            lat_deg = parts[2]
            lat_dir = parts[3]
            if not lat_deg or not lat_dir:
                return None
                
            lat_degrees = float(lat_deg[:2])
            lat_minutes = float(lat_deg[2:])
            latitude = lat_degrees + lat_minutes / 60.0
            if lat_dir == 'S':
                latitude = -latitude
                
            lon_deg = parts[4]
            lon_dir = parts[5]
            if not lon_deg or not lon_dir:
                return None
                
            lon_degrees = float(lon_deg[:3])
            lon_minutes = float(lon_deg[3:])
            longitude = lon_degrees + lon_minutes / 60.0
            if lon_dir == 'W':
                longitude = -longitude
            
            fix_quality = int(parts[6]) if parts[6] else 0
            hdop = float(parts[8]) if parts[8] else 0.0
            altitude = float(parts[9]) if parts[9] else 0.0
            
            return {
                'utc_time': utc_time,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'fix_quality': fix_quality,
                'hdop': hdop
            }
        except (ValueError, IndexError):
            return None
            
    def utc_to_epoch(self, utc_time_str):
        try:
            if len(utc_time_str) < 6:
                return 0, 0
            hours = int(utc_time_str[:2])
            minutes = int(utc_time_str[2:4])
            seconds = float(utc_time_str[4:])
            now = datetime.now()
            gps_time = datetime(now.year, now.month, now.day, hours, minutes, int(seconds))
            return int(gps_time.timestamp()), int((seconds % 1) * 1e9)
        except:
            return 0, 0
            
    def read_rtk_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line.startswith('$GNGGA'):
                    parsed_data = self.parse_gngga(line)
                    if parsed_data:
                        try:
                            utm_easting, utm_northing, zone, letter = utm.from_latlon(
                                parsed_data['latitude'], parsed_data['longitude'])
                        except:
                            return
                            
                        epoch_sec, epoch_nsec = self.utc_to_epoch(parsed_data['utc_time'])
                        
                        msg = Customrtk()
                        msg.header = Header()
                        msg.header.frame_id = 'GPS1_Frame'
                        msg.header.stamp.sec = epoch_sec
                        msg.header.stamp.nanosec = epoch_nsec
                        msg.latitude = parsed_data['latitude']
                        msg.longitude = parsed_data['longitude']
                        msg.altitude = parsed_data['altitude']
                        msg.utm_easting = utm_easting
                        msg.utm_northing = utm_northing
                        msg.zone = zone
                        msg.letter = letter
                        msg.fix_quality = parsed_data['fix_quality']
                        msg.hdop = parsed_data['hdop']
                        msg.gngga_read = line
                        
                        self.publisher.publish(msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: rtk_driver.py <port>")
        sys.exit(1)
    port = sys.argv[1]
    try:
        driver = RTKDriver(port)
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
