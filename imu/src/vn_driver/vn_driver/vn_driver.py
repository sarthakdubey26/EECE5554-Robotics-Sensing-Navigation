#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from custom_msgs.msg import Vectornav
import serial
import sys
import math
import time
from scipy.spatial.transform import Rotation as R

class VectorNavDriver(Node):
    def __init__(self, port):
        super().__init__('vn_driver')
        
        # Create publisher to /imu topic
        self.publisher = self.create_publisher(Vectornav, '/imu', 10)
        
        # Statistics tracking
        self.total_reads = 0
        self.successful_parses = 0
        self.parse_errors = 0
        
        # Open serial port with increased timeout
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=2.0,  # Increased timeout
                inter_byte_timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f'VectorNav IMU connected on {port}')
            
            # Wait for device to stabilize
            time.sleep(0.5)
            
            # Clear any old data in buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # Configure VectorNav to output at 40 Hz
            self.configure_output_rate(40)
            
            # Clear buffer again after configuration
            time.sleep(0.5)
            self.serial_port.reset_input_buffer()
            self.get_logger().info('Serial buffers cleared')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to IMU: {e}')
            sys.exit(1)
        
        # Create timer for reading data (50 Hz polling for 40 Hz data)
        self.timer = self.create_timer(0.02, self.read_imu_data)
        
        # Statistics timer (every 10 seconds)
        self.stats_timer = self.create_timer(10.0, self.print_statistics)
        
    def configure_output_rate(self, rate_hz):
        """
        Configure VectorNav output rate by writing to register.
        Register 7 controls serial output rate.
        Format: $VNWRG,07,RATE*CHECKSUM
        """
        try:
            # Create command string
            cmd_base = f"VNWRG,07,{rate_hz}"
            
            # Calculate checksum (XOR of all characters)
            checksum = 0
            for char in cmd_base:
                checksum ^= ord(char)
            
            # Format complete command
            cmd = f"${cmd_base}*{checksum:02X}\r\n"
            
            self.get_logger().info(f'Configuring to {rate_hz} Hz: {cmd.strip()}')
            
            # Send command
            self.serial_port.write(cmd.encode('utf-8'))
            
            # Wait for device to process
            time.sleep(0.5)
            
            self.get_logger().info(f'IMU configured to {rate_hz} Hz')
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure output rate: {e}')
    
    def convert_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        
        VectorNav outputs: Roll, Pitch, Yaw in DEGREES
        ROS expects: Quaternion (x, y, z, w)
        
        VectorNav uses ZYX (yaw-pitch-roll) intrinsic rotation order
        """
        # Convert degrees to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Use scipy for conversion (ZYX order for VectorNav)
        rotation = R.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        
        return quat[0], quat[1], quat[2], quat[3]
    
    def parse_vnymr(self, vnymr_string):
        """
        Parse VNYMR string from VectorNav.
        
        Format: $VNYMR,YAW,PITCH,ROLL,MAG_X,MAG_Y,MAG_Z,ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z*CHECKSUM
        
        VectorNav Units (from manual):
        - Yaw, Pitch, Roll: degrees
        - Magnetometer: Gauss → Convert to Tesla (×1e-4)
        - Acceleration: m/s² (already correct)
        - Angular Rate (Gyro): rad/s (already correct)
        """
        try:
            # Remove $ and whitespace
            vnymr_string = vnymr_string.strip().replace('$', '')
            
            # Verify checksum is present
            if '*' not in vnymr_string:
                return None
            
            # Split by comma
            parts = vnymr_string.split(',')
            
            # Verify we have exactly 13 parts (header + 12 data fields)
            if len(parts) < 13:
                self.get_logger().debug(f'Incomplete string: only {len(parts)} parts')
                return None
            
            # Verify header
            if parts[0] != 'VNYMR':
                return None
            
            # Parse each field with individual error handling
            try:
                yaw = float(parts[1])
                pitch = float(parts[2])
                roll = float(parts[3])
            except ValueError as e:
                self.get_logger().debug(f'Failed to parse orientation: {e}')
                return None
            
            try:
                mag_x = float(parts[4]) * 1e-4  # Gauss to Tesla
                mag_y = float(parts[5]) * 1e-4
                mag_z = float(parts[6]) * 1e-4
            except ValueError as e:
                self.get_logger().debug(f'Failed to parse magnetometer: {e}')
                return None
            
            try:
                accel_x = float(parts[7])
                accel_y = float(parts[8])
                accel_z = float(parts[9])
            except ValueError as e:
                self.get_logger().debug(f'Failed to parse accelerometer: {e}')
                return None
            
            try:
                gyro_x = float(parts[10])
                gyro_y = float(parts[11])
                # Remove checksum from last field
                gyro_z_str = parts[12].split('*')[0]
                gyro_z = float(gyro_z_str)
            except (ValueError, IndexError) as e:
                self.get_logger().debug(f'Failed to parse gyroscope: {e}')
                return None
            
            # Convert Euler angles to quaternion
            qx, qy, qz, qw = self.convert_to_quaternion(roll, pitch, yaw)
            
            return {
                'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
                'angular_velocity': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                'linear_acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
                'magnetic_field': {'x': mag_x, 'y': mag_y, 'z': mag_z}
            }
            
        except Exception as e:
            self.get_logger().debug(f'Unexpected parse error: {e}')
            return None
    
    def read_imu_data(self):
        """Read and publish IMU data with improved error handling"""
        try:
            # Check if data is available
            if self.serial_port.in_waiting > 0:
                self.total_reads += 1
                
                try:
                    # Read one complete line
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Skip if line is too short (incomplete)
                    if len(line) < 50:
                        return
                    
                    # Only process VNYMR strings with checksums
                    if line.startswith('$VNYMR') and '*' in line:
                        # Verify string has expected number of commas (should be 12)
                        comma_count = line.count(',')
                        if comma_count < 12:
                            self.get_logger().debug(f'Incomplete VNYMR: {comma_count} commas, expected 12')
                            return
                        
                        # Parse the string
                        parsed = self.parse_vnymr(line)
                        
                        if parsed is not None:
                            # Create Vectornav message
                            msg = Vectornav()
                            
                            # Header with timestamp and frame_id
                            msg.header = Header()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = 'imu1_frame'
                            
                            # IMU message
                            msg.imu = Imu()
                            msg.imu.header = msg.header
                            
                            # Orientation (quaternion)
                            msg.imu.orientation.x = parsed['orientation']['x']
                            msg.imu.orientation.y = parsed['orientation']['y']
                            msg.imu.orientation.z = parsed['orientation']['z']
                            msg.imu.orientation.w = parsed['orientation']['w']
                            
                            # Angular velocity (rad/s - already correct)
                            msg.imu.angular_velocity.x = parsed['angular_velocity']['x']
                            msg.imu.angular_velocity.y = parsed['angular_velocity']['y']
                            msg.imu.angular_velocity.z = parsed['angular_velocity']['z']
                            
                            # Linear acceleration (m/s² - already correct)
                            msg.imu.linear_acceleration.x = parsed['linear_acceleration']['x']
                            msg.imu.linear_acceleration.y = parsed['linear_acceleration']['y']
                            msg.imu.linear_acceleration.z = parsed['linear_acceleration']['z']
                            
                            # Magnetic field message
                            msg.mag_field = MagneticField()
                            msg.mag_field.header = msg.header
                            msg.mag_field.magnetic_field.x = parsed['magnetic_field']['x']
                            msg.mag_field.magnetic_field.y = parsed['magnetic_field']['y']
                            msg.mag_field.magnetic_field.z = parsed['magnetic_field']['z']
                            
                            # Raw IMU string
                            msg.raw_imu_string = line
                            
                            # Publish message
                            self.publisher.publish(msg)
                            self.successful_parses += 1
                        else:
                            self.parse_errors += 1
                            
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial exception: {e}')
                except UnicodeDecodeError as e:
                    self.get_logger().debug(f'Unicode decode error: {e}')
                    
        except Exception as e:
            self.get_logger().error(f'Unexpected read error: {e}')
    
    def print_statistics(self):
        """Print statistics about data collection"""
        if self.total_reads > 0:
            success_rate = (self.successful_parses / self.total_reads) * 100
            self.get_logger().info(
                f'Stats: {self.successful_parses} good / {self.total_reads} total '
                f'({success_rate:.1f}% success rate, {self.parse_errors} errors)'
            )

def main(args=None):
    rclpy.init(args=args)
    
    # Get port from command line arguments
    if len(sys.argv) < 2:
        print("Usage: ros2 run vn_driver vn_driver <port>")
        print("Example: ros2 run vn_driver vn_driver /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    node = VectorNavDriver(port)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down VectorNav driver')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
