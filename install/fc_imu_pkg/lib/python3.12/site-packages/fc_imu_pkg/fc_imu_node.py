#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class FlightControllerNode(Node):
    def __init__(self):
        super().__init__('fc_imu_node')
        
        # MSP command constants
        self.CMD_MSP_RAW_IMU = 102
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 100.0)  # Hz - FC can handle ~100Hz
        
        # Get parameter values
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.accel_publisher = self.create_publisher(Vector3, 'fc/accelerometer', 10)
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create MSP request packet for IMU data
        self.imu_request_packet = self.build_msp_packet(self.CMD_MSP_RAW_IMU)
        
        # Create timer for reading data
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def build_msp_packet(self, command):
        """Create a MSP request packet"""
        header = bytearray([36, 77, 60])  # $M<
        length = 0
        checksum = length ^ command
        return header + bytearray([length, command, checksum])
        
    def parse_accelerometer(self, data):
        """Parse accelerometer data from MSP response"""
        if len(data) >= 6:
            ax = int.from_bytes(data[0:2], 'little', signed=True)
            ay = int.from_bytes(data[2:4], 'little', signed=True)
            az = int.from_bytes(data[4:6], 'little', signed=True)
            return ax, ay, az
        return None
        
    def parse_gyroscope(self, data):
        """Parse gyroscope data from MSP response"""
        if len(data) >= 18:  # Accel (6 bytes) + Gyro (6 bytes) + Mag (6 bytes)
            gx = int.from_bytes(data[6:8], 'little', signed=True)
            gy = int.from_bytes(data[8:10], 'little', signed=True)
            gz = int.from_bytes(data[10:12], 'little', signed=True)
            return gx, gy, gz
        return None
    
    def timer_callback(self):
        """Read sensor data and publish"""
        try:
            # Clear any pending data
            self.ser.reset_input_buffer()
            
            # Send MSP request for IMU data
            self.ser.write(self.imu_request_packet)
            
            # Read response header
            header = self.ser.read(5)
            
            # Check if we got a valid response
            if len(header) == 5 and header[0:3] == b"$M>" and header[4] == self.CMD_MSP_RAW_IMU:
                data_len = header[3]
                data = self.ser.read(data_len)
                
                # Parse accelerometer data
                acc = self.parse_accelerometer(data)
                gyro = self.parse_gyroscope(data)
                
                if acc:
                    # Create and publish Vector3 message for raw accelerometer data
                    accel_msg = Vector3()
                    accel_msg.x = float(acc[0])
                    accel_msg.y = float(acc[1])
                    accel_msg.z = float(acc[2])
                    self.accel_publisher.publish(accel_msg)
                    
                    # Create and publish IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'
                    
                    # Convert raw values to m/s^2 
                    # Note: Adjust these scale factors based on your FC specifications
                    # Typical full scale is ±2g with 16-bit resolution
                    accel_scale = 9.80665 / 4096.0  # Convert from raw to m/s^2 (assumes ±2g range)
                    imu_msg.linear_acceleration.x = float(acc[0]) * accel_scale
                    imu_msg.linear_acceleration.y = float(acc[1]) * accel_scale
                    imu_msg.linear_acceleration.z = float(acc[2]) * accel_scale
                    
                    # Add gyro data if available
                    if gyro:
                        # Convert raw values to rad/s
                        # Typical full scale is ±2000 deg/s with 16-bit resolution
                        gyro_scale = 0.0174533 / 16.4  # Convert from raw to rad/s (assumes ±2000 deg/s range)
                        imu_msg.angular_velocity.x = float(gyro[0]) * gyro_scale
                        imu_msg.angular_velocity.y = float(gyro[1]) * gyro_scale
                        imu_msg.angular_velocity.z = float(gyro[2]) * gyro_scale
                    
                    # Set default orientation (identity quaternion)
                    imu_msg.orientation.w = 1.0
                    imu_msg.orientation.x = 0.0
                    imu_msg.orientation.y = 0.0
                    imu_msg.orientation.z = 0.0
                    
                    # Set covariance matrices (adjust based on your sensor's characteristics)
                    imu_msg.orientation_covariance = [-1.0] * 9  # -1 indicates orientation not available
                    imu_msg.angular_velocity_covariance = [0.01 if i == 0 or i == 4 or i == 8 else 0.0 for i in range(9)]
                    imu_msg.linear_acceleration_covariance = [0.01 if i == 0 or i == 4 or i == 8 else 0.0 for i in range(9)]
                    
                    # Publish the IMU message
                    self.imu_publisher.publish(imu_msg)
                    
                    self.get_logger().debug(f'Accel: x={acc[0]}, y={acc[1]}, z={acc[2]}')
                else:
                    self.get_logger().warn('Received incomplete IMU data')
            else:
                self.get_logger().warn('Invalid response header')
                
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {e}')
    
    def destroy_node(self):
        # Close serial port when node is shut down
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Closed serial port')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FlightControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()