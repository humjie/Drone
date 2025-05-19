#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import json
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import time

class AccelerometerNode(Node):
    def __init__(self):
        super().__init__('accelerometer_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # Get parameter values
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.accel_publisher = self.create_publisher(Vector3, 'accelerometer/raw', 10)
        
        # Create timer for reading data
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Buffer for incoming data
        self.buffer = ''
        
        # For calculating orientation
        self.prev_time = self.get_clock().now()
        
    def timer_callback(self):
        # Check if there's data available to read
        if self.ser.in_waiting:
            # Read data from serial port
            try:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8')
                self.buffer += data
                
                # Process complete lines
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    self.process_data(line)
            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')
    
    def process_data(self, data_line):
        try:
            # Try to parse the data as JSON
            data = json.loads(data_line)
            
            # Check if the expected accelerometer data is present
            if 'accel_x' in data and 'accel_y' in data and 'accel_z' in data:
                # Create and publish Vector3 message
                accel_msg = Vector3()
                accel_msg.x = float(data['accel_x'])
                accel_msg.y = float(data['accel_y'])
                accel_msg.z = float(data['accel_z'])
                self.accel_publisher.publish(accel_msg)
                
                # Create and publish IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'
                
                # Linear acceleration in m/s^2 (convert if your values are in g)
                gravity = 9.80665  # m/s^2
                imu_msg.linear_acceleration.x = float(data['accel_x']) * gravity
                imu_msg.linear_acceleration.y = float(data['accel_y']) * gravity
                imu_msg.linear_acceleration.z = float(data['accel_z']) * gravity
                
                # Set angular velocity to zero if not available
                imu_msg.angular_velocity.x = float(data.get('gyro_x', 0.0))
                imu_msg.angular_velocity.y = float(data.get('gyro_y', 0.0))
                imu_msg.angular_velocity.z = float(data.get('gyro_z', 0.0))
                
                # Set default orientation quaternion (identity)
                imu_msg.orientation.w = 1.0
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                
                # Set covariance matrices (high uncertainty for values we don't measure)
                imu_msg.orientation_covariance = [-1.0] * 9  # -1 indicates orientation not available
                imu_msg.angular_velocity_covariance = [0.01 if i == 0 or i == 4 or i == 8 else 0.0 for i in range(9)]
                imu_msg.linear_acceleration_covariance = [0.01 if i == 0 or i == 4 or i == 8 else 0.0 for i in range(9)]
                
                # Publish the IMU message
                self.imu_publisher.publish(imu_msg)
                
                self.get_logger().debug(f'Published accel data: x={accel_msg.x}, y={accel_msg.y}, z={accel_msg.z}')
            else:
                self.get_logger().warn('Received data missing accelerometer readings')
                
        except json.JSONDecodeError:
            self.get_logger().warn(f'Received non-JSON data: {data_line}')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {e}')
    
    def destroy_node(self):
        # Close serial port when node is shut down
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Closed serial port')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AccelerometerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()