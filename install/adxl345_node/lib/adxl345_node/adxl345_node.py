"""
ROS2 node for reading ADXL345 accelerometer data from an ESP device.
Publishes sensor data to ROS2 topics.
"""

import time
import json
import serial
from typing import Dict, Tuple, Any, Optional, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ADXL345Node(Node):
    """
    ROS2 Node for reading data from ADXL345 accelerometer via ESP.
    
    This node communicates with an ESP device that's connected to an
    ADXL345 accelerometer. The node publishes the accelerometer data
    as ROS2 sensor_msgs/Imu messages.
    """
    
    def __init__(self) -> None:
        """Initialize the ADXL345 ROS2 node."""
        super().__init__('adxl345_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Setup QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Setup publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile)
        
        # Setup publisher for raw acceleration data
        self.accel_pub = self.create_publisher(
            Float32MultiArray, 'imu/accel_raw', qos_profile)
        
        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        
        # Connect to serial
        self.connect()
        
        # Create timer for publishing data
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('ADXL345 node initialized')
        
    def connect(self) -> bool:
        """
        Establish connection to the ESP device.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.connected = True
            self.get_logger().info(f"Connected to ESP on {self.port}")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to ESP: {str(e)}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        """Close the serial connection to ESP."""
        if self.serial_conn and self.connected:
            self.serial_conn.close()
            self.connected = False
            self.get_logger().info("Disconnected from ESP")
    
    def read_acceleration(self) -> Optional[Dict[str, float]]:
        """
        Read acceleration data from ADXL345 via ESP.
        
        Returns:
            Dict with x, y, z acceleration values or None if reading failed
        """
        if not self.connected or not self.serial_conn:
            self.get_logger().error("Not connected to ESP")
            return None
            
        try:
            # Send command to ESP to request accelerometer data
            self.serial_conn.write(b'READ_ACCEL\n')
            
            # Read response with timeout
            start_time = time.time()
            response = ""
            while time.time() - start_time < self.timeout:
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode('utf-8').strip()
                    break
                time.sleep(0.01)
            
            if not response:
                self.get_logger().warning("No response from ESP")
                return None
            
            # Parse JSON response
            data = json.loads(response)
            
            return {
                'x': float(data.get('x', 0.0)),
                'y': float(data.get('y', 0.0)),
                'z': float(data.get('z', 0.0))
            }
        except (serial.SerialException, json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().error(f"Error reading acceleration data: {str(e)}")
            return None
    
    def timer_callback(self) -> None:
        """Read accelerometer data and publish it to ROS2 topics."""
        accel_data = self.read_acceleration()
        
        if accel_data:
            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Set acceleration data (in m/s^2, assuming ADXL345 returns in g's, multiply by 9.81)
            imu_msg.linear_acceleration.x = accel_data['x'] * 9.81
            imu_msg.linear_acceleration.y = accel_data['y'] * 9.81
            imu_msg.linear_acceleration.z = accel_data['z'] * 9.81
            
            # Set other fields to zero or NaN as we don't have them
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            
            # Set covariance matrices
            # -1 indicates that the covariance is unknown
            imu_msg.orientation_covariance = [-1.0] * 9
            imu_msg.angular_velocity_covariance = [-1.0] * 9
            imu_msg.linear_acceleration_covariance = [-1.0] * 9
            
            # Publish IMU message
            self.imu_pub.publish(imu_msg)
            
            # Also publish raw acceleration values
            accel_raw_msg = Float32MultiArray()
            accel_raw_msg.layout = MultiArrayLayout()
            accel_raw_msg.layout.dim = [MultiArrayDimension()]
            accel_raw_msg.layout.dim[0].label = "acceleration"
            accel_raw_msg.layout.dim[0].size = 3
            accel_raw_msg.layout.dim[0].stride = 3
            accel_raw_msg.data = [accel_data['x'], accel_data['y'], accel_data['z']]
            
            self.accel_pub.publish(accel_raw_msg)
            
            self.get_logger().debug(
                f"Published accel: X={accel_data['x']:.2f}, Y={accel_data['y']:.2f}, Z={accel_data['z']:.2f}"
            )
        else:
            self.get_logger().warning("Failed to read acceleration data")
            
            # Try to reconnect if needed
            if not self.connected:
                self.get_logger().info("Attempting to reconnect...")
                self.connect()

    def calibrate(self) -> bool:
        """
        Send calibration command to ESP for ADXL345.
        
        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self.connected or not self.serial_conn:
            self.get_logger().error("Not connected to ESP")
            return False
            
        try:
            # Send calibration command
            self.serial_conn.write(b'CALIBRATE\n')
            
            # Read response
            response = self.serial_conn.readline().decode('utf-8').strip()
            
            if "CALIBRATION_DONE" in response:
                self.get_logger().info("Calibration successful")
                return True
            else:
                self.get_logger().error(f"Calibration failed: {response}")
                return False
        except serial.SerialException as e:
            self.get_logger().error(f"Error during calibration: {str(e)}")
            return False


def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)
    
    adxl345_node = ADXL345Node()
    
    try:
        rclpy.spin(adxl345_node)
    except KeyboardInterrupt:
        pass
    finally:
        adxl345_node.disconnect()
        adxl345_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()