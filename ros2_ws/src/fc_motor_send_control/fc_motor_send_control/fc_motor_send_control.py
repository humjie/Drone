import serial
import struct
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

def msp_checksum(data_bytes):
    csum = 0
    for b in data_bytes:
        csum ^= b
    return csum

def build_msp_command(command, payload_bytes):
    '''
    Builds a full MSP command packet.

    Format:
    $M< length command payload checksum

    - command: uint8 command code (int)
    - payload_bytes: bytes payload
    '''
    length = len(payload_bytes)
    header = b"$M<"
    length_byte = struct.pack("B", length)
    command_byte = struct.pack("B", command)
    csum = msp_checksum(length_byte + command_byte + payload_bytes)
    packet = header + length_byte + command_byte + payload_bytes + struct.pack("B", csum)
    return packet

def send_motor_command(ser, motor_pwms):
    '''
    motor_pwms: list of 4 motor PWM values (uint16), typical 1000-2000us
    '''
    # Pack motor PWMs as 4 uint16 little endian
    payload = struct.pack("<HHHH", *motor_pwms)
    packet = build_msp_command(214, payload)  # 214 = MSP_SET_RAW_MOTOR
    ser.write(packet)
    ser.flush()

class FCMotorController(Node):
    def __init__(self):
        super().__init__('fc_motor_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('min_throttle', 1000)
        self.declare_parameter('max_throttle', 2000)
        self.declare_parameter('motor_mixing_strength', 0.5)  # How strongly roll/pitch/yaw affect motors
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.mixing_strength = self.get_parameter('motor_mixing_strength').value
        
        # Initialize serial connection
        self.get_logger().info(f"Connecting to flight controller at {self.serial_port}")
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            time.sleep(2)  # wait for FC to be ready
            self.get_logger().info("Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e
            
        # Create subscription to joystick data
        self.subscription = self.create_subscription(
            Vector3,
            'joystick_data',
            self.joystick_callback,
            10)
        
        # Print information about the subscription
        self.get_logger().info("====================================")
        self.get_logger().info("Subscribed to topic: joystick_data")
        self.get_logger().info("Message type: geometry_msgs/Vector3")
        self.get_logger().info("Adding roll/pitch/yaw control")
        self.get_logger().info("====================================")
        print("\033[1;32m") # Green and bold text
        print("⭐ FC Motor Controller Active ⭐")
        print("Subscribed to topic: \033[1;36mjoystick_data\033[1;32m")
        print("Message type: \033[1;36mgeometry_msgs/Vector3\033[1;32m")
        print("Using advanced motor mixing for roll/pitch/yaw control")
        print("\033[0m") # Reset text formatting
        
        # Initialize motor values
        self.motor_values = [self.min_throttle] * 4  # All motors at minimum throttle
        
        # Initialize control values
        self.throttle = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Count of received messages
        self.msg_count = 0
        
        # Motor layout for quadcopter (+) configuration:
        #    M1(front)
        #       |
        # M4 ---+--- M2
        #       |
        #    M3(back)
        
        self.get_logger().info("FC Motor Controller initialized with roll/pitch/yaw support")
    
    def joystick_callback(self, msg):
        """Process incoming joystick data"""
        # Extract control values from message
        self.throttle = msg.x  # Throttle from x component (0.0 to 1.0)
        self.pitch = msg.y     # Pitch from y component (-1.0 to 1.0)
        self.roll = msg.z      # Roll from z component (-1.0 to 1.0)
        
        # For this example, we're not using yaw (could add another message type or use Vector3Stamped)
        
        # Scale throttle from 0-1 to min_throttle-max_throttle
        throttle_base = int(self.min_throttle + self.throttle * (self.max_throttle - self.min_throttle))
        throttle_base = max(self.min_throttle, min(self.max_throttle, throttle_base))
        
        # Calculate motor mix based on roll, pitch, and yaw
        # For a quadcopter in + configuration:
        # Motor 1 (front): throttle + pitch - yaw
        # Motor 2 (right): throttle + roll + yaw
        # Motor 3 (back): throttle - pitch - yaw
        # Motor 4 (left): throttle - roll + yaw
        
        # Calculate the amount of adjustment for pitch/roll/yaw
        # The adjustment range depends on throttle to avoid stalling motors at low throttle
        max_adjustment = int((throttle_base - self.min_throttle) * self.mixing_strength)
        
        # Calculate motor values with mixing
        m1 = throttle_base + int(self.pitch * max_adjustment) - int(self.yaw * max_adjustment)
        m2 = throttle_base + int(self.roll * max_adjustment) + int(self.yaw * max_adjustment)
        m3 = throttle_base - int(self.pitch * max_adjustment) - int(self.yaw * max_adjustment)
        m4 = throttle_base - int(self.roll * max_adjustment) + int(self.yaw * max_adjustment)
        
        # Ensure all values are within min-max throttle range
        m1 = max(self.min_throttle, min(self.max_throttle, m1))
        m2 = max(self.min_throttle, min(self.max_throttle, m2))
        m3 = max(self.min_throttle, min(self.max_throttle, m3))
        m4 = max(self.min_throttle, min(self.max_throttle, m4))
        
        # Update motor values
        self.motor_values = [m1, m2, m3, m4]
        
        # Increment message counter
        self.msg_count += 1
        
        # Print received message data to terminal
        print(f"\033[K\rMsg #{self.msg_count}: thr={self.throttle:.2f}, pitch={self.pitch:.2f}, roll={self.roll:.2f} → motors=[{m1}, {m2}, {m3}, {m4}]", end="")
        
        # Send command to flight controller
        try:
            send_motor_command(self.ser, self.motor_values)
            self.get_logger().debug(f"Sent motor command: {self.motor_values}")
        except Exception as e:
            self.get_logger().error(f"Error sending motor command: {e}")
    
    def destroy_node(self):
        """Clean shutdown"""
        # Ensure motors are stopped before shutting down
        stop_values = [self.min_throttle] * 4
        try:
            send_motor_command(self.ser, stop_values)
            self.get_logger().info("Motors stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")
        
        # Close serial connection
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed")
        
        print("\n\033[1;33mShutting down FC Motor Controller...\033[0m")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = FCMotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()