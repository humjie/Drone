#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial
import time
import threading

class FlightControllerBridge(Node):
    def __init__(self):
        super().__init__('flight_controller_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/tty_betaflight_fc')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('throttle_scale', 1000.0)  # Scale for throttle values
        self.declare_parameter('roll_pitch_scale', 45.0)  # Scale for roll/pitch in degrees
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.throttle_scale = self.get_parameter('throttle_scale').value
        self.roll_pitch_scale = self.get_parameter('roll_pitch_scale').value
        
        # Initialize serial connection
        self.get_logger().info(f"Connecting to flight controller at {self.serial_port}")
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
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
        
        # Data structure to hold the latest control values
        self.controls = {
            'throttle': 0.0,  # altitude from joystick
            'roll': 0.0,      # left/right from joystick
            'pitch': 0.0,     # forward/backward from joystick
            'yaw': 0.0        # we're not controlling yaw in this example
        }
        
        # Start a thread to continuously send commands to flight controller
        self.running = True
        self.command_thread = threading.Thread(target=self.command_loop)
        self.command_thread.start()
        
        self.get_logger().info("Flight controller bridge initialized")
    
    def joystick_callback(self, msg):
        """Process incoming joystick data"""
        # Map Vector3 data to flight controls
        self.controls['throttle'] = msg.x  # altitude
        self.controls['pitch'] = -msg.y    # forward/backward (negative because forward is negative pitch)
        self.controls['roll'] = msg.z      # left/right
        
        self.get_logger().debug(f"Received joystick data: thr={self.controls['throttle']:.2f}, "
                               f"pitch={self.controls['pitch']:.2f}, roll={self.controls['roll']:.2f}")
    
    def command_loop(self):
        """Thread to continuously send commands to the flight controller"""
        while self.running and rclpy.ok():
            try:
                # Convert control values to the format expected by BetaFPV F4
                # This will vary depending on the protocol your FC uses (MSP, SBUS, etc.)
                command = self.format_command(
                    throttle=self.controls['throttle'],
                    roll=self.controls['roll'],
                    pitch=self.controls['pitch'],
                    yaw=self.controls['yaw']
                )
                
                # Send the command
                self.ser.write(command)
                
                # Pace the commands
                time.sleep(0.02)  # 50Hz update rate
                
            except Exception as e:
                self.get_logger().error(f"Error in command loop: {e}")
                time.sleep(1.0)  # Wait before retry
    
    def format_command(self, throttle, roll, pitch, yaw):
        """Format control values for BetaFPV F4 controller using MSP protocol
        
        Note: This is a simplified example for MSP protocol. You'll need to
        adjust this based on the specific protocol your FC uses.
        """
        # Scale the values to the expected range
        throttle_scaled = int(1000 + throttle * self.throttle_scale)
        throttle_scaled = max(1000, min(2000, throttle_scaled))  # Clamp to 1000-2000
        
        roll_scaled = int(1500 + roll * self.roll_pitch_scale)
        roll_scaled = max(1000, min(2000, roll_scaled))  # Clamp to 1000-2000
        
        pitch_scaled = int(1500 + pitch * self.roll_pitch_scale)
        pitch_scaled = max(1000, min(2000, pitch_scaled))  # Clamp to 1000-2000
        
        yaw_scaled = 1500  # Center yaw (no control in this example)
        
        # MSP protocol format for SET_RAW_RC command
        # This is a simplified implementation - adjust based on your FC's requirements
        msp_data = bytearray()
        msp_data.extend(b'$M<')  # MSP header
        msp_data.append(16)      # Data length (8 channels × 2 bytes)
        msp_data.append(200)     # Command (MSP_SET_RAW_RC)
        
        # Add channel data (throttle, roll, pitch, yaw and 4 aux channels)
        channels = [roll_scaled, pitch_scaled, throttle_scaled, yaw_scaled, 1000, 1000, 1000, 1000]
        for ch in channels:
            msp_data.append(ch & 0xFF)          # Low byte
            msp_data.append((ch >> 8) & 0xFF)   # High byte
        
        # Calculate checksum
        checksum = 0
        for i in range(3, len(msp_data)):
            checksum ^= msp_data[i]
        msp_data.append(checksum)
        
        return msp_data
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.command_thread.is_alive():
            self.command_thread.join(2.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = FlightControllerBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()