import serial
import struct
import time

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

def main():
    port = "/dev/ttyACM0"  # change this to your FC serial port
    baudrate = 115200

    print(f"Opening serial port {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # wait for FC to be ready

    try:
        print("Spinning motors at low throttle (1200)...")
        motor_pwms = [1050, 1050, 1050, 1050]  # low throttle
        send_motor_command(ser, motor_pwms)

        time.sleep(3)  # spin for 3 seconds

        print("Stopping motors...")
        motor_pwms = [1000, 1000, 1000, 1000]  # stop
        send_motor_command(ser, motor_pwms)

    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
