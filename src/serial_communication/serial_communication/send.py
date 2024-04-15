import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

ser = serial.Serial(port="/dev/ttyACM1", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

def listener_callback(msg, ser):
    print('Received: %s' % msg.data)
    ser.write(msg.data.encode())

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('SendToSerial')

    subscription = node.create_subscription(
        String,
        'serial_send',
        lambda msg: listener_callback(msg, ser),
        1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
