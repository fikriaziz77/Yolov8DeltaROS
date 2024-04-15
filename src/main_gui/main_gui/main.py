import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
# import library

class MultiArrayToText(Node):
    def __init__(self):
        super().__init__('main')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bbox_data',
            self.multi_array_callback,
            1)
        self.publisher_ = self.create_publisher(String, 'serial_send', 1)

        # declare variable (use self for global)
        self.i = 123

    def multi_array_callback(self, msg):
        print(self.i) # show 123
        # pecah data bbox (x, y, w, h, r)
        x = msg.data[0]
        y = msg.data[1]
        w = msg.data[2]
        h = msg.data[3]
        r = msg.data[4]

        # process the data show gui etc

        # end process the data show gui etc

        # Publish the string as a String message
        send_serial = String()
        send_serial.data = str(r) # data yang mau dikirim ke arduino
        self.publisher_.publish(send_serial)
        self.get_logger().info('Published: %s' % str(r)) # fungsi ngeprint

def main(args=None):
    rclpy.init(args=args)
    node = MultiArrayToText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
