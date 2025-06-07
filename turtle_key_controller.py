import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TurtleKeyController(Node):
    def __init__(self):
        # initializing the Node class using to enable ros2 communications
        super().__init__('turtle_key_controller')
        # creating the publisher
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("TurtleKeyController Node started. Use W/A/S/D to move the turtle.")
        self.run()

    def get_key(self):
        file_description = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_description)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(file_description, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()
            if key == 'w':  # Move forward
                twist.linear.x = 1.0
                twist.angular.z = 0.0
            elif key == 's':  # Move backward
                twist.linear.x = -1.0
                twist.angular.z = 0.0
            elif key == 'a':  # Turn left
                twist.linear.x = 0.0
                twist.angular.z = 1.0
            elif key == 'd':  # Turn right
                twist.linear.x = 0.0
                twist.angular.z = -1.0
            elif key == '\x03':  # Exit on Ctrl+C
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleKeyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
