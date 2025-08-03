import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.4

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
