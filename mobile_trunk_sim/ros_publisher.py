import std_msgs
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/summit_xl_control/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        linear = [0, 1, 1]
        angular = [0, 0, 0]
        msg = Float32MultiArray(layout=std_msgs.msg.MultiArrayLayout( data_offset=0), data=[linear[0],linear[1],
                                                                    linear[2],angular[0],angular[1],angular[2]])
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()