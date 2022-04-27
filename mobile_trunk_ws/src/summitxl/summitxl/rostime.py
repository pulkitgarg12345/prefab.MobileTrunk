from time import sleep
import rclpy
import std_msgs
from std_msgs.msg import Int32MultiArray

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('rostime')
    publisher = node.create_publisher(Int32MultiArray, "/summit_xl/clock", 10)

    while rclpy.ok():
        t = node.get_clock().now().to_msg()

        msg = Int32MultiArray(layout=std_msgs.msg.MultiArrayLayout( data_offset=0), data=[t.sec, t.nanosec])
        publisher.publish(msg)
        sleep(0.5)  # seconds

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()