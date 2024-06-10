import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
import time
import socket

from .ubisense import Ubisense


class UWBPublisher(Node):
    def __init__(self):
        super().__init__('ubisense_uwb_publisher')
        
        self.node_name = self.get_name()
        self.get_logger().info(f"{self.node_name} started")

        # Load the configuration file
        self.declare_parameter('udp_ip', '127.0.0.1')
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('tag', 'tag1')
        self.declare_parameter('debug', False)

        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.tag = self.get_parameter('tag').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value



        self.get_logger().info(f"Parameters loaded: {self.udp_ip}:{self.udp_port}")
        self.get_logger().info(f"Tag: {self.tag}")
        self.get_logger().info(f"Debug mode: {self.debug}")

        self.ubisense = Ubisense(self.udp_ip, self.udp_port)
        self.publisher_ = self.create_publisher(PoseWithCovariance, 'location', 10)
        self.timer = self.create_timer(0.1, self.spin)

    def spin(self):
        if self.debug:
            time.sleep(1)
            data = bytes.fromhex('e298026b0011ce000000f923000000011f6a9e41d80b46410000003fb2d65b3f1268311a8f13e700')
        else:
            data, _ = self.ubisense.sock.recvfrom(40)

        self.get_logger().info(f'Received message: {data.hex()}')

        ubisense_msg = self.ubisense.parse(data)

        if ubisense_msg.tag == self.tag:
            msg = PoseWithCovariance()
            pose = Pose()

            point = Point()
            point.x = ubisense_msg.x
            point.y = ubisense_msg.y
            point.z = ubisense_msg.z
            pose.position = point

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = 0.0
            quaternion.w = 1.0
            pose.orientation = quaternion

            msg.pose = pose

            msg.covariance[0] = ubisense_msg.variance ** 2
            msg.covariance[7] = ubisense_msg.variance ** 2
            msg.covariance[14] = ubisense_msg.variance ** 2
            msg.covariance[21] = 1000000000000.0
            msg.covariance[28] = 1000000000000.0
            msg.covariance[35] = 1000000000000.0

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    uwb_publisher = UWBPublisher()
    rclpy.spin(uwb_publisher)
    # uwb_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
