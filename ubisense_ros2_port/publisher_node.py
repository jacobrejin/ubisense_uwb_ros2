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
        # self.declare_parameter('udp_ip', '127.0.0.1')
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('tag', 'tag1')
        self.declare_parameter('debug', False)
        self.declare_parameter('logging_level', 0)
        #     topic: "ubisense_uwb/location"    # Topic name to which the position data needs to be published
        self.declare_parameter('topic', 'ubisense_uwb/pose')

        # self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.tag = self.get_parameter('tag').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.logging_level = self.get_parameter('logging_level').get_parameter_value().integer_value

        self.get_logger().info(f"Parameters loaded: {self.udp_port}")
        self.get_logger().info(f"Tag: {self.tag}")
        self.get_logger().info(f"Debug mode: {self.debug}")

        self.ubisense = Ubisense(self.udp_port)
        self.publisher_ = self.create_publisher(PoseWithCovariance, self.topic, 10)
        self.timer = self.create_timer(0.1, self.spin)

        self.buffer = b''
        self.got_udp_data = False


       

    def spin(self):
        if self.debug:
            time.sleep(1)
            data = bytes.fromhex('e298026b0011ce000000f923000000011f6a9e41d80b46410000003fb2d65b3f1268311a8f13e700')
        else:
            # data, _ = self.ubisense.sock.recvfrom(40)
            data, address = self.ubisense.sock.recvfrom(100)  # Receiving more bytes to handle extra data
            sender_ip, _ = address

            # There is no need to check for the senders IP as it keeps changing,
            # also we are binignd the port number of our socket to which the data is being send, so we can instead just use that
            # Check if the sender's IP matches the allowed IP
            # if sender_ip == self.udp_ip:
            if self.logging_level == 1:
                self.get_logger().info(f'Received message: {data.hex()}')
                self.get_logger().info(f"Received {len(data)} bytes from {address}")
            
            self.buffer += data
            
            while len(self.buffer) >= 40:
                # Find the start of the next message
                start_index = self.buffer.find(b'\xE2\x98')
                if start_index == -1:
                    # No valid start found, clear buffer
                    self.buffer = b''
                    break
                
                # Remove any leading bytes before the valid start
                if start_index > 0:
                    self.buffer = self.buffer[start_index:]
                
                # Check if we have enough bytes for a full message
                if len(self.buffer) < 40:
                    break

                # Extract the 40-byte message
                message_data = self.buffer[:40]
                self.buffer = self.buffer[40:]

                if message_data[2:4] == b'\x02\x6B':  # Check the Message ID
                    self.got_udp_data = True
                else:
                    print("Invalid message ID, discarded")

        if self.got_udp_data:
            self.got_udp_data = False
            # self.get_logger().info(f'Received message: {message_data.hex()}')

            ubisense_msg = self.ubisense.parse(message_data)
            
            if self.logging_level == 1:
                self.get_logger().info("parsed data")
                self.get_logger().info(f"Tag: {ubisense_msg.tag}")

            # if the data is valid, also checks for the flag which indicated a valid positon data
            if self.ubisense.success:
                # if the tag id is the same as the one we defined in the parameter
                if ubisense_msg.tag == self.tag:
                    msg = PoseWithCovariance()
                    pose = Pose()

                    point = Point()
                    point.x = ubisense_msg.x
                    point.y = ubisense_msg.y
                    point.z = ubisense_msg.z
                    quaternion = Quaternion()
                    quaternion.x = 0.0
                    quaternion.y = 0.0
                    quaternion.z = 0.0
                    quaternion.w = 1.0

                    pose.position = point
                    pose.orientation = quaternion
                    msg.pose = pose

                    msg.covariance[0] = ubisense_msg.variance
                    msg.covariance[7] = ubisense_msg.variance
                    msg.covariance[14] = ubisense_msg.variance
                    # we do not need to square or square root is as we are already getting the variance
                    # msg.covariance[0] = ubisense_msg.variance ** 2
                    # msg.covariance[7] = ubisense_msg.variance ** 2
                    # msg.covariance[14] = ubisense_msg.variance ** 2
                    msg.covariance[21] = 1000000000000.0
                    msg.covariance[28] = 1000000000000.0
                    msg.covariance[35] = 1000000000000.0

                    self.publisher_.publish(msg)

                    if self.logging_level == 1:
                        self.get_logger().info(f"Time {ubisense_msg.timestamp}: Tag {ubisense_msg.tag}\n ({ubisense_msg.x}m, {ubisense_msg.y}m, variance {ubisense_msg.variance}m")


def main(args=None):
    rclpy.init(args=args)
    uwb_publisher = UWBPublisher()
    rclpy.spin(uwb_publisher)
    uwb_publisher.ubisense.cleanup()
    uwb_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
