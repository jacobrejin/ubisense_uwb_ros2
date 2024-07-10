import rclpy
from rclpy.node import Node
from . import message_masks as masks
from . import message as ubisense_message
import socket
import struct
import time

class Ubisense():
    def __init__(self, udp_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", udp_port))
        self.print_flag = True
        self.success = False

    def parse_message_field(self, data, mask, indexes):
        field_bytes = b"".join([bytes([a & b]) for a, b in zip(data, mask)])
        field = field_bytes[indexes[0]:indexes[1]:indexes[2]].hex()
        return field

    def parse_message_number(self, data, mask, indexes, type):
        number_bytes = b"".join([bytes([a & b]) for a, b in zip(data, mask)])
        if type == "float":
            flag = 'f'
        elif type == "uint64":
            flag = 'Q'
        [number] = struct.unpack(flag, bytes.fromhex(number_bytes[indexes[0]:indexes[1]:indexes[2]].hex()))
        return number

    def spin(self):
        while rclpy.ok():
            # ONLY for debugging
            time.sleep(1)
            data = bytes.fromhex('e298026b0011ce000000f923000000011f6a9e41d80b46410000003fb2d65b3f1268311a8f13e700')
            
            # Uncomment the line below to connect to Ubisense 
            # data, _ = self.sock.recvfrom(40)
            
            self.get_logger().info(f'Received message: {data.hex()}')
            msg = self.parse(data)

            if msg and self.print_flag:
                self.get_logger().info(f"Time {msg.timestamp}: Tag {msg.tag}\n ({msg.x}m, {msg.y}m, {msg.z}m) variance {msg.variance}m")

    # create a method to be called to close the socket
    def cleanup(self):
        self.sock.close()

    def parse(self, data):
        protocol_id = self.parse_message_field(data, masks.protocol_id_mask, (0, 2, 1))
        if protocol_id != "e298":
            # not a valid protocol id
            self.success = False
            return

        message_id = self.parse_message_field(data, masks.message_id_mask, (2, 4, 1))
        if message_id != "026b":
            # not a valid message id
            self.success = False
            return

        flags = self.parse_message_field(data, masks.flags_mask, (15, 11, -1))
        if flags != "01000000":
            # flag is 0, so the data is not valid
            self.success = False
            return
        
        self.success = True

        tag = self.parse_message_field(data, masks.tag_mask, (4, 12, 1))
        timestamp = self.parse_message_number(data, masks.timestamp_mask, (39, 31, -1), "uint64")

        x = self.parse_message_number(data, masks.x_mask, (16, 20, 1), "float")
        y = self.parse_message_number(data, masks.y_mask, (20, 24, 1), "float")
        z = self.parse_message_number(data, masks.z_mask, (24, 28, 1), "float")
        variance = self.parse_message_number(data, masks.variance_mask, (28, 32, 1), "float")

        message = ubisense_message.Message(tag, timestamp, x, y, z, variance)
        return message

# def main(args=None):
#     rclpy.init(args=args)
#     udp_ip = '127.0.0.1'  # Replace with actual IP
#     udp_port = 5005  # Replace with actual port
#     ubisense_node = Ubisense(udp_ip, udp_port)
#     rclpy.spin(ubisense_node)
#     ubisense_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
