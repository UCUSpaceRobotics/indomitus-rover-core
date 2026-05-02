import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can
import time

class CanPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')
        self.publisher = self.create_publisher(String, '/can/rx', 10)
        self.bus = None
        self.connect()
        self.timer = self.create_timer(1.0, self.send_test_message)

    def connect(self):
        try:
            self.bus = can.interface.Bus(
                bustype='slcan',
                channel='/dev/ttyUSB0',
                bitrate=500000,
                ttyBaudrate=2000000
            )
            self.get_logger().info('CAN connected')
        except Exception as e:
            self.get_logger().error(f'CAN connect failed: {e}')
            self.bus = None

    def send_test_message(self):
        if self.bus is None:
            self.get_logger().warn('Trying to reconnect...')
            self.connect()
            return

        msg = can.Message(
            arbitration_id=0x123,
            data=[0x01, 0x02, 0x03, 0x04],
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            self.get_logger().info(f'Sent: ID={msg.arbitration_id:#x} data={msg.data.hex()}')
        except can.CanError as e:
            self.get_logger().error(f'CAN error: {e}')
            self.bus.shutdown()
            self.bus = None  # наступний тік таймера спробує reconnect

    def destroy_node(self):
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()