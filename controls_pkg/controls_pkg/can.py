import rclpy, threading 
from rclpy.node import Node
import can

from controls_msgs.msg import CanMessage

TIMEOUT_SEC = 0.01

class CanNode(Node):

    thread_lock = threading.Lock()

    def __init__(self):
        super().__init__(node_name='can_interface', namespace='/can_interface')

        self.declare_parameter('can_network_id', 'can0')

        self.can_network_id = self.get_parameter('can_network_id').get_parameter_value().string_value

        self.sub = self.create_subscription(
            CanMessage,
            'send',
            self.send_can_command,
            10,
        )

        self.pub = self.create_publisher(
            CanMessage,
            'rcvd',
            10,
        )

        self.bus = can.Bus(
            interface       = 'socketcan', 
            channel         = self.can_network_id, 
            is_extended_id  = False, 
            bitrate         = 1000000, 
        )
        
    def send_can_command(self, ros_can_msg: CanMessage):

        new_sent_msg = can.Message(
            arbitration_id=ros_can_msg.arbitration_id,
            data=ros_can_msg.data,
            is_extended_id=False,
            dlc=8,
        )
        
        with CanNode.thread_lock:
            try:
                self.bus.send(new_sent_msg, TIMEOUT_SEC)
                recv_msg = self.bus.recv(TIMEOUT_SEC)
            except Exception as e:
                recv_msg = None
                self.get_logger().error(f"Error communicating with the can bus: {e}")
            
        if recv_msg is not None:
            new_recv_msg = CanMessage()
            new_recv_msg.arbitration_id = recv_msg.arbitration_id
            new_recv_msg.data = recv_msg.data
            self.pub.publish(new_recv_msg)
        else:
            self.get_logger().error(f"The motor id {ros_can_msg.arbitration_id} was not found")
        
        
def main():
    rclpy.init()
    can_node = CanNode()
    rclpy.spin(can_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
