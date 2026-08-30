import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState  # replace with your actual gimbal msg type

class JoyToGimbal(Node):

    def __init__(self):
        super().__init__('joy_to_gimbal')

        self.current_yaw = 0.0
        self.current_pitch = 0.0

        self.subscription = self.create_subscription(
            Joy,
            '/base_station/joy',
            self.joy_callback,
            5
        )

        self.publisher = self.create_publisher(
            JointState,
            '/gimbal_move_pos',
            5
        )

    def joy_callback(self, msg):
        # D-pad is usually axes[6] (left/right) and axes[7] (up/down)
        dpad_x = msg.axes[6]
        dpad_y = msg.axes[7]

        moved = False

        out_msg = JointState()
        out_msg.name = ['motor1', 'motor2']
        if dpad_y > 0.5:  # UP
            self.current_pitch += 1.0
            moved = True
            out_msg.position = [self.current_yaw, self.current_pitch]  # Example position for UP

        elif dpad_y < -0.5:  # DOWN
            self.current_pitch -= 1.0
            moved = True
            out_msg.position = [self.current_yaw, self.current_pitch]  # Example position for DOWN

        elif dpad_x > 0.5:  # RIGHT
            self.current_yaw += 1.0
            moved = True
            out_msg.position = [self.current_yaw, self.current_pitch]  # Example position for RIGHT

        elif dpad_x < -0.5:  # LEFT
            self.current_yaw -= 1.0
            moved = True
            out_msg.position = [self.current_yaw, self.current_pitch]  # Example position for LEFT

        else:
            out_msg.position = [self.current_yaw, self.current_pitch]  # No change if D-pad is neutral

        if moved:
            out_msg.velocity = []
            out_msg.effort = []
            self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToGimbal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()