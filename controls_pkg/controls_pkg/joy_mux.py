# This node is meant to allow switching between joint and ik control

import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from controls_msgs.msg import SystemResetMsgSentParams as Reset
from std_msgs.msg import String, Empty, Bool
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data


class JoyMux(Node):

    def __init__(self):
        super().__init__('joy_mux')

        self.declare_parameter('toggle_button', 10)
        self.declare_parameter('reset_button', 7)
        self.declare_parameter('button_threshold', 20)
        self.declare_parameter('start_ik', False)
        self.declare_parameter('joystick_1', 0)
        self.declare_parameter('joystick_2', 2)
        self.declare_parameter('message_timeout', 0.5)

        self.toggle_button = self.get_parameter('toggle_button').get_parameter_value().integer_value
        self.reset_button = self.get_parameter('reset_button').get_parameter_value().integer_value
        self.button_threshold = self.get_parameter('button_threshold').get_parameter_value().integer_value
        self.isIK = self.get_parameter('start_ik').get_parameter_value().bool_value
        self.joy1 = self.get_parameter('joystick_1').get_parameter_value().integer_value
        self.joy2 = self.get_parameter('joystick_2').get_parameter_value().integer_value

        self.joy1_safe = False
        self.joy2_safe = False
        self.last_msg = 0
        self.is_zeroing = False

        # Joy Subscriber
        self.joy_sub = self.create_subscription( Joy, 'joy', self.joy_callback, qos_profile_sensor_data)

        # Zero Subscribers
        self.zero_rail      = self.create_subscription(Bool, 'zero_rail', self.zero_motor, 10)
        self.zero_pitch     = self.create_subscription(Empty, 'zero_wrist', self.zero_motor, 10)
        self.zero_four_bar  = self.create_subscription(Empty, 'zero_four_bar', self.zero_motor, 10)

        # Zero interrupt
        self.zero_interrupt = self.create_publisher(Empty, 'stop_zeroing', 10)

        # Joint Publisher
        self.joint_pub = self.create_publisher(Joy, 'joint_joy', 10)

        # IK publisher
        self.ik_pub = self.create_publisher(Joy, 'ik_joy', 10)

        # Reset Publishers
        self.shoulder_reset_pub = self.create_publisher(Reset, "/manipulator/shoulder/send/reset",    10)
        self.elbow_reset_pub    = self.create_publisher(Reset, "/manipulator/elbow/send/reset",       10)
        self.roll_reset_pub     = self.create_publisher(Reset, "/manipulator/wrist_roll/send/reset",  10)
        self.pitch_reset_pub    = self.create_publisher(Reset, "/manipulator/wrist_pitch/send/reset", 10)
        self.rail_reset_pub     = self.create_publisher(Reset, "/manipulator/linear_rail/send/reset", 10)
        self.gripper_reset_pub     = self.create_publisher(Reset, "/manipulator/gripper/send/reset", 10)

    def zero_motor(self, msg):
        self.get_logger().info("trying to zero motor")
        self.is_zeroing = True

    def controller_zero_check(self, msg : Joy):

        for axis in msg.axes:
            if abs(axis) >= 0.1 and axis != 1.0 and axis != -1.0:
                self.get_logger().info(f"Failed because axis is {axis}")
                return False

        for button in msg.buttons:
            if button != 0:
                self.get_logger().info(f"Failed because button is {button}")
                return False

        return True

    def joy_callback(self, msg : Joy):
        # Check if the controllers have disconnected

        # If this is the first message, save the time
        if (self.last_msg == 0):
            self.last_msg = time.time()
        
        # Get and compare the current time
        cur_msg = time.time()

        if (cur_msg - self.last_msg) > self.get_parameter('message_timeout').get_parameter_value().double_value:
            self.joy1_safe = False
            self.joy2_safe = False
            self.get_logger().info("Not Safe Joys")

        # Update for next iteration
        self.last_msg = cur_msg

        # Overwrite joy_values if it isn't safe to move yet
        if not self.joy1_safe:
            # If values are stuck at max, set them to zero
            if msg.axes[self.joy1] == 1.0:

                msg.axes[self.joy1] = 0.0
                msg.axes[self.joy1 + 1] = 0.0
            else:
                self.joy1_safe = True
                self.get_logger().info("Joy 1 Safe")

        if not self.joy2_safe:
            # If values are stuck at max, set them to zero
            if msg.axes[self.joy2] == 1.0:

                msg.axes[self.joy2] = 0.0
                msg.axes[self.joy2 + 1] = 0.0
            else:
                self.joy2_safe = True
                self.get_logger().info("Joy 2 Safe")

        
        # Are we currently trying to zero?
        if self.is_zeroing:

            if not self.controller_zero_check(msg):
                self.get_logger().info("Breaking out of zeroing")
                self.is_zeroing = False
                self.zero_interrupt.publish(Empty())
            
            else:
                return

        # Handle the Resets
        if msg.buttons[self.reset_button]:
            self.reset_count += 1

            if self.reset_count == self.button_threshold:
                # Clear the current message for the switch
                msg.buttons = [0] * len(msg.buttons)
                msg.axes = [0.0] * len(msg.axes)

                self.get_logger().info("Resetting All Arm Motors")

                reset_msg = Reset()
                self.shoulder_reset_pub.publish(reset_msg)
                self.elbow_reset_pub.publish(reset_msg)
                self.roll_reset_pub.publish(reset_msg)
                self.pitch_reset_pub.publish(reset_msg)
                self.rail_reset_pub.publish(reset_msg)
                self.gripper_reset_pub.publish(reset_msg)
                return
            
        else:
            self.reset_count = 0
        

        # Handle the mode toggling
        if msg.buttons[self.toggle_button]:
            self.toggle_count += 1

            if self.toggle_count == self.button_threshold:
                self.isIK = not self.isIK
                
                # Clear the current message for the switch
                msg.buttons = [0] * len(msg.buttons)
                msg.axes = [0.0] * len(msg.axes)

        else:
            self.toggle_count = 0

        print("IK Vs Joint")
        # Send to the correct place
        if self.isIK:
            self.ik_pub.publish(msg)
        else:
            print("Sending joint pub.")
            self.joint_pub.publish(msg)

            


def main(args=None):
    rclpy.init(args=args)

    joy_mux = JoyMux()
    
    multi_threaded_executor = MultiThreadedExecutor(num_threads=2)

    multi_threaded_executor.add_node(joy_mux)

    multi_threaded_executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_threaded_executor.shutdown()
    joy_mux.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()