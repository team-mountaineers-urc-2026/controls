import rclpy
import time
import yaml
from os import path
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from controls_msgs.msg import ReadMotorStatus2MsgSentParams as SendStatus2, ReadMotorStatus2MsgRecvParams as RecvStatus2, SpeedClosedLoopControlMsgSentParams as SendSpeed
from std_msgs.msg import Empty, Float32, Bool, String
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from numpy import sign, savetxt

# From setuptools import findpackages

class ArmZeroer(Node):

    def __init__(self):
        super().__init__('arm_zeroer')

        # Parameters
        self.declare_parameter('polling_freq', 200.0)
        self.declare_parameter('polling_timeout', 15.0)
        self.declare_parameter('config_filepath', '../config/zeros.yaml')
        self.declare_parameter('current_thresh', 1.0)

        self.polling_period     = 1.0 / self.get_parameter('polling_freq').get_parameter_value().double_value
        self.polling_timeout    = self.get_parameter('polling_timeout').get_parameter_value().double_value
        self.config_filepath    = self.get_parameter('config_filepath').get_parameter_value().string_value
        self.rail_thresh        = self.get_parameter('current_thresh').get_parameter_value().double_value

        # Topics to request motor positions
        self.rail_pos_req           = self.create_publisher(SendStatus2, 'linear_rail/send/status2', 10)
        self.shoulder_pos_req       = self.create_publisher(SendStatus2, 'shoulder/send/status2', 10)
        self.elbow_pos_req          = self.create_publisher(SendStatus2, 'elbow/send/status2', 10)
        self.wrist_pitch_pos_req    = self.create_publisher(SendStatus2, 'wrist_pitch/send/status2', 10)
        self.wrist_roll_pos_req     = self.create_publisher(SendStatus2, 'wrist_roll/send/status2', 10)

        # Topics to receive motor positions
        self.rail_pos_resp          = self.create_subscription(RecvStatus2, 'linear_rail/rcvd/status2', lambda msg, motor='linear_rail':        self.read_position_callback(msg, motor), qos_profile_sensor_data)
        self.shoulder_pos_resp      = self.create_subscription(RecvStatus2, 'shoulder/rcvd/status2',    lambda msg, motor='shoulder':    self.read_position_callback(msg, motor), qos_profile_sensor_data)
        self.elbow_pos_resp         = self.create_subscription(RecvStatus2, 'elbow/rcvd/status2',       lambda msg, motor='elbow':       self.read_position_callback(msg, motor), qos_profile_sensor_data)
        self.wrist_pitch_pos_resp   = self.create_subscription(RecvStatus2, 'wrist_pitch/rcvd/status2', lambda msg, motor='wrist_pitch': self.read_position_callback(msg, motor), qos_profile_sensor_data)
        self.wrist_roll_pos_resp    = self.create_subscription(RecvStatus2, 'wrist_roll/rcvd/status2',  lambda msg, motor='wrist_roll':  self.read_position_callback(msg, motor), qos_profile_sensor_data)

        # Topic to send motor speeds
        self.joint_vel = self.create_publisher(JointState,"joint_vel", 10)

        # Topic to publish the rail zero position
        self.rail_zero_pub = self.create_publisher(Float32, "zero_rail_pos", 10)

        # Command topic
        self.command_sub = self.create_subscription(String, "zeroer_command_input", self.command_parser, 10)

        # Timers
        self.poll_timer = self.create_timer(self.polling_period, self.polling_callback)
        self.poll_timer.cancel()

        self.velocity_timer = self.create_timer(0.5, self.send_velocity_callback)
        self.velocity_timer.cancel()

        # Global Variables
        self.zeroing_status = {
            'linear_rail' : [False, self.rail_pos_req, 'FWD'],
            'shoulder' : [False, self.shoulder_pos_req],
            'elbow' : [False, self.elbow_pos_req],
            'wrist_pitch' : [False, self.wrist_pitch_pos_req],
            'wrist_roll' : [False, self.wrist_roll_pos_req]
        }
        self.zeroing_positions = {}
        self.rail_speed = -100
        self.timeout_value = 0.0

    def command_parser(self, command : String):
        cmd_word = command.data

        match cmd_word:

            case 'RAIL':
                self.zeroing_status['linear_rail'][0] = True
                self.zeroing_status['linear_rail'][2] = 'FWD'
                self.timeout_value = time.time() + self.polling_timeout
                self.poll_timer.reset()
                self.get_logger().info("Velocity started")
                self.velocity_timer.reset()

            case 'STOP':
                self.zeroing_status['linear_rail'][0] = False
                self.zeroing_status['shoulder'][0]    = False
                self.zeroing_status['elbow'][0]       = False
                self.zeroing_status['wrist_pitch'][0] = False
                self.zeroing_status['wrist_roll'][0]  = False

                self.stop_zeroing()

            # Manipulator w/out the rail, or manipulator SANS rail, lol
            case 'UNDERTALE':
                self.zeroing_status['shoulder'][0]    = True
                self.zeroing_status['elbow'][0]       = True
                self.zeroing_status['wrist_pitch'][0] = True
                self.zeroing_status['wrist_roll'][0]  = True
                self.timeout_value = time.time() + self.polling_timeout
                self.poll_timer.reset()

            case _:
                self.get_logger().info(f"UNKNOWN MESSAGE {cmd_word} RECIEVED")

    def polling_callback(self):
        
        # Check if the timeout has happened:
        if time.time() > self.timeout_value:
            self.get_logger().info("Timeout for polling")
            self.stop_zeroing()
            return

        requested_status = False
        # Check and see which messages we should send a value to
        for joint in self.zeroing_status.keys():
            status = self.zeroing_status[joint]

            if status[0]:
                # self.get_logger().info(f"publishing to {joint}")
                status[1].publish(SendStatus2())
                requested_status = True

        # If this timer isn't useful, turn it off
        if not requested_status:
            self.poll_timer.cancel()

    def stop_zeroing(self):
        vel_state = JointState()

        vel_state.name = ["linear_rail"]
        vel_state.velocity = [0.0]
        
        self.joint_vel.publish(vel_state)

        self.zeroing_status['linear_rail'][0] = False
        self.zeroing_status['shoulder'][0]    = False
        self.zeroing_status['elbow'][0]       = False
        self.zeroing_status['wrist_pitch'][0] = False
        self.zeroing_status['wrist_roll'][0]  = False

        self.poll_timer.cancel()
        self.get_logger().info("Velocity canceled")
        self.velocity_timer.cancel()

    def read_position_callback(self, msg, motor):

        # If we are looking for this motor's value
        if self.zeroing_status[motor][0]:

            # If the motor we are looking at is not the rail
            if motor != 'linear_rail':

                self.zeroing_positions[motor] = ((msg.angle_degrees - 180) % 360) - 180

                self.zeroing_status[motor][0] = False

                # If we have values for each motor, save them and clear the list
                if len(self.zeroing_positions) == 4:
                    self.save_zeros()
                    self.zeroing_positions = {}

            # Otherwise check the current of the rail and see if we need to reverse
            else:
                if abs(msg.current_amps) > self.rail_thresh and self.zeroing_status['linear_rail'][2] != 'REV':
                    self.get_logger().info("Switchinf Rail Directions!")
                    self.rail_zero_pub.publish(Float32(data = float(msg.angle_degrees)))
                    self.zeroing_status['linear_rail'][2] = 'REV'
                    self.timeout_value = time.time() + 5.0

    def send_velocity_callback(self):

        # Check if the timeout has happened:
        if time.time() > self.timeout_value:
            self.get_logger().info("Timeout for sending velocity")
            self.stop_zeroing()
            return

        vel_state = JointState()
        vel_state.name = ["linear_rail"]

        if self.zeroing_status['linear_rail'][0] and self.zeroing_status['linear_rail'][2] == 'FWD':
            vel_state.velocity.append(100.0)

        elif self.zeroing_status['linear_rail'][0] and self.zeroing_status['linear_rail'][2] == 'REV':
            vel_state.velocity.append(-100.0)

        else:
            vel_state.velocity.append(0.0)

        self.joint_vel.publish(vel_state)

    def save_zeros(self):
        self.get_logger().info(f"ZERO DATA: {self.zeroing_positions}")

        # Turn it into a yaml and save that yaml

        file_contents = yaml.dump(self.zeroing_positions, sort_keys=False)

        self.get_logger().info(f"YAML DATA: {file_contents}")

        # file_path = path.normpath(path.join(__file__, self.config_filepath))
        file_path = '/home/daedalus/workspace-daedalus/src/control_packages/controls/controls_pkg/config/zeros.yaml'

        file = open(file_path, 'w+')
        file.write(file_contents)
        file.close()

def main(args=None):
    rclpy.init(args=args)

    arm_zeroer = ArmZeroer()
    rclpy.spin(arm_zeroer)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_zeroer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()