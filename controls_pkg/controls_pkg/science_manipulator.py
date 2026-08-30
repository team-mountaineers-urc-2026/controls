import rclpy
import yaml
from os import path
from rclpy.executors import MultiThreadedExecutor
from controls_msgs.msg import SpeedClosedLoopControlMsgSentParams as SendSpeed,\
    ReadMotorStatus1MsgSentParams as SendStatus1,\
    ReadMotorStatus2MsgRecvParams as RecvStatus2,\
    ReadMotorStatus2MsgSentParams as SendStatus2, \
    AbsolutePositionClosedLoopControlMsgSentParams as SendPosition
from robot_interfaces.msg import STargetedFloat
from rclpy.node import Node
from std_msgs.msg import Float32, Empty, Bool, String
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from math import floor
import numpy as np
from numpy import sign
from enum import Enum, auto

DEBUGGING = False
RAIL_DPM = -45300.0 # Degrees Per Meter for the linear rail, found empirically

class Motor():
    def __init__(self, offset = None, current_pos = None, protection_enabled = False):
        self.offset = offset
        self.current_pos = current_pos
        self.protection_enabled = protection_enabled
        self.rotation_number = 0
        self.linear_state = None
        self.upper_limit = 0
        self.lower_limit = 0
        self.amperage = 0.0
        self.last_raw = None

class RailHomingState(Enum):
    IDLE = auto()
    MOVING_LEFT = auto()
    MOVING_RIGHT = auto()
    MOVING_CENTER = auto()
    HOMED = auto()
    FAULT = auto()

class Science_Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')
        
        # Limit Switch Linear Rail Setup
        self.limit_switch_1_state = 1
        self.limit_switch_2_state = 1
        # Declare Parameters
        self.declare_parameter("timeout_delay_sec", 0.5)
        self.declare_parameter("timeout_check_freq", 2.0)
        self.declare_parameter("motor_status_freq", 20.0)
        self.declare_parameter("fwd_kinematics_freq", 15.0)
        self.declare_parameter("publish_vel_freq", 15.0)

        self.declare_parameter("lower_limit_y", 0.04)
        self.declare_parameter("upper_limit_y", 0.42)
        # self.declare_parameter("upper_limit_pitch", 225.0)#dont need to declare uper and lower limits for drum now
        # self.declare_parameter("lower_limit_pitch", 20.0)
        # self.declare_parameter("upper_limit_roll", 0.0)
        # self.declare_parameter("lower_limit_roll", -430.0)

        self.declare_parameter('linear_rail_upper_limit', 0.41)
        self.declare_parameter('linear_rail_lower_limit', 0.03)
        self.declare_parameter('shoulder_upper_limit', 0.0)
        self.declare_parameter('shoulder_lower_limit', 0.0)
        self.declare_parameter('elbow_upper_limit', 0.0)
        self.declare_parameter('elbow_lower_limit', 0.0)
        # self.declare_parameter('wrist_pitch_upper_limit', 88)#dont need to declare uper and lower limits for drum now
        # self.declare_parameter('wrist_pitch_lower_limit', -126)
        # self.declare_parameter('wrist_roll_upper_limit', 140)
        # self.declare_parameter('wrist_roll_lower_limit', -320)
        # self.declare_parameter('gripper_upper_limit', 0.15) #Meters
        # self.declare_parameter('gripper_lower_limit', 0.03)#Meters

        self.declare_parameter('config_filepath', '../config/zeros.yaml')

        # Get Parameter Values
        self.timeout_delay_sec   = self.get_parameter("timeout_delay_sec").get_parameter_value().double_value
        self.timeout_check_freq  = self.get_parameter("timeout_check_freq").get_parameter_value().double_value
        self.motor_status_freq   = self.get_parameter("motor_status_freq").get_parameter_value().double_value
        self.fwd_kinematics_freq = self.get_parameter("fwd_kinematics_freq").get_parameter_value().double_value
        self.publish_vel_freq    = self.get_parameter("publish_vel_freq").get_parameter_value().double_value

        self.lower_limit_y      = self.get_parameter("lower_limit_y").get_parameter_value().double_value
        self.upper_limit_y      = self.get_parameter("upper_limit_y").get_parameter_value().double_value
        # self.upper_limit_pitch  = self.get_parameter("upper_limit_pitch").get_parameter_value().double_value#do not need to limit pitch angles
        # self.lower_limit_pitch  = self.get_parameter("lower_limit_pitch").get_parameter_value().double_value
        # self.upper_limit_roll   = self.get_parameter("upper_limit_roll").get_parameter_value().double_value
        # self.lower_limit_roll   = self.get_parameter("lower_limit_roll").get_parameter_value().double_value

        self.config_filepath = self.get_parameter('config_filepath').get_parameter_value().string_value

        # Speed Publishers         
        self.shoulder_pub = self.create_publisher(SendSpeed, "/science_manipulator/shoulder/send/speed_control",     10)
        self.elbow_pub    = self.create_publisher(SendSpeed, "/science_manipulator/elbow/send/speed_control",        10)
        # self.roll_pub     = self.create_publisher(SendSpeed, "/science_manipulator/wrist_roll/send/speed_control",   10)
        self.drum_pub    = self.create_publisher(SendSpeed, "/science_manipulator/wrist_pitch/send/speed_control",  10)
        self.rail_pub     = self.create_publisher(SendSpeed, "/science_manipulator/linear_rail/send/speed_control",  10)
        self.finger_pub  = self.create_publisher(STargetedFloat, "/science/actuator_pos", 5)
        
        # Status Publishers
        self.shoulder_status1_pub = self.create_publisher(SendStatus1, "/science_manipulator/shoulder/send/status1",     10)
        self.elbow_status1_pub    = self.create_publisher(SendStatus1, "/science_manipulator/elbow/send/status1",        10)
        # self.roll_status1_pub     = self.create_publisher(SendStatus1, "/science_manipulator/wrist_roll/send/status1",   10)
        self.drum_status1_pub    = self.create_publisher(SendStatus1, "/science_manipulator/wrist_pitch/send/status1",  10)
        self.rail_status1_pub     = self.create_publisher(SendStatus1, "/science_manipulator/linear_rail/send/status1",  10)
        # self.gripper_status1_pub = self.create_publisher(SendStatus1,"/science_manipulator/gripper/send/status1",10)

        self.shoulder_status2_pub    = self.create_publisher(SendStatus2, "/science_manipulator/shoulder/send/status2",     10)
        self.elbow_status2_pub       = self.create_publisher(SendStatus2, "/science_manipulator/elbow/send/status2",        10)
        # self.roll_status2_pub        = self.create_publisher(SendStatus2, "/science_manipulator/wrist_roll/send/status2",   10)
        self.drum_status2_pub       = self.create_publisher(SendStatus2, "/science_manipulator/wrist_pitch/send/status2",  10)
        self.rail_status2_pub        = self.create_publisher(SendStatus2, "/science_manipulator/linear_rail/send/status2",  10)
        # self.gripper_status2_pub        = self.create_publisher(SendStatus2, "/science_manipulator/gripper/send/status2",  10)

        # Send Position Publishers
        self.rail_pos_send          = self.create_publisher(SendPosition, 'linear_rail/send/position', 10)
        self.shoulder_pos_send      = self.create_publisher(SendPosition, 'shoulder/send/position', 10)
        self.elbow_pos_send         = self.create_publisher(SendPosition, 'elbow/send/position', 10)
        self.drum_pos_send   = self.create_publisher(SendPosition, 'wrist_pitch/send/position', 10)
        # self.wrist_roll_pos_send    = self.create_publisher(SendPosition, 'wrist_roll/send/position', 10)
        # self.gripper_pos_send          = self.create_publisher(SendPosition, 'gripper/send/position', 10)
        

        # Position Publisher
        self.joint_pos = self.create_publisher(JointState, 'joint_pos', 10)
        self.end_pos = self.create_publisher(Twist, 'end_pos', 10)
        self.goal_arrival = self.create_publisher(Bool, 'goal_arrival', 10)

        # Subscriptions
        self.create_subscription(RecvStatus2, "linear_rail/rcvd/status2",  lambda msg, motor='linear_rail': self.update_current_theta(msg, motor), 10)
        self.create_subscription(RecvStatus2, "shoulder/rcvd/status2",     lambda msg, motor='shoulder':    self.update_current_theta(msg, motor), 10)
        self.create_subscription(RecvStatus2, "elbow/rcvd/status2",        lambda msg, motor='elbow':       self.update_current_theta(msg, motor), 10)
        self.create_subscription(RecvStatus2, "wrist_pitch/rcvd/status2",  lambda msg, motor='wrist_pitch': self.update_current_theta(msg, motor), 10)
        # self.create_subscription(RecvStatus2, "wrist_roll/rcvd/status2",   lambda msg, motor='wrist_roll':  self.update_current_theta(msg, motor), 10)
        # self.create_subscription(RecvStatus2, "gripper/rcvd/status2",   lambda msg, motor='gripper':  self.update_current_theta(msg, motor), 10)

        self.create_subscription(Empty, "reload_offsets", self.load_offsets, 10)
        self.create_subscription(Float32, "zero_rail_pos", self.set_railoffset, 10)


        self.create_subscription(String, 'enable_protection', lambda msg, status=True: self.set_protection(msg, status), 10)
        self.create_subscription(String, 'disable_protection', lambda msg, status=False: self.set_protection(msg, status), 10)
        
        # Limit Switch Subscriptions
        self.create_subscription(Bool, "/limit_switch_1", self.limit_switch_1_cb, 10)
        self.create_subscription(Bool, "/limit_switch_2", self.limit_switch_2_cb, 10)
        self.create_subscription(Empty, "home_linear_rail",         self.start_rail_homing,       10)
        self.create_subscription(Empty, "stop_home_linear_rail",    self.stop_rail_homing,        10)
        self.create_subscription(Empty, "center_linear_rail",       self.move_to_center,          10)
        self.create_subscription(Empty, "disable_rail_soft_limits", self.disable_rail_soft_limits, 10)
        self.create_subscription(Empty, "enable_rail_soft_limits",  self.enable_rail_soft_limits,  10)
        self.create_subscription(Empty, "zero_arm_motors",          self.zero_arm_motors,          10)

        self.create_timer(0.05, self.rail_homing_loop)

        self.joint_vel = self.create_subscription(JointState, "joint_vel", self.set_motor_speeds, 10)
        self.next_pos = self.create_subscription(JointState, "next_pos", self.set_next_positions, 10)

        self.create_timer(1.0/self.timeout_check_freq,  self.check_message_timeout)
        self.create_timer(1.0/self.publish_vel_freq,    self.pub_both)
        self.create_timer(1.0/self.motor_status_freq,   self.send_status_msgs)
        self.create_timer(1.0/self.fwd_kinematics_freq, self.pos_timer_callback)

        self.last_received_time = self.get_clock().now()

        self.rail_homing_state = RailHomingState.IDLE
        self.rail_homing_active = False
        self.rail_soft_limits_enabled = True

        self.left_limit_position = None
        self.right_limit_position = None
        self.rail_center_position = None

        self.rail_soft_limit_left = None
        self.rail_soft_limit_right = None
        self.rail_stow_center = False

        self.rail_soft_limit_margin = 0.012
        self.homing_speed = 125
        self.centering_speed = 300
        self.limit_switch_timeout_sec = 5
        self.last_limit_switch_update = self.get_clock().now()
        # Rail centered status — add to __init__ after last_limit_switch_update line:
        self.rail_stowed_at_center_flag = False
        self.rail_centered_pub = self.create_publisher(Bool, '/science_manipulator/rail_centered', 10)
        self.create_timer(1.0, self._publish_centered_status)
        self.motor_data = {
            'linear_rail'   : Motor(protection_enabled=True),
            'shoulder'      : Motor(),
            'elbow'         : Motor(),
            'wrist_pitch'   : Motor(),
            'finger'        : float
            # 'wrist_roll'    : Motor(),
            # 'gripper'       : Motor(protection_enabled=True)
        }

        self.load_bounds()
        self.load_offsets(Empty())
    
        self.finger_val_to_send = False
        self.finger_vel = 0.5
        self.current_speed = JointState()
        self.current_position_goals = JointState()
        self.current_end_pos = Twist()

    #Limit Switch Callbacks
    def limit_switch_1_cb(self, msg):
        self.limit_switch_1_state = msg.data
        self.last_limit_switch_update = self.get_clock().now()

    def limit_switch_2_cb(self, msg):
        self.limit_switch_2_state = msg.data
        self.last_limit_switch_update = self.get_clock().now()

    def pub_both(self):
        self.send_motor_speeds()
        # self.send_next_positions()

    def load_bounds(self):
        self.motor_data['linear_rail'].upper_limit = self.get_parameter('linear_rail_upper_limit').get_parameter_value().double_value
        self.motor_data['linear_rail'].lower_limit = self.get_parameter('linear_rail_lower_limit').get_parameter_value().double_value

        self.motor_data['shoulder'].upper_limit = self.get_parameter('shoulder_upper_limit').get_parameter_value().integer_value
        self.motor_data['shoulder'].lower_limit = self.get_parameter('shoulder_lower_limit').get_parameter_value().integer_value

        self.motor_data['elbow'].upper_limit = self.get_parameter('elbow_upper_limit').get_parameter_value().integer_value
        self.motor_data['elbow'].lower_limit = self.get_parameter('elbow_lower_limit').get_parameter_value().integer_value

        # self.motor_data['wrist_pitch'].upper_limit = self.get_parameter('wrist_pitch_upper_limit').get_parameter_value().integer_value
        # self.motor_data['wrist_pitch'].lower_limit = self.get_parameter('wrist_pitch_lower_limit').get_parameter_value().integer_value

        # self.motor_data['wrist_roll'].upper_limit = self.get_parameter('wrist_roll_upper_limit').get_parameter_value().integer_value
        # self.motor_data['wrist_roll'].lower_limit = self.get_parameter('wrist_roll_lower_limit').get_parameter_value().integer_value

        # self.motor_data['gripper'].upper_limit = self.get_parameter('gripper_upper_limit').get_parameter_value().double_value
        # self.motor_data['gripper'].lower_limit = self.get_parameter('gripper_lower_limit').get_parameter_value().double_value

    def load_offsets(self, msg: Empty):
        file_path = '/home/pathfinder/workspace-deimos/src/control_packages/controls/controls_pkg/config/zeros.yaml'
        try:
            with open(file_path, 'r+') as file:
                offsets = yaml.load(file.read(), Loader=yaml.SafeLoader) or {}
        except:
            offsets = {}

        for motor in self.motor_data.keys():
            if motor == 'finger':
                continue
            if motor in offsets:
                self.motor_data[motor].offset = offsets[motor]

        self.rail_soft_limit_left  = offsets.get('rail_soft_limit_left',  None)
        self.rail_soft_limit_right = offsets.get('rail_soft_limit_right', None)
        self.left_limit_position   = offsets.get('rail_left_limit',       None)
        self.right_limit_position  = offsets.get('rail_right_limit',      None)
        self.rail_center_position  = offsets.get('rail_center_position',  None)
        self.rail_stowed_at_center_flag = bool(offsets.get('rail_stowed_at_center', False))
        self.get_logger().info(f"Loaded rail_stowed_at_center: {self.rail_stowed_at_center_flag}")

        if self.rail_soft_limit_left is not None and self.rail_soft_limit_right is not None:
            if self.rail_soft_limit_left <= self.rail_soft_limit_right:
                self.get_logger().error("INVALID RAIL LIMITS IN YAML")
                self.rail_soft_limit_left = None
                self.rail_soft_limit_right = None
            else:
                self.get_logger().info(
                    f"Loaded rail limits:\n"
                    f" Left Soft:  {self.rail_soft_limit_left}\n"
                    f" Right Soft: {self.rail_soft_limit_right}\n"
                    f" Center:     {self.rail_center_position}"
                )

        if self.rail_center_position is not None:
            self.rail_stow_center = True
            self.get_logger().info("Rail stowed at center assumed. Will auto-zero on first encoder read.")

    # TODO prove this works with wrapping    
    def set_railoffset(self, msg : Float32):
        self.motor_data['linear_rail'].offset = msg.data

    # Could this be replaced with the single turn angle? Might get more resolution...
    def update_current_theta(self, msg, motor):
        self.motor_data[motor].amperage = msg.current_amps
        self.motor_data[motor].last_raw = msg.angle_degrees  # ← add this


        if self.motor_data[motor].offset is not None:
            self.set_motor_value(motor, msg.angle_degrees)

        if motor == 'linear_rail' and self.rail_stow_center:
            if self.motor_data['linear_rail'].current_pos is not None:
                unwrapped = msg.angle_degrees
                new_offset = unwrapped - (self.rail_center_position * RAIL_DPM)
                self.motor_data['linear_rail'].offset = new_offset
                self.rail_stow_center = False
                self.get_logger().info(f"Auto-zeroed rail to center. offset={new_offset:.2f}")
                self.set_motor_value(motor, unwrapped)

    # Sets the motor current position based on the wrappings
    def set_motor_value(self, motor, unwrapped_value):
        wrapped_value = (unwrapped_value + 180 - self.motor_data[motor].offset) % 360 - 180

        # Check the last value and try to determine a sign flip
        wrapped_sign = sign(wrapped_value)
        old_sign = None if self.motor_data[motor].current_pos == None else sign(self.motor_data[motor].current_pos)

        # Add or subtract a rotation depending on which way we turned
        if old_sign != None and motor != 'linear_rail':
            if (wrapped_sign < 0 and old_sign > 0) and (abs(wrapped_value) > 150):
                self.motor_data[motor].rotation_number += 1

        # For the linear rail, do some fancier calculations
        if old_sign != None and motor == 'linear_rail':
            old_value = self.motor_data[motor].current_pos
            new_value = unwrapped_value
            
            if (sign(new_value) != sign(old_value)):
                self.get_logger().info(f"New Sign {sign(new_value)} \t old Sign {sign(old_value)}")
                self.get_logger().info(f"NEW VALUE {new_value} \t \t OLD VALUE {old_value}")

            if (sign(new_value) != sign(old_value)) and abs(new_value) > 12000:

                # We have flipped, calculate the offset
                if (int(sign(new_value))) == 1:
                    self.get_logger().info(f"Border Reached.")
                    self.get_logger().info(f"Subtracting 32768")
                    self.motor_data[motor].rotation_number -= 1

                else:
                    self.get_logger().info(f"Border Reached.")
                    self.get_logger().info(f"Adding 32768")
                    self.motor_data[motor].rotation_number += 1

            # self.get_logger().info(f"uw: {unwrapped_value}; rv: {unwrapped_value - self.motor_data[motor].offset}")

            # self.get_logger().info(f"""
            # Rotation Number = {self.motor_data[motor].rotation_number}
            # Offset          = {(self.motor_data[motor].rotation_number * 32768)/RAIL_DPM}
            # UnwrappedDist   = {unwrapped_value/RAIL_DPM}
            # UnwrappedOffset = {(unwrapped_value - self.motor_data[motor].offset)/RAIL_DPM}
            # """)

            self.motor_data['linear_rail'].linear_state = (unwrapped_value - self.motor_data[motor].offset + self.motor_data[motor].rotation_number * 65535)/RAIL_DPM

        if motor != 'linear_rail':
            self.motor_data[motor].current_pos = wrapped_value
        
        else:
            self.motor_data[motor].current_pos = unwrapped_value

    def set_protection(self, msg : String, status : bool):
        
        if msg.data == 'all':
            self.get_logger().info(f"Status for ALL set to {status}")
            for motor in self.motor_data.keys(): self.motor_data[motor].protection_enabled = status
        
        elif msg.data not in self.motor_data.keys():
            self.get_logger().info(f"Unknown command {msg.data}")

        else:
            self.get_logger().info(f"Status for {msg.data.upper()} set to {status}")
            self.motor_data[msg.data].protection_enabled = status

    def send_status_msgs(self):
        status1_msg = SendStatus1()
        self.shoulder_status1_pub.publish(status1_msg)
        self.elbow_status1_pub.publish(status1_msg)
        # self.roll_status1_pub.publish(status1_msg)
        self.drum_status1_pub.publish(status1_msg)
        self.rail_status1_pub.publish(status1_msg)
        # self.gripper_status1_pub.publish(status1_msg)

        status2_msg = SendStatus2()
        self.shoulder_status2_pub.publish(status2_msg)
        self.elbow_status2_pub.publish(status2_msg)
        # self.roll_status2_pub.publish(status2_msg)
        self.drum_status2_pub.publish(status2_msg)
        self.rail_status2_pub.publish(status2_msg)
        # self.gripper_status2_pub.publish(status2_msg)

    def wrap_rail(self, new_value):
    #     # Check if the sign is different and we are not around zero
    #     if (sign(new_value) != sign(self.last_rail_value)) and abs(new_value) > 12000:

    #         # We have flipped, calculate the offset
    #         if (int(sign(new_value))) == 1:
    #             

    #         else:
    #             self.get_logger().info(f"Border Reached. {self.last_rail_value} is switching to {new_value}")
    #             self.get_logger().info(f"Adding 32768")
    #             self.rail_offset = self.rail_offset + 32768

    #     self.last_rail_value = new_value
    #     return new_value + self.rail_offset
        pass

    def pos_timer_callback(self):
        pos_state = JointState()

        for motor in self.motor_data.keys():

            # If the motor has been zeroed and we have a current position
            if motor == "finger":
                continue

            if self.motor_data[motor].offset != None and self.motor_data[motor].current_pos != None:
                
                pos_state.name.append(motor)
                if motor == 'linear_rail':
                    if self.motor_data[motor].linear_state is None:
                        pos_state.name.pop()
                        continue
                    pos_state.position.append(self.motor_data[motor].linear_state)
                else:
                    pos_state.position.append(self.motor_data[motor].current_pos + 360 * self.motor_data[motor].rotation_number)
                pos_state.effort.append(float(self.motor_data[motor].amperage))
        
        # for i in range(0, len(pos_state.name)):
        #     self.get_logger().info(f"{pos_state.name[i]} : {pos_state.position[i]}")

        self.joint_pos.publish(pos_state)
        
    def set_motor_speeds(self, msg : JointState):
        self.current_speed = msg
        self.last_received_time = self.get_clock().now()
        # self.get_logger().info(f"CALLBACK: {self.current_speed}")
        self.send_motor_speeds()
    def send_motor_speeds(self):
        names = self.current_speed.name
        velocities = self.current_speed.velocity

        messages_to_send = {}

        for i in range(0, len(names)):

            if names[i] == "finger":
                if self.finger_vel == velocities[i]:
                    continue
                self.finger_val_to_send = True
                self.finger_vel = velocities[i]
                continue

            current_motor = self.motor_data[names[i]]
            motor_vel = velocities[i]

            if current_motor.protection_enabled:
                if names[i] == 'linear_rail':
                    left_pressed  = (self.limit_switch_1_state == 0)
                    right_pressed = (self.limit_switch_2_state == 0)

                    if left_pressed and motor_vel < 0:
                        self.get_logger().info("LEFT LIMIT HIT → blocking left motion")
                        motor_vel = 0

                    if right_pressed and motor_vel > 0:
                        self.get_logger().info("RIGHT LIMIT HIT → blocking right motion")
                        motor_vel = 0

                    if self.rail_soft_limits_enabled:
                        if self.rail_soft_limit_left is not None:
                            if current_motor.linear_state is not None and current_motor.linear_state >= self.rail_soft_limit_left and motor_vel < 0:
                                self.get_logger().warn("LEFT SOFT LIMIT")
                                motor_vel = 0
                        if self.rail_soft_limit_right is not None:
                            if current_motor.linear_state is not None and current_motor.linear_state <= self.rail_soft_limit_right and motor_vel > 0:
                                self.get_logger().warn("RIGHT SOFT LIMIT")
                                motor_vel = 0

                elif current_motor.current_pos is not None:
                    if (current_motor.current_pos + 360 * current_motor.rotation_number) > current_motor.upper_limit and motor_vel > 0:
                        self.get_logger().info(f'Motor {names[i]} stopped: over upper limit')
                        motor_vel = 0
                    elif (current_motor.current_pos + 360 * current_motor.rotation_number) < current_motor.lower_limit and motor_vel < 0:
                        self.get_logger().info(f'Motor {names[i]} stopped: under lower limit')
                        motor_vel = 0

            messages_to_send[names[i]] = SendSpeed(speed_dps=int(motor_vel))

        if 'linear_rail' in messages_to_send:
            self.rail_pub.publish(messages_to_send['linear_rail'])
        if 'shoulder' in messages_to_send:
            self.shoulder_pub.publish(messages_to_send['shoulder'])
        if 'elbow' in messages_to_send:
            self.elbow_pub.publish(messages_to_send['elbow'])
        if 'wrist_pitch' in messages_to_send:
            self.drum_pub.publish(messages_to_send['wrist_pitch'])

        if self.finger_val_to_send:
            finger_msg = STargetedFloat()
            finger_msg.data = self.finger_vel
            finger_msg.target = 'act'
            self.finger_val_to_send = False
            self.finger_pub.publish(finger_msg)

    def set_next_positions(self, msg : JointState):
        self.current_position_goals = msg
        if 'wrist_pitch' in msg.name:
            names = self.current_position_goals.name
            goals = self.current_position_goals.position
            self.drum_pos_send.publish(SendPosition(speed_limit_dps=15, goal_angle_degrees=int(goals[names.index('wrist_pitch')])))
        # self.send_next_positions()

    def zero_arm_motors(self, msg):
        file_path = '/home/pathfinder/workspace-deimos/src/control_packages/controls/controls_pkg/config/zeros.yaml'
        try:
            try:
                with open(file_path, 'r') as file:
                    data = yaml.safe_load(file) or {}
            except:
                data = {}

            stow_motors = ['shoulder', 'elbow', 'wrist_pitch']
            for motor in stow_motors:
                raw = self.motor_data[motor].last_raw
                if raw is None:
                    self.get_logger().warn(f"Cannot zero {motor} - no position yet")
                    continue
                self.motor_data[motor].offset = raw
                self.motor_data[motor].rotation_number = 0
                self.motor_data[motor].current_pos = 0.0
                data[motor] = float(raw)
                self.get_logger().info(f"Zeroed {motor} at {raw:.2f}")

            with open(file_path, 'w') as file:
                yaml.dump(data, file)
            self.get_logger().info("Arm motors zeroed and saved to zeros.yaml")
        except Exception as e:
            self.get_logger().error(f"Failed to zero arm motors: {e}")

    def command_rail_speed(self, speed):
        msg = JointState()
        msg.name = ['linear_rail']
        msg.velocity = [float(speed)]
        self.current_speed = msg
        self.send_motor_speeds()

    def move_to_center(self, msg):
        if self.rail_center_position is None:
            self.get_logger().error("Cannot move to center - not homed yet")
            return
        if self.motor_data['linear_rail'].linear_state is None:
            self.get_logger().error("Cannot move to center - position unknown")
            return
        self.get_logger().info(f"Moving to center: {self.rail_center_position:.4f}m")
        self.rail_homing_active = True
        self.rail_soft_limits_enabled = False
        self.rail_homing_state = RailHomingState.MOVING_CENTER

    def start_rail_homing(self, msg):
        if self.rail_homing_active:
            self.get_logger().warn("Rail already homing")
            return
        current_time = self.get_clock().now()
        age = (current_time - self.last_limit_switch_update).nanoseconds / 1e9
        if age > self.limit_switch_timeout_sec:
            self.get_logger().error(f"HOMING ABORTED: switch age {age:.2f}s exceeds timeout")
            return
        if self.limit_switch_1_state == 0:
            self.get_logger().error("LEFT switch already pressed")
            return
        if self.limit_switch_2_state == 0:
            self.get_logger().error("RIGHT switch already pressed")
            return
        self.get_logger().info("STARTING LINEAR RAIL HOMING")
        self.rail_homing_active = True
        self.rail_homing_state = RailHomingState.MOVING_LEFT

    def stop_rail_homing(self, msg):
        self.get_logger().warn("STOPPING RAIL HOMING")
        if self.rail_homing_state != RailHomingState.HOMED:
            self.rail_stowed_at_center_flag = False
            self._save_centered_flag(False)
        self.rail_homing_active = False
        self.rail_homing_state = RailHomingState.IDLE
        # self.command_rail_speed(0)

    def disable_rail_soft_limits(self, msg):
        self.rail_soft_limits_enabled = False
        self.get_logger().warn("RAIL SOFT LIMITS DISABLED")

    def enable_rail_soft_limits(self, msg):
        self.rail_soft_limits_enabled = True
        self.get_logger().info("RAIL SOFT LIMITS ENABLED")

    def rail_homing_loop(self):
        current_time = self.get_clock().now()
        age = (current_time - self.last_limit_switch_update).nanoseconds / 1e9
        if age > self.limit_switch_timeout_sec:
            if self.rail_homing_active:
                self.get_logger().error(f"HOMING ABORTED: lost switch comms age={age:.2f}s")
            self.stop_rail_homing(None)
            return
        if not self.rail_homing_active:
            return

        rail_motor = self.motor_data['linear_rail']

        if self.rail_homing_state == RailHomingState.MOVING_LEFT:
            if self.limit_switch_1_state == 0:
                self.command_rail_speed(0)
                self.left_limit_position = rail_motor.linear_state
                self.get_logger().info(f"HOMING: LEFT LIMIT FOUND at {self.left_limit_position}")
                self.rail_homing_state = RailHomingState.MOVING_RIGHT
            else:
                self.command_rail_speed(-self.homing_speed)

        elif self.rail_homing_state == RailHomingState.MOVING_RIGHT:
            if self.limit_switch_2_state == 0:
                self.command_rail_speed(0)
                self.right_limit_position = rail_motor.linear_state
                if self.left_limit_position is None or self.right_limit_position is None:
                    self.get_logger().error("HOMING: Cannot compute center - positions are None")
                    self.rail_homing_active = False
                    self.rail_homing_state = RailHomingState.FAULT
                    return
                self.get_logger().info(f"HOMING: RIGHT LIMIT FOUND at {self.right_limit_position}")
                self.rail_center_position  = (self.left_limit_position + self.right_limit_position) / 2.0
                self.rail_soft_limit_left  = self.left_limit_position  - self.rail_soft_limit_margin
                self.rail_soft_limit_right = self.right_limit_position + self.rail_soft_limit_margin
                self.rail_soft_limits_enabled = False
                self.rail_homing_state = RailHomingState.MOVING_CENTER
            else:
                self.command_rail_speed(self.homing_speed)

        elif self.rail_homing_state == RailHomingState.MOVING_CENTER:
            if rail_motor.linear_state is None:
                self.get_logger().warn("HOMING: Cannot center - linear_state None, saving and finishing")
                self.command_rail_speed(0)
                self.rail_homing_active = False
                self.rail_homing_state = RailHomingState.HOMED
                self.save_rail_homing_data()
                return
            error = self.rail_center_position - rail_motor.linear_state
            if abs(error) < 0.005:
                self.command_rail_speed(0)
                self.rail_soft_limits_enabled = True
                self.rail_homing_active = False
                self.rail_homing_state = RailHomingState.HOMED
                self.save_rail_homing_data()
                self.get_logger().info("HOMING COMPLETE")
            else:
                speed = -self.centering_speed if error > 0 else self.centering_speed
                self.command_rail_speed(speed)

    def save_rail_homing_data(self):
        file_path = '/home/pathfinder/workspace-deimos/src/control_packages/controls/controls_pkg/config/zeros.yaml'
        try:
            try:
                with open(file_path, 'r') as file:
                    data = yaml.safe_load(file) or {}
            except:
                data = {}
            data['rail_soft_limit_left']  = float(self.rail_soft_limit_left)
            data['rail_soft_limit_right'] = float(self.rail_soft_limit_right)
            data['rail_left_limit']       = float(self.left_limit_position)
            data['rail_right_limit']      = float(self.right_limit_position)
            data['rail_center_position']  = float(self.rail_center_position)
            self.rail_stowed_at_center_flag = True
            data['rail_stowed_at_center'] = True
            with open(file_path, 'w') as file:
                yaml.dump(data, file)
            self.get_logger().info("Saved rail homing data to zeros.yaml")
        except Exception as e:
            self.get_logger().error(f"Failed to save homing data: {e}")
    def _save_centered_flag(self, value: bool):
        file_path = '/home/pathfinder/workspace-deimos/src/control_packages/controls/controls_pkg/config/zeros.yaml'
        try:
            try:
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            except:
                data = {}
            data['rail_stowed_at_center'] = value
            with open(file_path, 'w') as f:
                yaml.dump(data, f)
        except Exception as e:
            self.get_logger().error(f"Failed to save centered flag: {e}")

    def _publish_centered_status(self):
        self.rail_centered_pub.publish(Bool(data=self.rail_stowed_at_center_flag))
    def send_next_positions(self):
        names = self.current_position_goals.name
        goals = self.current_position_goals.position

        for i in range(0, len(names)):
            match names[i]:

                case 'linear_rail':
                    # Check where we are vs the goal
                    if self.motor_data['linear_rail'].current_pos == None:
                        self.get_logger().error("Linear Rail not Zeroed")
                        self.current_position_goals.name.pop(i)
                        self.current_position_goals.position.pop(i)
                        continue

                    diff = abs(goals[i] - self.motor_data['linear_rail'].current_pos)
                    if diff < 2:
                        self.current_position_goals.name.pop(i)
                        self.current_position_goals.position.pop(i)
                        continue
                    self.rail_pos_send.publish(SendPosition(speed_limit_dps=900, goal_angle_degrees=int(goals[i])))

                # case 'gripper':
                #     # Check where we are vs the goal
                #     if self.motor_data['gripper'].current_pos == None:
                #         self.get_logger().error("Gripper not Zeroed")
                #         self.current_position_goals.name.pop(i)
                #         self.current_position_goals.position.pop(i)
                #         continue

                #     diff = abs(goals[i] - self.motor_data['gripper'].current_pos)
                #     if diff < 2:
                #         self.current_position_goals.name.pop(i)
                #         self.current_position_goals.position.pop(i)
                #         continue
                #     self.gripper_pos_send.publish(SendPosition(speed_limit_dps=900, goal_angle_degrees=int(goals[i])))

                case 'shoulder':
                    # Check where we are vs the goal
                    diff = abs(goals[i] - self.motor_data['shoulder'].current_pos)
                    if diff < 1:
                        self.current_position_goals.name.pop(i)
                        self.current_position_goals.position.pop(i)
                        continue
                    self.shoulder_pos_send.publish(SendPosition(speed_limit_dps=10, goal_angle_degrees=int(goals[i])))

                case 'elbow':
                    # Check where we are vs the goal
                    diff = abs(goals[i] - self.motor_data['elbow'].current_pos)
                    if diff < 1:
                        self.current_position_goals.name.pop(i)
                        self.current_position_goals.position.pop(i)
                        continue
                    self.elbow_pos_send.publish(SendPosition(speed_limit_dps=10, goal_angle_degrees=int(goals[i])))

                case 'wrist_pitch':
                    # Check where we are vs the goal
                    diff = abs(goals[i] - self.motor_data['wrist_pitch'].current_pos)
                    if diff < 1:
                        self.current_position_goals.name.pop(i)
                        self.current_position_goals.position.pop(i)
                        continue
                    self.drum_pos_send.publish(SendPosition(speed_limit_dps=15, goal_angle_degrees=int(goals[i])))

                # case 'wrist_roll':
                #     # Check where we are vs the goal
                #     diff = abs(goals[i] - self.motor_data['wrist_roll'].current_pos)
                #     if diff < 1:
                #         self.current_position_goals.name.pop(i)
                #         self.current_position_goals.position.pop(i)
                #         continue
                #     self.wrist_roll_pos_send.publish(SendPosition(speed_limit_dps=15, goal_angle_degrees=int(goals[i])))

                case _:
                    self.get_logger().error(f"Unknown motor {names[i]}")

        # If the list is empty
        if len(self.current_position_goals.name) == 0:
            self.goal_arrival.publish(Bool(data=True))
        else:
            self.goal_arrival.publish(Bool(data=False))

    def check_message_timeout(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_received_time).nanoseconds / 1e9 > self.timeout_delay_sec:  
            # self.get_logger().warning(f'No velocity control message received in the last {self.timeout_delay_sec} seconds, zeroing motor speeds')
            self.current_speed = JointState()
            self.last_received_time = self.get_clock().now()
    
def main(args=None):
    
    rclpy.init(args=args)
    science_manipulator = Science_Manipulator()
    rclpy.spin(science_manipulator)
    science_manipulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
