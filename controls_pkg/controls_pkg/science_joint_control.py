

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy, JointState
from rclpy.qos import qos_profile_sensor_data
from math import floor

DEBUGGING = False

def clamp(val) -> int:
    return max(-1,min(val,1))

def normalize_joystick_axes_vals(value: float):
    return clamp(-1 * value)
        
def normalize_bumper_axes_vals(value: float):
    return clamp(1 - ((value + 1)/2))

class Science_Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')

        # Declare Parameters
        self.declare_parameter("restart_button", 7)
        self.declare_parameter("rail_left_button", 4)
        self.declare_parameter("rail_right_button", 5)
        self.declare_parameter("finger_extend_button",2)
        self.declare_parameter("finger_retract_button", 1)
        self.declare_parameter("shoulder_axes", 1)
        self.declare_parameter("elbow_axes", 4)
        self.declare_parameter("wrist_speed_axes", 5)
        # self.declare_parameter("wrist_roll_axes", 6)
        self.declare_parameter("wrist_drum_axes", 7)
  
        self.declare_parameter("max_dps", 20)

        self.declare_parameter("timeout_delay_sec", 2.0)
        self.declare_parameter("timeout_check_freq", 2.0)
        self.declare_parameter("motor_status_freq", 1.0)

        # Get Parameter Values
        self.restart_button = self.get_parameter("restart_button").get_parameter_value().integer_value
        self.rail_left_button = self.get_parameter("rail_left_button").get_parameter_value().integer_value
        self.rail_right_button = self.get_parameter("rail_right_button").get_parameter_value().integer_value
        self.shoulder_axes = self.get_parameter("shoulder_axes").get_parameter_value().integer_value
        self.elbow_axes = self.get_parameter("elbow_axes").get_parameter_value().integer_value
        self.wrist_speed_axes = self.get_parameter("wrist_speed_axes").get_parameter_value().integer_value
        # self.wrist_roll_axes = self.get_parameter("wrist_roll_axes").get_parameter_value().integer_value
        self.wrist_drum_axes = self.get_parameter("wrist_drum_axes").get_parameter_value().integer_value
        self.finger_extend_button = self.get_parameter("finger_extend_button").get_parameter_value().integer_value
        self.finger_retract_button = self.get_parameter("finger_retract_button").get_parameter_value().integer_value

        self.max_dps = self.get_parameter("max_dps").get_parameter_value().integer_value
        
        self.timeout_delay_sec = self.get_parameter("timeout_delay_sec").get_parameter_value().double_value
        self.timeout_check_freq = self.get_parameter("timeout_check_freq").get_parameter_value().double_value
        self.motor_status_freq = self.get_parameter("motor_status_freq").get_parameter_value().double_value

        # Speed Publisher
        self.joint_vel = self.create_publisher(JointState,"joint_vel", 10)

        # Subscriptions
        self.joy_input = self.create_subscription(Joy, "joint_joy", self.calculate_motor_speeds, qos_profile_sensor_data)

        self.send_speed_commands(0,0,0,0,0)#removed roll and gripper commands

    def send_speed_commands(self, rail, shoulder, elbow, drum, finger):#removed roll, gripper
        vel_state = JointState()

        vel_state.name = ["linear_rail", "shoulder", "elbow", "wrist_pitch", "finger"] #removed "wrist_roll", "gripper"   && wrist pitch is now the ros topic for wrist_drum, didnt rename it for ease of integration
        vel_state.velocity = [float(rail), float(shoulder), float(elbow), float(drum), float(finger)]#removed float(roll), float(gripper)
        
        self.joint_vel.publish(vel_state)

    def calculate_motor_speeds(self, joy_msg: Joy):
        self.last_received_time = self.get_clock().now()

        
        wrist_speed = normalize_bumper_axes_vals(joy_msg.axes[self.wrist_speed_axes])
        shoulder_dps = -floor(normalize_joystick_axes_vals(joy_msg.axes[self.shoulder_axes]) * self.max_dps * 0.5) #Negated to reverse controls for Deimos
        elbow_dps = -floor(normalize_joystick_axes_vals(joy_msg.axes[self.elbow_axes]) * self.max_dps * 0.3)       #Negated to reverse controls for Deimos
        # roll_dps = int(joy_msg.axes[self.wrist_roll_axes] * wrist_speed * self.max_dps * 3)
        drum_dps =  -int(joy_msg.axes[self.wrist_drum_axes] * wrist_speed * self.max_dps * 20)
        #drum_dps =  floor(normalize_joystick_axes_vals(joy_msg.axes[self.wrist_drum_axes] * self.max_dps * 5))
        rail_dps = floor((joy_msg.buttons[self.rail_left_button] - joy_msg.buttons[self.rail_right_button]) * 1000)
        if (joy_msg.buttons[self.finger_extend_button] and joy_msg.buttons[self.finger_retract_button] ):
            # Extend:
            finger_dps = 0.5
        elif (joy_msg.buttons[self.finger_retract_button] ):
            finger_dps = 0
        elif (joy_msg.buttons[self.finger_extend_button] ):
            finger_dps = 1
        else:
            finger_dps = 0.5
        # finger_dps = floor((joy_msg.buttons[self.finger_extend_button] - joy_msg.buttons[self.finger_retract_button]))

        # Put button maps here


        # If the speed commands are too low, don't send them
        if abs(shoulder_dps) < 6:
            shoulder_dps = 0

        if abs(elbow_dps) < 6:
            elbow_dps = 0
        

        if DEBUGGING:
            self.get_logger().info(f"rail_dps{rail_dps}\nshoulder_dps{shoulder_dps}\nelbow_dps{elbow_dps}\ndrum_dps{drum_dps}")#removed roll_dps{roll_dps}\n
        self.send_speed_commands(rail_dps, shoulder_dps, elbow_dps, drum_dps, finger_dps) #removed roll_dps
       
    
def main(args=None):
    rclpy.init(args=args)
    science_manipulator = Science_Manipulator()
    multi_threaded_executor = MultiThreadedExecutor(num_threads=2)

    multi_threaded_executor.add_node(science_manipulator)

    multi_threaded_executor.spin()

    multi_threaded_executor.shutdown()
    science_manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

