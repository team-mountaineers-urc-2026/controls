
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float32
from math import floor, pi, cos, sin
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from numpy import sign

RESTART_BTN = 4 # Top Face Button
RAIL_LEFT_BTN = 6 # Left Bumper
RAIL_RIGHT_BTN = 7 # Right Buper
SHOULDER_AXES = 1 # Left Stick
ELBOW_AXES = 3 # Right Stick
WRIST_SPEED_AXES = 4 # Right Trigger
WRIST_ROLL_AXES = 6 # Left/Right D Pad
WRIST_PITCH_AXES = 7 #Up/Down D Pad 

TIMEOUT_DELAY_SEC = 2
TIMEOUT_CHECK_HZ = 2

MOTOR_STATUS_HZ = 2

MAX_DPS = 20

SHOULDER_LENGTH_MM = 437
ELBOW_LENGTH_MM = 462.25109
WRIST_LENGTH_MM = 236.45653

MAX_LINEAR_VEL_MPS = 0.05
SE_MIN = 30
EW_MIN = 45

RAIL_DPM = 44500 # Degrees Per Meter for the linear rail, found empirically
IK_FREQ = 5

DEBUGGING = True 

def clamp(val) -> int:
    return max(-1,min(val,1))

def normalize_joystick_axes_vals(value: float):
    return clamp(-1 * value)
        
def normalize_bumper_axes_vals(value: float):
    return clamp(1 - ((value + 1)/2))

class IK_Manipulator(Node):

    def __init__(self):
        super().__init__('ik_manipulator')
    
        # Declare Parameters
        self.declare_parameter("y_pos_btn", 6)
        self.declare_parameter("y_neg_btn", 7)
        self.declare_parameter("x_axis", 1)
        self.declare_parameter("z_axis", 3)

        # Get Parameter Values
        self.y_pos_btn  = self.get_parameter("y_pos_btn").get_parameter_value().integer_value
        self.y_neg_btn  = self.get_parameter("y_neg_btn").get_parameter_value().integer_value
        self.x_axis     = self.get_parameter("x_axis").get_parameter_value().integer_value
        self.z_axis     = self.get_parameter("z_axis").get_parameter_value().integer_value

        # PUBLISHERS
        self.joint_vel = self.create_publisher(JointState, 'joint_vel', 10)

        # SUBSCRIBERS
        self.joint_pos = self.create_subscription(JointState, 'joint_pos', self.get_arm_pos, 10)
        self.joy_input = self.create_subscription(Joy, 'ik_joy', self.update_joy_value, 10)

        # TIMERS 
        self.create_timer(1.0/IK_FREQ, self.calculate_motor_speeds)
        self.last_received_time = self.get_clock().now()

        # INITIALIZATION TASKS
        self.current_pos = JointState()
        self.current_joy = Joy()

        # self.send_reset_msgs()
        self.send_speed_commands(0,0,0,0,0)

    # Save the current arm position for usage
    def get_arm_pos(self, msg : JointState):
        self.current_pos = msg

    # Save the current joy value for usage
    def update_joy_value(self, msg : Joy):
        self.current_joy = msg

    # Wrap motor speeds into a joy message
    def send_speed_commands(self, rail, shoulder, elbow, roll, pitch):
        vel_state = JointState()

        vel_state.name = ["linear_rail", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]
        vel_state.velocity = [rail, shoulder, elbow, roll, pitch]
        
        self.joint_vel.publish(vel_state)

    # Callback for main functionality
    def calculate_motor_speeds(self):

        # What does the user want to do?

        x_vel = self.current_joy.axes[self.x_axis]
        y_vel = self.current_joy.buttons[self.y_pos_btn] - self.current_joy.buttons[self.y_neg_btn]
        z_vel = self.current_joy.axes[self.z_axis]

        # TODO Implement Inverse Kinematics Later

        rail_dps = floor(y_vel * 500)
        shoulder_dps = 0
        elbow_dps = 0
        wrist_roll_dps = 0
        wrist_pitch_dps = 0
        
        self.send_speed_commands(rail_dps, shoulder_dps, elbow_dps, wrist_roll_dps, wrist_pitch_dps)
    
def main(args=None):
    rclpy.init(args=args)
    ik_manipulator = IK_Manipulator()
    rclpy.spin(ik_manipulator)
    ik_manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()