from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Float32
from controls_msgs.msg import SpeedClosedLoopControlMsgSentParams as SendSpeed, ReadMotorStatus1MsgSentParams as SendStatus1
import math, rclpy

'''
TO DO:
- Change py constants to ros-accessible params
- If not recieved messsage in x seconds, send zero speed
- If recieved many move msgs, then stopped, make sure to send zero speed
- Autodetect uart or can failure and switch to different system
'''

TRACK_WIDTH_M = 1.0
WHEEL_RADIUS = 0.26
GEAR_RATIO = 1/36

TIMEOUT_DELAY_SEC = 1
TIMEOUT_CHECK_HZ = 4

MOTOR_STATUS_HZ = 1

SPEED_COEFF = 16

DEBUGGING = False

class Drivetrain(Node):

    def __init__(self):

        super().__init__('drivetrain')
        
        self.declare_parameter("safety_threshold", math.pi/4.0)
        self.declare_parameter("slow_factor", 0.5)
        self.declare_parameter("speed_multiplier", 1.0)

        self.safety_threshold = self.get_parameter('safety_threshold').get_parameter_value().double_value
        self.slow_factor = self.get_parameter('slow_factor').get_parameter_value().double_value
        self.speed_multiplier = self.get_parameter('speed_multiplier').get_parameter_value().double_value

        self.create_subscription(Twist,"cmd_vel",self.send_drivebase_command,qos_profile_sensor_data)
        self.create_subscription(Bool, "toggle_safety", self.toggle_safety, 10)
        self.create_subscription(Vector3, "/health_monitor/chassis_orientation", self.toggle_danger, qos_profile_sensor_data)

        self.create_subscription(Float32, "speed_multiplier", self.update_speed_mult, 10)
        
        # Speed Publishers
        self.front_left_pub = self.create_publisher(SendSpeed,  "front_left/send/speed_control",    10)
        self.front_right_pub = self.create_publisher(SendSpeed, "front_right/send/speed_control",   10)
        self.back_left_pub = self.create_publisher(SendSpeed,   "back_left/send/speed_control",     10)
        self.back_right_pub = self.create_publisher(SendSpeed,  "back_right/send/speed_control",    10)

        # Status Publishers 
        self.front_left_status1_pub = self.create_publisher(SendStatus1,"front_left/send/status1",10)
        self.front_right_status1_pub = self.create_publisher(SendStatus1,"front_right/send/status1",10)
        self.back_left_status1_pub = self.create_publisher(SendStatus1,"back_left/send/status1",10)
        self.back_right_status1_pub = self.create_publisher(SendStatus1,"back_right/send/status1",10)

        self.create_timer(1/MOTOR_STATUS_HZ,self.send_status_msgs)
        self.create_timer(1.0/TIMEOUT_CHECK_HZ, self.check_message_timeout)
        self.last_received_time = self.get_clock().now()

        self.in_danger = False
        self.drive_safety = True

    def send_status_msgs(self):
        status_msg = SendStatus1()
        self.front_left_status1_pub.publish(status_msg)
        self.front_right_status1_pub.publish(status_msg)
        self.back_left_status1_pub.publish(status_msg)
        self.back_right_status1_pub.publish(status_msg)

    def update_speed_mult(self, msg):
        # self.get_logger().info(f'{msg.data}')
        self.speed_multiplier = msg.data

    def send_speed_commands(self, left_dps, right_dps):
        right_msg = SendSpeed(); right_msg.speed_dps = right_dps
        left_msg = SendSpeed(); left_msg.speed_dps = left_dps

        self.front_left_pub.publish(left_msg)
        self.front_right_pub.publish(right_msg)
        self.back_left_pub.publish(left_msg)
        self.back_right_pub.publish(right_msg)

        
    def check_message_timeout(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_received_time).nanoseconds / 1e9 > TIMEOUT_DELAY_SEC:  
            # self.get_logger().warning(f'No cmd_vel message received in the last {TIMEOUT_DELAY_SEC} seconds, zeroing motor speeds')
            self.send_speed_commands(0,0)
            self.last_received_time = self.get_clock().now() # repeated zeros has TIMEOUT_DELAY_SEC delay between them

    
    def toggle_safety(self, bool_msg: Bool):
        self.drive_safety = bool_msg.data

    def toggle_danger(self, msg: Vector3):

        if abs(msg.x) > self.safety_threshold or abs(msg.y) > self.safety_threshold:
            if not self.in_danger:
                self.get_logger().info("DANGER DETECTED, SLOWING DOWN")
            self.in_danger = True
        else:
            if self.in_danger:
                self.get_logger().info("DANGER HAS PASSED")
            self.in_danger = False

    def send_drivebase_command(self, twist_msg: Twist):
        try:
            self.last_received_time = self.get_clock().now()

            # Adjust to match last year
            lin_vel = twist_msg.linear.x * 1.85 * self.speed_multiplier
            ang_vel = twist_msg.angular.z * 6 * self.speed_multiplier
            # self.get_logger().info(f"{ang_vel}")

            left_velocity = -lin_vel + ang_vel * (TRACK_WIDTH_M / 2)
            right_velocity = lin_vel + ang_vel * (TRACK_WIDTH_M / 2)
            
            left_dps = int(math.floor(SPEED_COEFF * (left_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360))
            right_dps = int(math.floor(SPEED_COEFF * (right_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360))

            # Safety Stuff
            if self.drive_safety and self.in_danger:
                left_dps = int(left_dps * self.slow_factor)
                right_dps = int(right_dps * self.slow_factor)

            # If the speed is low enough to cause the PID controller to explode, don't send it
            if (abs(left_dps) < 6):
                left_dps = 0

            elif (left_dps > 200):
                left_dps = 200

            elif (left_dps < -200):
                left_dps = -200

            if (abs(right_dps) < 6):
                right_dps = 0

            elif (right_dps > 200):
                right_dps = 200

            elif (right_dps < -200):
                right_dps = -200

            # if DEBUGGING:
            #     self.get_logger().info(f"left_dps{left_dps}\nright_dps{right_dps}")
            # self.get_logger().info(f"Left: {left_dps}\tRight: {right_dps}")

            self.send_speed_commands(left_dps,right_dps)

        except Exception as e:
            self.get_logger().warning(f"Error in handling of drivetrain control {e.with_traceback()}, sending zero speeds")
            self.send_speed_commands(0,0)


            
def main(args=None):

    rclpy.init(args=args)
    drivetrain = Drivetrain()
    rclpy.spin(drivetrain)
    drivetrain.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()