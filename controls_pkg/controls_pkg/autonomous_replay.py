from rclpy.node import Node
import rclpy
from controls_msgs.msg import ReadMotorStatus2MsgSentParams as SendStatus2, ReadMotorStatus2MsgRecvParams as RecvStatus2, SpeedClosedLoopControlMsgSentParams as SendSpeed
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, Empty, String
import numpy
from time import time

class AutonomousReplay(Node):

    def __init__(self):

        super().__init__('autonomous_replay', namespace='manipulator')
        
        # Get Position Publishers
        self.rail_pos_req           = self.create_publisher(SendStatus2, 'linear_rail/send/status2', 10)
        self.shoulder_pos_req       = self.create_publisher(SendStatus2, 'shoulder/send/status2', 10)
        self.elbow_pos_req          = self.create_publisher(SendStatus2, 'elbow/send/status2', 10)
        self.wrist_pitch_pos_req    = self.create_publisher(SendStatus2, 'wrist_pitch/send/status2', 10)
        self.wrist_roll_pos_req     = self.create_publisher(SendStatus2, 'wrist_roll/send/status2', 10)

        # Read Position Subscribers
        self.rail_pos_resp          = self.create_subscription(RecvStatus2, 'linear_rail/rcvd/status2', lambda msg, motor='linear_rail': self.listener_callback(msg, motor), 10)
        self.shoulder_pos_resp      = self.create_subscription(RecvStatus2, 'shoulder/rcvd/status2',    lambda msg, motor='shoulder':    self.listener_callback(msg, motor), 10)
        self.elbow_pos_resp         = self.create_subscription(RecvStatus2, 'elbow/rcvd/status2',       lambda msg, motor='elbow':       self.listener_callback(msg, motor), 10)
        self.wrist_pitch_pos_resp   = self.create_subscription(RecvStatus2, 'wrist_pitch/rcvd/status2', lambda msg, motor='wrist_pitch': self.listener_callback(msg, motor), 10)
        self.wrist_roll_pos_resp    = self.create_subscription(RecvStatus2, 'wrist_roll/rcvd/status2',  lambda msg, motor='wrist_roll':  self.listener_callback(msg, motor), 10)

        # Additional Subscriptions
        self.store_position_sub     = self.create_subscription(Bool, 'store_position', self.store_position, 10)
        self.pop_position_sub       = self.create_subscription(Empty, 'pop_position', self.pop_position, 10)
        self.reset_playback_sub     = self.create_subscription(Empty, 'reset_playback', self.reset_playback, 10)
        self.playback_status        = self.create_subscription(Bool, 'playback_status', self.playback_status, 10)
        self.intervention_sub       = self.create_subscription(Bool, 'human_intervention_over', self.intervention_callback, 10)

        # Send Speed Publishers
        self.joint_vel = self.create_publisher(JointState,"joint_vel", 10)

        # Autonomy Light Publishers
        self.light_pub = self.create_publisher(String, '/led_color_topic', 10)

        self.req_timer = self.create_timer(1.0/200.0, self.requester_callback)
        self.med_timer = self.create_timer(1.0/60.0, self.median_callback)
        self.med_timer.cancel()

        self.rail_pos       = None
        self.shoulder_pos   = None
        self.elbow_pos      = None
        self.pitch_pos      = None
        self.roll_pos       = None

        # Rail Shoulder Elbow Pitch Roll Solenoid
        self.rail_complete = False
        self.shoulder_complete = False
        self.elbow_complete = False
        self.pitch_complete = False
        self.roll_complete = False
        self.solenoid_complete = False

        self.waiting_for_intervention = False

        self.pos_list = []

        self.current_place = 0

        self.stopping_threshold = 2
        self.intervention_counter = 0

    def requester_callback(self):
        self.rail_pos_req.publish(SendStatus2())
        self.shoulder_pos_req.publish(SendStatus2())
        self.elbow_pos_req.publish(SendStatus2())
        self.wrist_pitch_pos_req.publish(SendStatus2())
        self.wrist_roll_pos_req.publish(SendStatus2())

    def listener_callback(self, msg, motor):

        match motor:

            case "linear_rail":
                self.rail_pos = msg.angle_degrees

            case "shoulder":
                self.shoulder_pos = msg.angle_degrees

            case "elbow":
                self.elbow_pos = msg.angle_degrees

            case "wrist_roll":
                self.roll_pos = msg.angle_degrees

            case "wrist_pitch":
                self.pitch_pos = msg.angle_degrees

            case _:
                self.get_logger().info(f"{motor} not found, setting all motor positions to None")
                self.rail_pos       = None
                self.shoulder_pos   = None
                self.elbow_pos      = None
                self.roll_pos       = None
                self.pitch_pos      = None

    def median_callback(self):

        if self.current_place >= len(self.pos_list):
            # Publish Blue lights
            self.light_pub.publish(String(data="teleop"))
            self.get_logger().info("Finished")
            self.current_place = 0
            self.med_timer.cancel()
            return
        curr_target = self.pos_list[self.current_place]
        output_motors = ["linear_rail", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]
        output_vels = []

        # RAIL
        if abs(self.rail_pos - curr_target[0]) < 30:
            self.rail_complete = True
            output_vels.append(0.0)

        else:
            self.rail_complete = False
            rail_vel = 30 * numpy.sign(curr_target[0] - self.rail_pos) + (curr_target[0] - self.rail_pos) * 0.5
            if abs(rail_vel) > 900:
                rail_vel =  900 * numpy.sign(curr_target[0] - self.rail_pos)
            output_vels.append(rail_vel)

        # SHOULDER
        if abs(self.shoulder_pos - curr_target[1]) < 3:
            self.shoulder_complete = True
            output_vels.append(0.0)

        else:
            self.shoulder_complete = False
            shoulder_vel = numpy.sign(curr_target[1] - self.shoulder_pos)*6 + 0.1 * (curr_target[1] - self.shoulder_pos)
            output_vels.append(shoulder_vel)

        # ELBOW
        # self.elbow_complete = True
        # output_vels.append(0.0)
        if abs(self.elbow_pos - curr_target[2]) < 3:
            self.elbow_complete = True
            output_vels.append(0.0)

        else:
            self.elbow_complete = False
            elbow_vel = numpy.sign(curr_target[2] - self.elbow_pos) * 6 + 0.1 * (curr_target[2] - self.elbow_pos)
            output_vels.append(elbow_vel)

        # PITCH
        if abs(self.pitch_pos - curr_target[3]) < 4:
            self.pitch_complete = True
            output_vels.append(0.0)

        else:
            self.pitch_complete = False
            pitch_vel = numpy.sign(curr_target[3] - self.pitch_pos) * 10 + 0.1 * (curr_target[3] - self.pitch_pos)
            output_vels.append(pitch_vel)

        # ROLL
        if abs(self.roll_pos - curr_target[4]) < 5:
            self.roll_complete = True
            output_vels.append(0.0)

        else:
            self.roll_complete = False
            roll_vel = numpy.sign(curr_target[4] - self.roll_pos) * 24 + 0.1 * (curr_target[4] - self.roll_pos)
            output_vels.append(roll_vel)


        # If we are done
        if self.rail_complete and self.shoulder_complete and self.elbow_complete and self.pitch_complete and self.roll_complete:
            
            # Check for solenoid
            if curr_target[5] == 1 and not self.solenoid_complete:

                self.intervention_counter += 1
                if self.intervention_counter >= 30:
                    self.get_logger().info("Waiting For Human Intervention")
                    self.intervantion_counter = 0

                # set request for human intervention
                self.waiting_for_intervention = True

        # If each of the different things are done and we are allowed to move one
            if not self.waiting_for_intervention:
                self.get_logger().info("Continuing to next place")
                self.current_place += 1
                self.rail_complete = False
                self.shoulder_complete = False
                self.elbow_complete = False
                self.pitch_complete = False
                self.roll_complete = False
                self.solenoid_complete = False

                self.waiting_for_intervention = False
                # continue;

        # No matter what, tell it to move how it should
        vel_state = JointState(name = output_motors, velocity=output_vels)
        self.joint_vel.publish(vel_state)

    def intervention_callback(self, msg : Bool):
        self.get_logger().info("Human Intervened")
        if msg.data:
            self.solenoid_complete = True
            self.waiting_for_intervention = False
            self.get_logger().info("Successful Intervention")

        else:
            self.get_logger().info("Unsuccessful Intervention")


    def store_position(self, msg):
        solenoid = 1 if msg.data else 0
        positions = [self.rail_pos, self.shoulder_pos, self.elbow_pos, self.pitch_pos, self.roll_pos, solenoid]
        self.pos_list.append(positions)
        self.get_logger().info("Added place")
        for pos in self.pos_list:
            self.get_logger().info(f"{pos[0]}, {pos[1]}, {pos[2]}, {pos[3]}, {pos[4]}, {pos[5]}")

        self.get_logger().info("")

    def pop_position(self, msg):
        self.pos_list.pop()
        self.get_logger().info("Removed Last added place")

    def reset_playback(self, msg):
        self.current_place = 0
            
    def playback_status(self, msg):
        if msg.data:
            self.med_timer.reset()
            self.light_pub.publish(String(data="autonomous"))
            # pass
        else:
            self.med_timer.cancel()
            output_motors = ["linear_rail", "shoulder", "elbow", "pitch", "roll"]
            self.joint_vel.publish(JointState(name=output_motors, velocity=[0.0, 0.0, 0.0, 0.0, 0.0]))

def main(args=None):

    rclpy.init(args=args)
    autonomous_replay = AutonomousReplay()
    rclpy.spin(autonomous_replay)
    autonomous_replay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()