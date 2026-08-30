import rclpy
from rclpy.node import Node
from controls_msgs.msg import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

VIEWABLE_DATA_POINTS = 600

class DebugGraphing(Node):
    def __init__(self):
        super().__init__('manipulator_debugging')

        # Speed Topics
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/shoulder/rcvd/speed_control',
            self.update_shoulder_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/elbow/rcvd/speed_control',
            self.update_elbow_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/wrist_roll/rcvd/speed_control',
            self.update_roll_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/wrist_pitch/rcvd/speed_control',
            self.update_pitch_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/linear_rail/rcvd/speed_control',
            self.update_rail_recv_speed,
            10
        )

        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/gripper/rcvd/speed_control',
            self.update_gripper_recv_speed,
            10
        )

        # Status Topics
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/shoulder/rcvd/status1',
            self.update_shoulder_status,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/elbow/rcvd/status1',
            self.update_elbow_status,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/wrist_roll/rcvd/status1',
            self.update_roll_status,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/wrist_pitch/rcvd/status1',
            self.update_pitch_status,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/linear_rail/rcvd/status1',
            self.update_rail_status,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/manipulator/gripper/rcvd/status1',
            self.update_gripper_status,
            10
        )

        self.shoulder_current = []
        self.elbow_current = []
        self.roll_current = []
        self.pitch_current = []
        self.rail_current = []
        self.gripper_current = []

        self.shoulder_voltage = []
        self.elbow_voltage = []
        self.roll_voltage = []
        self.pitch_voltage = []
        self.rail_voltage = []
        self.gripper_voltage = []

        # Set up the plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

    # Current
    def update_shoulder_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.shoulder_current.append(abs(msg.current_amps))
        self.shoulder_current = self.shoulder_current[-1*VIEWABLE_DATA_POINTS:]
        
    def update_elbow_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.elbow_current.append(abs(msg.current_amps))
        self.elbow_current = self.elbow_current[-1*VIEWABLE_DATA_POINTS:]

    def update_roll_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.roll_current.append(abs(msg.current_amps))
        self.roll_current = self.roll_current[-1*VIEWABLE_DATA_POINTS:]

    def update_pitch_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.pitch_current.append(abs(msg.current_amps))
        self.pitch_current = self.pitch_current[-1*VIEWABLE_DATA_POINTS:]

    def update_rail_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.rail_current.append(abs(msg.current_amps))
        self.rail_current = self.rail_current[-1*VIEWABLE_DATA_POINTS:]
    def update_gripper_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.gripper_current.append(abs(msg.current_amps))
        self.gripper_current = self.gripper_current[-1*VIEWABLE_DATA_POINTS:]

    # Status
    def update_shoulder_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.shoulder_voltage.append(msg.voltage_volts)
        self.shoulder_voltage = self.shoulder_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_elbow_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.elbow_voltage.append(msg.voltage_volts)
        self.elbow_voltage = self.elbow_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_pitch_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.pitch_voltage.append(msg.voltage_volts)
        self.pitch_voltage = self.pitch_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_roll_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.roll_voltage.append(msg.voltage_volts)
        self.roll_voltage = self.roll_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_rail_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.rail_voltage.append(msg.voltage_volts)
        self.rail_voltage = self.rail_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_gripper_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.gripper_voltage.append(msg.voltage_volts)
        self.gripper_voltage = self.gripper_voltage[-1*VIEWABLE_DATA_POINTS:] 

    # Plot updating function
    def update_plot(self, frame):
        self.ax1.clear()
        self.ax2.clear()

        # Plot currents
        self.ax1.plot(self.shoulder_current, label='Shoulder')
        self.ax1.plot(self.elbow_current, label='Elbow')
        self.ax1.plot(self.pitch_current, label='Pitch')
        self.ax1.plot(self.roll_current, label='Roll')
        self.ax1.plot(self.rail_current, label='Rail')
        self.ax1.plot(self.gripper_current, label='Gripper')
        self.ax1.set_title('Current (Amps)')
        self.ax1.set_ylabel('Current (A)')
        self.ax1.legend()

        # Plot voltages
        self.ax2.plot(self.shoulder_voltage, label='Shoulder')
        self.ax2.plot(self.elbow_voltage, label='Elbow')
        self.ax2.plot(self.pitch_voltage, label='Pitch')
        self.ax2.plot(self.roll_voltage, label='Roll')
        self.ax2.plot(self.rail_voltage, label='Rail')
        self.ax2.plot(self.gripper_voltage, label='Rail')
        self.ax2.set_title('Voltage (Volts)')
        self.ax2.set_ylabel('Voltage (V)')
        self.ax2.legend()

        self.fig.suptitle('Manipulator Data')
        plt.tight_layout()

    def show_plot(self):
        plt.show()


def main():  
    rclpy.init()
    node = DebugGraphing()

    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    try:
        node.show_plot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
