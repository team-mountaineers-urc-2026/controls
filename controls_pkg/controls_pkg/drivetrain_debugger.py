import rclpy
from rclpy.node import Node
from controls_msgs.msg import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

VIEWABLE_DATA_POINTS = 600

class DebugGraphing(Node):
    def __init__(self):
        super().__init__('drivetrain_debugging')

        # Speed Topics
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/drivetrain/front_right/rcvd/speed_control',
            self.update_fr_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/drivetrain/front_left/rcvd/speed_control',
            self.update_fl_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/drivetrain/back_right/rcvd/speed_control',
            self.update_br_recv_speed,
            10
        )
        self.create_subscription(
            SpeedClosedLoopControlMsgRecvParams,
            '/drivetrain/back_left/rcvd/speed_control',
            self.update_bl_recv_speed,
            10
        )

        # Status Topics
        self.create_subscription(
            ReadMotorStatus1MsgRecvParams,
            '/drivetrain/front_right/rcvd/status1',
            self.update_fr_recv_status,
            10
        )
        self.create_subscription(
            ReadMotorStatus1MsgRecvParams,
            '/drivetrain/front_left/rcvd/status1',
            self.update_fl_recv_status,
            10
        )
        self.create_subscription(
            ReadMotorStatus1MsgRecvParams,
            '/drivetrain/back_right/rcvd/status1',
            self.update_br_recv_status,
            10
        )
        self.create_subscription(
            ReadMotorStatus1MsgRecvParams,
            '/drivetrain/back_left/rcvd/status1',
            self.update_bl_recv_status,
            10
        )

        # Initialize data lists
        self.fr_current = []
        self.fl_current = [] 
        self.br_current = [] 
        self.bl_current = [] 
        self.fr_voltage = []
        self.fl_voltage = []
        self.br_voltage = []
        self.bl_voltage = []

        # Set up the plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

    # Speed
    def update_fr_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.fr_current.append(abs(msg.current_amps))
        self.fr_current = self.fr_current[-1*VIEWABLE_DATA_POINTS:]
        
    def update_fl_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.fl_current.append(abs(msg.current_amps))
        self.fl_current = self.fl_current[-1*VIEWABLE_DATA_POINTS:]

    def update_br_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.br_current.append(abs(msg.current_amps))
        self.br_current = self.br_current[-1*VIEWABLE_DATA_POINTS:]

    def update_bl_recv_speed(self, msg: SpeedClosedLoopControlMsgRecvParams):
        self.bl_current.append(abs(msg.current_amps))
        self.bl_current = self.bl_current[-1*VIEWABLE_DATA_POINTS:]

    # Status
    def update_fr_recv_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.fr_voltage.append(msg.voltage_volts)
        self.fr_voltage = self.fr_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_fl_recv_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.fl_voltage.append(msg.voltage_volts)
        self.fl_voltage = self.fl_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_br_recv_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.br_voltage.append(msg.voltage_volts)
        self.br_voltage = self.br_voltage[-1*VIEWABLE_DATA_POINTS:]

    def update_bl_recv_status(self, msg: ReadMotorStatus1MsgRecvParams):
        self.bl_voltage.append(msg.voltage_volts)
        self.bl_voltage = self.bl_voltage[-1*VIEWABLE_DATA_POINTS:]


    # Plot updating function
    def update_plot(self, frame):
        self.ax1.clear()
        self.ax2.clear()

        # Plot currents
        self.ax1.plot(self.fr_current, label='Front Right')
        self.ax1.plot(self.fl_current, label='Front Left')
        self.ax1.plot(self.br_current, label='Back Right')
        self.ax1.plot(self.bl_current, label='Back Left')
        self.ax1.set_title('Current (Amps)')
        self.ax1.set_ylabel('Current (A)')
        self.ax1.legend()

        # Plot voltages
        self.ax2.plot(self.fr_voltage, label='Front Right')
        self.ax2.plot(self.fl_voltage, label='Front Left')
        self.ax2.plot(self.br_voltage, label='Back Right')
        self.ax2.plot(self.bl_voltage, label='Back Left')
        self.ax2.set_title('Voltage (Volts)')
        self.ax2.set_ylabel('Voltage (V)')
        self.ax2.legend()

        self.fig.suptitle('Drivetrain Data')
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
