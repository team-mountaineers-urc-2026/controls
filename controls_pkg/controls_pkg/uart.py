import rclpy, threading, serial
from rclpy.node import Node
from controls_msgs.srv import UartSendRecv
import time

'''
TO DO:
- Utilize SingleThreadedExecutor or make node for each serial port (most likely)
- Determine timing and delays for send/recv portion
- Change mapping values to reflect real system
- Ensure that the id's are stored somewhere only once and manipulator, drivetrain, and interfaces  
'''

def make_serial(port):
    return serial.Serial(
        port = port,
        baudrate=115200,
        timeout=3,
    )

_PORT_NAME_MAPPINGS = {
    # Drivebase
    0x141 : '/dev/ttyUSB0',
    0x142 : '/dev/ttyUSB1',
    0x143 : '/dev/ttyUSB2',
    0x144 : '/dev/ttyUSB3',

    # Manipulator
    0x145 : '/dev/ttyUSB4',
    0x146 : '/dev/ttyUSB5',
    0x147 : '/dev/ttyUSB6',
    0x148 : '/dev/ttyUSB7',
    0x149 : '/dev/ttyUSB8',

}

_PORT_MAPPINGS = { key: make_serial(port) for key, port in _PORT_NAME_MAPPINGS.items() }


class UartInterfaceNode(Node):

    thread_lock = threading.Lock()

    def __init__(self):

        super().__init__('uart_interface_node')
        self.can_service = self.create_service(
            UartSendRecv,
            '/uart_send_recv',
            self.send_uart_command
            )

    def send_uart_command(self, request, response):

        requested_port = _PORT_MAPPINGS.get(request.arbitration_id)

        if requested_port == None:
            raise ValueError(f"The arbitration id {request.arbitration_id} is not in use")
    
        with UartInterfaceNode.thread_lock:

            # Delays and timings may need to be played with here
            # in_waiting method ensures data HAS to be there
            # need to add error handling for if no response
            requested_port.write(request.uart_data)
            while requested_port.in_waiting < 13: 
                time.sleep(0.001)
            recv = requested_port.readline()

        response.arbitration_id = request.arbitration_id
        response.uart_data = recv
        return response
        
def main():
    rclpy.init()
    uart_interface_node = UartInterfaceNode()
    rclpy.spin(uart_interface_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()