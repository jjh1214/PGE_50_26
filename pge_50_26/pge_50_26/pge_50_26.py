import rclpy
from rclpy.node import Node
import serial
import time

class DHrobotics_Gripper(Node):
    def __init__(self):
        super().__init__('pge_50_26_node')

        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200  
        self.timeout = 1
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'Successfully connected to {self.port} at {self.baudrate} baudrate.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {str(e)}')
            return

        self.initialization_state()

        time.sleep(5)

        self.initialize_gripper()

        time.sleep(5)

        self.initialization_state()

        time.sleep(5)

        self.set_position_gripper()

        self.timer = self.create_timer(1.0, self.read_data_callback)

    def initialize_gripper(self):
        init_command = b'\x01\x06\x01\x00\x00\x01\x49\xF6'

        try:
            self.serial.write(init_command)
            self.get_logger().info('Sent initialization command to the gripper.')
            response = self.serial.read(8)
            if len(response) < 8:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response == init_command:
                self.get_logger().info('Gripper initialization successful.')
            else:
                self.get_logger().warn('Unexpected response during initialization.')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending initialization command: {str(e)}')

    def initialization_state(self):
        read_initialization_state_command = b'\x01\x03\x02\x00\x00\x01\x85\xB2'

        try:
            self.serial.write(read_initialization_state_command)
            self.get_logger().info('Check initialization state.')
            response = self.serial.read(8)
            if len(response) < 6:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response[3:5] == b'\x00\x01':
                self.get_logger().info('Gripper is initialized.')
            elif response[3:5] == b'\x00\x00':
                self.get_logger().warn('Gripper is not initialized.')
            else:
                self.get_logger().error(f'Unexpected initialization state: {response[3:5].hex()}')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending initialization state command: {str(e)}')

    def reinitialize_gripper(self):
        reinit_command = b'\x01\x06\x01\x00\x00\xA5\x48\x4D'
        try:
            self.serial.write(reinit_command)
            self.get_logger().info('Sent reinitialization command to the gripper.')
            response = self.serial.read(8)
            self.get_logger().info(f'Received response: {response.hex()}')

            if response == reinit_command:
                self.get_logger().info('Gripper reinitialization successful.')
            else:
                self.get_logger().warn('Unexpected response during reinitialization.')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending reinitialization command: {str(e)}')

    def set_position_gripper(self):
        set_position_command = b'\x01\x06\x01\x03\x01\xF4\x78\x21'

        try:
            self.serial.write(set_position_command)
            self.get_logger().info('Sent set_position_command to the gripper.')
            response = self.serial.read(8)
            if len(response) < 8:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response == set_position_command:
                self.get_logger().info('Gripper position set successful.')
            else:
                self.get_logger().warn('Unexpected response during initialization.')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending set_position_command command: {str(e)}')

    def read_data_callback(self):
        if self.serial.in_waiting > 0:
            try:
                data = self.serial.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received data: {data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading data: {str(e)}')

    def send_data(self, data):
        try:
            self.serial.write(data.encode('utf-8'))
            self.get_logger().info(f'Sent data: {data}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error sending data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DHrobotics_Gripper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down RS485 node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()