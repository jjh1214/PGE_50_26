import rclpy
from rclpy.node import Node
import serial
import time
import struct

class PGE5026Controller(Node):
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

        # self.set_automatic_initialization(False)

        # time.sleep(5)

        # self.use_IO_mode(False)

        # time.sleep(5)

        # self.read_IO_mode_state()

        # time.sleep(5)

        self.set_speed_gripper(100)

        time.sleep(5)

        self.set_power_value_gripper(100)

        time.sleep(5)

        self.set_position_gripper(500)

        time.sleep(5)

        self.read_current_position()

        time.sleep(5)

        self.read_current_speed()

        time.sleep(5)

        self.read_power_value()

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

    def set_position_gripper(self, pos):
        position_hex = struct.pack('>H', pos)

        base_command = b'\x01\x06\x01\x03' + position_hex

        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)

        set_position_command = base_command + crc_hex

        try:
            self.serial.write(set_position_command)
            self.get_logger().info(f'Sent set position command to the gripper: {set_position_command.hex()}')
            
            response = self.serial.read(8)
            if len(response) < 8:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response == set_position_command:
                self.get_logger().info('Gripper position set successfully.')
            else:
                self.get_logger().warn('Unexpected response during gripper position set.')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending set position command: {str(e)}')

    def set_speed_gripper(self, speed):
        speed_hex = struct.pack('>H', speed)

        base_command = b'\x01\x06\x01\x04' + speed_hex

        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)

        set_speed_command = base_command + crc_hex

        try:
            self.serial.write(set_speed_command)
            self.get_logger().info(f'Sent set speed command to the gripper: {set_speed_command.hex()}')
            
            response = self.serial.read(8)
            if len(response) < 8:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response == set_speed_command:
                self.get_logger().info('Gripper speed set successfully.')
            else:
                self.get_logger().warn('Unexpected response during gripper speed set.')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending set speed command: {str(e)}')        

    def set_power_value_gripper(self, power):
        power_hex = struct.pack('>H', power)

        base_command = b'\x01\x06\x01\x01' + power_hex

        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)

        set_power_value_command = base_command + crc_hex

        try:
            self.serial.write(set_power_value_command)
            self.get_logger().info(f'Sent set power value command to the gripper: {set_power_value_command.hex()}')
            
            response = self.serial.read(8)
            if len(response) < 8:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response == set_power_value_command:
                self.get_logger().info('Gripper power value set successfully.')
            else:
                self.get_logger().warn('Unexpected response during gripper power value set.')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending set power value command: {str(e)}')            

    def set_automatic_initialization(self, bool):
        if bool == True:

            base_command = b'\x01\x06\x05\x04\x00\x01'

            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)

            set_automatic_initialization_command = base_command + crc_hex
            try:
                self.serial.write(set_automatic_initialization_command)
                self.get_logger().info(f'Sent set automatic initialization true command to the gripper: {set_automatic_initialization_command.hex()}')
                
                response = self.serial.read(8)
                if len(response) < 8:
                    self.get_logger().warn(f'Received incomplete response: {response.hex()}')
                else:
                    self.get_logger().info(f'Received response: {response.hex()}')

                if response == set_automatic_initialization_command:
                    self.get_logger().info('Gripper automatic initialization set successfully.')
                else:
                    self.get_logger().warn('Unexpected response during gripper automatic initialization set.')

            except serial.SerialException as e:
                self.get_logger().error(f'Error sending set automatic initialization command: {str(e)}')
        
        elif bool == False:
            
            base_command = b'\x01\x06\x05\x04\x00\x00'

            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)

            set_automatic_initialization_command = base_command + crc_hex
            try:
                self.serial.write(set_automatic_initialization_command)
                self.get_logger().info(f'Sent set automatic initialization false command to the gripper: {set_automatic_initialization_command.hex()}')
                
                response = self.serial.read(8)
                if len(response) < 8:
                    self.get_logger().warn(f'Received incomplete response: {response.hex()}')
                else:
                    self.get_logger().info(f'Received response: {response.hex()}')

                if response == set_automatic_initialization_command:
                    self.get_logger().info('Gripper automatic initialization set successfully.')
                else:
                    self.get_logger().warn('Unexpected response during gripper automatic initialization set.')

            except serial.SerialException as e:
                self.get_logger().error(f'Error sending set automatic initialization command: {str(e)}')

        else:
            pass

    def use_IO_mode(self, bool):
        if bool == True:

            base_command = b'\x01\x06\x04\x02\x00\x01'

            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)

            use_IO_mode_command = base_command + crc_hex
            try:
                self.serial.write(use_IO_mode_command)
                self.get_logger().info(f'Sent use IO mode true command to the gripper: {use_IO_mode_command.hex()}')
                
                response = self.serial.read(8)
                if len(response) < 8:
                    self.get_logger().warn(f'Received incomplete response: {response.hex()}')
                else:
                    self.get_logger().info(f'Received response: {response.hex()}')

                if response == use_IO_mode_command:
                    self.get_logger().info('Gripper IO mode set successfully.')
                else:
                    self.get_logger().warn('Unexpected response during gripper use IO mode set.')

            except serial.SerialException as e:
                self.get_logger().error(f'Error sending use IO mode command: {str(e)}')
        
        elif bool == False:
            
            base_command = b'\x01\x06\x04\x02\x00\x00'

            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)

            use_IO_mode_command = base_command + crc_hex
            try:
                self.serial.write(use_IO_mode_command)
                self.get_logger().info(f'Sent use IO mode false command to the gripper: {use_IO_mode_command.hex()}')
                
                response = self.serial.read(8)
                if len(response) < 8:
                    self.get_logger().warn(f'Received incomplete response: {response.hex()}')
                else:
                    self.get_logger().info(f'Received response: {response.hex()}')

                if response == use_IO_mode_command:
                    self.get_logger().info('Gripper use IO mode set false successfully.')
                else:
                    self.get_logger().warn('Unexpected response during gripper use IO mode set.')

            except serial.SerialException as e:
                self.get_logger().error(f'Error sending set use IO mode command: {str(e)}')

        else:
            pass        


    def read_IO_mode_state(self):
        base_command = b'\x01\x03\x04\x02'

        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)

        read_IO_mode_state_command = base_command + crc_hex

        try:
            self.serial.write(read_IO_mode_state_command)
            self.get_logger().info('Check IO mode state.')
            response = self.serial.read(8)
            if len(response) < 6:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            if response[3:5] == b'\x00\x01':
                self.get_logger().info('IO mode ON.')
            elif response[3:5] == b'\x00\x00':
                self.get_logger().warn('IO mode OFF.')
            else:
                self.get_logger().error(f'Unexpected IO mode state: {response[3:5].hex()}')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending IO mode state command: {str(e)}')

    def read_current_position(self):
        read_current_position_command = b'\x01\x03\x02\x02\x00\x01\x24\x72'

        try:
            self.serial.write(read_current_position_command)
            self.get_logger().info('Read real time position.')
            response = self.serial.read(8)
            if len(response) < 6:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            position = int.from_bytes(response[3:5], byteorder='big')
            self.get_logger().info(f'Current position (decimal): {position}‰ (thousand ratio)')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending read current position command: {str(e)}')        

    def read_current_speed(self):
        read_current_speed_command = b'\x01\x03\x01\x04\x00\x01\xC4\x37'

        try:
            self.serial.write(read_current_speed_command)
            self.get_logger().info('Read current speed set.')
            response = self.serial.read(8)
            if len(response) < 6:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            position = int.from_bytes(response[3:5], byteorder='big')
            self.get_logger().info(f'Current speed set (decimal): {position}%')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending read current speed command: {str(e)}')      

    def read_power_value(self):
        read_power_value_command = b'\x01\x03\x01\x01\x00\x01\xD4\x36'

        try:
            self.serial.write(read_power_value_command)
            self.get_logger().info('Read current power value set.')
            response = self.serial.read(8)
            if len(response) < 6:
                self.get_logger().warn(f'Received incomplete response: {response.hex()}')
            else:
                self.get_logger().info(f'Received response: {response.hex()}')

            position = int.from_bytes(response[3:5], byteorder='big')
            self.get_logger().info(f'Current power value set (decimal): {position}%')

        except serial.SerialException as e:
            self.get_logger().error(f'Error sending read power value command: {str(e)}')      

    def calculate_crc(self, command):
        crc = 0xFFFF
        for pos in command:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

def main(args=None):
    rclpy.init(args=args)
    node = PGE5026Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pge_50_26_node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()