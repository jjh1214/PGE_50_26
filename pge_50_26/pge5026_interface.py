import serial
import struct

class PGE5026Interface:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f'Successfully connected to {self.port} at {self.baudrate} baudrate.')
        except serial.SerialException as e:
            print(f'Error opening serial port: {str(e)}')

    def initialize_gripper(self):
        init_command = b'\x01\x06\x01\x00\x00\x01\x49\xF6'
        try:
            self.serial.write(init_command)
            print(f'Sent initialization command to the gripper.')
            response = self.serial.read(8)
            if len(response) < 8:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response == init_command:
                print('Gripper initialization successful.')
            else:
                print('Unexpected response during initialization.')
        except serial.SerialException as e:
            print(f'Error sending initialization command: {str(e)}')

    def initialization_state(self):
        read_initialization_state_command = b'\x01\x03\x02\x00\x00\x01\x85\xB2'
        try:
            self.serial.write(read_initialization_state_command)
            print(f'Check initialization state.')
            response = self.serial.read(8)
            if len(response) < 6:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response[3:5] == b'\x00\x01':
                print('Gripper is initialized.')
            elif response[3:5] == b'\x00\x00':
                print('Gripper is not initialized.')
            else:
                print(f'Unexpected initialization state: {response[3:5].hex()}')
        except serial.SerialException as e:
            print(f'Error sending initialization state command: {str(e)}')

    def reinitialize_gripper(self):
        reinit_command = b'\x01\x06\x01\x00\x00\xA5\x48\x4D'
        try:
            self.serial.write(reinit_command)
            print(f'Sent reinitialization command to the gripper.')
            response = self.serial.read(8)
            print(f'Received response: {response.hex()}')
            if response == reinit_command:
                print('Gripper reinitialization successful.')
            else:
                print('Unexpected response during reinitialization.')
        except serial.SerialException as e:
            print(f'Error sending reinitialization command: {str(e)}')

    def set_position_gripper(self, pos):
        position_hex = struct.pack('>H', pos)
        base_command = b'\x01\x06\x01\x03' + position_hex
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        set_position_command = base_command + crc_hex
        try:
            self.serial.write(set_position_command)
            print(f'Sent set position command to the gripper: {set_position_command.hex()}')
            response = self.serial.read(8)
            if len(response) < 8:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response == set_position_command:
                print('Gripper position set successfully.')
            else:
                print('Unexpected response during gripper position set.')
        except serial.SerialException as e:
            print(f'Error sending set position command: {str(e)}')

    def set_speed_gripper(self, speed):
        speed_hex = struct.pack('>H', speed)
        base_command = b'\x01\x06\x01\x04' + speed_hex
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        set_speed_command = base_command + crc_hex
        try:
            self.serial.write(set_speed_command)
            print(f'Sent set speed command to the gripper: {set_speed_command.hex()}')
            response = self.serial.read(8)
            if len(response) < 8:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response == set_speed_command:
                print('Gripper speed set successfully.')
            else:
                print('Unexpected response during gripper speed set.')
        except serial.SerialException as e:
            print(f'Error sending set speed command: {str(e)}')

    def set_power_value_gripper(self, power):
        power_hex = struct.pack('>H', power)
        base_command = b'\x01\x06\x01\x01' + power_hex
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        set_power_value_command = base_command + crc_hex
        try:
            self.serial.write(set_power_value_command)
            print(f'Sent set power value command to the gripper: {set_power_value_command.hex()}')
            response = self.serial.read(8)
            if len(response) < 8:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response == set_power_value_command:
                print('Gripper power value set successfully.')
            else:
                print('Unexpected response during gripper power value set.')
        except serial.SerialException as e:
            print(f'Error sending set power value command: {str(e)}')

    def set_automatic_initialization(self, bool):
        if bool == True:
            base_command = b'\x01\x06\x05\x04\x00\x01'
            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)
            set_automatic_initialization_command = base_command + crc_hex
            try:
                self.serial.write(set_automatic_initialization_command)
                print(f'Sent set automatic initialization true command to the gripper: {set_automatic_initialization_command.hex()}')
                response = self.serial.read(8)
                if len(response) < 8:
                    print(f'Received incomplete response: {response.hex()}')
                else:
                    print(f'Received response: {response.hex()}')
                if response == set_automatic_initialization_command:
                    print('Gripper automatic initialization set successfully.')
                else:
                    print('Unexpected response during gripper automatic initialization set.')
            except serial.SerialException as e:
                print(f'Error sending set automatic initialization command: {str(e)}')
        
        elif bool == False:
            base_command = b'\x01\x06\x05\x04\x00\x00'
            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)
            set_automatic_initialization_command = base_command + crc_hex
            try:
                self.serial.write(set_automatic_initialization_command)
                print(f'Sent set automatic initialization false command to the gripper: {set_automatic_initialization_command.hex()}')
                response = self.serial.read(8)
                if len(response) < 8:
                    print(f'Received incomplete response: {response.hex()}')
                else:
                    print(f'Received response: {response.hex()}')
                if response == set_automatic_initialization_command:
                    print('Gripper automatic initialization set successfully.')
                else:
                    print('Unexpected response during gripper automatic initialization set.')
            except serial.SerialException as e:
                print(f'Error sending set automatic initialization command: {str(e)}')

        else:
            print(f'Wrong input. Please request True or False')

    def use_IO_mode(self, bool):
        if bool == True:
            base_command = b'\x01\x06\x04\x02\x00\x01'
            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)
            use_IO_mode_command = base_command + crc_hex
            try:
                self.serial.write(use_IO_mode_command)
                print(f'Sent use IO mode true command to the gripper: {use_IO_mode_command.hex()}')
                response = self.serial.read(8)
                if len(response) < 8:
                    print(f'Received incomplete response: {response.hex()}')
                else:
                    print(f'Received response: {response.hex()}')
                if response == use_IO_mode_command:
                    print('Gripper IO mode set successfully.')
                else:
                    print('Unexpected response during gripper use IO mode set.')
            except serial.SerialException as e:
                print(f'Error sending use IO mode command: {str(e)}')
        
        elif bool == False:
            base_command = b'\x01\x06\x04\x02\x00\x00'
            crc = self.calculate_crc(base_command)
            crc_hex = struct.pack('<H', crc)
            use_IO_mode_command = base_command + crc_hex
            try:
                self.serial.write(use_IO_mode_command)
                print(f'Sent use IO mode false command to the gripper: {use_IO_mode_command.hex()}')
                response = self.serial.read(8)
                if len(response) < 8:
                    print(f'Received incomplete response: {response.hex()}')
                else:
                    print(f'Received response: {response.hex()}')
                if response == use_IO_mode_command:
                    print('Gripper use IO mode set false successfully.')
                else:
                    print('Unexpected response during gripper use IO mode set.')
            except serial.SerialException as e:
                print(f'Error sending set use IO mode command: {str(e)}')

        else:
            print(f'Wrong input. Please request True or False')

    def read_IO_mode_state(self):
        base_command = b'\x01\x03\x04\x02'
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        read_IO_mode_state_command = base_command + crc_hex
        try:
            self.serial.write(read_IO_mode_state_command)
            print(f'Check IO mode state.')
            response = self.serial.read(8)
            if len(response) < 6:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response[3:5] == b'\x00\x01':
                print('IO mode ON.')
            elif response[3:5] == b'\x00\x00':
                print('IO mode OFF.')
            else:
                print(f'Unexpected IO mode state: {response[3:5].hex()}')
        except serial.SerialException as e:
            print(f'Error sending IO mode state command: {str(e)}')

    def read_current_position(self):
        read_current_position_command = b'\x01\x03\x02\x02\x00\x01\x24\x72'
        try:
            self.serial.write(read_current_position_command)
            print(f'Read real time position.')
            response = self.serial.read(8)
            if len(response) < 6:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            position = int.from_bytes(response[3:5], byteorder='big')
            print(f'Current position (decimal): {position}‰ (thousand ratio)')
        except serial.SerialException as e:
            print(f'Error sending read current position command: {str(e)}')

    def read_current_speed(self):
        read_current_speed_command = b'\x01\x03\x01\x04\x00\x01\xC4\x37'
        try:
            self.serial.write(read_current_speed_command)
            print(f'Read current speed set.')
            response = self.serial.read(8)
            if len(response) < 6:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            speed = int.from_bytes(response[3:5], byteorder='big')
            print(f'Current speed set (decimal): {speed}%')
        except serial.SerialException as e:
            print(f'Error sending read current speed command: {str(e)}')

    def read_power_value(self):
        read_power_value_command = b'\x01\x03\x01\x01\x00\x01\xD4\x36'
        try:
            self.serial.write(read_power_value_command)
            print(f'Read current power value set.')
            response = self.serial.read(8)
            if len(response) < 6:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            power = int.from_bytes(response[3:5], byteorder='big')
            print(f'Current power value set (decimal): {power}%')
        except serial.SerialException as e:
            print(f'Error sending read power value command: {str(e)}')

    def read_device_id(self):
        base_command = b'\x01\x03\x03\x02'
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        read_device_id_command = base_command + crc_hex
        try:
            self.serial.write(read_device_id_command)
            print(f'Read current device modbus ID.')
            response = self.serial.read(8)
            if len(response) < 6:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            ID = int.from_bytes(response[3:5], byteorder='big')
            print(f'Current device modbus ID : {ID}')
        except serial.SerialException as e:
            print(f'Error sending read device id command: {str(e)}')

    def gripper_open(self):
        pos = 1000
        position_hex = struct.pack('>H', pos)
        base_command = b'\x01\x06\x01\x03' + position_hex
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        gripper_open_command = base_command + crc_hex
        try:
            self.serial.write(gripper_open_command)
            print(f'Sent open command to the gripper: {gripper_open_command.hex()}')
            response = self.serial.read(8)
            if len(response) < 8:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response == gripper_open_command:
                print('Gripper open successfully.')
            else:
                print('Unexpected response during gripper open.')
        except serial.SerialException as e:
            print(f'Error sending open command: {str(e)}')

    def gripper_close(self):
        pos = 0
        position_hex = struct.pack('>H', pos)
        base_command = b'\x01\x06\x01\x03' + position_hex
        crc = self.calculate_crc(base_command)
        crc_hex = struct.pack('<H', crc)
        gripper_close_command = base_command + crc_hex
        try:
            self.serial.write(gripper_close_command)
            print(f'Sent close command to the gripper: {gripper_close_command.hex()}')
            response = self.serial.read(8)
            if len(response) < 8:
                print(f'Received incomplete response: {response.hex()}')
            else:
                print(f'Received response: {response.hex()}')
            if response == gripper_close_command:
                print('Gripper close successfully.')
            else:
                print('Unexpected response during gripper close.')
        except serial.SerialException as e:
            print(f'Error sending close command: {str(e)}')

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