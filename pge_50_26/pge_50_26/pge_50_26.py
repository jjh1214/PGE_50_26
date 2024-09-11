import sys
import os
import rclpy
from rclpy.node import Node
import time

os.chdir(os.path.expanduser('~'))
sys.path.append("./dh_gripper_ws/src/PGE_50_26/pge_50_26")  ## get import pass: pge5026_interface.py
from pge5026_interface import PGE5026Interface

class PGE5026Controller(Node):
    def __init__(self):
        super().__init__('pge_50_26_node')

        self.gripper = PGE5026Interface()
        self.gripper.connect()

        time.sleep(5)

        self.gripper.initialize_gripper()

        time.sleep(5)

        self.gripper.initialization_state()

        time.sleep(5)

        self.gripper.set_speed_gripper(100)

        time.sleep(5)

        self.gripper.set_power_value_gripper(100)

        time.sleep(5)

        self.gripper.set_position_gripper(500)

        time.sleep(5)

        self.gripper.read_current_position()

        time.sleep(5)

        self.gripper.read_current_speed()

        time.sleep(5)

        self.gripper.read_power_value()

        time.sleep(5)

        self.gripper.gripper_open()

        time.sleep(5)

        self.gripper.gripper_close()

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