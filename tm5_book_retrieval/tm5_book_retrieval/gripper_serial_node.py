import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GripperSerialNode(Node):

    def __init__(self):
        super().__init__('gripper_serial_node')

        self.port = '/dev/ttyACM0' #ttyACM1 / 0
        self.baudrate = 115200

        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1.0
            )
              
            time.sleep(2.0)
            self.serial.reset_input_buffer()

            self.get_logger().info(
                f'Connected to Arduino on {self.port}'
            )

        except serial.SerialException as e:
            self.serial = None

            self.get_logger().error(
                f'Failed to connect to Arduino: {e}'
            )

        self.command_sub = self.create_subscription(
            String,
            '/tm5/gripper_command',
            self.command_callback,
            10
        )

        self.response_pub = self.create_publisher(
            String,
            '/tm5/gripper_response',
            10
        )

    def command_callback(self, msg):

        command = msg.data.strip()

        if self.serial is None:
            self.get_logger().error(
                'Arduino serial connection is not available'
            )
            return

        try:
            self.serial.write((command + '\n').encode())

            self.get_logger().info(
                f'Sent to Arduino: {command}'
            )

            response = self.serial.readline().decode().strip()

            if response:
                self.get_logger().info(
                    f'Arduino: {response}'
                )

                response_msg = String()
                response_msg.data = response
                self.response_pub.publish(response_msg)

        except serial.SerialException as e:

            self.get_logger().error(
                f'Serial communication error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = GripperSerialNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node.serial is not None:
            node.serial.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
