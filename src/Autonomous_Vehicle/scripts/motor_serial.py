#!/usr/bin/env python3

import threading
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray


class MotorSerial(Node):

    def __init__(self):
        super().__init__('motor_serial')

        # exclusive=True: a second copy of this node (e.g. left over from a
        # launch that died) fails here instead of silently splitting the
        # Arduino's encoder messages with this one.
        try:
            self.serial_port = serial.Serial(
                '/dev/ttyACM0',
                115200,
                timeout=0.05,
                exclusive=True
            )
        except serial.SerialException as e:
            self.get_logger().fatal(
                f"Cannot open /dev/ttyACM0 exclusively ({e}). Another "
                "motor_serial.py may still be running: "
                "pgrep -af motor_serial.py"
            )
            raise

        # Opening the Uno serial port resets the Arduino.
        time.sleep(2)
        self.serial_port.reset_input_buffer()

        self.get_logger().info(
            "Connected to Arduino on /dev/ttyACM0"
        )

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_cmd_time = time.monotonic()

        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.encoder_pub = self.create_publisher(
            Int64MultiArray,
            '/encoder_counts',
            10
        )

        # Send commands every 50 ms to maintain the Arduino watchdog.
        self.timer = self.create_timer(
            0.05,
            self.send_command
        )

        self.reader = threading.Thread(
            target=self.read_serial,
            daemon=True
        )
        self.reader.start()

        self.get_logger().info("Motor Serial Node Ready")

    def cmd_callback(self, msg):

        with self.lock:
            self.linear_velocity = float(msg.linear.x)
            self.angular_velocity = float(msg.angular.z)
            self.last_cmd_time = time.monotonic()

    def send_command(self):

        with self.lock:
            linear = self.linear_velocity
            angular = self.angular_velocity
            last_cmd = self.last_cmd_time

        # Stop if the command publisher becomes inactive.
        if time.monotonic() - last_cmd > 0.20:
            linear = 0.0
            angular = 0.0

        command = f"V,{linear:.3f},{angular:.3f}\n"

        try:
            self.serial_port.write(command.encode())

        except serial.SerialException as e:
            self.get_logger().error(
                f"Serial Write Error: {e}"
            )

    def read_serial(self):

        while rclpy.ok() and not self.stop_event.is_set():

            try:
                line = (
                    self.serial_port.readline()
                    .decode(errors='ignore')
                    .strip()
                )

                if not line:
                    continue

                parts = line.split(',')

                # Accept only Arduino encoder messages.
                if len(parts) != 3 or parts[0] != 'E':
                    continue

                left = int(parts[1])
                right = int(parts[2])

                msg = Int64MultiArray()
                msg.data = [left, right]

                self.encoder_pub.publish(msg)

            except (ValueError, serial.SerialException) as e:
                self.get_logger().warning(
                    f"Serial Read Error: {e}"
                )

    def shutdown_bridge(self):

        self.stop_event.set()

        try:
            self.serial_port.write(b'V,0.000,0.000\n')
            self.serial_port.flush()
            time.sleep(0.05)
        except serial.SerialException:
            pass

        if self.reader.is_alive():
            self.reader.join(timeout=0.2)

        if self.serial_port.is_open:
            self.serial_port.close()


def main(args=None):

    rclpy.init(args=args)

    node = MotorSerial()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown_bridge()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
