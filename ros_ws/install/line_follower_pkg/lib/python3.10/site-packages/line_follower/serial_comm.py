#!/usr/bin/env python3

"""

serial_comm_node.py

-------------------

ROS2 node for Bluetooth serial communication with the ATmega32 robot.


Serial → ROS topic mapping

  E:120,118\n  →  /encoder_raw   (std_msgs/String)

  S:10101\n    →  /ir_raw        (std_msgs/String)


ROS topic → Serial

  /motor_cmd   (std_msgs/String)  →  written verbatim to the port

"""


import threading

import time


import rclpy

from rclpy.node import Node

import serial

from std_msgs.msg import String


# ── tuneable constants ────────────────────────────────────────────────────────

SERIAL_PORT       = '/dev/rfcomm0'

BAUD_RATE         = 9600

CONNECT_INTERVAL  = 2.0

READ_INTERVAL     = 0.05

BT_WARMUP_DELAY   = 2

CMD_DELAY         = 0.05   # seconds between consecutive commands to the AVR

# ─────────────────────────────────────────────────────────────────────────────



class SerialReader(Node):


    def __init__(self):

        super().__init__('serial_reader_node')


        self.port      = SERIAL_PORT

        self.baud      = BAUD_RATE

        self.ser       = None

        self.connected = False


        self._serial_lock = threading.Lock()


        # ── publishers ────────────────────────────────────────────────────

        self.ir_publisher  = self.create_publisher(String, '/ir_raw',      10)

        self.enc_publisher = self.create_publisher(String, '/encoder_raw', 10)


        # ── subscriber (ROS → Serial) ─────────────────────────────────────

        self.subscription = self.create_subscription(

            String,

            '/motor_cmd',

            self._send_motor_cmd_callback,

            10,

        )


        self.get_logger().info(f'Waiting for device on {self.port}...')


        self.connect_timer = self.create_timer(CONNECT_INTERVAL, self._try_connect)

        self.read_timer    = self.create_timer(READ_INTERVAL,    self._read_serial)


    # ── connection ────────────────────────────────────────────────────────────

    def _try_connect(self):

        if self.connected:

            return


        try:

            if self.ser is None:

                self.ser = serial.Serial(self.port, self.baud, timeout=1)

                time.sleep(BT_WARMUP_DELAY)

                self.ser.reset_input_buffer()

                self.get_logger().info('Port opened — sending PING...')

            else:

                self.get_logger().info('Retrying PING...')


            self.ser.write(b'PING\n')

            self.ser.flush()


            response = self.ser.readline().decode('utf-8', errors='ignore').rstrip()


            if 'PONG' in response:

                self.connected = True

                self.get_logger().info(f'✅ Device connected — response: {response}')

            else:

                self.get_logger().warn(

                    f'No PONG yet (got: "{response}") — retrying in {CONNECT_INTERVAL}s...'

                )


        except serial.SerialException as e:

            self.get_logger().warn(f'Cannot open port: {e} — retrying...')

            self._close_port()

        except Exception as e:

            self.get_logger().warn(f'Unexpected error during connect: {e}')

            self._close_port()


    # ── motor command ─────────────────────────────────────────────────────────

    def _send_motor_cmd_callback(self, msg: String):

        if not self.connected or self.ser is None:

            self.get_logger().warn('Cannot send command — robot not connected yet!')

            return


        command = msg.data if msg.data.endswith('\n') else msg.data + '\n'


        with self._serial_lock:

            if self.ser is None:

                return

            try:

                self.ser.write(command.encode('utf-8'))

                self.ser.flush()

                time.sleep(CMD_DELAY)  # give AVR time to process before next command

                self.get_logger().info(f'Sent to Robot: {msg.data.strip()}')

            except Exception as e:

                self.get_logger().error(f'Failed to send command: {e}')


    # ── serial reader ─────────────────────────────────────────────────────────

    def _read_serial(self):

        if not self.connected or self.ser is None:

            return


        with self._serial_lock:

            if self.ser is None:

                return

            try:

                while self.ser.in_waiting > 0:

                    raw  = self.ser.readline()

                    line = raw.decode('utf-8', errors='ignore').rstrip()


                    if not line:

                        continue


                    if line.startswith('E:'):

                        payload = line[2:]

                        parts   = payload.split(',')

                        if len(parts) == 2 and all(p.lstrip('-').isdigit() for p in parts):

                            msg      = String()

                            msg.data = line

                            self.enc_publisher.publish(msg)

                            self.get_logger().debug(f'Encoder: {line}')

                        else:

                            self.get_logger().warn(f'Malformed encoder packet: "{line}"')


                    elif line.startswith('S:') and len(line) == 7:

                        msg      = String()

                        msg.data = line

                        self.ir_publisher.publish(msg)

                        self.get_logger().debug(f'IR: {line}')


                    else:

                        self.get_logger().info(f'Serial RX: {line}')


            except serial.SerialException as e:

                self.get_logger().error(f'Connection lost: {e}')

                self._close_port()

            except Exception as e:

                self.get_logger().error(f'Error reading serial: {e}')


    # ── helpers ───────────────────────────────────────────────────────────────

    def _close_port(self):

        self.connected = False

        if self.ser is not None:

            try:

                self.ser.close()

            except Exception:

                pass

            self.ser = None

        self.get_logger().warn(f'Port closed — will retry in {CONNECT_INTERVAL}s...')


    # ── lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self):

        with self._serial_lock:

            if self.ser is not None and self.ser.is_open:

                self.ser.close()

                self.get_logger().info('Serial port closed.')

        super().destroy_node()



# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):

    rclpy.init(args=args)

    node = SerialReader()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()
