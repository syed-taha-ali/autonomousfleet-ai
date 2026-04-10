#!/usr/bin/env python3
# encoding: utf-8
"""
STM32 RRC Lite bridge node (ported from HiWonder inspo project).

Fixes applied vs. inspo:
- Renamed the attribute shadowing the callback: ``self.enable_reception``
  (bool) is now ``self._reception_enabled``; the callback is
  ``enable_reception_callback``. The original code had the subscription
  callback replace the boolean attribute, causing the pub thread to
  permanently stop publishing sensor data after the first message.
- Removed the hardcoded ``~/workspace/ros2_ws/src/...`` YAML path. Servo
  offsets now load from a parameter ``servo_config_path`` (defaults to the
  package-share ``config/servo_config.yaml``).
- Uses ``get_package_share_directory('af_hal')`` via ament_index.
"""
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, UInt16
from std_srvs.srv import Trigger

from af_hal.ros_robot_controller_sdk import Board, PacketReportKeyEvents
from af_msgs.msg import (
    BusServoState,
    ButtonState,
    BuzzerState,
    LedState,
    MotorsState,
    OLEDState,
    PWMServoState,
    RGBStates,
    Sbus,
    ServosPosition,
    SetBusServoState,
    SetPWMServoState,
)
from af_msgs.srv import GetBusServoState, GetPWMServoState


class RosRobotController(Node):
    gravity = 9.80665

    def __init__(self, name):
        super().__init__(name)
        self.board = Board()
        self.board.enable_reception()
        self._reception_enabled = True
        self.running = True

        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('init_finish', False)
        self.declare_parameter(
            'servo_config_path',
            os.path.join(
                get_package_share_directory('af_hal'),
                'config',
                'servo_config.yaml',
            ),
        )
        self.IMU_FRAME = self.get_parameter('imu_frame').value

        self.imu_pub = self.create_publisher(Imu, '~/imu_raw', 1)
        self.joy_pub = self.create_publisher(Joy, '~/joy', 1)
        self.sbus_pub = self.create_publisher(Sbus, '~/sbus', 1)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)

        self.create_subscription(LedState, '~/set_led', self.set_led_state, 5)
        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_state, 5)
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_state, 5)
        self.create_subscription(MotorsState, '~/set_motor', self.set_motor_state, 10)
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception_callback, 1)
        self.create_subscription(SetBusServoState, '~/bus_servo/set_state', self.set_bus_servo_state, 10)
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)
        self.create_subscription(SetPWMServoState, '~/pwm_servo/set_state', self.set_pwm_servo_state, 10)
        self.create_subscription(RGBStates, '~/set_rgb', self.set_rgb_states, 10)

        self.create_service(GetBusServoState, '~/bus_servo/get_state', self.get_bus_servo_state)
        self.create_service(GetPWMServoState, '~/pwm_servo/get_state', self.get_pwm_servo_state)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        self.load_servo_offsets()
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.clock = self.get_clock()
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.get_logger().info('ros_robot_controller started')

    # ------------------------------------------------------------------
    # Configuration

    def load_servo_offsets(self):
        config_path = self.get_parameter('servo_config_path').value
        if not config_path or not os.path.exists(config_path):
            self.get_logger().warn(
                f"servo_config_path '{config_path}' not found; skipping offset load"
            )
            return
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            if not isinstance(config, dict):
                self.get_logger().error(
                    f"Servo config {config_path} is not a mapping; skipping"
                )
                return
            for servo_id in range(1, 5):
                offset = config.get(servo_id, 0)
                try:
                    self.board.pwm_servo_set_offset(servo_id, offset)
                    self.get_logger().info(
                        f"servo {servo_id} offset -> {offset}"
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"failed to set offset for servo {servo_id}: {exc}"
                    )
        except yaml.YAMLError as exc:
            self.get_logger().error(f"YAML parse error in {config_path}: {exc}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"error reading {config_path}: {exc}")

    # ------------------------------------------------------------------
    # Lifecycle

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pub_callback(self):
        while self.running and rclpy.ok():
            if self._reception_enabled:
                self.pub_button_data(self.button_pub)
                self.pub_joy_data(self.joy_pub)
                self.pub_imu_data(self.imu_pub)
                self.pub_sbus_data(self.sbus_pub)
                self.pub_battery_data(self.battery_pub)
            time.sleep(0.02)

    def enable_reception_callback(self, msg):
        self.get_logger().info(f'enable_reception <- {msg.data}')
        self._reception_enabled = bool(msg.data)
        self.board.enable_reception(msg.data)

    # ------------------------------------------------------------------
    # Actuator subscribers

    def set_led_state(self, msg):
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    def set_buzzer_state(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def set_rgb_states(self, msg):
        pixels = [(s.index, s.red, s.green, s.blue) for s in msg.states]
        self.board.set_rgb(pixels)

    def set_motor_state(self, msg):
        data = [[i.id, i.rps] for i in msg.data]
        self.board.set_motor_speed(data)

    def set_oled_state(self, msg):
        self.board.set_oled_text(int(msg.index), msg.text)

    def set_pwm_servo_state(self, msg):
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.append([i.id[0], i.position[0]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])
        if data:
            self.board.pwm_servo_set_position(msg.duration, data)

    def get_pwm_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            states.append(data)
        response.success = True
        response.state = states
        return response

    def set_bus_servo_position(self, msg):
        data = [[i.id, i.position] for i in msg.position]
        if data:
            self.board.bus_servo_set_position(msg.duration, data)

    def set_bus_servo_state(self, msg):
        data = []
        servo_ids_to_stop = []
        for i in msg.state:
            if not (i.present_id and i.present_id[0]):
                continue
            pid = i.present_id[1]
            if i.target_id and i.target_id[0]:
                self.board.bus_servo_set_id(pid, i.target_id[1])
            if i.position and i.position[0]:
                data.append([pid, i.position[1]])
            if i.offset and i.offset[0]:
                self.board.bus_servo_set_offset(pid, i.offset[1])
            if i.position_limit and i.position_limit[0]:
                self.board.bus_servo_set_angle_limit(pid, i.position_limit[1:])
            if i.voltage_limit and i.voltage_limit[0]:
                self.board.bus_servo_set_vin_limit(pid, i.voltage_limit[1:])
            if i.max_temperature_limit and i.max_temperature_limit[0]:
                self.board.bus_servo_set_temp_limit(pid, i.max_temperature_limit[1])
            if i.enable_torque and i.enable_torque[0]:
                self.board.bus_servo_enable_torque(pid, i.enable_torque[1])
            if i.save_offset and i.save_offset[0]:
                self.board.bus_servo_save_offset(pid)
            if i.stop and i.stop[0]:
                servo_ids_to_stop.append(pid)
        if data:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_ids_to_stop:
            self.board.bus_servo_stop(servo_ids_to_stop)

    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None:
                    i.id = state[0]
                    data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            if i.get_voltage:
                state = self.board.bus_servo_read_voltage(i.id)
                if state is not None:
                    data.voltage = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None:
                    data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None:
                    data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None:
                    data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None:
                    data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque(i.id)
                if state is not None:
                    data.enable_torque = state
            states.append(data)
        response.state = states
        response.success = True
        return response

    # ------------------------------------------------------------------
    # Sensor publishers

    def pub_battery_data(self, pub):
        data = self.board.get_battery()
        if data is not None:
            msg = UInt16()
            msg.data = data
            pub.publish(msg)

    def pub_button_data(self, pub):
        data = self.board.get_button()
        if data is None:
            return
        key_id, key_event = data
        state_map = {
            PacketReportKeyEvents.KEY_EVENT_PRESSED: 1,
            PacketReportKeyEvents.KEY_EVENT_LONGPRESS: 2,
            PacketReportKeyEvents.KEY_EVENT_LONGPRESS_REPEAT: 3,
            PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_LP: 4,
            PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_SP: 0,
            PacketReportKeyEvents.KEY_EVENT_CLICK: 5,
            PacketReportKeyEvents.KEY_EVENT_DOUBLE_CLICK: 6,
            PacketReportKeyEvents.KEY_EVENT_TRIPLE_CLICK: 7,
        }
        state = state_map.get(key_event, -1)
        if state == -1:
            self.get_logger().error(f"unhandled button event: {key_event}")
            return
        msg = ButtonState()
        msg.id = key_id
        msg.state = state
        pub.publish(msg)

    def pub_joy_data(self, pub):
        data = self.board.get_gamepad()
        if data is None:
            return
        msg = Joy()
        msg.axes = data[0]
        msg.buttons = data[1]
        msg.header.stamp = self.clock.now().to_msg()
        pub.publish(msg)

    def pub_sbus_data(self, pub):
        data = self.board.get_sbus()
        if data is None:
            return
        msg = Sbus()
        msg.channel = data
        msg.header.stamp = self.clock.now().to_msg()
        pub.publish(msg)

    def pub_imu_data(self, pub):
        data = self.board.get_imu()
        if data is None:
            return
        ax, ay, az, gx, gy, gz = data
        msg = Imu()
        msg.header.frame_id = self.IMU_FRAME
        msg.header.stamp = self.clock.now().to_msg()
        # Raw IMU has no orientation estimate; the filter chain will fill it in.
        msg.orientation.w = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.linear_acceleration.x = ax * self.gravity
        msg.linear_acceleration.y = ay * self.gravity
        msg.linear_acceleration.z = az * self.gravity
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.linear_acceleration_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.004,
        ]
        pub.publish(msg)


def main():
    rclpy.init()
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        except Exception:  # noqa: BLE001
            pass
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
