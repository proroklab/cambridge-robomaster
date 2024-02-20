import time
import rclpy

import Jetson.GPIO as GPIO
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from functools import partial
import socket

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from emergency_stop_msgs.srv import EmergencyStop
from sensor_msgs.msg import BatteryState
from robomaster_msgs.msg import WheelSpeed
from enum import Enum
from .display_util import show_stats, show_battery, show_network


class Button:
    class State(Enum):
        RELEASED = 0
        DEBOUNCE = 1
        SHORT = 2
        LONG = 3

    def __init__(self, pin):
        self.gpio_pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN)

        self.prev_state = Button.State.RELEASED
        self.state = Button.State.RELEASED
        self.last_state_transition = time.perf_counter()

        self.transition_callbacks = {}

    def register_transition_callback(self, state_a, state_b, callback):
        transition = (state_a, state_b)
        if transition not in self.transition_callbacks:
            self.transition_callbacks[transition] = []
        self.transition_callbacks[transition].append(callback)

    def get_state(self):
        return self.state

    def run(self):
        if self.state == Button.State.RELEASED:
            if self.is_pressed():
                self.state = Button.State.DEBOUNCE
                self.last_state_transition = time.perf_counter()
        elif self.state == Button.State.DEBOUNCE:
            if time.perf_counter() - self.last_state_transition > 0.01:
                if self.is_pressed():
                    self.state = Button.State.SHORT
                else:
                    self.state = Button.State.RELEASED
        elif self.state == Button.State.SHORT:
            if not self.is_pressed():
                self.state = Button.State.RELEASED
            elif time.perf_counter() - self.last_state_transition > 1.0:
                self.state = Button.State.LONG
        elif self.state == Button.State.LONG:
            if not self.is_pressed():
                self.state = Button.State.RELEASED

        transition = (self.prev_state, self.state)
        if transition in self.transition_callbacks:
            for callback in self.transition_callbacks[transition]:
                callback()
        self.prev_state = self.state

    def is_pressed(self):
        return not bool(GPIO.input(self.gpio_pin))


class EmergencyStopClient:
    class Status(Enum):
        OPERATIONAL = 0
        REQUESTED = 1
        NOT_READY = 2
        STOPPED = 3

    future = None
    future_timestamp = None
    last_request_stop = None

    def __init__(self, client, stop):
        self.client = client
        self.send_request(stop)

    def update(self):
        if not self.client.service_is_ready():
            self.cancel_request()
            self.status = self.Status.NOT_READY
        elif (
            self.status == self.Status.NOT_READY and self.last_request_stop is not None
        ):
            self.send_request(self.last_request_stop)

        if self.future is not None and self.future.done():
            if self.future.result().success:
                if self.last_request_stop:
                    self.status = self.Status.STOPPED
                else:
                    self.status = self.Status.OPERATIONAL
            else:
                self.status = self.Status.NOT_READY
            self.future = None
            self.future_timestamp = None

    def cancel_request(self):
        if self.future is not None:
            self.future.cancel()
            self.future_timestamp = None

    def send_request(self, stop):
        self.cancel_request()
        self.last_request_stop = stop

        req = EmergencyStop.Request(stop=stop)
        self.future_timestamp = time.time()
        self.future = self.client.call_async(req)
        self.status = self.Status.REQUESTED


class Display:
    def __init__(self):
        self.display = ssd1306(i2c(port=7, address=0x3C))
        self.disp_idx = 0
        self.battery_percentage = 0.0
        self.top_status = ""

        self.display_funcs = [
            show_stats,
            partial(
                lambda draw: show_battery(
                    self.display.bounding_box, self.battery_percentage, draw
                )
            ),
            partial(show_network, "wlan0"),
            partial(show_network, "wlan1"),
            partial(show_network, "eth0"),
        ]

    def set_battery_percentage(self, battery_percentage):
        self.battery_percentage = battery_percentage

    def set_top_status(self, msg):
        self.top_status = msg

    def show_top_bar(self, draw):
        _, _, w, h = self.display.bounding_box
        hostname = socket.gethostname()
        draw.text((0, 0), hostname, fill="white")
        draw.text((80, 0), self.top_status, fill="white")
        draw.text(
            (110, 0), f"{self.disp_idx + 1}/{len(self.display_funcs)}", fill="white"
        )
        draw.line((0, 12, w, 12), fill="white")

    def update(self):
        with canvas(self.display) as draw:
            self.show_top_bar(draw)
            self.display_funcs[self.disp_idx](draw)

    def next_page(self):
        self.disp_idx = (self.disp_idx + 1) % len(self.display_funcs)


class UserInterface(Node):
    class RunState(Enum):
        RUNNING = 0
        IDLE = 1
        STOPPED = 2

    def __init__(self):
        super().__init__("user_interface")
        self.is_emergency = False
        self.run_state = UserInterface.RunState.IDLE
        self.time_last_vel = time.perf_counter()

        self.emergency_stop_client = EmergencyStopClient(
            self.create_client(EmergencyStop, "emergency_stop"), self.is_emergency
        )

        self.create_subscription(
            BatteryState,
            "battery_state",
            self.battery_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.create_subscription(
            WheelSpeed,
            "cmd_wheels",
            self.wheelspeed_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.display = Display()
        self.btn = Button(4)

        self.timer_run_btn = self.create_timer(1 / 20, self.btn.run)
        self.timer_update_display_status = self.create_timer(2.0, self.loop_display)

        self.btn.register_transition_callback(
            Button.State.SHORT, Button.State.RELEASED, self.short_press_release
        )
        self.btn.register_transition_callback(
            Button.State.LONG, Button.State.RELEASED, self.long_press_release
        )
        self.btn.register_transition_callback(
            Button.State.SHORT, Button.State.LONG, self.long_press
        )

    def battery_callback(self, msg):
        self.display.set_battery_percentage(msg.percentage)

    def wheelspeed_callback(self, msg):
        self.time_last_vel = time.perf_counter()

    def short_press_release(self):
        if self.run_state == UserInterface.RunState.RUNNING:
            self.emergency_stop_client.send_request(True)
            self.emergency_stop_client.update()

            self.run_state = UserInterface.RunState.STOPPED

            self.update_display()
        else:
            self.skip_page()

    def long_press_release(self):
        if self.run_state == UserInterface.RunState.IDLE:
            self.skip_page()

    def long_press(self):
        if self.run_state in [
            UserInterface.RunState.RUNNING,
            UserInterface.RunState.STOPPED,
        ]:
            self.emergency_stop_client.send_request(False)
            self.emergency_stop_client.update()

            self.run_state = UserInterface.RunState.IDLE

            self.update_display()

    def skip_page(self):
        self.display.next_page()
        self.display.update()
        self.timer_update_display_status.reset()

    def update_display(self):
        top_str = {
            UserInterface.RunState.STOPPED: "STP",
            UserInterface.RunState.RUNNING: "RUN",
            UserInterface.RunState.IDLE: "IDL",
        }[self.run_state]
        self.display.set_top_status(top_str)

        if self.run_state != UserInterface.RunState.STOPPED:
            if time.perf_counter() - self.time_last_vel > 1.0:  # No vel cmd for 1 sec
                self.run_state = UserInterface.RunState.IDLE
            else:
                self.run_state = UserInterface.RunState.RUNNING

        self.display.update()

    def loop_display(self):
        if self.btn.get_state() == Button.State.RELEASED:
            self.display.next_page()
        self.update_display()


def main(args=None):
    rclpy.init(args=args)
    publisher = UserInterface()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
