import time
import neopixel
import board
import numpy as np
import functools
import rclpy
import RPi.GPIO as GPIO

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from emergency_stop_msgs.srv import EmergencyStop
from enum import Enum


class EmergencyButton:
    def __init__(self, pin):
        self.gpio_pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.last_state = self.is_emergency()

    def is_emergency(self):
        return bool(GPIO.input(self.gpio_pin))

    def has_changed(self):
        current_state = self.is_emergency()
        has_changed = current_state != self.last_state
        self.last_state = current_state
        return has_changed


class AlarmRing:
    def __init__(self, pixels, color=(80, 0, 0), sigma=1.0, rays=[1.0], pos_offset=0.0):
        self.pixels = pixels
        self.color = color
        self.rays = rays
        self.sigma = sigma
        self.pos_offset = pos_offset
        self.reset()

    def get_values_for_pos(self, pos):
        n = len(self.pixels)
        x = np.arange(-n, 2 * n)  # wrap around
        p = (pos + self.pos_offset) % 1.0
        vals = np.exp(-((x - (p * n)) ** 2) / (2 * self.sigma ** 2))
        vals[n : 2 * n] += vals[:n]  # wrap around
        vals[n : 2 * n] += vals[2 * n :]  # wrap around
        return vals[n : 2 * n]

    def reset(self):
        self.pos = 0.0

    def run(self, increment=0.05):
        values = np.sum(
            [self.get_values_for_pos(self.pos + ray) for ray in self.rays], axis=0
        )
        for i in range(len(self.pixels)):
            self.pixels[i] = (np.array(self.color) * values[i]).astype(int)
        self.pos += increment
        completed_cycle = False
        if self.pos >= 1.0:
            completed_cycle = True
            self.pos -= 1.0

        self.pixels.show()

        return completed_cycle


class Pulse:
    def __init__(self, pixels, color=(0, 40, 0)):
        self.pixels = pixels
        self.color = color
        self.reset()

    def reset(self):
        self.increment_fac = 1.0
        self.pos = 0.0

    def run(self, increment=0.05):
        self.pos += increment * self.increment_fac
        if self.pos >= 1.0:
            self.pos = 1.0
            self.increment_fac *= -1.0
        elif self.pos <= 0.0:
            self.pos = 0.0
            self.increment_fac *= -1.0

        self.pixels.fill((np.array(self.color) * self.pos).astype(int))
        self.pixels.show()

        return self.pos == 0.0


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


class EmergencyStopButton(Node):
    class LEDState(Enum):
        BUTTON = 0
        AGENTS = 1

    def __init__(self):
        super().__init__("emergency_stop_button")
        self.emergency_stop_clients = {}
        self.timer_refresh_pose_subs = self.create_timer(
            10.0, self.update_stoppable_agents
        )
        self.timer_run = self.create_timer(1 / 10, self.run)
        self.timer_update_led_status = self.create_timer(1 / 30, self.update_led_status)
        self.timer_status_agents_led = self.create_timer(
            1.0, self.cb_switch_status_button
        )

        self.btn = EmergencyButton(17)

        self.pixels = neopixel.NeoPixel(board.D18, 39, auto_write=False)

        self.ring = AlarmRing(self.pixels, rays=[0.0, 0.5], pos_offset=0.2)
        self.led_cycles = 0
        self.pulse = Pulse(self.pixels, color=(0, 20, 0))
        self.is_emergency = False
        self.led_status_state = self.LEDState.BUTTON

    def update_stoppable_agents(self):
        services = self.get_service_names_and_types()
        for service_name, service_type in services:
            uuid = service_name.strip("/").split("/")[0]
            if (
                service_type == ["emergency_stop_msgs/srv/EmergencyStop"]
                and uuid not in self.emergency_stop_clients.keys()
            ):
                self.emergency_stop_clients[uuid] = EmergencyStopClient(
                    self.create_client(EmergencyStop, service_name), self.is_emergency
                )

    def run(self):
        self.is_emergency = self.btn.is_emergency()

        emergency_status_changed = self.btn.has_changed()
        if emergency_status_changed:
            self.timer_status_agents_led.reset()
            self.ring.reset()
            self.pulse.reset()
            self.led_status_state = self.LEDState.AGENTS

        for client in self.emergency_stop_clients.values():
            if emergency_status_changed:
                client.send_request(self.is_emergency)
            client.update()

    def cb_switch_status_button(self):
        self.led_status_state = self.LEDState.BUTTON
        self.timer_status_agents_led.cancel()

    def update_led_status(self):
        if self.led_status_state == self.LEDState.BUTTON:
            if self.is_emergency:
                self.led_cycles += int(self.ring.run(0.05))
            else:
                self.led_cycles += int(self.pulse.run(0.03))
            if self.led_cycles == 2:
                self.led_cycles = 0
                self.led_status_state = self.LEDState.AGENTS
                self.timer_status_agents_led.reset()

        elif self.led_status_state == self.LEDState.AGENTS:
            self.pixels.fill((0, 0, 0))
            n_clients = len(self.emergency_stop_clients)
            for i, client in enumerate(self.emergency_stop_clients.values()):
                for i_center in [7, 27]:  # , 17]:
                    idx = int(i_center - n_clients / 2 + 1 + i)
                    self.pixels[idx] = {
                        client.Status.OPERATIONAL: (0, 20, 0),
                        client.Status.REQUESTED: (10, 10, 0),
                        client.Status.NOT_READY: (20, 3, 0),
                        client.Status.STOPPED: (20, 0, 0),
                    }[client.status]
            self.pixels.show()


def main(args=None):
    rclpy.init(args=args)
    publisher = EmergencyStopButton()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
