"""Vehicle interface: the app calls set_speed() about 20 times per second."""
import time


class Vehicle:
    def set_speed(self, speed: float):
        """speed is a fraction of full speed, 0..1."""
        raise NotImplementedError

    def stop(self):
        self.set_speed(0.0)


class SimulatedVehicle(Vehicle):
    def __init__(self):
        self.speed = 0.0

    def set_speed(self, speed):
        self.speed = speed


class Esp32Vehicle(Vehicle):
    """Drives the motors through an ESP32 + L293D (firmware in firmware/obstacle_esp32).

    Sends lines like "S0.30". The command is repeated at least every KEEPALIVE seconds,
    because the firmware stops the motors if it hears nothing for 0.5 s (app crashed,
    cable unplugged...).
    """

    KEEPALIVE = 0.2

    def __init__(self, link):
        self.link = link  # esp32_link.Esp32Link
        self.speed = 0.0
        self._last = (None, 0.0)

    def set_speed(self, speed):
        self.speed = speed
        command = f"S{speed:.2f}"
        last_command, last_time = self._last
        now = time.monotonic()
        if command != last_command or now - last_time >= self.KEEPALIVE:
            if self.link.send(command):
                self._last = (command, now)
