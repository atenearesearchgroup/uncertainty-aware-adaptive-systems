from math import pi
from random import gauss, seed


class Battery:
    def __init__(self, capacity: float, initial_charge: int = 100):
        self.capacity = capacity
        self.nominal_voltage = 12
        self.energy = self.capacity * initial_charge / 100

    def get_voltage(self):
        ratio = self.energy/self.capacity
        if ratio > 0.90:
            return self.nominal_voltage * 1.05 * ratio
        if ratio > 0.20:
            return self.nominal_voltage * 0.945
        if ratio > 0.10:
            return self.nominal_voltage * (ratio+0.8)
        return self.nominal_voltage * ratio

    def discharge(self, power: float):
        self.energy -= power

    def charge(self, percent: int):
        self.energy = self.capacity * percent / 100


class Mobile:
    def __init__(self, name: str, position: float = 0.0, max_speed: int = 10):
        self._position = position
        self.name = name
        self.max_speed = abs(max_speed)

    sensor_accuracy = 0.01

    def get_position(self) -> float:  # return a blurred value of the current position
        return self._position + gauss(0.0, self.sensor_accuracy)

    def get_real_distance(self, other: __init__) -> float:
        return other._position - self._position

    def get_distance(self, other: __init__) -> float:
        real_distance = self.get_real_distance(other)
        return gauss(real_distance, self.sensor_accuracy*(1+0.25*real_distance))

    def move(self, speed: float, time: float = 1.0):
        if speed > self.max_speed:
            speed = self.max_speed
        if speed < - self.max_speed:
            speed = - self.max_speed
        # use gauss to simulate friction etc. on the actual movement of the robot
        self._position += speed * time # * gauss(1.0, 0.01)
        #  print(self.name + ': Exact Position=' + str(self._position) + ' speed=' + str(speed))


class Robot (Mobile):
    def __init__(self, name: str, position: float = 0.0, max_speed: int = 10, capacity: float = 1000):
        super().__init__(name, position, max_speed)
        self.battery = Battery(capacity)
        self.average_real_distance = 0.0
        self.min_real_distance = 999999999999999999
        self.count = 0

    def move(self, speed: float, time: float):
        super().move(speed, time)
        self.battery.discharge(speed*speed*time * gauss(1.0, 0.1))
        # print(' charge='+str(self.battery.energy))

    def update_distance(self, other: Mobile):
        self.count += 1
        real_distance = self.get_real_distance(other)
        self.average_real_distance += real_distance
        self.min_real_distance = min(self.min_real_distance, real_distance)

    def get_average_distance(self):
        return self.average_real_distance / self.count

    def get_min_distance(self):
        return self.min_real_distance





