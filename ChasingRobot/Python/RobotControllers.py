from random import seed
from typing import List, Any
from collections import Counter
from matplotlib import pyplot, use

from uncertainties import ufloat

from robot import Robot, Battery, Mobile


class BaseLine:
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float):
        self.robot = robot
        self.speed = 0.0
        self.max_acceleration = 5
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.target = None
        self.error = [0.0, 0.0, 0.0]

    def get_error(self, target_distance: float):
        distance = self.robot.get_distance(self.target)
        return distance - target_distance

    def compute_target_speed(self, target_distance: float, dt: float) -> float:
        error = self.get_error(target_distance)
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = error
        derivative = (error - self.error[1]) / dt
        integral = (error+self.error[1]+self.error[2]) * dt
        return self.Kp * error + self.Ki * integral + self.Kd * derivative

    def move_robot(self, target: Mobile, target_distance: float, dt: float):
        speed = self.compute_target_speed(target_distance, dt)
        speed = max(min(self.robot.max_speed, speed), 0)
        delta_speed = speed-self.speed
        if delta_speed != 0:
            sign = delta_speed/abs(delta_speed)
            if abs(delta_speed) < self.max_acceleration*dt:
                self.speed = speed
            else:
                self.speed += sign * self.max_acceleration*dt
        self.robot.move(self.speed, dt)


class RobustController (BaseLine):
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float):
        super().__init__(robot, kp, ki, kd)

    def get_error(self, target_distance: float):
        distance = self.robot.get_distance(self.target)
        return distance - target_distance - Mobile.sensor_accuracy * 10  # margin with initial distance


class ProbabilisticController (BaseLine):
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float, risk: float):
        super().__init__(robot, kp, ki, kd)
        self.confidence = self.confidence_interval[risk]

    confidence_interval = {.90: 1.64, .99: 2.67, .999: 3.29, .9999: 3.89, .99999: 4.41}

    def get_error(self, target_distance: float) -> ufloat:
        # distance = ufloat(self.robot.get_distance(target), 2.0)
        distance = self.robot.get_distance(self.target)
        # print(self.__class__.__name__+": "+str(distance))
        return ufloat(distance, abs((1+0.25*distance)*Mobile.sensor_accuracy))


class PXNinesController (ProbabilisticController):
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float, risk: float):
        super().__init__(robot, kp, ki, kd, risk)

    def get_error(self, target_distance: float) -> float:
        distance = super().get_error(target_distance)
        return distance.nominal_value - distance.std_dev * self.confidence - target_distance  # p>risk


class P3NinesController (PXNinesController):
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float):
        super().__init__(robot, kp, ki, kd, .999)


class P5NinesController (PXNinesController):
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float):
        super().__init__(robot, kp, ki, kd, .99999)


class StochasticController (ProbabilisticController):
    def __init__(self, robot: Robot, kp: float, ki: float, kd: float):
        super().__init__(robot, kp, ki, kd, .999)

    # def compute_target_speed(self, target_distance: float, dt: float):
    #     speed = super().compute_target_speed(target_distance, dt)
    #     return speed.nominal_value - speed.std_dev * 4
    def get_error(self, target_distance: float) -> float:
        distance = self.robot.get_distance(self.target)
        udistance = ufloat(distance, (1+0.25*distance)*self.robot.sensor_accuracy)  # ojo: with distance
        return (udistance - target_distance).nominal_value - self.robot.sensor_accuracy


def setup_controller(target: Mobile, * controllers):
    result: list[BaseLine] = []
    for controller in controllers:
        robot = Robot(controller.__name__, 0.0)
        rc = controller(robot, 1.60, 0.75, 0.51)
        rc.target = target
        result.append(rc)
    return result


def run(run_seed: int, time: float, dt: float):
    seed(run_seed)
    result_average = dict()
    result_min = dict()
    target = Mobile('Target', 10.0)
    controllers = setup_controller(target, BaseLine, RobustController, P3NinesController, P5NinesController) # , StochasticController)
    for t in range(1, int(time/dt)):
        #print(t)
        target.move(2, dt)
        for c in controllers:
            c.move_robot(target, 1, dt)
            c.robot.update_distance(target)

    print('--------')
    for c in controllers:
        robot = c.robot
        # result[robot.name] = (robot.get_average_distance(), robot.get_min_distance())
        result_average[robot.name] = robot.get_average_distance()
        result_min[robot.name] = robot.get_min_distance()

        print(robot.name+' distance to target, average='+str(robot.get_average_distance())+' Min='+str(robot.get_min_distance()))

    return result_average, result_min


def multiple_run(run_number: int, time: float, dt: float):
    def dict_divide(d: dict, num: int):
        for k in d.keys():
            d[k] = d[k] / num

    def dict_min(d1: dict, d2:dict):
        for k in d1.keys():
            d1[k] = min(d1[k], d2[k])

    total_averages,  total_mins = run(1, time, dt)
    # min_mins = total_mins.copy()
    for r in range(1, run_number):
        averages, mins = run(42*r, time, dt)
        total_averages = dict(Counter(total_averages)+Counter(averages))
        total_mins = dict(Counter(total_mins)+Counter(mins))
        # dict_min(min_mins, mins)
    dict_divide(total_averages, run_number)
    dict_divide(total_mins, run_number)
    return total_averages, total_mins # , min_mins


if __name__ == '__main__':

    x_axis = list()
    average_results = dict()
    min_results = dict()
    Limit = list()
    for p in range(0, 51):
        Mobile.sensor_accuracy = p / 1000.0
        x_axis.append(Mobile.sensor_accuracy)
        averages, mins = multiple_run(30, 30, 0.2)
        for k in averages.keys():
            if p == 0:
                average_results[k] = list()
                min_results[k] = list()
            average_results[k].append(averages[k])
            min_results[k].append(mins[k])
        Limit.append(1.0)

    pyplot.xlabel('Sensor Precision (m)')
    pyplot.ylabel('Distance (m)')
    pyplot.plot(x_axis, Limit, 'k', label='Limit')
    colors = ['r','b','c','g','y']
    i = 0
    for k in average_results.keys():
        pyplot.plot(x_axis,average_results[k],colors[i]+'--',label=k+' (Aver.)')
        pyplot.plot(x_axis,min_results[k],colors[i],label=k+' (Min.)')
        i += 1
    pyplot.legend(loc='upper center', shadow=True)
    pyplot.show()


