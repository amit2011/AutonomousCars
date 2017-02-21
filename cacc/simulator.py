
from enum import Enum
from .errors import *
from .util import *

class ControlMode(Enum):
    CACC_CA = 1
    CACC_GC = 2
    ACC = 3

class Car:

    def __init__(self, name, init_pos, init_vel, init_accel, max_vel, max_deccel, length):
        self.name = name
        self.points = [(init_pos, init_vel, init_accel, ControlMode.CACC_GC)]
        self.max_vel = max_vel
        self.max_deccel = max_deccel
        self.length = length

    def __str__(self):
        point = self.points[-1]
        return "%s p=%.2f, v=%.2f, a=%.2f, mode=%s, length=%.2f, max_deccel=%.2f" % \
            (self.name, point[0], point[1], point[2], point[3], self.length, self.max_deccel)

    def __repr__(self):
        return str(self)

    @property
    def pos(self):
        return self.points[-1][0]

    @property
    def vel(self):
        return self.points[-1][1]

    @property
    def accel(self):
        return self.points[-1][2]

    def update_constant_speed(self, dt):
        last = self.points[-1]
        # TODO Make sure speed limits aren't exceeded
        accel = last[2]
        vel = last[1]
        pos = last[0] + vel * dt
        self.points.append((pos, vel, accel, ControlMode.CACC_GC))

    def update_speed(self, dt):
        last = self.points[-1]
        accel = last[2]
        vel = min(last[1] + accel * dt, self.max_vel)
        pos = last[0] + last[1]*dt + 0.5 * last[2] * dt ** 2
        self.points.append((pos, vel, accel, ControlMode.CACC_GC))


class Simulator:

    def __init__(self, conf):
        self.config = conf
        self.cars = []
        for car in conf.cars:
            self.cars.append(
                Car(car['name'],
                    car['pos'],
                    car['vel'],
                    car['accel'],
                    conf.sim_conf['max_vel'],
                    conf.sim_conf['max_deccel'],
                    conf.sim_conf['car_length']))

    def run(self):
        dt = self.config.sim_conf['time_unit']
        for car in self.cars:
            print("time=0, %s" % car)
        try:
            leader = self.cars[0]
            for car in self.cars[1:]:
                compute_accel(leader, car, dt)
                leader = car
            for i in range(self.config.sim_conf['trajectory_points']):
                self.cars[0].update_constant_speed(dt)
                print("time=%.2f, %s" % (i * dt, self.cars[0]))
                for car in self.cars[1:]:
                    car.update_speed(dt)
                    print("time=%.2f, %s" % (i * dt, car))
                leader = self.cars[0]
                for car in self.cars[1:]:
                    compute_accel(leader, car, dt)
                    leader = car
        except (CollisionException, NegativeSafegapException, CollisionAvoidanceException) as e:
            print(e)
            exit(1)
