
from enum import Enum

from cacc import util
from .util import *
import os

class ControlMode(Enum):
    CACC_CA = 1
    CACC_GC = 2
    ACC = 3


class Car:

    def __init__(self, world, global_conf, car_conf):
        self.name = car_conf['name']
        self.points = [(
            0.00,
            car_conf['position'],
            car_conf['velocity'],
            car_conf['acceleration'],
            ControlMode.CACC_GC
        )]

        self.max_vel = global_conf['max_vel']
        self.max_deccel = global_conf['max_deccel']
        self.length = global_conf['car_length']

        guide_strat = car_conf['guide_strat']
        comm_strat = car_conf['comm_strat']
        sensor = car_conf['sensor']

        self.guide_strat = cls_from_str(guide_strat['class'])(world, **guide_strat)
        self.comm_strat = cls_from_str(comm_strat['class'])(world, **comm_strat)
        self.sensor = cls_from_str(sensor['class'])(world, **sensor)

    def __str__(self):
        time, pos, vel, acc, mode = self.points[-1]
        return "%s p=%.2f, v=%.2f, a=%.2f, mode=%s, length=%.2f, max_deccel=%.2f, %s, %s, %s" % \
            (self.name, pos, vel, acc, mode, self.length, self.max_deccel, self.guide_strat, self.comm_strat, self.sensor)

    def __repr__(self):
        return str(self)

    def update(self, world):
        accel = self.accel
        vel = self.vel + self.accel * world.dt
        pos = self.pos + self.vel * world.dt + .5 * self.accel * world.dt ** 2
        mode = self.mode
        self.points.append((world.time, pos, vel, accel, mode))

    def update_guidance(self, i):
        time, pos, vel, _, _ = self.points[-1]
        new_acc, new_mode = self.guide_strat.compute_guidance(self, i)
        self.points[-1] = (time, pos, vel, new_acc, new_mode)

    def prepare(self):
        self.sensor.orient(self)
        self.guide_strat.orient(self)

    def communicate(self):
        self.comm_strat.communicate(self)

    @property
    def pos(self):
        return self.points[-1][1]

    @property
    def vel(self):
        return self.points[-1][2]

    @property
    def accel(self):
        return self.points[-1][3]

    @property
    def mode(self):
        return self.points[-1][4]

    def get_pos(self, i):
        return self.points[i][1]

    def get_vel(self, i):
        return self.points[i][2]


class World:

    def __init__(self, config):
        self.safe_time_gap = config['simulation']['safe_time_gap']
        self.dt = config['simulation']['time_unit']
        self.cars = [Car(self, config['simulation'], car_conf)
            for car_conf in config['cars']]
        self.messages = dict()
        self.i = 0
        self.time = 0.0


    def update(self):
        self.time += self.dt
        self.i += 1

        for car in self.cars:
            car.update(self)

        for car in self.cars:
            car.communicate()

        for car in self.cars:
            car.update_guidance(self.i)

    def prepare(self):
        for car in self.cars:
            car.prepare()

        for car in self.cars:
            car.communicate()

        for car in self.cars:
            car.update_guidance(0)

    def send_msg(self, src, data):
        self.messages[src] = data

    def recv_msg(self, src):
        return self.messages.get(src, None)

    def __str__(self):
        string = ""
        for car in self.cars:
            string += "t=%.2f: %s\n" % (self.time, car)
        return string


class Simulator:

    def __init__(self, config):
        self.config = config
        self.world = World(config)

    def run(self):
        point_count = self.config['simulation']['trajectory_points']
        print(self.world)
        self.world.prepare()
        for _ in range(point_count-1):
            print(self.world)
            self.world.update()
        print(self.world)

    def output(self):
        for car in self.world.cars:
            self.car_to_file(car)

    def car_to_file(self, car):
        output = '"time":"'
        for p in car.points:
            output += "%3.2f " % p[0]
        output += '","speed":"'
        for p in car.points:
            output += "%4.2f " % p[2]
        output += '","location":"'
        for p in car.points:
            output += "%4.2f " % p[1]
        output += '"\n'
        os.makedirs("output", exist_ok=True)
        with open(os.path.join("output", car.name), 'w') as f:
            f.write(output)
