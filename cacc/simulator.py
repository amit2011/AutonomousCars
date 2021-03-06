from enum import Enum
import os
import json

import matplotlib.pyplot as plt
import numpy as np

from cacc import errors
from cacc.util import *


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

    def check_contraints(self):
        for i in range(len(self.cars)-1):
            leader = self.cars[i]
            follower = self.cars[i+1]
            gap = leader.pos - leader.length - follower.pos
            if gap <= 0:
                raise errors.CollisionException(leader=leader, follower=follower, gap=gap)

    def prepare(self):
        for car in self.cars:
            car.prepare()

        for car in self.cars:
            car.communicate()

        for car in self.cars:
            car.update_guidance(0)

        self.cars = sorted(self.cars, key=lambda car: -car.pos)

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
        #print(self.world)
        self.world.prepare()
        for _ in range(point_count-1):
            #print(self.world)
            self.world.update()
            self.world.check_contraints()
        #print(self.world)

    def output(self, directory):
        for car in self.world.cars:
            self.car_to_file(car, directory)
        self.plot_data(directory)
        with open(directory + "config.json", 'w') as fp:
            json.dump(self.config, fp, indent=4)

    def car_to_file(self, car, directory):
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
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, car.name), 'w') as f:
            f.write(output)

    def plot_data(self, directory):
        os.makedirs(directory, exist_ok=True)
        point_count = len(self.world.cars[0].points)
        dt = self.config['simulation']['time_unit']
        x = np.arange(0, point_count * dt, dt)

        plt.figure(dpi=400)
        plt.grid(True)
        plt.title("Position")
        plt.xlabel("secs")
        plt.ylabel("meters")
        for car in self.world.cars:
            positions = [point[1] for point in car.points]
            line, = plt.plot(x, positions, label=car.name)
        plt.legend()
        plt.savefig(os.path.join(directory, "pos.png"))

        plt.figure(dpi=400)
        plt.grid(True)
        plt.title("Velocity")
        plt.xlabel("secs")
        plt.ylabel("m/s")
        for car in self.world.cars:
            positions = [point[2] for point in car.points]
            line, = plt.plot(x, positions, label=car.name)
        plt.legend()
        plt.savefig(os.path.join(directory, "vel.png"))

        plt.figure(dpi=400)
        plt.grid(True)
        plt.title("Acceleration")
        plt.xlabel("secs")
        plt.ylabel("$m/s^2$")
        for car in self.world.cars:
            positions = [point[3] for point in car.points]
            line, = plt.plot(x, positions, label=car.name)
        plt.legend()
        plt.savefig(os.path.join(directory, "accel.png"))
