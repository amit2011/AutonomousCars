import random

class OneAheadSensor:

    def __init__(self, world, **kwargs):
        self.world = world
        self.leader = None
        self.pos_bias = eval("lambda car: " + kwargs.get('pos_bias', "0"))
        self.pos_std = eval("lambda car: " + kwargs.get('pos_std', "0"))
        self.vel_bias = eval("lambda car: " + kwargs.get('vel_bias', "0"))
        self.vel_std = eval("lambda car: " + kwargs.get('vel_std', "0"))

    def orient(self, car):
        """
        find the car that is directly ahead of car
        :param car:
        :return:
        """
        leader = None
        cars = sorted(self.world.cars, key=lambda car: -car.pos)
        for c in cars:
            if car.name == c.name:
                self.leader = leader
                break
            else:
                leader = c

    def sense_pos(self, car, i):
        bias = self.pos_bias(car)
        std = self.pos_std(car)
        return random.gauss(self.leader.get_pos(i-1) + bias, std)

    def sense_vel(self, car, i):
        bias = self.vel_bias(car)
        std = self.vel_std(car)
        return random.gauss(self.leader.get_vel(i-1) + bias, std)

    def sense(self, car, i):
        if self.leader is not None:
            return self.sense_pos(car, i), self.sense_vel(car, i)
        else:
            return None, None

    def __str__(self):
        if self.leader is not None:
            return "ONE_AHEAD(%s)" % (self.leader.name)
        else:
            return "NONE_AHEAD"
