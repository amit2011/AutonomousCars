import random


class  BasicCommStrategy:
    """
    This is the communication strategy that a good actor would use.
    Basically, it communicates the correct information.
    """

    def __init__(self, world, **kwargs):
        self.world = world

    def communicate(self, car):
        self.world.send_msg(car.name, car.accel)

    def __str__(self):
        return "BASIC_COMM"


class RandomDropCommStrat:
    """
    This is a malicious strategy where communcation packets will randomly
    be dropped, so data will be out of date at the other cars in the platoon
    """

    def __init__(self, world, **kwargs):
        self.world = world
        self.drop_rate = kwargs.get('drop_rate')

    def communicate(self, car):
        if random.random() > self.drop_rate:
            self.world.send_msg(car.name, car.accel)
        else:
            print("%s dropping packet" % (car.name))

    def __str__(self):
        return "DROP_COMM(%.2f)" % self.drop_rate

class LiarCommStrat:
    """
    This is a malicious strategy where a certain percentage of packets
    will randomly lie.
    """

    def __init__(self, world, **kwargs):
        self.world = world
        self.lie_rate = kwargs.get('lie_rate')
        self.lie_value = kwargs.get('lie_value')

    def communicate(self, car):
        if random.random() > self.lie_rate:
            self.world.send_msg(car.name, car.accel)
        else:
            print("%s is lying about it's accel" % (car.name))
            self.world.send_msg(car.name, self.lie_value)

    def __str__(self):
        return "LIAR_COMM(%.2f, %.4f)" % (self.lie_rate, self.lie_value)
