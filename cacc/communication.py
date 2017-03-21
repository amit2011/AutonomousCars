import random

class ConfigurableCommStrat:
    """
    This is a configurable communication stategy which can be configured
    to drop and mutate packets from config.

    Expectations:
    - 'drop_func': An boolean expression that is True if the packet should be
        dropped. (Default: False)
    - 'mutate_func': An expression which returns a float representing the value
        which should be sent as the acceleration of the car. (Default: car.accel)
    """

    def __init__(self, world, **kwargs):
        """
        :param world: The world object
        :param **kwargs: A dictionary containing the arguments for the strat.
        """
        self.world = world
        self.drop_func = eval("lambda car: " + kwargs['drop_func']) \
            if 'drop_func' in kwargs \
            else lambda car: False
        self.mutate_func = eval("lambda car: " + kwargs['mutate_func']) \
            if 'mutate_func' in kwargs \
            else lambda car: car.accel

    def communicate(self, car):
        """
        :param car: The car to base the communication on.
        """
        if not self.drop_func(car):
            val = self.mutate_func(car)
            self.world.send_msg(car.name, val)
        else:
            print("%s is dropping a packet" % (car.name))

    def __str__(self):
        return "CONFIG_COMM"
