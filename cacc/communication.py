
class  BasicCommStrategy:
    """
    This is the communication strategy that a good actor would use.
    Basically, it communicates the correct information.
    """

    def __init__(self, world):
        self.world = world

    def communicate(self, car):
        self.world.send_msg(car.name, car.accel)

    def __str__(self):
        return "BASIC_COMM"
