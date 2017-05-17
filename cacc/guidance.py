import time

from cacc import util
from cacc.simulator import ControlMode
from cacc.errors import *

class LeaderGuidanceStrategy:

    def __init__(self, world, **kwargs):
        """
        Needed to override the parameters
        """
        pass

    def orient(self, car):
        pass

    def compute_guidance(self, car, i):
        """
        Computes new acceleration for car.
        :param car:
        :param world:
        :return:
        """
        return car.accel, ControlMode.CACC_GC

    def __str__(self):
        return "LEADER"

class FollowerGuidanceStrategy:
    def __init__(self, world, **kwargs):
        self.world = world
        self.leader = None
        self.leader_accel = 0.0

    def orient(self, car):
        last = None
        cars = sorted(self.world.cars, key=lambda car: -car.pos)
        for c in cars:
            if car.name == c.name:
                self.leader = last
                break
            else:
                last = c

    def listen(self):
        lead_accel = self.world.recv_msg(self.leader.name)
        if lead_accel is not None:
            self.leader_accel = lead_accel
        return self.leader_accel

    def compute_guidance(self, car, i):
        # Use the sensor to get the position and velocity of the car ahead
        lead_pos, lead_vel = car.sensor.sense(car, i)
        # Listen for the message from the car in front, which should contain
        # the acceleration of that car.
        lead_accel = self.listen()

        if lead_pos is None or lead_vel is None:
            raise RuntimeError("Sensor observed None: This shouldn't happen for a follower")

        gap = lead_pos - car.pos - self.leader.length
        safe_gap = max(1.0,
            0.1 * car.vel + car.vel ** 2 / (2.0 * car.max_deccel) \
            - lead_vel ** 2 / (2 * self.leader.max_deccel) + 1.0
        )

        if safe_gap < 0:
            raise NegativeSafegapException(self.leader, car, safe_gap)
        if gap < safe_gap:
            # In actual use the car would:
            # - set follower mode to CACC_CA (collision avoidance mode)
            # - set follower accel to -MAX_DECEL (stop as fast as possible)
            # but we're only interested in whether we're in the right mode
            # at the moment.
            #raise CollisionAvoidanceException(self.leader, car, gap, safe_gap)
            return -car.max_deccel, ControlMode.CACC_CA

        desired_accel = 0.66 * lead_accel \
            + 0.99 * (lead_vel - car.vel) \
            + 4.08 * (gap - car.vel * self.world.safe_time_gap - 2.0)
        accel_control = (desired_accel - car.accel) / 0.4 * 0.1 + car.accel

        return max(min(accel_control, 3), -3), ControlMode.CACC_GC

    def __str__(self):
        if self.leader is not None:
            return "FOLLOW(%s)" % self.leader.name
        else:
            return "FOLLOW(None)"
