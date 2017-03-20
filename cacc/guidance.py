from cacc import util
from cacc.simulator import ControlMode
from cacc.errors import *


class LeaderGuidanceStrategy:

    def __init__(self, world, **kwargs):
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
        leader = None
        cars = sorted(self.world.cars, key=lambda car: -car.pos)
        for c in cars:
            if car.name == c.name:
                self.leader = leader
                break
            else:
                leader = c

    def observe(self, sensor, i):
        return sensor.sense(i)

    def listen(self):
        lead_accel = self.world.recv_msg(self.leader.name)
        if lead_accel is not None:
            self.leader_accel = lead_accel
        return self.leader_accel

    def compute_guidance(self, car, i):
        lead_pos, lead_vel = self.observe(car.sensor, i)
        if lead_pos is None or lead_vel is None:
            raise RuntimeError("OneAheadSensor observed None: This shouldn't happen")
        lead_accel = self.listen()
        gap = util.compute_gap(self.leader, car)
        if gap < 0:
            raise CollisionException(self.leader, car, gap)

        safe_gap = util.compute_safegap(self.leader, car)
        if safe_gap < 0:
            raise NegativeSafegapException(self.leader, car, safe_gap)

        if gap < safe_gap:
            # set follower mode to Cacc_ca
            # set follower accel to -MAX_DECEL
            # mode = ControlMode.CACC_CA
            # accel = -car.max_deccel
            raise CollisionAvoidanceException(self.leader, car, gap, safe_gap)
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
