
class CollisionException(Exception):

    def __init__(self, leader, follower, gap):
        self.leader = leader
        self.follower = follower
        self.gap = gap

    def __str__(self):
        return "CollisionException: %s hit the rear of %s (gap = %.2f)" \
            %  (self.follower.name, self.leader.name, self.gap)

class NegativeSafegapException(Exception):
        def __init__(self, leader, follower, safe_gap):
            self.leader = leader
            self.follower = follower
            self.safe_gap = safe_gap

        def __str__(self):
            return "NegativeSafegapException: leader: %s, follower: %s, safe_gap = %.2f" \
                %  (self.leader.name, self.follower.name, self.safe_gap)

class CollisionAvoidanceException(Exception):
    def __init__(self, leader, follower, gap, safe_gap):
        self.leader = leader
        self.follower = follower
        self.gap = gap
        self.safe_gap = safe_gap

    def __str__(self):
        return "CollisionAvoidanceException: gap (%4.2f) between %s and its follower, %s, is less than safegap (%4.2f).  Entering collision avoidance mode with MAX_DECCEL=%4.2f." \
            % (self.gap, self.leader.name, self.follower.name, self.safe_gap, self.follower.max_deccel)
