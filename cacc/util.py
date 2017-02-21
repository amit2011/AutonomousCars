from .errors import *
import importlib

def compute_accel(leader, follower, safeTime):
    gap = compute_gap(leader, follower)
    if gap < 0:
        raise CollisionException(leader, follower, gap)

    safe_gap = compute_safegap(leader, follower)
    if safe_gap < 0:
        raise NegativeSafegapException(leader, follower, safe_gap)

    if gap < safe_gap:
        # set follower mode to Cacc_ca
        # set follower accel to -MAX_DECEL
        """
            f->p[i].mode = Cacc_ca;
            fprintf(stderr, " Currentgap %4.2f is less than safegap %4.2f, entering ca with max deceleration %4.2f\n", gap, safegap, f->maxda);
            f->p[i].accel = -MAX_DECEL;
        """
        raise CollisionAvoidanceException(leader, follower, gap, safe_gap)

    # set follower mode to CACC_GC
    # adjust follower acceleration appropriately
    """
        f->p[i].mode = Cacc_gc;
        double desiredaccel = 0.66 * r->p[i].accel
            + 0.99 * (r->p[i].vel - f->p[i].vel)
            + 4.08 * (gap - (f->p[i].vel * SAFETIMEGAP) - 2.0);
        double accelcontrol = (desiredaccel - f->p[i].accel) / 0.4 * 0.1
            + f->p[i].accel;
        f->p[i].accel = max(min(accelcontrol, 3), -3);
    """

    return None # TODO What should this be?

def compute_gap(leader, follower):
    return leader.pos - follower.pos - leader.length

def compute_safegap(leader, follower):
    return max(1.0,
        0.1 * follower.vel + follower.vel ** 2 / (2.0 * follower.max_deccel) \
        - leader.vel ** 2 / (2 * leader.max_deccel) + 1.0
    )

def cls_from_str(clsname):
    module_name, class_name = clsname.rsplit(".", 1)
    module = importlib.import_module(module_name, package='cacc')
    cls = getattr(module, class_name)
    return cls
