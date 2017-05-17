#!/usr/bin/env python

import json
import argparse
import pprint
import time
import os
from multiprocessing import Pool, cpu_count

from cacc.simulator import Simulator
from cacc.errors import CollisionException, NegativeSafegapException, CollisionAvoidanceException

def sim_worker(pair):
    i, conf = pair
    if i % 50 == 0:
        print(i)
    try:
        sim = Simulator(conf)
        sim.run()
        return 0
    except (NegativeSafegapException, CollisionAvoidanceException) as e:
        print(e)
        return 0
    except CollisionException as e:
        return 1

def main():
    parser = argparse.ArgumentParser(
        description="Collaborative Adaptive Cruise Control (CACC) Simulator"
    )
    parser.add_argument(
        'config',
        metavar='config_file',
        type=argparse.FileType('r'),
        help='A config file containing run information'
    )
    parser.add_argument(
        '--runs',
        type=int,
        help="""
        The number of simulations to run.  It runs many simulations in
        parallel and counts the number of crashes that occur.  If runs=1, then
        charts can be generated for a single run.  Charts will not be generated
        with multiple runs. (default: 1)
        """,
        default=1
    )
    results = parser.parse_args()
    conf = json.load(results.config)
    pprint.pprint(conf)

    if results.runs == 1:
        try:
            sim = Simulator(conf)
            sim.run()
        except (
            NegativeSafegapException,
            CollisionException,
            CollisionAvoidanceException
        ) as e:
            print(e)
        finally:
            sim.output("output/%s/" % time.ctime())
    else:
        pool = Pool(cpu_count())
        crashes = sum(pool.map(sim_worker, enumerate([conf] * results.runs)))
        print("Crashes: %d" % crashes)

if __name__ == "__main__":
    main()
