#!/usr/bin/env python

from cacc.simulator import Simulator
from cacc.errors import CollisionException, NegativeSafegapException, CollisionAvoidanceException
import json
import argparse
import pprint


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
    results = parser.parse_args()
    conf = json.load(results.config)
    pprint.pprint(conf)
    sim = Simulator(conf)
    try:
        sim.run()
    except (CollisionException, NegativeSafegapException, CollisionAvoidanceException) as e:
        print(e)
        exit(1)
    sim.output()

if __name__ == "__main__":
    main()
