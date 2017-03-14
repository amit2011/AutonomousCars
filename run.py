#!/usr/bin/env python

from cacc.simulator import Simulator
from cacc.config import SimulatorConfig
import argparse

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
    config = SimulatorConfig.from_file(results.config)
    print(config)
    sim = Simulator(config)
    sim.run()
    sim.output()

if __name__ == "__main__":
    main()
