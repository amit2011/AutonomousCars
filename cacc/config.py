import configparser

class SimulatorConfig:

    def __init__(self, sim_conf, cars):
        self.sim_conf = sim_conf
        self.cars = cars

    def __str__(self):
        string = "Simulation Config:\n"
        for name, value in self.sim_conf.items():
            string += "\t%-20s: %s\n" % (name, value)
        string += "Cars:\n"
        for car in self.cars:
            string += "\t%s\n" % car
        return string

    @classmethod
    def from_file(cls, config_file):
        print("Reading config from: %s" % config_file.name)
        parser = configparser.ConfigParser()
        parser.read_file(config_file)
        sim_conf = cls.parse_sim_conf(parser)
        cars = []
        for name, values in parser.items():
            if name in {"Simulation", 'DEFAULT'}:
                continue
            cars.append(cls.parse_car_conf(name, values))
        config = cls(sim_conf, cars)
        return config

    @staticmethod
    def parse_sim_conf(conf):
        return {
            'safe_time_gap': conf.getfloat('Simulation', 'safe_time_gap'),
            'car_length': conf.getfloat('Simulation', 'car_length'),
            'max_deccel': conf.getfloat('Simulation', 'max_deccel'),
            'max_vel': conf.getfloat('Simulation', 'max_vel'),
            'time_unit': conf.getfloat('Simulation', 'time_unit'),
            'trajectory_points': conf.getint('Simulation', 'trajectory_points')
        }

    @staticmethod
    def parse_car_conf(car_name, car_conf):
        if 'position' not in car_conf:
            raise RuntimeError("Missing position config for %s", car_name)
        if 'velocity' not in car_conf:
            raise RuntimeError("Missing velocity config for %s", car_name)
        if 'acceleration' not in car_conf:
            raise RuntimeError("Missing acceleration config for %s", car_name)
        if 'guide_strat' not in car_conf:
            raise RuntimeError("Missing guide_strat config for %s", car_name)
        if 'comm_strat' not in car_conf:
            raise RuntimeError("Missing comm_strat config for %s", car_name)
        if 'sensor' not in car_conf:
            raise RuntimeError("Missing sensor config for %s", car_name)
        return {
            'name': car_name,
            'pos': float(car_conf.get('position')),
            'vel': float(car_conf.get('velocity')),
            'accel': float(car_conf.get('acceleration')),
            'guide_strat': car_conf.get('guide_strat'),
            'comm_strat': car_conf.get('comm_strat'),
            'sensor': car_conf.get('sensor')
        }
