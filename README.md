# AutonomousCars
Research work on autonomous vehicle security

## Setup:
Designed and tested on Python 3.4

```bash
python -m venv cacc-env
source cacc-env/bin/activate
pip install -r requirements.txt
deactivate
```

## How to run:
```bash
source cacc-env/bin/activate
./run.py conf/<config_file>
deactivate
```

Output from the runs will be stored under `output/<timestamp>/`.  It should
contain, the run configuration, position info for each car, and charts
containing position, velocity, and acceleration info.

## New Run Configurations:
You can create new simulation configurations with any
number of cars. For imformation on which args you need,
read the source and see examples in `conf/`. 

