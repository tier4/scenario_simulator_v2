# Run test scenario with Docker

Before you try this documentation, please check [this document](RunWithDocker.md) and setup docker and run scenario_simulator_v2 inside docker.

## Run test scenarios in docker

Please check isEgo value is set false before running this script.
For example.

```
ObjectController:
    Controller:
    name: ''
    Properties:
        Property:
        - { name: isEgo, value: 'false' }
```

Download [example scenario](https://gist.github.com/hakuturu583/5e6a651df9abdf25dca7071ff5ea8ac3) in your local machine.

```
wget https://gist.github.com/hakuturu583/5e6a651df9abdf25dca7071ff5ea8ac3/archive/d4225e88169749b748b3f3cd70d8b75198846362.zip
unzip d4225e88169749b748b3f3cd70d8b75198846362.zip
mv 5e6a651df9abdf25dca7071ff5ea8ac3-d4225e88169749b748b3f3cd70d8b75198846362 scenarios
sudo pip3 install git+https://github.com/sloretz/off-your-rocker.git
docker pull tier4/scenario_simulator_v2
rocker --x11 --oyr-mount $PWD/scenarios/UC-001-0001-Kashiwa.yaml -- tier4/scenario_simulator_v2 ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:=$PWD/scenarios/UC-001-0001-Kashiwa.yaml with-rviz:=True
```
