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


Please execute commands below.

```
wget https://gist.github.com/hakuturu583/5e6a651df9abdf25dca7071ff5ea8ac3/archive/1448057aeebc34cbfc04598b965440fcf7ecb636.zip
unzip 1448057aeebc34cbfc04598b965440fcf7ecb636.zip
mv 5e6a651df9abdf25dca7071ff5ea8ac3-1448057aeebc34cbfc04598b965440fcf7ecb636 scenarios
sudo pip3 install git+https://github.com/sloretz/off-your-rocker.git
docker pull tier4/scenario_simulator_v2:galactic
rocker --x11 --oyr-mount $PWD/scenarios/UC-001-0001-Kashiwa.yaml -- tier4/scenario_simulator_v2:galactic ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:=$PWD/scenarios/UC-001-0001-Kashiwa.yaml launch_rviz:=True
```

You can see output like below.

<video
  class="c-video__embed"
  src="https://user-images.githubusercontent.com/10348912/126959705-040368bc-7d56-4b81-b456-876da58f763f.mp4"
  width="100%"
  loop
  autoplay
  muted
  playsinline>
</video>
