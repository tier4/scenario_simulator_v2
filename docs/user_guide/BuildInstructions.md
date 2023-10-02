# Build Instructions

This setup instruction is based on ROS 2 humble.

## Setup ROS 2 environment

This framework only supports ROS 2 Galactic Geochelone now. (We are planning to support ROS 2 Humble Hawksbill)  
You can install Galactic by executing the command below in your terminal.

```bash
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update
sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
sudo apt install -y python3-pip python3-rosdep2 python3-vcstool python3-colcon-common-extensions
rosdep update
```
reference : <https://docs.ros.org/en/humble/Installation.html>
## Setup workspace

```bash
mkdir -p ~/scenario_simulator_ws/src
cd ~/scenario_simulator_ws/src
git clone https://github.com/tier4/scenario_simulator_v2.git
# These lines are necessary right now, but it will be removed in the near future
cd scenario_simulator_v2
# This script clones the part of the source codes in Autoware and add it to the workspace
vcs import external < dependency_humble.repos
```

## Install dependencies via rosdep

```bash
cd ~/scenario_simulator_ws
source /opt/ros/humble/setup.bash
rosdep install -iry --from-paths src/scenario_simulator_v2 --rosdistro humble
```

## Build scenario_simulator_v2

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
