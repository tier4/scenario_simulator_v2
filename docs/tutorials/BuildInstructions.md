# Build Instructions

This setup instruction is based on ROS2-foxy.
If you want to setup in galactic environment, please replace foxy to galactic.

## Setup ROS 2 environment

This framework only supports ROS 2 Foxy Fitzroy. (<https://docs.ros.org/en/foxy/Installation.html>)  
You can install Foxy by typing the command below in your terminal.

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
sudo apt install ros-foxy-desktop
source /opt/ros/foxy/setup.bash
sudo apt install -y python3-pip python3-rosdep2 python3-vcstool python3-colcon-common-extensions
rosdep update
```

## Setup workspace

```bash
mkdir -p ~/scenario_simulator_ws/src
cd ~/scenario_simulator_ws/src
git clone https://github.com/tier4/scenario_simulator_v2.git
# These lines are necessary right now, but it will be removed in the near future
cd scenario_simulator_v2
# This script clones the part of the source codes in Autoware and add it to the workspace
vcs import external < dependency_foxy.repos
```

## Install dependencies via rosdep

```bash
cd ~/scenario_simulator_ws
source /opt/ros/foxy/setup.bash
rosdep install -iry --from-paths src/scenario_simulator_v2 --rosdistro foxy
```

## Build scenario_simulator_vs

```bash
colcon build --symlink-install
```
