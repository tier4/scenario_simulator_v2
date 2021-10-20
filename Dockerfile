ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes
ARG ROS_DISTRO

RUN sudo apt-get update && sudo apt-get -y install python3-pip python3-rospkg python3-rosdep software-properties-common
RUN add-apt-repository ppa:kisak/kisak-mesa -y
RUN apt-get update && apt-get install libegl-mesa0 -y

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
COPY . $WORKDIR

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/
RUN mkdir -p /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator/external
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
RUN vcs import external < dependency_${ROS_DISTRO}.repos
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install -iry --from-paths . --rosdistro ${ROS_DISTRO}

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY ./docker-entrypoint.sh /
RUN chmod a+x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
