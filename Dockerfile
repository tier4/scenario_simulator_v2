FROM ros:foxy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes

RUN sudo apt-get update && sudo apt-get -y install python3-pip python3-rospkg python3-rosdep

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
COPY . $WORKDIR

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/
RUN mkdir -p /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator/external
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
RUN sh install_depends.sh foxy
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src
RUN source /opt/ros/foxy/setup.bash && rosdep install -iry --from-paths . --rosdistro foxy

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN source /opt/ros/foxy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY ./docker-entrypoint.sh /
RUN chmod a+x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
