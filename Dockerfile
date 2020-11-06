FROM osrf/ros:dashing-desktop
SHELL ["/bin/bash", "-c"]

WORKDIR /root/scenario_simulator_ws/src/scenario_simulator
COPY . $WORKDIR

RUN apt-get update \
    && apt-get install -y python3-rosdep python3-vcstool python3-colcon-common-extensions python3-pip \
    && rm /etc/ros/rosdep/sources.list.d/20-default.list

RUN rosdep init \
    && rosdep update
WORKDIR /root/scenario_simulator_ws/
RUN vcs import src < src/scenario_simulator/dependency.repos
WORKDIR /root/scenario_simulator_ws/src
RUN source /opt/ros/dashing/setup.bash && rosdep install -r -y --from-paths . --ignore-src
WORKDIR /root/scenario_simulator_ws
RUN source /opt/ros/dashing/setup.bash && colcon build --symlink-install

COPY entrypoint.bash /entrypoint.bash
RUN chmod 777 /entrypoint.bash

WORKDIR /root/scenario_simulator_ws/src/scenario_simulator/test_runner/scenario_test_utility
RUN pip3 install -r requirements.txt

WORKDIR /root/scenario_simulator_ws
RUN echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
RUN echo "source /root/scenario_simulator_ws/install/local_setup.bash" >> ~/.bashrc
ENTRYPOINT ["/entrypoint.bash"]