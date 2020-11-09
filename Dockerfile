FROM tiryoh/ros2-desktop-vnc:dashing
SHELL ["/bin/bash", "-c"]

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
COPY . $WORKDIR

RUN apt-get update && apt-get -y install python3-pip

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/
RUN mkdir -p /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator/external
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
RUN vcs import external < dependency.repos
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src
RUN source /opt/ros/dashing/setup.bash && rosdep install -r -y --from-paths . --ignore-src
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN source /opt/ros/dashing/setup.bash && colcon build --symlink-install

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator/test_runner/scenario_test_utility
RUN pip3 install -r requirements.txt

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
RUN echo "source /home/ubuntu/Desktop/scenario_simulator_ws/install/local_setup.bash" >> ~/.bashrc