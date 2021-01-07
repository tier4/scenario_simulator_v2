FROM tiryoh/ros2-desktop-vnc:foxy
SHELL ["/bin/bash", "-c"]

RUN sudo apt-get update && sudo apt-get -y install python3-pip python3-rospkg python3-rosdep

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
COPY . $WORKDIR

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/
RUN mkdir -p /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator/external
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
RUN vcs import external < dependency.repos
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src
RUN source /opt/ros/foxy/setup.bash && rosdep install -r -y --from-paths . --ignore-src

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /home/ubuntu/Desktop/scenario_simulator_ws/install/local_setup.bash" >> ~/.bashrc
RUN chown -R ubuntu:ubuntu /home/ubuntu/Desktop

RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
RUN dpkg -i ./google-chrome-stable_current_amd64.deb
RUN rm google-chrome-stable_current_amd64.deb

USER ubuntu
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN source /opt/ros/foxy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

USER root
