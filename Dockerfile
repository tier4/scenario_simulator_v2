FROM tiryoh/ros2-desktop-vnc:dashing
ARG GITHUB_USER
ARG GITHUB_TOKEN

RUN mkdir -p /home/ubuntu/Desktop/scenario_simulator_ws/src

RUN touch /home/ubuntu/.netrc \
    echo machine github.com >> /home/ubuntu/.netrc \
    echo login $GITHUB_USER >> /home/ubuntu/.netrc \
    echo passowrd $GITHUB_TOKEN >> /home/ubuntu/.netrc \
    git clone https://github.com/tier4/scenario_simulator.auto.git /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
    rm /home/ubuntu/.netrc

RUN apt-get install -y python-rosdep python3-vcstool python3-colcon-common-extensions
RUN rosdep init
RUN rosdep update
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src
RUN rosdep install -r -y --from-paths . --ignore-src
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws
RUN ["/bin/bash", "-c", "source /opt/ros/dashing/setup.bash && colcon build --symlink-install"]