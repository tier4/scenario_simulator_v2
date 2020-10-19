FROM tiryoh/ros2-desktop-vnc:dashing
ARG GITHUB_USER
ARG GITHUB_TOKEN

RUN mkdir -p /home/ubuntu/Desktop/scenario_simulator_ws/src

RUN touch /root/.netrc \
    echo machine github.com >> /root/.netrc \
    echo login $GITHUB_USER >> /root/.netrc \
    echo passowrd $GITHUB_TOKEN >> /root/.netrc \
    git clone https://github.com/tier4/scenario_simulator.auto.git /home/ubuntu/Desktop/scenario_simulator_ws/src/scenario_simulator
    rm /root/.netrc