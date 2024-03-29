ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} as build-stage
RUN mkdir -p /colcon_ws/src
WORKDIR /colcon_ws/src
COPY embree_vendor .
RUN echo "repositories:" > dependency_empty.repos
ARG DEPENDENCY_REPOS=dependency_empty.repos
RUN vcs import . < ${DEPENDENCY_REPOS}
WORKDIR /colcon_ws
RUN apt-get update && \
    rosdep install -iry --from-paths src && \
    apt-get clean \
    && rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon test
