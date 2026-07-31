ARG ROS_DISTRO="humble"
# ===================================================================
# Development Stage: Build the full workspace with all tools for development
# ===================================================================
FROM docker.io/library/ros:${ROS_DISTRO} AS development
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN --mount=type=cache,id=apt-cache-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lib-${TARGETARCH},target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rospkg \
        python3-rosdep \
        software-properties-common \
        ccache && \
    add-apt-repository ppa:kisak/kisak-mesa -y && \
    apt-get update && \
    apt-get install -y --no-install-recommends libegl-mesa0

# Keep the cached APT files to speed up subsequent builds
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Copy all necessary source directories
WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws/src
COPY . .

# Install dependencies for all copied packages
RUN git clone --depth 1 https://github.com/autowarefoundation/autoware_launch.git /tmp/autoware_launch && \
    mv /tmp/autoware_launch/vehicle/sample_vehicle_launch/sample_vehicle_description /home/ubuntu/Desktop/scenario_simulator_ws/src/sample_vehicle_description && \
    rm -rf /tmp/autoware_launch && \
    mkdir -p external && \
    vcs import external < dependency_humble.repos

RUN --mount=type=cache,id=apt-cache-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lib-${TARGETARCH},target=/var/lib/apt,sharing=locked \
    bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install -iy --from-paths . --rosdistro ${ROS_DISTRO}"

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws

ENV CC="/usr/lib/ccache/gcc"
ENV CXX="/usr/lib/ccache/g++"
ENV CCACHE_DIR="/ccache"
RUN --mount=type=cache,target=/ccache bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DBUILD_CPP_MOCK_SCENARIOS=ON \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

# ===================================================================
# Runtime Stage: Create a minimal final image
# ===================================================================
FROM docker.io/library/ros:${ROS_DISTRO}-ros-base AS runtime
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Copy source first so rosdep can read package.xml files to resolve
# only the runtime (exec) dependencies of every package in the workspace.
COPY --from=development /home/ubuntu/Desktop/scenario_simulator_ws/src/ /home/ubuntu/Desktop/scenario_simulator_ws/src/

WORKDIR /home/ubuntu/Desktop/scenario_simulator_ws

RUN --mount=type=cache,id=apt-cache-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lib-${TARGETARCH},target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends python3-pip && \
    rosdep update && \
    rosdep install -iy --from-paths src --rosdistro ${ROS_DISTRO} --dependency-types=exec && \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    apt-get remove --purge -y python3-pip && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /root/.ros/rosdep

COPY --from=development /home/ubuntu/Desktop/scenario_simulator_ws/install/ /home/ubuntu/Desktop/scenario_simulator_ws/install/
COPY --from=development /home/ubuntu/Desktop/scenario_simulator_ws/log/ /home/ubuntu/Desktop/scenario_simulator_ws/log/

# Remove unnecessary development files from the copied artifacts
RUN find /home/ubuntu/Desktop/scenario_simulator_ws/install -name cmake -type d -exec rm -rf {} + && \
    find /home/ubuntu/Desktop/scenario_simulator_ws/install -name '*.a' -type f -delete && \
    rm -rf /home/ubuntu/Desktop/scenario_simulator_ws/install/include

COPY ./docker-entrypoint.sh /
RUN chmod a+x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]

# ===================================================================  
# Desktop Stage: Inherit from runtime and add desktop tools like rviz  
# ===================================================================  
FROM runtime AS desktop  

# Install ros2 desktop packages and rviz2  
RUN --mount=type=cache,id=apt-cache-${TARGETARCH},target=/var/cache/apt,sharing=locked \  
    --mount=type=cache,id=apt-lib-${TARGETARCH},target=/var/lib/apt,sharing=locked \  
    apt-get update && \  
    apt-get install -y --no-install-recommends \  
        ros-${ROS_DISTRO}-desktop \  
        ros-${ROS_DISTRO}-rviz2 && \  
    apt-get clean && \  
    rm -rf /var/lib/apt/lists/*
