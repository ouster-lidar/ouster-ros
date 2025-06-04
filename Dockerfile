ARG ROS_DISTRO=rolling
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp

FROM ros:${ROS_DISTRO}-ros-core AS build-env
ARG RMW_IMPLEMENTATION
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
    BUILD_HOME=/var/lib/build \
    OUSTER_ROS_PATH=/opt/ros2_ws/src/ouster-ros

RUN set -xue && \
    apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        fakeroot \
        dpkg-dev \
        python3-rosdep \
        python3-rospkg \
        python3-bloom \
        python3-colcon-common-extensions

RUN if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then \
        apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp; \
    elif [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then \
        apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp; \
    fi

# Set up non-root build user
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -o -g ${BUILD_GID} build \
&& useradd -o -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

# Set up build environment
COPY --chown=build:build . $OUSTER_ROS_PATH

RUN set -xe         \
&& apt-get update   \
&& rosdep init      \
&& rosdep update --rosdistro=$ROS_DISTRO \
&& rosdep install --from-paths $OUSTER_ROS_PATH -y --ignore-src


USER build:build
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& mkdir src \
&& cp -R $OUSTER_ROS_PATH ./src
FROM build-env
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Wno-deprecated-declarations"

RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon test \
    --ctest-args tests ouster_ros --rerun-failed --output-on-failure

# Entrypoint for running Ouster ros:
#
# Usage: docker run --rm -it ouster-ros [sensor.launch parameters ..]
#
ENTRYPOINT ["bash", "-c", "set -e \
&& source ./install/setup.bash \
&& ros2 launch ouster_ros sensor.launch.xml \"$@\" \
", "ros-entrypoint"]
