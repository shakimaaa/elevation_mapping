#!/bin/bash

# This only runs once time after the docker image is built.
set -e
cd /cj_gazebo_sim
# Set the default build type
source /opt/ros/$ROS_DISTRO/setup.bash
BUILD_TYPE=RelWithDebInfo #Debug, Release, RelWithDebInfo, MinSizeRel
colcon build \
        --continue-on-error \
        --parallel-workers $(nproc) \
        --symlink-install \
        --event-handlers console_cohesion+ \
        --base-paths /cj_gazebo_sim/src/elevation/elevation_mapping/elevation_map_msgs /cj_gazebo_sim/src/elevation/elevation_mapping/elevation_mapping_cupy /cj_gazebo_sim/src/elevation/elevation_mapping/plane_segmentation /cj_gazebo_sim/src/elevation/elevation_mapping/sensor_processing \
        --cmake-args \
                "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
                "-DBUILD_TESTING=OFF" \
                "-DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined" \
                -Wall -Wextra -Wpedantic -Wshadow \
        --packages-skip \
                convex_plane_decomposition \
                convex_plane_decomposition_ros \
                robot_sim \
                fastdds_ros2_bridge \
                lr_pro_description