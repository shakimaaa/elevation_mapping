#!/bin/bash
cd ~/workspace
# vcs import < src/elevation_mapping_cupy/docker/src.repos src/ --recursive -w $(($(nproc)/2))

sudo apt update

# If install fails due to numpy version, you can uncomment the following line to install numpy 1.26.4
rosdep update
rosdep install --from-paths src --ignore-src -y -r # --skip-keys "numpy_lessthan_2"

# rosdep install --from-paths src --ignore-src -y -r --skip-keys "numpy_lessthan_2"