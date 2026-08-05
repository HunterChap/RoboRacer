RoboRacer Raspberry Pi Source Workspace
=======================================

This archive contains complete ROS 2 source-package structures for:
- roboracer_py
- roboracer_cpp

It also contains the real-car start, stop, and alias scripts.

The zero-byte file below is intentional and required by ament_python:
  roboracer_ws/src/roboracer_py/resource/roboracer_py

Install
-------
Extract the archive in the Raspberry Pi home directory:

  cd ~
  unzip -o ~/Downloads/roboracer_ws_source_complete.zip

Install dependencies and build:

  sudo apt update
  sudo apt install -y python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-joy ros-jazzy-ackermann-msgs ros-jazzy-nav-msgs

  source /opt/ros/jazzy/setup.bash
  cd ~/roboracer_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install \
    --packages-select roboracer_py roboracer_cpp
  source ~/roboracer_ws/install/setup.bash

Install aliases:

  chmod +x ~/roboracer_ws/scripts/*.sh
  bash ~/roboracer_ws/scripts/setup_roboracer_aliases.sh
  source ~/.bashrc

Commands:
  start_rc_real
  stop_rc_real
  rcbuild
  rcsource
  rcnodes

Important
---------
- Do not copy build, install, or log directories from another computer.
- vehicle_driver_node still does not perform real VESC/servo writes.
- Keep hardware_output_enable=false until hardware communication and
  calibration are implemented and tested.
- The real LiDAR vendor driver is not included.
