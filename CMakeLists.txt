# This file is part of the jetbot_ros_webconsole package (https://github.com/bobboteck/jetbot_ros_webconsole).
# Copyright (c) 2020 Roberto D'Amico.
# 

cmake_minimum_required(VERSION 2.8.3)
project(jetbot_ros_webconsole)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package(
  CATKIN_DEPENDS
    roswww
    rosbridge_server
)

roslaunch_add_file_check(launch)