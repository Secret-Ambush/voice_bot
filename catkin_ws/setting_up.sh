#!/bin/bash

catkin_make
source devel/setup.bash
roslaunch voice_bot voice_control.launch
