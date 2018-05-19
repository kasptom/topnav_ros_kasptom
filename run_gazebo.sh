#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
#roslaunch topnav_gazebo topnav_world.launch
roslaunch topnav_gazebo capo.launch

