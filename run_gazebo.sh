#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
#roslaunch topnav_gazebo topnav_d17.launch
roslaunch topnav_gazebo capo.launch

