#!/usr/bin/env sh
roslaunch urdf_tutorial display.launch model:=`find . -name capo.urdf.xacro`
