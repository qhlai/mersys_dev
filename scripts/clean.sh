#!/bin/bash
path_dataset='/media/lqh/WDC/dataset/r3live_offical'
path_workspace='~/ros/r3live'

#gnome-terminal -t "catkin_make" -x bash -c "catkin_make;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;catkin_make;exec bash;"
cd ../../../
catkin_make clean
