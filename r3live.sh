#!/bin/bash
path_dataset='/media/lqh/WDC/dataset/r3live_offical'
path_workspace='~/ros/r3live'

gnome-terminal -t "build" -x bash -c "./build.sh;exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch r3live r3live_bag.launch;exec bash;"


gnome-terminal -t "rosbag play" -x bash -c "rosbag play ${path_dataset}/hku_campus_seq_00.bag;exec bash;"
