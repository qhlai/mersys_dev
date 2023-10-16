#!/bin/bash
path_dataset='/home/uestc/dataset/mersys/2023-09-23-16-38-52fixed.bag'
path_dataset='/media/lqh/WDC/dataset/mersys/2023-08-02-22-37-57fixed.bag'
# path_dataset='/media/lqh/WDC/dataset/r3live_offical/hku_campus_seq_00.bag'
path_workspace='~/ros/r3live'

# gnome-terminal -t "build" -x bash -c "./build.sh;exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch r3live r3live_bag.launch;exec bash;"
sleep 1
gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch r3live rviz.launch;exec bash;"
#gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia.launch;exec bash;"
# gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh;exec bash;"

gnome-terminal -t "rosbag play" -x bash -c "rosbag play ${path_dataset};exec bash;"


# gnome-terminal -t "rosbag play" -x bash -c "rosbag play ${path_dataset}/hku_campus_seq_00.bag;exec bash;"
# smb://172.168.3.59/dataset/mersys
