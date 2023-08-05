#!/bin/bash
path_dataset='/media/lqh/WDC/dataset/colive/nice/2023-08-02-23-10-07fixed_part2.bag'
# path_workspace='~/ros/r3live'


# sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia.launch;exec bash;"

sleep 3

# gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh;exec bash;"

gnome-terminal -t "rosbag play" -x bash -c "rosbag play -r 2 ${path_dataset};exec bash;"


