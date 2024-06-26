#!/bin/bash
path_dataset='/media/lqh/WDC/dataset/mersys/nice/2023-08-02-22-48-44fixed_part1.bag'
path_dataset='/home/uestc/dataset/mersys/2023-08-09-16-26-08fixed.bag'
# path_workspace='~/ros/r3live'

# rosbag filter 2023-08-02-22-48-44fixed.bag  2023-08-02-22-48-44fixed_part1.bag  "t.secs >= 1690987730 and t.secs <= 1690988121"
# 1690987080
# 1690987325
# sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch fast_lio mapping_mid70.launch;exec bash;"

sleep 3

# gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh;exec bash;"
# gnome-terminal -t "rosbag play" -x bash -c "rosbag play ${path_dataset}/hku_campus_seq_00.bag;exec bash;"
# gnome-terminal -t "rosbag play" -x bash -c "rosbag play -r 3 ${path_dataset};exec bash;"
#rosbag play -r 2 /home/lqh/dataset/mersys/2023-09-19-19-33-05fixed.bag_1.bag
rosbag play -r 2 /home/uestc/dataset/HILTI/Construction_Site_2.bag

