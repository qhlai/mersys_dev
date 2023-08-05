#!/bin/bash

username=$(id $uid | awk -F'(' '{print $2}' | cut -d')' -f1)
if [ "$username" = "lqh" ];then
path_dataset='/media/lqh/WDC/dataset/colive/nice/'
# path_dataset='/media/lqh/WDC/dataset/colive/fine/'
echo "lqh"
elif [ "$username" = "viosus" ];then
path_dataset='/home/viosus/dataset/colive/'
fi
client_num=$1



file_name="2023-08-02-23-10-07fixed_part2.bag"

namespace="client1"
gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend dataset.launch namespace:=$namespace file_path:=$path_dataset file_name:=$file_name ;exec bash;"
# sleep 1

if [ "$client_num" != "" ];then
file_name="2023-08-02-23-10-07fixed_part1.bag"
namespace="client2"
gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend dataset.launch namespace:=$namespace file_path:=$path_dataset file_name:=$file_name ;exec bash;"
fi


# rosbag filter 2023-08-02-23-10-07fixed_part2.bag  2023-08-02-23-10-07fixed_part21.bag  "t.secs >= 1690989215" and t.secs <= 1690987315"

# rosbag filter 2023-08-02-23-10-07fixed.bag 2023-08-02-23-10-07fixed_part3.bag "t.secs >= 1690989203 and t.secs <= 1690989193"
# name_dataset='2023-08-02-23-10-07fixed.bag'
# # path_dataset='/media/lqh/WDC/dataset/r3live_offical/hku_campus_seq_00.bag'

# rosbag play --clock -r 2  ${path_dataset}${name_dataset}


