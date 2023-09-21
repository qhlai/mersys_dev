#!/bin/bash

username=$(id $uid | awk -F'(' '{print $2}' | cut -d')' -f1)
if [ "$username" = "lqh" ];then
path_dataset='/media/lqh/WDC/dataset/colive/nice/'

file_name_1="2023-08-02-22-48-44fixed_part1_1.bag"
file_name_2="2023-08-02-22-48-44fixed_part1_2.bag"

path_dataset='/home/lqh/dataset/colive/'
file_name_1="2023-09-19-19-33-05fixed.bag_1.bag"
file_name_2="2023-09-19-19-33-05fixed.bag_2.bag"
#file_name_3="2023-09-19-19-33-05fixed.bag_3.bag"

# path_dataset='/media/lqh/WDC/dataset/colive/fine/'
echo "lqh"
elif [ "$username" = "viosus" ];then
path_dataset='/home/viosus/dataset/colive/'
file_name_1="2023-08-09-16-26-08fixed_part1.bag"
file_name_2="2023-08-09-16-26-08fixed_part2.bag"
file_name_3="2023-08-02-23-10-07fixed_part1.bag"
file_name_4="2023-08-02-23-10-07fixed_part1.bag"
fi
client_num=$1
play_speed=1


# file_name="2023-08-09-16-26-08fixed_part1.bag"
namespace="client1"
gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend dataset.launch namespace:=$namespace file_path:=$path_dataset file_name:=$file_name_1 play_speed:=$play_speed;exec bash;"
# sleep 1

if [ "$client_num" = "2" ];then
# file_name="2023-08-09-16-26-08fixed_part2.bag"
namespace="client2"
gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend dataset.launch namespace:=$namespace file_path:=$path_dataset file_name:=$file_name_2 play_speed:=$play_speed;exec bash;"


# namespace="client3"
# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend dataset.launch namespace:=$namespace file_path:=$path_dataset file_name:=$file_name_3 play_speed:=$play_speed;exec bash;"

fi

# rosbag filter  2023-08-02-22-48-44fixed_part1.bag 2023-08-02-22-48-44fixed_part1_1.bag "t.secs >= 1690987730 and t.secs <= 1690987950"

# rosbag filter  2023-08-02-22-48-44fixed_part1.bag 2023-08-02-22-48-44fixed_part1_2.bag "t.secs >= 1690987900 and t.secs <= 1690988121"

# rosbag filter 2023-08-02-23-10-07fixed_part2.bag  2023-08-02-23-10-07fixed_part21.bag  "t.secs >= 1690989215" and t.secs <= 1690987315"

# rosbag filter 2023-08-02-23-10-07fixed.bag 2023-08-02-23-10-07fixed_part3.bag "t.secs >= 1690989203 and t.secs <= 1690989193"
# name_dataset='2023-08-02-23-10-07fixed.bag'
# # path_dataset='/media/lqh/WDC/dataset/r3live_offical/hku_campus_seq_00.bag'

# rosbag play --clock -r 2  ${path_dataset}${name_dataset}


