#!/bin/bash




gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch mersys_backend rviz.launch rviz:=true enable:=false;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch mersys_backend mersys_backend.launch rviz:=true;exec bash;"
username=$(id $uid | awk -F'(' '{print $2}' | cut -d')' -f1)
if [ "$username" = "lqh" ];then

gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/lqh/ros/r3live_ws/devel/lib/mersys_backend/mersys_backend_node;exec bash;"

elif [ "$username" = "uestc" ];then

gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/uestc/colive_ws/devel/lib/mersys_backend/mersys_backend_node;exec bash;"
fi



sleep 8

gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client1;exec bash;"

sleep 2

gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client2;exec bash;"

sleep 2

gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client3;exec bash;"


sleep 5

gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh 3;exec bash;"
