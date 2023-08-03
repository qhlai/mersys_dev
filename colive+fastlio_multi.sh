#!/bin/bash


gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend colive_backend.launch rviz:=true;exec bash;"




sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client1;exec bash;"

sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client2;exec bash;"

sleep 5

gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh 2;exec bash;"