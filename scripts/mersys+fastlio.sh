#!/bin/bash

gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch mersys_backend mersys_backend.launch rviz:=true;exec bash;"


sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false ;exec bash;"
# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch fast_lio mapping_mid70_client.launch rviz:=false ;exec bash;"

sleep 3

# gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh;exec bash;"
path_dataset='/home/uestc/dataset/HILTI/Campus_2.bag'
gnome-terminal -t "rosbag play" -x bash -c "rosbag play -r 1 ${path_dataset};exec bash;"


# gnome-terminal -t "band" -x bash -c "sudo iftop -i lo -f "port 9033";"

#gnome-terminal -t "backend_node" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;rosrun covins_backend covins_backend_node;exec bash;"

#gnome-terminal -t "visual launch" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;roslaunch ~/ros/covins_g_ws/src/covins/covins_backend/launch/tf.launch;exec bash;"

# catkin_make
# source ~/catkin_ws/devel/setup.bash
