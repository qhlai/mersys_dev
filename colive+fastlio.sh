#!/bin/bash
# path_dataset='/media/lqh/WDC/dataset/r3live_offical/hku_campus_seq_00.bag'
path_dataset='/home/viosus/dataset/r3live_offical/hku_campus_seq_00.bag'
# path_dataset='/home/viosus/dataset/colive/2023-07-28-19-51-53.bag'

path_workspace='~/ros/r3live'

#gnome-terminal -t "catkin_make" -x bash -c "catkin_make;exec bash;"
# gnome-terminal -t "build" -x bash -c "./build.sh;exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend colive_backend.launch;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;catkin_make;source ./devel/setup.bash;roslaunch colive_backend colive_backend.launch;exec bash;"

sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch;exec bash;"


sleep 3

gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh;exec bash;"

# gnome-terminal -t "rosbag play" -x bash -c "rosbag play --clock -r 1 ${path_dataset};exec bash;"


# gnome-terminal -t "band" -x bash -c "sudo iftop -i lo -f "port 9033";"

#gnome-terminal -t "backend_node" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;rosrun covins_backend covins_backend_node;exec bash;"

#gnome-terminal -t "visual launch" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;roslaunch ~/ros/covins_g_ws/src/covins/covins_backend/launch/tf.launch;exec bash;"

# catkin_make
# source ~/catkin_ws/devel/setup.bash
