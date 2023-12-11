#!/bin/bash
# path_dataset='/media/uestc/WDC/dataset/r3live_offical'
# path_dataset='/home/uestc/dataset/r3live_offical'
path_dataset='/media/uestc/WDC/dataset/mersys/2023-08-02-22-37-57fixed.bag'
path_workspace='~/ros/r3live'

#gnome-terminal -t "catkin_make" -x bash -c "catkin_make;exec bash;"
# gnome-terminal -t "build" -x bash -c "./build.sh;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch mersys_backend mersys_backend.launch rviz:=true;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch mersys_backend rviz.launch rviz:=true enable:=false;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch mersys_backend mersys_backend.launch rviz:=true;exec bash;"
gnome-terminal -t "roslaunch vis" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch vins vins_rviz.launch;exec bash;"

username=$(id $uid | awk -F'(' '{print $2}' | cut -d')' -f1)
if [ "$username" = "lqh" ];then

gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/lqh/ros/r3live_ws/devel/lib/mersys_backend/mersys_backend_node;exec bash;"

elif [ "$username" = "uestc" ];then

gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/uestc/mersys_ws/devel/lib/mersys_backend/mersys_backend_node;exec bash;"
fi


# gnome-terminal -t "roslaunch" -x bash -c "cd ../../../;catkin_make;source ./devel/setup.bash;roslaunch mersys_backend mersys_backend.launch;exec bash;"

sleep 3

gnome-terminal -t "roslaunch vins node" -x bash -c "cd ../../../;source ./devel/setup.bash;roslaunch vins vins_run.launch;exec bash;"


sleep 3

gnome-terminal -t "rosbag play" -x bash -c "rosbag play ${path_dataset};exec bash;"


# gnome-terminal -t "band" -x bash -c "sudo iftop -i lo -f "port 9033";"

#gnome-terminal -t "backend_node" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;rosrun covins_backend covins_backend_node;exec bash;"

#gnome-terminal -t "visual launch" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;roslaunch ~/ros/covins_g_ws/src/covins/covins_backend/launch/tf.launch;exec bash;"

# catkin_make
# source ~/catkin_ws/devel/setup.bash
