#!/bin/bash
path_dataset='/home/uestc/dataset/r3live_offical'
path_workspace='~/ros/r3live'

#gnome-terminal -t "catkin_make" -x bash -c "catkin_make;exec bash;"

# gnome-terminal -t "build" -x bash -c "./build.sh;exec bash;"

gnome-terminal -t "roslaunch vis" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch livox_camera_calib calib_3.launch;exec bash;"


# gnome-terminal -t "roslaunch vis" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch livox_camera_calib calib_2.launch;exec bash;"
# # rosrun image_transport republish raw in:=/d400/color/image_raw compressed out:=/d400/color/image_raw

# gnome-terminal -t "roslaunch vins node" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch vins vins_run.launch;exec bash;"



# gnome-terminal -t "rosbag play" -x bash -c "rosbag play ${path_dataset}/hku_campus_seq_00.bag;exec bash;"

# #gnome-terminal -t "backend_node" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;rosrun covins_backend covins_backend_node;exec bash;"

# #gnome-terminal -t "visual launch" -x bash -c "source ~/ros/covins_g_ws/devel/setup.bash;roslaunch ~/ros/covins_g_ws/src/covins/covins_backend/launch/tf.launch;exec bash;"

# # catkin_make
# # source ~/catkin_ws/devel/setup.bash
