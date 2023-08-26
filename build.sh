#!/bin/bash
# path_dataset='/media/lqh/WDC/dataset/r3live_offical'
path_dataset='/media/viosus/WDC/dataset/r3live_offical'
path_workspace='~/ros/r3live'


if [ $# -eq 0 ]
then
    NR_JOBS=""
    CATKIN_JOBS=""
else
    NR_JOBS=${1:-}
    CATKIN_JOBS="-j${NR_JOBS}"
fi

#gnome-terminal -t "catkin_make" -x bash -c "catkin_make;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;catkin_make;exec bash;"
cd ../../
# catkin_make -j11

#r3live

# catkin build ${CATKIN_JOBS} livox_ros_driver r3live

#colive
clear

catkin build ${CATKIN_JOBS} colive_backend 
catkin build ${CATKIN_JOBS} livox_ros_driver  fast_lio 
catkin build ${CATKIN_JOBS}  vins camera_models global_fusion loop_fusion
catkin build ${CATKIN_JOBS}  livox_camera_calib
# catkin build ${CATKIN_JOBS}  r3live

# catkin build catkin_simple
