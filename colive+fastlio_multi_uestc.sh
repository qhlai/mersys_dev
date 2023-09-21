#!/bin/bash




# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend rviz.launch rviz:=true enable:=false;exec bash;"

# # gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend colive_backend.launch rviz:=true;exec bash;"
# username=$(id $uid | awk -F'(' '{print $2}' | cut -d')' -f1)
# if [ "$username" = "lqh" ];then

# gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/lqh/ros/r3live_ws/devel/lib/colive_backend/colive_backend_node;exec bash;"

# elif [ "$username" = "uestc" ];then

# gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/uestc/colive_ws/devel/lib/colive_backend/colive_backend_node;exec bash;"
# fi



# sleep 8

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client1;exec bash;"

# sleep 3

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client2;exec bash;"

# sleep 5

# gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh 2;exec bash;"


gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend rviz.launch rviz:=true enable:=false;exec bash;"

# gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch colive_backend colive_backend.launch rviz:=true;exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/lqh/ros/r3live_ws/devel/lib/colive_backend/colive_backend_node;exec bash;"
gnome-terminal -t "roslaunch" -x bash -c "gdb  -ex run  /home/uestc/colive_ws/devel/lib/colive_backend/colive_backend_node;exec bash;"

sleep 8

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client1;exec bash;"

sleep 3

gnome-terminal -t "roslaunch" -x bash -c "cd ../../;source ./devel/setup.bash;roslaunch fast_lio mapping_avia_client.launch rviz:=false namespace:=client2;exec bash;"

sleep 5

gnome-terminal -t "rosbag play" -x bash -c "./dataset.sh 2;exec bash;"