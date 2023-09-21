#!/bin/bash

dataset='2023-09-19-19-33-05fixed.bag'

gnome-terminal -t "roslaunch" -x bash -c "rosbag filter ${dataset}  ${dataset}-1.bag "t.secs >= 1695123196 and t.secs <= 1695124080";exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "rosbag filter ${dataset}  ${dataset}-2.bag "t.secs >= 1695123735  and t.secs <= 1695124536";exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "rosbag filter ${dataset}  ${dataset}-3.bag "t.secs >= 1695124450 and t.secs <= 1695124860";exec bash;"
