#!/bin/bash

dataset='2023-09-19-19-33-05fixed.bag'

gnome-terminal -t "roslaunch" -x bash -c "rosbag filter ${dataset}  ${dataset}-1.bag "t.secs <= 1695126500";exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "rosbag filter ${dataset}  ${dataset}-2.bag "t.secs >= 1695126285  and t.secs <= 1695127015";exec bash;"

gnome-terminal -t "roslaunch" -x bash -c "rosbag filter ${dataset}  ${dataset}-3.bag "t.secs >= 1695126900";exec bash;"



dataset='2023-09-19-20-18-04fixed.bag'

rosbag filter ${dataset}  ${dataset}-1.bag "t.secs <= 1695126500"

rosbag filter ${dataset}  ${dataset}-2.bag "t.secs >= 1695126285  and t.secs <= 1695127015"

rosbag filter ${dataset}  ${dataset}-3.bag "t.secs >= 1695126880"
