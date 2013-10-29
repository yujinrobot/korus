#!/bin/bash

sleep 15

export SIZE=`du -s /usr/local/log | awk '{print $1}'`
if [ $SIZE -ge 800000 ] ; then rm -rf /usr/local/log/* ; fi

export SIZE=`du -s /home/yujin/.ros/log | awk '{print $1}'`
if [ $SIZE -ge 800000 ] ; then rm -rf /home/yujin/.ros/log/* ; fi

source /opt/korus_workspace/korus.bash


#roscd
#./upgrade /dev/ttyACM0 reset.cmd > reset.txt 2>&1 & 

sleep 5

roslaunch korus_meta navigation_standalone.launch map_name:="/home/yujin/testmap" > /usr/local/log/korus.log 2>&1 &
