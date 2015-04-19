#!/bin/bash

cd $HOME/davis_driver/trunk
xterm -hold -e "./caer" &
xterm -hold -e "roscore" &
xterm -hold -e "rosrun davis_caer davis_caer" &
xterm -hold -e "rosrun rviz rviz" &
xterm -hold -e "roslaunch svo_ros davis.launch"

exit 0
