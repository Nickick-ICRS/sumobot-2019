#!/bin/bash

# Go to classic mode (to use ROS stuff)
#classic

# Load ROS stuff
. /opt/ros/kinetic/setup.bash

# Load the program
. /home/nrh16/sumobot-2019/devel/setup.bash

# Launch the script
roslaunch sumobot_motion_master sumobot.launch

