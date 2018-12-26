# sumobot-2019

This repository contains a catkin workspace to allow The Shoveller to be programmed in ROS. The Shoveller is an example robot for ICRS' (Imperial College Robotics Society) Sumobot2019 competition.

The robot is programmed in ROS, so before proceeding make sure you have ROS installed (this was made on Melodic) and follow the [installation instructions](http://wiki.ros.org/ROS/Installation). It might be a good idea to do some of the [beginner tutorials](http://wiki.ros.org/ROS/Tutorials) if you don't know anything about ROS!

## Packages

Each catkin package can be made and run the same way:

1. open a terminal
2. `cd <workspace>` (e.g. `cd ~/catkin_ws`)
3. `catkin_make <package_name>` (or `catkin_make` to make all packages)
4. `source devel/setup.bash` (unless you're not using bash - In which case you'll know and I'll assume you can work it out)
5. `rosrun <package_name> node` (make sure you have roscore running in a separate terminal first!)

To run multiple packages at once you only need to run catkin_make once, but you'll need to make sure you've sourced the setup file on each new terminal.

The packages within are outlined as follows:

### sumobot_msgs

This package contains msgs used by other ROS nodes to communicate. The messages included are:

 - MotorPowers
     - uint8 left_power (left motor power %, 0-100)
     - uint8 right_power (right motor power %, 0-100)
     - uint8 left_direction (left motor direction, 0-1)
     - uint8 right_direction (right motor direction, 0-1)

### sumobot_drive
##### (C++)

This package contains code to control the motors by sending messages via a ROS topic. The packages uses the following external libraries:

 - WiringPi - This needs to be installed via their website [here](http://wiringpi.com/download-and-install/). This allows for control of the GPIO pins.
 - jsoncpp - The amalgamated version of jsoncpp is included in this package, so no installation is necessary. To use this yourself see [the github repo](https://github.com/open-source-parsers/jsoncpp).

The layout within the package is as follows:
 - sumobot_drive_node -> Contains `main()`. Directs the pwm controller based on the most recently received values by the listener.
     - pwm_controller -> Controls the speed of the motors (as a %) via Pulse Width Modulation.
     - listener -> Listens on a topic defined in the config file for MotorPowers messages.
     - config -> Handles loading the config file (in `~/.config/sumobot/sumobot_drive/config.json`).

### sumobot_xbox_controller
##### (Python)

This package contains code to read button and joystick states of an xbox controller. The package uses the following external libraries:

 - xboxdrv - This is used by Xbox (below) and needs to be installed before being run via `sudo apt install xboxdrv` see [xboxdrv](http://xboxdrv.gitlab.io/) for more info.

 - Xbox - The python file used by this is xbox.py and is already included in this repo. To use this yourself see [the github repo](https://github.com/FRC4564/Xbox).

The layout of this package is as follows:
 - xbox_listener.py -> Executable script. Listens to an xbox controller and outputs a sensor_msgs/Joy message on a channel specified in the config.
     - config.py -> Handles loading the config file (in `~/.config/sumobot/sumobot_xbox_controller/config.json`).


## Sumobot2019 Rules

The current rules for Sumobot2019 can be found [here](http://sumobotrules.icrs.io). If you are interested in joining the competition, please get in contact with Nick: nrh16@ic.ac.uk.
