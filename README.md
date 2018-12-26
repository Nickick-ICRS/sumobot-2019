# sumobot-2019

This repository contains a catkin workspace to allow The Shoveller to be programmed in ROS. The Shoveller is an example robot for ICRS' (Imperial College Robotics Society) Sumobot2019 competition.

## Packages

The packages within are outlined as follows:

### sumobot_drive

This package contains code to control the motors by sending messages via a ROS topic. The packages uses the following external libraries:

 - WiringPi - This needs to be installed via their website [here](http://wiringpi.com/download-and-install/). This allows for control of the GPIO pins.
 - jsoncpp - The amalgamated version of jsoncpp is included in this package, so no installation is necessary. TO use this yourself see [the github repo](https://github.com/open-source-parsers/jsoncpp).

The layout within the package is as follows:
 - main -> directs the pwm controller based on the most recently received values by the listener
     - pwm_controller -> Controls the speed of the motors (as a %) via Pulse Width Modulation
     - listener -> Listens on a topic defined in the config file for MotorPowers messages
     - config -> Handles loading the config file (in ~/.config/sumobot/sumobot_drive)

## Sumobot2019 Rules

The current rules for Sumobot2019 can be found [here](http://sumobotrules.icrs.io). If you are interested in joining the competition, please get in contact with Nick: nrh16@ic.ac.uk.
