####################################################################
####################################################################
# Copyright 2019 Nick Hafner                                       #
                                                                   #
# Permission is hereby granted, free of charge, to any person      #
# obtaining a copy of this software and associated documentation   #
# files (the "Software"), to deal in the Software without          #
# restriction, including without limitation the rights to use,     #
# copy, modify, merge, publish, distribute, sublicense, and/or     #
# sell copies of the Software, and to permit persons to whom the   #
# Software is furnished to do so, subject to the following         #
# conditions:                                                      #
                                                                   #
# The above copyright notice and this permission notice shall be   #
# included in all copies or substantial portions of the Software.  #
                                                                   #
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,  #
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES  #
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND         #
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT      #
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,     #
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     #
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR    #
# OTHER DEALINGS IN THE SOFTWARE.                                  #
####################################################################
####################################################################

import rospy
from sensor_msgs.msg import Joy
import pygame
import config
import datetime

def publisher(controller):
    pub = rospy.Publisher(config.data["topic_name"], Joy, queue_size=10)
    rospy.init_node(config.data["node_name"], anonymous=True)
    rate = rospy.Rate(config.data["polling_frequency"]) # 10hz

    controller.init()

    last_input = datetime.datetime.now()
    
    (olx, oly, orx, ory, oa, ob, ox, oy) = (0, 0, 0, 0, 0, 0, 0, 0)
    (lx, ly, rx, ry, a, b, x, y) = (0, 0, 0, 0, 0, 0, 0, 0)

    while not rospy.is_shutdown():
        dt = datetime.datetime.now() - last_input
        if(dt.seconds > 2):
            # Check that the controller is still connected
            pygame.joystick.quit()
            pygame.joystick.init()
            while(pygame.joystick.get_count() == 0):
                pygame.joystick.quit()
                pygame.joystick.init()
            controller = pygame.joystick.Joystick(0)
            controller.init()
            last_input = datetime.datetime.now()

        if(config.data["use_controller"]):
            # Get joystick data
            lx = controller.get_axis(0)
            ly = -controller.get_axis(1)
            rx = controller.get_axis(3)
            ry = -controller.get_axis(4)
            #(rx, ry) = controller.get_axis(1)
            # Get button states
            a = controller.get_button(0)
            b = controller.get_button(1)
            x = controller.get_button(2)
            y = controller.get_button(3)
            if(olx != lx or oly != ly or orx != rx or ory != ry \
               or oa != a or ob != b or ox != x or oy != y):
               last_input = datetime.datetime.now()
               (olx, oly, orx, ory, oa, ob, ox, oy) = (lx, ly, rx, ry, a, b, x, y)
        
        # Store controller data ready to be sent
        controller_data = Joy(axes=(lx, ly, rx, ry),
                              buttons=(a, b, x, y))
        pub.publish(controller_data)
        
        # Handle pygame events
        pygame.event.pump()

        # Maintain publishing frequency
        rate.sleep()

        # Check that Joystick is connected
        while(pygame.joystick.get_count() == 0):
            pygame.joystick.init()
        controller = pygame.joystick.Joystick(0)
        controller.init()

if __name__ == '__main__':
    pygame.init()
    # Initialise config
    config.init()
    # If the config tells us to ignore the xbox controller then don't use it
    # This lets us debug code without a controller
    if(config.data["use_controller"]):
        # Initialise the xbox controller
        pygame.joystick.init()
        try:
            while(pygame.joystick.get_count() == 0):
                pygame.joystick.quit()
                pygame.joystick.init()
            controller = pygame.joystick.Joystick(0)
            publisher(controller)
        except rospy.ROSInterruptException:
            pygame.joystick.quit()

    else:
        try:
            publisher(False)
        except rospy.ROSInterruptException:
            controller.close()
        
