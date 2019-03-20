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
import xbox
import config

def publisher(controller):
    pub = rospy.Publisher(config.data["topic_name"], Joy, queue_size=10)
    rospy.init_node(config.data["node_name"], anonymous=True)
    rate = rospy.Rate(config.data["polling_frequency"]) # 10hz
    
    while not rospy.is_shutdown():
        (lx, ly, rx, ry, a, b, x, y) = (0, 0, 0, 0, 0, 0, 0, 0)
        if(config.data["use_controller"]):
            # Get joystick data
            (lx, ly) = controller.leftStick()
            (rx, ry) = controller.rightStick()
            # Get button states
            a = controller.A()
            b = controller.B()
            x = controller.X()
            y = controller.Y()
            print((lx, ly, rx, ry))
        
        # Store controller data ready to be sent
        controller_data = Joy(axes=(lx, ly, rx, ry),
                              buttons=(a, b, x, y))

        pub.publish(controller_data)
        # Maintain publishing frequency
        rate.sleep()

if __name__ == '__main__':
    # Initialise config
    config.init()
    # If the config tells us to ignore the xbox controller then don't use it
    # This lets us debug code without a controller
    if(config.data["use_controller"]):
        # Keep trying to connect to the controller
        while(True):
            # Initialise the xbox controller
            controller = xbox.Joystick()
            try:
                publisher(controller)
            except rospy.ROSInterruptException:
                controller.close()
            except IOError:
                pass;

    else:
        try:
            publisher(False)
        except rospy.ROSInterruptException:
            controller.close()
        
