#!/usr/bin/env python

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
        # Initialise the xbox controller
        controller = xbox.Joystick()
        try:
            publisher(controller)
        except rospy.ROSInterruptException:
            controller.close()

    else:
        try:
            publisher(False)
        except rospy.ROSInterruptException:
            controller.close()
        
