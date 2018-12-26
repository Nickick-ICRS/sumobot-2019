#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('my_topic', String, queue_size=10)
    rospy.init_node('my_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        my_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(my_str)
        pub.publish(my_str)
        # Maintain publishing frequency
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
