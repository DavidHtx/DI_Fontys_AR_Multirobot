#!/usr/bin/env python

import rospy
import RPi.GPIO

from std_msgs.msg import String

def PinPub():
    pub = rospy.Publisher('PinRelay', String, queue_size=10)
    rospy.init_node('PinRelay', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        PinPub()
    except rospy.ROSInterruptException:
        pass