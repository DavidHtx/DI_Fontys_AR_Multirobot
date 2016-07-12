#!/usr/bin/env python

# This node turns
# nav_msgs/OccupancyGrid Message
# to
# adhoc_communication/MmMapUpdate Message
#
# From Topic gmapping map
# publish map_other

import rospy
from nav_msgs.msg import OccupancyGrid
from adhoc_communication.msg import MmMapUpdate

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