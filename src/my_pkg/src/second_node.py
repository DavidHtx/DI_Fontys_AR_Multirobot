#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



