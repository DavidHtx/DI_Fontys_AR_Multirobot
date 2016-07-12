#!/usr/bin/env python
import rbha
import test
import rosinterface
import rospy
from geometry_msgs.msg import Twist
cmd_vel = Twist()

def callback(msg):
    global cmd_vel
    cmd_vel = msg


def talker():
    global cmd_vel
    #rosinterface.init_robot()
    #rosinterface.enable_robot()
    rospy.init_node('talker', anonymous=True)
    sub = rospy.Subscriber ('/cmd_vel', Twist, callback)
    rate = rospy.Rate(10) # 10hz
    #rbha.init_serial()
    ser = test.robot()
    while not rospy.is_shutdown():
        print (cmd_vel.linear.x)
        ser.send_speed(cmd_vel.linear.x)
        #rbha.rb1.setspeed = cmd_vel.linear.x
        #rbha.rb1.setsteer = 0
        #rosinterface.set_movesteer (cmd_vel.linear.x, 0)
        #rosinterface.get_update_from_rosbee()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass