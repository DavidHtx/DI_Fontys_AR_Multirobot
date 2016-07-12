#!/usr/bin/env python

# Import libraries
import rospy
import tf.transformations
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

# Global Variables
possition = Twist()
distance_to_marker = 0
orientation_left_right = 0
ID_marker = 0

movement_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)


# movement_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10) for kuka

# Callback function, defines global variables reading from /vizmark
def callback(msg):
    global distance_to_marker
    global orientation_left_right
    global ID_marker
    distance_to_marker = msg.pose.position.z
    orientation_left_right = msg.pose.position.x  # minus to left plus to right
    ID_marker = msg.id


# Accel function for turtlebot
def smooth_acceleration_stratight():
    interrupt_occured = False
    step_value = 0.01
    step_time = 0.1
    maximum_speed = 0.2
    if (maximum_speed == 0.2):
        movement_publisher.publish(possition)
        rospy.sleep(step_time)
    while (possition.linear.x != maximum_speed and interrupt_occured == False):
        possition.linear.x += step_value
        movement_publisher.publish(possition)
        rospy.sleep(step_time)


# not being used right now see rotate_to_marker
def smooth_rotation(angle):
    step_angle_value = 0.1
    step_angle_value = 0.1
    # while (possition.angular.z !=angle):


# Function that detects if marker is further away then 0.7 m if so call Accel func.
# Function not being used
def follow_the_marker():
    while (distance_to_marker > 0.7):
        smooth_acceleration_stratight()


# Function which rotates the turtlebot towards the marker
def rotate_to_marker():
    global orientation_left_right
    while (orientation_left_right > 0.2 or orientation_left_right < -0.2):
        if (orientation_left_right > 0.1):
            possition.angular.z = -0.5
        if (orientation_left_right < -0.1):
            possition.angular.z = 0.5
    orientation_left_right = 0


# This is the main function see __main__ for call
def le_function():
    # Initiate THENODE
    rospy.init_node('THENODE', anonymous=True)
    # Define refresh rate/action rate
    rate = rospy.Rate(10)
    # Subscribe THENODE to /vizmark to allow it to read data according to callback function
    rospy.Subscriber("/visualization_marker", Marker, callback)
    # Start of while(TRUE)
    while not rospy.is_shutdown():
        # When distance to marker is greater then 0.6 move towards marker
        if (distance_to_marker > 1):
            print "the distance is bigger than 0.6"
        #    smooth_acceleration_stratight()
        else:
            possition.linear.x = 0
            movement_publisher.publish(possition)
            # Check if marker is in front of robot if not rotate towards it
        rotate_to_marker()
        # When orientation of marker is detected write in console (for test purposes)
        rospy.sleep(0.1)
        if (orientation_left_right > 0.1):
            print (" right ")
            # movement_publisher.publish(possition.angular.x);
        else:
            if (orientation_left_right < -0.1):
                print(" left ")
            else:
                print(" center ")
                # thestr = "distance is %s" % distance_to_marker
                # print thestr

    rate.sleep()


if __name__ == '__main__':
    try:
        le_function()
    except rospy.ROSInitException:
        pass
