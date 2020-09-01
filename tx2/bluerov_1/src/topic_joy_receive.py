#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy

def callback(msg):
    print msg.buttons[0]
    print msg.buttons[1]
    print msg.axes[0]
    print msg.axes[1]

rospy.init_node('topic_joy_receive', anonymous=True)
rospy.Subscriber('joy', Joy, callback)

rospy.spin()
