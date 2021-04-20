#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def talker():
	msg = Twist()
	pub = rospy.Publisher('/cmd_vel', String, queue_size=10)
	rospy.init_node('robot talker',anonymous=True)
	while not rospy.is_shutdown():
	msg.linear.z = 0.7
	pub.publish(msg)
	time.sleep(0.3)
	
if __name__ == '__main__':
	talker()

