#!/usr/bin/env python
import rospy
import numpy as np
import sys
from std_msgs.msg import Float32
import math

t = rospy.get_param("t",1) 



if __name__=='__main__':
    rospy.init_node("signal_generator")
    pub_signal=rospy.Publisher("cmd_pwm", Float32, queue_size=10)
    pub_time=rospy.Publisher("time", Float32, queue_size=10)
    rate=rospy.Rate(5)
    t=0
      
    while not rospy.is_shutdown():
	t+= 0.1
	y=t*(np.sin(t))
	y=(y+20)*75

	print_info = "%3f | %3f" %(y,t)
	rospy.loginfo(print_info)
	pub_signal.publish(y)
	pub_time.publish(t)
        rate.sleep()
	
