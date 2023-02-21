#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
import math
if __name__=='__main__':
    rospy.init_node("signal_generator")
    pub_signal=rospy.Publisher("signal", Float32, queue_size=10)
    pub_time=rospy.Publisher("time", Float32, queue_size=10)
    rate=rospy.Rate(10)
    t=0
    while not rospy.is_shutdown():
        t+= 0.1
        y= np.sin(t)
        print_info = "%3f | %3f" %(y,t)
        rospy.loginfo(print_info)
        pub_signal.publish(y)
        pub_time.publish(t)

        rate.sleep()