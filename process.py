#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
          
original_signal = 0
original_time = 0

def callbacksignal(msgsignal):
    global original_signal
    original_signal = msgsignal.data

def callbacktime(msgtime):
    global original_time
    original_time = msgtime.data

if __name__ =='__main__':
    rospy.init_node('process')
    rospy.Subscriber("signal", Float32, callbacksignal)
    rospy.Subscriber("time", Float32, callbacktime)

    pub_signal = rospy.Publisher("process", Float32, queue_size = 10)
    pub_time = rospy.Publisher("time_2", Float32, queue_size = 10)
    rate = rospy.Rate(10) #10 Hz

    while not rospy.is_shutdown():
        t=original_time

        y=((original_signal * np.cos(5)) + np.cos(t) * np.sin(5))* -np.cos(t)
        print_info = "%3f | %3f" %(y,t)
        rospy.loginfo(print_info)
        pub_signal.publish(y)
        pub_time.publish(t)

        rate.sleep()