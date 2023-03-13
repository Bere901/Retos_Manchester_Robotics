#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point
from std_msgs.msg import Float32

msg = set_point()
msg.time_x = 0.0
msg.signal_y = 0.0




if __name__=='__main__':
    pub_signal=rospy.Publisher("set_point",set_point, queue_size=10)
    pub_motor=rospy.Publisher("signal",Float32, queue_size=10)
    rospy.init_node("Set_Point_Generator")
    rate = rospy.Rate(10)
    t0= rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
	P=rospy.get_param("P",21)
        phase= rospy.get_param("phase",0)
        amplitud= rospy.get_param("amplitud",1)
        offset= rospy.get_param("offset",0)
        w=2*np.pi/P
        t= rospy.Time.now().to_sec()-t0
        timeset=t
        signal=(np.sin(w*t+phase)*amplitud)+offset
	
	msg.time_x = t
        msg.signal_y = signal
	pub_signal.publish(msg)
        pub_motor.publish(signal)
        print_info = "%3f | %3f" %(signal,t)
        rospy.loginfo(print_info)

        rate.sleep()
