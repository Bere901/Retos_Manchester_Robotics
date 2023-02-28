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
    t=0

    while not rospy.is_shutdown():
	t+= 0.1
        signal= np.sin(t)
        signal=((t * np.cos(5)) + np.cos(t) * np.sin(5))* -np.cos(t)

	msg.time_x = t
        msg.signal_y = signal
	pub_signal.publish(msg)
        pub_motor.publish(signal)
        print_info = "%3f | %3f" %(signal,t)
        rospy.loginfo(print_info)

        rate.sleep()
