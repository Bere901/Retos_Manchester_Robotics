#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
#from pid_control.msg import set_point
from pid_control.msg import motor_input
from pid_control.msg import motor_output

signal_data2 = 0.0
time_data2 = 0.0

output = motor_output()
output.time = 0.0
output.output = 0.0



#Setup parameters, vriables and callback functions here (if required)
def callback(msg):
    global signal_data2, time_data2
    signal_data2 = msg.input
    time_data2 = msg.time

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("system")
    rospy.Subscriber(rospy.get_namespace()+"motor_input", motor_input, callback)
    rate = rospy.Rate(10)
    pub=rospy.Publisher("motor_output",motor_output, queue_size=10)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        #Write your code here
	signal = signal_data2
        #rospy.loginfo("The proessed signal value is: %f at a time %f",signal, time_data)
        #pub.publish(signal_data)


	output.time = time_data2
        output.output = signal
	pub.publish(output)
	#rospy.loginfo("The generated signal value is: %f at a time %f", signal, time_data2)
	print_info = "%3f | %3f" %(signal,time_data2)
        rospy.loginfo(print_info)




        rate.sleep()
