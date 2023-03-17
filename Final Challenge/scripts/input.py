#!/usr/bin/env python
import rospy
import numpy as np
from reto_final.msg import set_point
from std_msgs.msg import Float32

msg = set_point()
msg.time_x = 0.0
msg.signal_y = 0.0




if __name__=='__main__':
    pub_signal=rospy.Publisher("set_point",set_point, queue_size=10)
    pub_motor=rospy.Publisher("signal",Float32, queue_size=10)
    rospy.init_node("input")
    rate = rospy.Rate(10)
    t0= rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        tipo=rospy.get_param("tipo",2)
	if tipo== 1:
            P=rospy.get_param("P",21)
            phase= rospy.get_param("phase",0)
            amplitud= rospy.get_param("amplitud",1)
            offset= rospy.get_param("offset",0)
            w=2*np.pi/P
            t= rospy.Time.now().to_sec()-t0
            timeset=t
            signal=(np.sin(w*t+phase)*amplitud)+offset
        
        #estructura del square
        elif tipo==2:
            P=rospy.get_param("P",21)
            phase= rospy.get_param("phase",0)
            amplitud= rospy.get_param("amplitud",1)
            offset= rospy.get_param("offset",0)
            w=2*np.pi/P
            t= rospy.Time.now().to_sec()-t0
            if(np.sin(w*t+phase)*amplitud)>=0:
                signal=rospy.get_param("tope",1)
                timeset=t
            else:
                signal=rospy.get_param("fondo",-1)
                timeset=t0
        
        #estructura del step 
        elif tipo==3:
            P=rospy.get_param("P",21)
            phase= rospy.get_param("phase",0)
            amplitud= rospy.get_param("amplitud",1)
            offset= rospy.get_param("offset",0)
            #w=2*np.pi/P
            t= rospy.Time.now().to_sec()-t0
            signal=offset
            timeset=t
	#t+= 0.1
        #signal= np.sin(t)
        

	msg.time_x = t
        msg.signal_y = signal
	pub_signal.publish(msg)
        pub_motor.publish(signal)
        print_info = "%3f | %3f" %(signal,t)
        rospy.loginfo(print_info)

        rate.sleep()
