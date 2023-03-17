#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from reto_final.msg import set_point
from reto_final.msg import motor_input
from reto_final.msg import motor_output





signal_data = 0.0
time_data = 0.0

output_data = 0.0
time_data2 = 0.0

error = 0.0
angularVelocity = 0.0 

error_acumulado = 0.0
ultima_medicion = 0.0

original_signal = 0

msg = motor_input()
msg.time = 0.0
msg.input = 0.0
error_anterior=0


def callback(msg):
    global signal_data, time_data
    signal_data = msg.signal_y
    time_data = msg.time_x 

def callback2(msgsignal):
    global original_signal
    original_signal = msgsignal.data



def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    pub_signal=rospy.Publisher("cmd_pwm", Float32, queue_size=10)
    rospy.Subscriber("my_topic", Float32, callback2)
    pub_time=rospy.Publisher("time", Float32, queue_size=10)
    rospy.init_node("controller")
    rospy.Subscriber(rospy.get_namespace()+"set_point", set_point, callback)
    rate = rospy.Rate(10)
    rospy.on_shutdown(stop)
 
    while not rospy.is_shutdown():
	kp = rospy.get_param("kp",1.3) 
	#0.8 para senoidal
	#1.4 para la cuadrada
	
	ki = rospy.get_param("ki",2.4)
	#0.8 para senoidal
	#2.2 para la cuadrada	

	kd = rospy.get_param("kd",0.02)

	tau = rospy.get_param("tau",0.1) 
	R = rospy.get_param("R",0.1) 
	dt = rospy.get_param("dt",0.4) 

	error = signal_data - original_signal
        error_acumulado += error * dt  ## multiplicar por "dt"
	error_res= (error - error_anterior)/ dt
 

        accion_proporcional = kp * error
        accion_integral = ki * error_acumulado
        accion_derivativa = kd * error_res

        accion_control = accion_proporcional + accion_integral + accion_derivativa +1.5
        u = accion_control * R
        velocidad = ultima_medicion + ((u - ultima_medicion))
        #pub.publish(Float32(velocidad))
        ultima_medicion = velocidad
        signal=ultima_medicion
        error_anterior = error



	print_info = "%3f | %3f | %3f| %3f| %3f" %(signal_data,time_data,original_signal,signal,error)
        rospy.loginfo(print_info)
        #signal = signal_data
        time = time_data
        pub_signal.publish(signal)
	pub_time.publish(time)
	rate.sleep()
