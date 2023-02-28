#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from pid_control.msg import set_point
from pid_control.msg import motor_input
from pid_control.msg import motor_output

kp = rospy.get_param("kp",5) 
ki = rospy.get_param("ki",0.1) 
tau = rospy.get_param("tau",6.28) 
R = rospy.get_param("R",0.01) 
dt = rospy.get_param("dt",5) 

signal_data = 0.0
time_data = 0.0

output_data = 0.0
time_data2 = 0.0

error = 0.0
angularVelocity = 0.0 

error_acumulado = 0.0
ultima_medicion = 0.0


msg = motor_input()
msg.time = 0.0
msg.input = 0.0


def callback(msg):
    global signal_data, time_data
    signal_data = msg.signal_y
    time_data = msg.time_x 

def callback2(msg):
    global output_data, time_data2
    output_data = msg.output
    time_data2 = msg.time 

def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    pub=rospy.Publisher("motor_input",motor_input, queue_size=10)
    pub_controlador=rospy.Publisher("control",Float32, queue_size=10)
    rospy.init_node("controller")
    rospy.Subscriber(rospy.get_namespace()+"set_point", set_point, callback)
    rospy.Subscriber(rospy.get_namespace()+"motor_output", motor_output, callback2)
    rate = rospy.Rate(10)
    rospy.on_shutdown(stop)
 
    while not rospy.is_shutdown():

	error = 0.0 - signal_data
        error_acumulado += error * dt  ## multiplicar por "dt"
        accion_proporcional = kp * error
        accion_integral = ki * error_acumulado
        accion_control = accion_proporcional + accion_integral
        u = accion_control * R
        velocidad = ultima_medicion + ((u - ultima_medicion) / tau)
        #pub.publish(Float32(velocidad))
        ultima_medicion = velocidad
        signal=ultima_medicion

	msg.time = time_data
        msg.input = signal
	pub.publish(msg)
	pub_controlador.publish(signal)
        print_info = "%3f | %3f" %(signal,time_data)
        rospy.loginfo(print_info)
        rate.sleep()
