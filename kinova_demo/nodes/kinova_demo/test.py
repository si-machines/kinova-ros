#! /usr/bin/env python
"""A test program to test action servers for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')

import actionlib

import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
import thread
from kinova_msgs.srv import *
import argparse
from robot_control_modules import *

prefix = 'j2s7s300_'
nbJoints = 7
interactive = True
duration_sec = 100


if __name__ == '__main__':
    try:        
        prefix, nbJoints = argumentParser(None)	
        rospy.init_node('torque_compensated_mode')
        if (interactive == True):        
            nb = raw_input("Moving robot to candle like position, and setting zero torques, press return to start, n to skip")
            if (nb != "n" and nb != "N"):
                result = joint_position_client([180, 180, 180, 180, 180, 180, 180], prefix)
                if (interactive == True):        
                    nb = raw_input('Setting torques to zero, press return')			
                    ZeroTorque(prefix)
                    
                if (interactive == True):
                    nb = raw_input('Starting gravity compensation mode')
                    #use service to set torque control parameters	
                    service_address = '/' + prefix + 'driver/in/set_torque_control_parameters'	
                    rospy.wait_for_service(service_address)
                        
                    #use service to switch to torque control	
                    service_address = '/' + prefix + 'driver/in/set_torque_control_mode'	
                    rospy.wait_for_service(service_address)
                    try:
                        switchTorquemode = rospy.ServiceProxy(service_address, SetTorqueControlMode)
                        switchTorquemode(1)           
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    
                    #subscriber to get feedback    
                    topic_name = '/' + prefix + 'driver/out/joint_state'
                    max_error = [0,0,0,0,0,0,0]
                    jointCmds = [0,0,0,0,0,0,0]
                    counter = [0]
                    sub = rospy.Subscriber(topic_name, JointState, getFeedbackCallback, (jointCmds,'torque',max_error,counter))
                    
                    #publish joint torque commands
                    topic_name = '/' + prefix + 'driver/in/joint_torque'
                    pub = rospy.Publisher(topic_name, kinova_msgs.msg.JointTorque, queue_size=1)
                    jointCmd = kinova_msgs.msg.JointTorque()
                    jointCmd.joint1 = jointCmds[0];
                    jointCmd.joint2 = jointCmds[1];
                    jointCmd.joint3 = jointCmds[2];
                    jointCmd.joint4 = jointCmds[3];
                    jointCmd.joint5 = jointCmds[4];
                    jointCmd.joint6 = jointCmds[5];
                    jointCmd.joint7 = jointCmds[6];
                    count = 0		
                    rate = rospy.Rate(100)
                    L = []
                    thread.start_new_thread(input_thread, (L,))
                    while (count<100*duration_sec):		
                        pub.publish(jointCmd)
                        count = count + 1
                        rate.sleep()
                        if L: break
                    sub.unregister()
                    print "max error %f %f %f %f %f %f %f" %(max_error[0], max_error[1], max_error[2], max_error[3], max_error[4], max_error[5], max_error[6])
                    
                    #use service to switch to position control	
                    try:           
                        switchTorquemode(0)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                
            print("Done!")
    except rospy.ROSInterruptException:
        print "program interrupted before completion"



