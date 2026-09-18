#!/usr/bin/env python3
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python example how to use ROS and EXUDYN
#
# Details:  This example shows how to communicate between an exudyn simulation
#			and ROS
#           To make use of this example, you need to 
#           install ROS (ROS1 noetic) including rospy (see rosInterface.py)
#           prerequisite to use: 
#           use a bash terminal to start the roscore with: 
#               roscore 
#           then run the simulation:
#               python 3 ROSMobileManipulator.py
#           you can use the prepared ROS node, ROSControlMobileManipulator to control the simulation
#           use a bash terminal to start the recommended file:
#               python3 ROSControlMobileManipulator.py
#           for even more ROS functionality create a ROS package (e.q. myExudynInterface) in a catkin workspace, 
#           copy files ROSMobileManipulator.py, bodykairos.stl and ROSControlMobileManipulator.py in corresponding folders within the package
#           for more functionality see also: ROSMassPoint.py, ROSBringupTurtle.launch, ROSControlVelocity.py from the EXUDYN examples folder
# Author:   Martin Sereinig
# Date:     2023-07-15 (created)
# last Update: 2023-09-11
# Copyright:This file is part of Exudyn. Exudyn is free software. 
# You can redistribute it and/or modify it under the terms of the Exudyn license. 
# See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# general imports
import numpy as np 
from exudyn.utilities  import *

# ros imports
import rospy
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from std_msgs.msg import String

#**class: class to establish a ROS communication 
#**author: Martin Sereinig
#**notes: used to build ROS publisher
class MobileManipulatorCommander:
    def __init__(self):
        self.forwardSpeed = 1.0
        self.forwardTime = 1.0
        self.sidewardSpeed = 1.0
        self.sidewardTime = 1.0
        self.turnSpeed = np.pi/2
        self.turnTime = 1.0
        
        self.vPubTopic = '/cmd_vel'     # define topic
        self.vTalkerPub = rospy.Publisher(self.vPubTopic,Twist,queue_size=10)  # define publisher for topic name and message type 
        
        self.sPubTopic = '/my_string'
        self.sTalkerPub = rospy.Publisher(self.sPubTopic,String,queue_size=10)
        
        self.pPubTopic = '/my_pose'
        self.pTalkerPub = rospy.Publisher(self.pPubTopic,Pose,queue_size=10)
        return

    #**classFunction: function to create a ROS twist publisher
    #**input:
    #       platformSpeed: desired velocity as list [vx,vy,wz]
    #**author: Martin Sereinig
    def VelocityTalker(self,platformSpeed=[0,0,0]):
        self.mobileManipulatorLog = "Speed:{} at time {}".format(platformSpeed,rospy.get_time() )
        self.mobileManipulatorTwistVariable = Twist()    # initialize variable defined by standard message Twist(), it will be initialized with zero
        self.mobileManipulatorTwistVariable.linear.x = platformSpeed[0]
        self.mobileManipulatorTwistVariable.linear.y = platformSpeed[1]
        self.mobileManipulatorTwistVariable.angular.z = platformSpeed[2]
        rospy.loginfo(self.mobileManipulatorLog)  # ros print 
        self.vTalkerPub.publish(self.mobileManipulatorTwistVariable)  # publish of new velocity on topic defined in publisher 
        return True

    #**classFunction: function to create a ROS string publisher
    #**input:
    #       controlString: control string as string
    #**author: Martin Sereinig
    def StringTalker(self,controllerString=''):
        self.controllerString = controllerString
        self.sTalkerPub.publish(self.controllerString)  # publish controller string
        return True

    #**classFunction: function to create a ROS Pose publisher
    #**input:
    #       desiredArmPose: desired arm pose as list [x,y,z,qx,qy,qz,qw]
    #**author: Martin Sereinig
    def PoseTalker(self,desiredArmPose=[0,0,0,0,0,0,1]):
        self.desiredArmPose = Pose()    # initialize variable defined by standard message 
        self.desiredArmPose.position.x = desiredArmPose[0]
        self.desiredArmPose.position.y = desiredArmPose[1]
        self.desiredArmPose.position.z = desiredArmPose[2]
        self.desiredArmPose.orientation.x = desiredArmPose[3]
        self.desiredArmPose.orientation.y = desiredArmPose[4]
        self.desiredArmPose.orientation.z = desiredArmPose[5]
        self.desiredArmPose.orientation.w = desiredArmPose[6]
        self.pTalkerPub.publish(self.desiredArmPose)
        return True

# main function
if __name__ == '__main__':
    # initialize ROS node 
    rospy.init_node('Control_Mobile_Manipulator', anonymous=True)
    # create myTurtle object
    MobileManipulatorKairos = MobileManipulatorCommander()
    rospy.sleep(1)
    print('node and object initialized')
    controllerString = 'r'
    try:
        while not rospy.is_shutdown() and controllerString != 'q':
            print('Get controller string: \n (q)...quit \n (mk)...move by external node \n (ms)...move in square \n (a)...arm   ')
            controllerString = input()
            MobileManipulatorKairos.StringTalker(controllerString)
            if controllerString=='q': 
                break
            elif controllerString == 'mk': 
                menueHelper = 'g'
                while menueHelper!='e':
                    print('please use any node to send /cmd_vel commands ')
                    print('for keyboard use teleop_twist_keyboard package (has to be installed)')
                    print ('to return into menue press: e')
                    menueHelper = input()
                
            elif controllerString == 'a': 
                print('move with arm')
                # send arm position to ROS
                armPosition = [0.5,0.5,0.5]
                armRotation = RotationMatrixY(pi/2)
                eulerRot = RotationMatrix2EulerParameters(armRotation)       
                armOrientationQ = [eulerRot[1],eulerRot[2],eulerRot[3],eulerRot[0]]
                MobileManipulatorKairos.PoseTalker(desiredArmPose = armPosition + armOrientationQ)
                rospy.sleep(0.5)
                controllerString = 'r'

            elif controllerString == 'ms':
                # move in square
                for item in [1,-1]:
                    MobileManipulatorKairos.VelocityTalker(platformSpeed = [item*MobileManipulatorKairos.forwardSpeed,0,0])
                    rospy.sleep(MobileManipulatorKairos.forwardTime)
                    MobileManipulatorKairos.VelocityTalker(platformSpeed = [0,0,0])
                    rospy.sleep(0.5)

                    MobileManipulatorKairos.VelocityTalker(platformSpeed = [0,item*MobileManipulatorKairos.sidewardSpeed,0])
                    rospy.sleep(MobileManipulatorKairos.sidewardTime)
                    MobileManipulatorKairos.VelocityTalker(platformSpeed = [0,0,0])
                    rospy.sleep(0.5)

                controllerString = 'r'
                MobileManipulatorKairos.StringTalker(controllerString)
            else:
                MobileManipulatorKairos.VelocityTalker(platformSpeed = [0,0,0])
                print('no valid control string received')
    except rospy.ROSInitException:
                controllerString = 'q'
                print('ROS Init Exception')
                MobileManipulatorKairos.VelocityTalker(platformSpeed = [0,0,0])


