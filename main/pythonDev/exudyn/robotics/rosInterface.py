#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  This interface collects interfaces and functionality for ROS comunication
#           This library is under construction (2023-05);
#           To make use of this libraries, you need to 
#           install ROS (ROS1 noetic) including rospy
#           Please consider following workflow:
#           make sure to have a working ROS1-NOETIC installation, ROS2 is not supported yet
#           tested only with ROS1-NOETIC, ubuntu 20.04, and Python 3.8.10
#           you find all ROS1 installation steps on: 
#           http://wiki.ros.org/noetic/Installation/Ubuntu
#           Step 1.4 we recommend to install: (sudo apt install ros-noetic-desktop)
#           Check the installation of the turtlesim package (rosrun turtlesim turtlesim\_node )
#           if not installed: sudo apt install ros-noetic-turtlesim
#           use a catkin workspace and build a ROS1 Package  
#           Follow instructions on:
#           http://wiki.ros.org/ROS/Tutorials (recommend go trough step 1 to 6)
#           Minimal example to use:
#           create catkin workspace: 
#               mkdir -p ~/catkin\_ws/src
#               cd ~/catkin\_ws
#               catkin\_make
#               source devel/setup.bash
#           build ROS package:
#               cd ~/catkin\_ws/src
#               catkin\_create\_pkg my\_pkg\_name rospy roscpp std\_msgs geometry\_msgs sensor\_msgs 
#           build catkin workspace and sourcing setup file
#               cd ~/catkin\_ws
#               cakin\_make
#               source ~/catkin\_ws/devel/setup.bash
#           for more functionality see also: ROSExampleMassPoint.py, ROSExampleBringup.launch, ROSExampleControlVelocity.py
# Author:   Martin Sereinig, Peter Manzl 
# Date:     2023-05-31 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. 
# You can redistribute it and/or modify it under the terms of the Exudyn license. 
# See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn as exu
from exudyn.utilities import *
import time
import os

# import needed ROS modules and messages
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, Twist
from std_msgs.msg import Float64MultiArray, Empty, String, Time

#**class: interface super class to establish a ROS Exudyn interface 
#           see specific class functions which can be used and extended 
#           by inheritance with class MyClass(ROSInterface)
#**author: Martin Sereinig, Peter Manzl 
#**notes: some inspiration can be taken from 
class ROSInterface: 
    def __init__(self, name = 'ExudynRobotInterface'): 

        # check ROS version 
        self.CheckROSversion()

        # init ros node
        rospy.init_node(name, anonymous=True)        

        # topics namespace, will be used in all topics
        self.topicBase = '/exudyn/'

        # set ROS node start time Time 
        self.t0 = rospy.get_time()
        print('ROS node initialization at time {} done!'.format(self.t0))
        # initialize simulation runtime
        self.tsim = self.t0 -  rospy.get_time()

        # initialization of some timing variables
        self.rosPoseSendInterval = 0.01  #ms
        self.rosTwistSendInterval = 0.01 #ms
        self.lastPoseSendTime = -self.rosPoseSendInterval
        self.lastTwistSendTime = -self.rosTwistSendInterval
        self.systemStateUpdateInterval = 0.01  #ms
        self.lastSystemStateUpdateTime = -self.systemStateUpdateInterval
        self.lastStepTime = 0    # last step time (exudyn time)
        
        # initialize standard publisher
        # publisher for time message 
        self.exuTimePublisher =  self.InitPublisher(pubTopicName='SimiulationTime', 
                                        pubType = Time, queueSize = 10)
        # publisher for simple string message 
        self.exuStringPublisher =  self.InitPublisher(pubTopicName='SimpleString', 
                                        pubType = String, queueSize = 10)
        # publisher for velocities (Twist)
        self.exuTwistPublisher =  self.InitPublisher(pubTopicName='Twist', 
                                        pubType = Twist, queueSize = 10)
        # publisher for Pose
        self.exuPosePublisher =  self.InitPublisher(pubTopicName='Pose', 
                                        pubType = PoseStamped, queueSize = 10)
        # publisher for Wrench
        self.exuWrenchPublisher =  self.InitPublisher(pubTopicName='Wrensh', 
                                        pubType = WrenchStamped, queueSize = 10)
        # publisher for system data
        self.exuSystemstatePublisher =  self.InitPublisher(pubTopicName='Systemstate', 
                                        pubType = Float64MultiArray, queueSize = 1)
        # add further publisher if needed in inherited class
        # initialization of subscriber and needed callback function will be done in inherited class   
        # self.callbackData =
        return
    #**classFunction: function to create a publisher
    #**input:
    #       pubTopicName: topic name to publish, actual topic will be /exudyn/pubTopicName
    #       pubType: data type used in topic
    #       queSize: length of queue to hold messages, should be as small as sending frequency (= simulation sample time)
    #**author: Martin Sereinig
    #**notes: find msgs types here
    #  http://docs.ros.org/en/melodic/api/std\_msgs/html/index-msg.html
    #**examples: 
    #       publisher for poses, pubType = PoseStamped, 
    #       publisher for system data, pubType = Float64MultiArray,
    #       publisher for filtered force, pubType = WrenchStamped,
    #       publisher for velocities, pubType = Twist,
    def InitPublisher(self, pubTopicName='', pubType = Empty, queueSize = 10): 
        exuPublisher = rospy.Publisher(self.topicBase  + pubTopicName, 
                            pubType, queue_size=queueSize)
        return exuPublisher 
    
    #**classFunction: function to create a generic callback function for a subscriber
    #**input:
    #       topic: topic name generated by init Subscriber
    #       data: data structure for regarding individual topic 
    #**author: Peter Manzl 
    def ExuCallbackGeneric(self,subTopicName, data): 
        setattr(self, subTopicName, data)
        return True
    
    #**classFunction: function to create a subscriber
    #**input:
    #       subTopicNameSpace: topic namespace: 'exudyn/'
    #       subTopicName: topic name to subscribe
    #       subType: data type for topic to subscribe
    #**author: Peter Manzl 
    #**note: callback function will be automatic generated for each subscriber, depending
    #        on subTopicName. Data will be found under self.subTopicName
    def InitSubscriber(self,subTopicNameSpace, subTopicName, subType): 
        exuSubscriber = rospy.Subscriber(subTopicNameSpace+subTopicName, subType, 
                                    lambda data: self.ExuCallbackGeneric(subTopicName, data))
        return exuSubscriber

    #**classFunction: check the current used ROS version
    #**author: Martin Sereinig
    #**note: just supports ROS1, ROS2 support will be given in future releases 
    def CheckROSversion(self):
        # check and set ROSVersion 
        #os.system('rosversion -d')
        #self.myROSversionString = os.popen('rosversion -d').read()
        self.myROSversionEnvInt = int(os.getenv('ROS_VERSION','0'))
        self.myROSdistriEnv = os.getenv('ROS_DISTRO','unknown')
        rospy.loginfo('-ROS'+str(self.myROSversionEnvInt)+' '+self.myROSdistriEnv+'-')  
        if self.myROSversionEnvInt != 0:
            return True
        else:
            return False        

    #**classFunction: Example method to be called once per frame/control cycle in Exudyn PreStepUserFunction
    #**note:        reads sensor values, creates message, publish and subscribe to ROS 
    #**input:
    #  mbs:     mbs (exudyn.exudynCPP.MainSystem), multi-body simulation system from exudyn
    #  tExu:    tExu (float), elapsed time since simulation start
    #  getData: getData (string), get pose information from 'node' or from 'sensor'
    #**author: Martin Sereinig
    #**notes: 
    #           reads sensor values, creates message, publish and subscribe to ROS
    #           publishing each and every step is too much, this would slow down the connection
    #           thus: publish every few seconds, only
    #           furthermore, as vrInterface is only updating the graphics with f=60Hz, we don't have to update
    #           system state every 1ms, so with f=1000Hz. Instead f=60Hz equivalents to update every 1/60=17ms
    #           timing variable to know when to send new command to robot or when to publish new mbs system state update
    def PublishPoseUpdate(self, mbs, tExu, getData = 'node'):
        if tExu - self.lastPoseSendTime >= self.rosPoseSendInterval:
            self.lastPoseSendTime = tExu
            if getData == 'sensor':
                # read current kinematic state and orientation from predefined variables send via mbs.variables
                posFromExu = mbs.GetSensorValues(mbs.variables['pos'])
                oriFromExu = mbs.GetSensorValues(mbs.variables['ori'])
                # convert data to numpy arrays
                pos = np.array(posFromExu)
                rot = np.array(oriFromExu)
                rotE = np.roll(RotXYZ2EulerParameters(rot),-1)  # use roll to meet ROS eulerparameter convention [x,y,z,w]

            elif getData == 'node':
                nodeDic = mbs.GetNode(mbs.variables['nodeNumber'])
                if nodeDic['nodeType'] == 'RigidBodyEP':
                    # get position [x,y,z] and orientation (eulerparameter) [w,x,y,z] from exudyn via node coordinates 
                    # nodeCoordinates = [x,y,z, w,x,y,z]
                    nodeCoordinates = list(np.array(nodeDic['referenceCoordinates']) + 
                                        np.array(mbs.GetNodeOutput(mbs.variables['nodeNumber'],variableType = exu.OutputVariableType.Coordinates)))
                    # reformulation for ROS 
                    pos = nodeCoordinates[0:3] # pos = [x,y,z]
                    rotE = np.roll(nodeCoordinates[3:7],-1) # rotE = [x,y,z,w]

                    if False: # for debug
                        print('position: ',nodeCoordinates[0:3])
                        print('orientation: ',nodeCoordinates[3:7]) # eulerparameter exu [w,x,y,z]
                else: 
                    print('node type not supported')
            else:
                print('Error! Please choose how to get pose information!')
            
            
            poseExu = np.append(pos,rotE)
            # compose message and publish
            msgData = PoseStamped()
            # postion
            msgData.pose.position.x = poseExu[0]
            msgData.pose.position.y = poseExu[1]
            msgData.pose.position.z = poseExu[2]
            # orientation given in unit quarternions
            msgData.pose.orientation.x = poseExu[3]
            msgData.pose.orientation.y = poseExu[4]
            msgData.pose.orientation.z = poseExu[5]
            msgData.pose.orientation.w = poseExu[6]
            # write current time into message
            msgData.header.stamp = rospy.Time.now()
            self.exuPosePublisher.publish(msgData)



    #**classFunction: Example method to be called once per frame/control cycle in Exudyn PreStepUserFunction
    #**note:        reads sensor values, creates message, publish and subscribe to ROS 
    #**input:
    #  mbs:     mbs (exudyn.exudynCPP.MainSystem), multi-body simulation system from exudyn
    #  tExu:    tExu (float), elapsed time since simulation start
    #  getData: getData (string), get pose information from 'node' or from 'sensor'
    #**author: Martin Sereinig
    #**notes: 
    #           reads sensor values, creates message, publish and subscribe to ROS
    def PublishTwistUpdate(self, mbs, tExu, getData='node'):
        if tExu - self.lastTwistSendTime >= self.rosTwistSendInterval:
            self.lastTwistSendTime = tExu

            if getData == 'sensor':
                # read current kinematic state and orientation from predefined variables
                velocityLinearFromExu = mbs.GetSensorValues(mbs.variables['velt'])
                velocityAngularFromExu = mbs.GetSensorValues(mbs.variables['velr'])
                # convert data to numpy arrays
                velLin = np.array(velocityLinearFromExu)
                velAng = np.array(velocityAngularFromExu)

            elif getData == 'node':
                nodeDic = mbs.GetNode(mbs.variables['nodeNumber'])
                if nodeDic['nodeType'] == 'RigidBodyEP':
                    # get linear and angular velocity form exudyn via node 
                    velLin= mbs.GetNodeOutput(mbs.variables['nodeNumber'],variableType = exu.OutputVariableType.Velocity)
                    velAng = mbs.GetNodeOutput(mbs.variables['nodeNumber'],variableType = exu.OutputVariableType.AngularVelocity)
                    # same could be done to get acceleration from exudyn node 
                    # nUIPLinearAcc= mbs.GetNodeOutput(nUIP,variableType = exu.OutputVariableType.Acceleration)
                    # nUIPAngularAcc= mbs.GetNodeOutput(nUIP,variableType = exu.OutputVariableType.AngularAcceleration)
            
            twistExu = np.append(velLin,velAng)
            # compose message and publish
            msgData = Twist()
            # linear velocities
            msgData.linear.x = twistExu[0]
            msgData.linear.y = twistExu[1]
            msgData.linear.z = twistExu[2]
            # angular velocities
            msgData.angular.x = twistExu[3]
            msgData.angular.y = twistExu[4]
            msgData.angular.z = twistExu[5]
            # write current time into message
            self.exuTwistPublisher.publish(msgData)

    #**classFunction: method to be send system state data once per frame/control cycle in Exudyn PreStepUserFunction
    #**input:
    #  mbs:     mbs (exudyn.exudynCPP.MainSystem), multi-body simulation system from exudyn
    #  tExu:       tExu (float),  simulation time
    # systemStateData:   systemStateData (list), full Exudyn SystemState
    #**author: Martin Sereinig
    #**note:        collects important exudyn system data and send it to ros-topic
    def PublishSystemStateUpdate(self, mbs, tExu):
        if tExu - self.lastSystemStateUpdateTime >= self.systemStateUpdateInterval:
            self.lastSystemStateUpdateTime = tExu
            
            systemStateList1d = []
            # publish system state to ros-topic 
            systemStateData = mbs.systemData.GetSystemState()

            # first entry is current time
            systemStateList1d.append(tExu)

            # then systemData itself follows
            for array in systemStateData:
                # add length of next array
                systemStateList1d.append(float(len(array)))
                # add array itself
                for i in range(len(array)):
                    systemStateList1d.append(array[i])

            msg = Float64MultiArray()
            msg.data = systemStateList1d # dataList
            self.exuSystemstatePublisher.publish(msg)


