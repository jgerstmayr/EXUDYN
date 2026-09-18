#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python example how to use ROS and EXUDYN
#
# Details:  This example shows how to use a ROS publisher to send command velocity 
#           prerequisite to be able to use together with exudyn see rosInterface.py: 
#
# use a bash terminal to start the roscore: 
#           roscore 
# start the exudyn simulation (ROSTurtle.py)
# run this to send cmd_vel data to the exudyn simulation 
#           for more functionality see also: ROSMassPoint.py, ROSEBringupTurtle.launch, ROSControlTurtleVelocity.py
# Author:   Martin Sereinig, Peter Manzl 
# Date:     2023-05-31 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. 
# You can redistribute it and/or modify it under the terms of the Exudyn license. 
# See 'LICENSE.txt' for more details.q
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import numpy as np 
# import rospy for ROS communication
import rospy
# import standart messages and services  
from geometry_msgs.msg import Twist   
from std_srvs.srv import Empty


#**class: class to establish a ROS communication 
#**author: Martin Sereinig
#**notes: used to build ROS publisher
class TurtleBotCommander:
    def __init__(self):
        self.forwardSpeed = 2
        self.forwardTime = 1.0
        self.turnSpeed = np.pi/2
        self.turnTime = 2.0
        #   turtle1/cmd_vel to move the ROS turtlesim vizualisation
        #   /cmd_vel to move the EXUDYN turtle simulation (exuROSexample3DTurtle.py) 
        self.pubTopic = '/cmd_vel'     # define topic
        self.vTalkerPub = rospy.Publisher(self.pubTopic,Twist,queue_size=10)  # define publisher for topic name and message type 
        return
    

    #**classFunction: function to create a ROS publisher
    #**input:
    #       turtleSpeed: desired velocity as list [vx,vy,wz]
    #**author: Martin Sereinig
    def VelocityTalker(self,turtleSpeed=[0,0,0]):
        self.turtleLog = "Speed:{} at time {}".format(turtleSpeed,rospy.get_time() )
        self.turtleTwistVariable = Twist()    # initialize variable defined by standard message Twist(), it will be initialized with zero
        self.turtleTwistVariable.linear.x = turtleSpeed[0]
        self.turtleTwistVariable.linear.y = turtleSpeed[1]
        self.turtleTwistVariable.angular.z = turtleSpeed[2]
        rospy.loginfo(self.turtleLog)  # ros print 
        self.vTalkerPub.publish(self.turtleTwistVariable)  # publish of new velocity on topic defined in publisher 
        return True

    #**classFunction: function to create a ROS service call to reset the turtlesim if needed
    #**author: Martin Sereinig
    #**notes: only used when ROS turtlesim_node is used 
    def resetService(self):
        rospy.wait_for_service('reset')  # wait for existing service 
        try:
            self.resetTurtle = rospy.ServiceProxy('reset',Empty)   # generate a service call
            self.resetTurtle()  # do the actual service call
            rospy.sleep(0.5)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            return False 


if __name__ == '__main__':
    # initialize ROS node 
    rospy.init_node('VelocityTalker', anonymous=True)
    # create myTurtle object
    myTurtle = TurtleBotCommander()
    rospy.sleep(1)
    print('node and object initialized')
    stopLoop = 'r'
    try:
        i = 0
        # move in a square with turtle 
        # modify this to command the turtle for different movements
        while not rospy.is_shutdown() and i<4 and stopLoop != 'q':
            # forward
            myTurtle.VelocityTalker(turtleSpeed = [myTurtle.forwardSpeed,0,0])
            rospy.sleep(myTurtle.forwardTime)
            # turn
            myTurtle.VelocityTalker(turtleSpeed = [0,0,myTurtle.turnSpeed])
            rospy.sleep(myTurtle.turnTime)
            i = i+1
            if i==4:
                myTurtle.VelocityTalker(turtleSpeed=[0,0,0])
                print('stop turtle (q) or redo (r)')
                stopLoop = input()
                if stopLoop == 'r': i = 0
        # stop turtle
        # reset turtle if used with ROS-turtlesim
        if myTurtle.pubTopic != '/cmd_vel':
            myTurtle.resetService()
    except rospy.ROSInitException:
        pass


