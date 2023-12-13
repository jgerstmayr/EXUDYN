#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectRigidBody2D
#+++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')
sys.path.append('../../TestModels') #for direct run in directory

import exudyn as exu
from exudyn.utilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

exu.Print("start mini example for class ObjectRigidBody2D")
try: #puts example in safe environment
    node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = [1,1,0.25*np.pi], 
                                       initialCoordinates=[0.5,0,0],
                                       initialVelocities=[0.5,0,0.75*np.pi]))
    mbs.AddObject(RigidBody2D(nodeNumber = node, physicsMass=1, physicsInertia=2))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result
    exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
    exudynTestGlobals.testResult+= mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[2]
    #final x-coordinate of position shall be 2, angle theta shall be np.pi

except BaseException as e:
    exu.Print("An error occured in test example for ObjectRigidBody2D:", e)
else:
    exu.Print("example for ObjectRigidBody2D completed, test result =", exudynTestGlobals.testResult)

