#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectRigidBody2D
#+++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../../bin/WorkingRelease')
sys.path.append('../TestModels')
from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import exudyn as exu
from exudynUtilities import *
from itemInterface import *
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

testError=1 #set default error, if failed
print("start mini example for class ObjectRigidBody2D")
try: #puts example in safe environment
    node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = [1,1,0.25*np.pi], 
                                       initialCoordinates=[0.5,0,0],
                                       initialVelocities=[0.5,0,0.75*np.pi]))
    mbs.AddObject(RigidBody2D(nodeNumber = node, physicsMass=1, physicsInertia=2))

    #assemble and solve system for default parameters
    mbs.Assemble()
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', exu.SimulationSettings())

    #check result
    testError = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0] - 2
    testError+= mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[2] - 0.75*np.pi
    #final x-coordinate of position shall be 2, angle theta shall be np.pi

except BaseException as e:
    print("An error occured in test example for ObjectRigidBody2D:", e)
else:
    print("example for ObjectRigidBody2D completed, test error =", testError)

