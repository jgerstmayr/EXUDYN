#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectMass1D
#+++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')

import exudyn as exu
from exudyn.utilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

exu.Print("start mini example for class ObjectMass1D")
try: #puts example in safe environment
    node = mbs.AddNode(Node1D(referenceCoordinates = [1], 
                              initialCoordinates=[0.5],
                              initialVelocities=[0.5]))
    mass = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result, get current mass position at local position [0,0,0]
    exudynTestGlobals.testResult = mbs.GetObjectOutputBody(mass, exu.OutputVariableType.Position, [0,0,0])[0]
    #final x-coordinate of position shall be 2

except BaseException as e:
    exu.Print("An error occured in test example for ObjectMass1D:", e)
else:
    exu.Print("example for ObjectMass1D completed, test result =", exudynTestGlobals.testResult)

