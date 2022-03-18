#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class LoadMassProportional
#+++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

exu.Print("start mini example for class LoadMassProportional")
try: #puts example in safe environment
    node = mbs.AddNode(NodePoint(referenceCoordinates = [1,0,0]))
    body = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=2))
    mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=body))
    mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=[0,0,-9.81]))

    #assemble and solve system for default parameters
    mbs.Assemble()
    exu.SolveDynamic(mbs)

    #check result
    exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]
    #final z-coordinate of position shall be -g/2 due to constant acceleration with g=-9.81
    #result independent of mass

except BaseException as e:
    exu.Print("An error occured in test example for LoadMassProportional:", e)
else:
    exu.Print("example for LoadMassProportional completed, test result =", exudynTestGlobals.testResult)

