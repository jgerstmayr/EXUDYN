#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectConnectorCoordinateSpringDamper
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

testError=1 #set default error, if failed
exu.Print("start mini example for class ObjectConnectorCoordinateSpringDamper")
try: #puts example in safe environment
    def springForce(mbs, t, itemNumber, u, v, k, d, offset, dryFriction, dryFrictionProportionalZone):
        return 0.1*k*u+k*u**3+v*d

    nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
    
    #Spring-Damper between two marker coordinates
    mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                         stiffness = 5000, damping = 80, springForceUserFunction = springForce)) 
    loadCoord = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, load = 1)) #static linear solution:0.002

    #assemble and solve system for default parameters
    mbs.Assemble()
    exu.SolveDynamic(mbs)

    #check result at default integration time
    exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[0]

except BaseException as e:
    exu.Print("An error occured in test example for ObjectConnectorCoordinateSpringDamper:", e)
else:
    exu.Print("example for ObjectConnectorCoordinateSpringDamper completed, test error =", testError)

