#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectConnectorCoordinate
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
print("start mini example for class ObjectConnectorCoordinate")
try: #puts example in safe environment
    def OffsetUF(t, lOffset): #gives 0.05 at t=1
        return 0.5*(1-np.cos(2*3.141592653589793*0.25*t))*lOffset

    nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
    
    #Spring-Damper between two marker coordinates
    mbs.AddObject(CoordinateConstraint(markerNumbers = [groundMarker, nodeMarker], 
                                       offset = 0.1, offsetUserFunction = OffsetUF)) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', exu.SimulationSettings())

    #check result at default integration time
    testError = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[0] - 0.049999999999272404

except BaseException as e:
    print("An error occured in test example for ObjectConnectorCoordinate:", e)
else:
    print("example for ObjectConnectorCoordinate completed, test error =", testError)

