#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectConnectorDistance
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
print("start mini example for class ObjectConnectorDistance")
try: #puts example in safe environment
    #example with 1m pendulum, 50kg under gravity
    nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[1,0]))
    oMass = mbs.AddObject(MassPoint2D(physicsMass = 50, nodeNumber = nMass))
    
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
    oDistance = mbs.AddObject(DistanceConstraint(markerNumbers = [mGround, mMass], distance = 1))
    
    mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -50*9.81, 0])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    
    sims=exu.SimulationSettings()
    sims.timeIntegration.generalizedAlpha.spectralRadius=0.7
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', sims)

    #check result at default integration time
    testError = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0] - (-0.9845225086606828)

except BaseException as e:
    print("An error occured in test example for ObjectConnectorDistance:", e)
else:
    print("example for ObjectConnectorDistance completed, test error =", testError)

