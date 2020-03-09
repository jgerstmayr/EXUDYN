#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectConnectorCartesianSpringDamper
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
print("start mini example for class ObjectConnectorCartesianSpringDamper")
try: #puts example in safe environment
    #example with mass at [1,1,0], 5kg under load 5N in -y direction
    k=5000
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[1,1,0]))
    oMass = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [1,1,0]))
    mbs.AddObject(CartesianSpringDamper(markerNumbers = [mGround, mMass], 
                                        stiffness = [k,k,k], 
                                        damping = [0,k*0.05,0], offset = [0,0,0]))
    mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -5, 0])) #static solution=-5/5000=-0.001m

    #assemble and solve system for default parameters
    mbs.Assemble()
    SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', exu.SimulationSettings())

    #check result at default integration time
    testError = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[1] - (-0.00099999999997058)

except BaseException as e:
    print("An error occured in test example for ObjectConnectorCartesianSpringDamper:", e)
else:
    print("example for ObjectConnectorCartesianSpringDamper completed, test error =", testError)

