#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectConnectorTorsionalSpringDamper
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

exu.Print("start mini example for class ObjectConnectorTorsionalSpringDamper")
try: #puts example in safe environment
    #example with rigid body at [0,0,0], with torsional load
    k=2e3
    nBody = mbs.AddNode(RigidRxyz())
    oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                    nodeNumber=nBody))
    
    mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                            localPosition = [0,0,0]))
    mbs.AddObject(RevoluteJointZ(markerNumbers = [mGround, mBody])) #rotation around ground Z-axis
    mbs.AddObject(TorsionalSpringDamper(markerNumbers = [mGround, mBody], 
                                        stiffness = k, damping = k*0.01, offset = 0))

    #torque around z-axis; expect approx. phiZ = 1/k=0.0005
    mbs.AddLoad(Torque(markerNumber = mBody, loadVector=[0,0,1])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    
    #check result at default integration time
    exudynTestGlobals.testResult = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Rotation)[2]

except BaseException as e:
    exu.Print("An error occured in test example for ObjectConnectorTorsionalSpringDamper:", e)
else:
    exu.Print("example for ObjectConnectorTorsionalSpringDamper completed, test result =", exudynTestGlobals.testResult)

