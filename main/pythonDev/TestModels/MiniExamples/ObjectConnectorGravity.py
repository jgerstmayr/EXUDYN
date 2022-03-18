#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectConnectorGravity
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

exu.Print("start mini example for class ObjectConnectorGravity")
try: #puts example in safe environment
    mass0 = 1e25
    mass1 = 1e3
    r = 1e5
    G = 6.6743e-11
    vInit = np.sqrt(G*mass0/r)
    tEnd = (r*0.5*np.pi)/vInit #quarter period
    node0 = mbs.AddNode(NodePoint(referenceCoordinates = [0,0,0])) #star
    node1 = mbs.AddNode(NodePoint(referenceCoordinates = [r,0,0], 
                                  initialVelocities=[0,vInit,0])) #satellite
    oMassPoint0 = mbs.AddObject(MassPoint(nodeNumber = node0, physicsMass=mass0))
    oMassPoint1 = mbs.AddObject(MassPoint(nodeNumber = node1, physicsMass=mass1))
    
    m0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node0))
    m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node1))
    
    mbs.AddObject(ObjectConnectorGravity(markerNumbers=[m0,m1],
                                         mass0 = mass0, mass1=mass1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    sims = exu.SimulationSettings()
    sims.timeIntegration.endTime = tEnd
    exu.SolveDynamic(mbs, sims, solverType=exu.DynamicSolverType.RK67)

    #check result at default integration time
    #expect y=x after one period of orbiting (got: 100000.00000000485)
    exudynTestGlobals.testResult = mbs.GetNodeOutput(node1, exu.OutputVariableType.Position)[1]

except BaseException as e:
    exu.Print("An error occured in test example for ObjectConnectorGravity:", e)
else:
    exu.Print("example for ObjectConnectorGravity completed, test result =", exudynTestGlobals.testResult)

