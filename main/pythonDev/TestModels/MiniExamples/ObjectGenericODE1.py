#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectGenericODE1
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

exu.Print("start mini example for class ObjectGenericODE1")
try: #puts example in safe environment
    #set up a 2-DOF system
    nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                        initialCoordinates=[1,0],
                                        numberOfODE1Coordinates=2))

    #build system matrix and force vector
    #undamped mechanical system with m=1, K=100, f=1
    A = np.array([[0,1],
                  [-100,0]])
    b = np.array([0,1])
    
    oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1], 
                                                   systemMatrix=A, 
                                                   rhsVector=b))
    
    #assemble and solve system for default parameters
    mbs.Assemble()
    
    sims=exu.SimulationSettings()
    solverType = exu.DynamicSolverType.RK44
    mbs.SolveDynamic(solverType=solverType, simulationSettings=sims)

    #check result at default integration time
    exudynTestGlobals.testResult = mbs.GetNodeOutput(nODE1, exu.OutputVariableType.Coordinates)[0]

except BaseException as e:
    exu.Print("An error occured in test example for ObjectGenericODE1:", e)
else:
    exu.Print("example for ObjectGenericODE1 completed, test result =", exudynTestGlobals.testResult)

