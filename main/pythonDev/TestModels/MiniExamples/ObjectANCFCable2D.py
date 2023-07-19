#+++++++++++++++++++++++++++++++++++++++++++
# Mini example for class ObjectANCFCable2D
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

exu.Print("start mini example for class ObjectANCFCable2D")
try: #puts example in safe environment
    rhoA = 78.
    EA = 1000000.
    EI = 833.3333333333333
    cable = Cable2D(physicsMassPerLength=rhoA, 
                    physicsBendingStiffness=EI, 
                    physicsAxialStiffness=EA, 
                    )

    ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                    positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                    numberOfElements=32, #converged to 4 digits
                    cableTemplate=cable, #this defines the beam element properties
                    massProportionalLoad = [0,-9.81,0],
                    fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                    )
    lastNode = ancf[0][-1]

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveStatic()

    #check result
    exudynTestGlobals.testResult = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)[0]
    #ux=-0.5013058140308901

except BaseException as e:
    exu.Print("An error occured in test example for ObjectANCFCable2D:", e)
else:
    exu.Print("example for ObjectANCFCable2D completed, test result =", exudynTestGlobals.testResult)

