#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for ANCFCable2D test user functions
#
# Author:   Johannes Gerstmayr
# Date:     2023-12-13
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

from exudyn.beams import *
from math import atan

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

rhoA = 78.
EA = 100000.
EI = 2000

#example of bending moment user function
#limit bending moment with atan function
def bendingMomentUserFunction(mbs, t, itemNumber, axialPositionNormalized, curvature, curvature_t, curvatureRef, bendingStiffness, bendingDamping,
                                    axialStrain, axialStrain_t, axialStrainRef):
    #m = bendingStiffness*(curvature-curvatureRef) + bendingDamping*curvature_t #this is the linear, conventional case
    kappa=(curvature-curvatureRef)
    kappa = 0.1*atan(10*kappa) #nonlinear behavior, somehow like elasto-plastic
    return bendingStiffness*(kappa) + bendingDamping*curvature_t

#example of axial force user function
#reduce stiffness over time
def axialForceUserFunction(mbs, t, itemNumber, axialPositionNormalized, axialStrain, axialStrain_t, axialStrainRef, axialStiffness, axialDamping,
            curvature, curvature_t, curvatureRef):
    fact = max(0.02,(2-t**0.5)) #make axial stiffness it softer over time
    return fact*axialStiffness*(axialStrain-axialStrainRef) + axialDamping*axialStrain_t

#create ANCF cable object:
cable = ObjectANCFCable2D(massPerLength=rhoA, 
                bendingStiffness=EI, 
                bendingDamping = EI*0.1,
                axialStiffness=EA,
                axialDamping=EA*0.05,
                bendingMomentUserFunction=bendingMomentUserFunction,
                axialForceUserFunction=axialForceUserFunction,
                )

#create several cable elements
ancf=GenerateStraightLineANCFCable(mbs=mbs,
                positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                numberOfElements=16, #converged to 4 digits
                cableTemplate=cable, #this defines the beam element properties
                massProportionalLoad = [0,-9.81,0],
                fixedConstraintsNode0 = [1,1, 0,1],
                )

#assemble and solve system for default parameters
mbs.Assemble()

endTime = 0.5
stepSize = 5e-3

simulationSettings = exu.SimulationSettings()

simulationSettings.solution.file.write = False
simulationSettings.timeIntegration.verboseMode = 1 #turn off, because of lots of output
simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
# simulationSettings.show.computationTime = True
# simulationSettings.show.statistics = True

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True

SC.visualizationSettings.view0.window.renderWindowSize=[1200,1024]
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++


if not testIsActive:
    SC.renderer.Start()              #start graphics visualization
    SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue

mbs.SolveDynamic(simulationSettings)

if not testIsActive:
    SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
    SC.renderer.Stop()               #safely close rendering window!

#evaluate final (=current) output values
node = ancf[0][-1]
p = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)
exu.Print('ANCFcable2DuserFunction test tip pos=',p)

u=sum(p)
exu.Print('solution of ANCFcable2DuserFunction test =',u)

exu.sys['testResult'] = u










