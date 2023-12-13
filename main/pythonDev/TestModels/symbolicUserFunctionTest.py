#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Tests for symbolic user function
#
# Author:   Johannes Gerstmayr
# Date:     2023-11-28
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys
sys.exudynFast = True

import exudyn as exu
esym = exu.symbolic
from exudyn.utilities import *
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

useSymbolicUF = True
if useSymbolicUF:
    mysym = esym #np or esym
    myabs = esym.abs
    myReal = esym.Real
else: #regular mode with numpy / Python functions
    mysym = np #np or esym
    myabs = abs
    myReal = float

#variable can be used to switch behavior
esym.variables.Set('flag',1)

def springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    #f0 = damping*deltaL_t + stiffness*deltaL + force #linear
    #f0 = damping*esym.sign(deltaL_t) + stiffness*deltaL*(1+deltaL*deltaL)
    fact = esym.variables.Get('flag')
    f0 = fact*10*damping*deltaL_t + stiffness*mysym.sign(deltaL) * (myabs(deltaL))**myReal(1.2) + force
    return f0

def UFload(mbs, t, load):
    return load*mysym.sin(10*(2*pi)*t)

oGround = mbs.CreateGround()

oMassPoint = mbs.CreateMassPoint(referencePosition=[1.+0.05,0,0], physicsMass=1)
co = mbs.CreateSpringDamper(bodyList=[oGround, oMassPoint],
                            referenceLength = 0.1, stiffness = 100, 
                            damping = 1)

mc = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=0))
load = mbs.AddLoad(LoadCoordinate(markerNumber=mc, load=10,
                                  loadUserFunction=UFload))

if useSymbolicUF:
    symbolicFunc = CreateSymbolicUserFunction(mbs, springForceUserFunction, co, 'springForceUserFunction')
    symbolicFunc.TransferUserFunction2Item(mbs, co, 'springForceUserFunction')    

    symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, load, 'loadUserFunction',1)
    symFuncLoad.TransferUserFunction2Item(mbs, load, 'loadUserFunction')    
    
    #check function:
    print('spring user function:',symbolicFunc)
    print('load user function:  ',symFuncLoad)
else:
    mbs.SetObjectParameter(co, 'springForceUserFunction', springForceUserFunction)

#assemble and solve system for default parameters
mbs.Assemble()

fact = 1000
endTime = 1*fact
stepSize = 0.005

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.solutionWritePeriod = 0.01
# simulationSettings.solutionSettings.writeSolutionToFile = False

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True

import time
ts = time.time()
print('start simulation')
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
print('finished: ', time.time()-ts, 'seconds')
# mbs.SolutionViewer()

# esym.variables.Set('flag',0)


#i7-1390, power saving
#results for ExplicitMidpoint, 2e6 steps, best of three, exudynFast=False:
#C++:                   2.99s
#Python user function:  16.01s
#Symbolic user function:4.37s

#i7-1390, boost
#results for ExplicitMidpoint, 2e6 steps, best of three, exudynFast=True:
#C++:                   1.14s  #570ns / step
#Python user function:  5.62s
#Symbolic user function:1.34s  #100ns overhead for user function
# => speedup 22.4

#i9
#results for ExplicitMidpoint, 2e6 steps, best of three, exudynFast=True:
#C++:                   1.80s  #570ns / step
#Python user function:  9.52s
#Symbolic user function:2.09s  #100ns overhead for user function

