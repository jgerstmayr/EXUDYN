#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Tests for symbolic user function; also those of items whose signature an earlier item has (#2664)
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
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', True)

esym = exu.symbolic
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

useSymbolicUF = True

#variable can be used to switch behavior
#esym.variables.Set('flag',1)

if useSymbolicUF:
    def springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
        #f0 = damping*deltaL_t + stiffness*deltaL + force #linear
        #fact = esym.variables.Get('flag') #potential way to couple behaviour to external triggers
        f0 = 10*damping*deltaL_t + stiffness*esym.sign(deltaL) * (esym.abs(deltaL))**1.2 + force
        return f0
    
    def UFload(mbs, t, load):
        return load*esym.sin(10*(2*pi)*t)
else:
    def springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
        f0 = 10*damping*deltaL_t + stiffness*np.sign(deltaL) * (np.abs(deltaL))**1.2 + force
        return f0
    
    def UFload(mbs, t, load):
        return load*np.sin(10*(2*pi)*t)

# no user function:    
# UFload=0
# springForceUserFunction=0

oGround = mbs.CreateGround()

oMassPoint = mbs.CreateMassPoint(referencePosition=[1.+0.05,0,0], physicsMass=1)
co = mbs.CreateSpringDamper(bodyNumbers=[oGround, oMassPoint],
                            referenceLength = 0.1, stiffness = 100, 
                            damping = 1,
                            springForceUserFunction = springForceUserFunction,
                            )

mc = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=0))
load = mbs.AddLoad(LoadCoordinate(markerNumber=mc, load=10,
                                  loadUserFunction=UFload))

if useSymbolicUF:
    symbolicFunc = CreateSymbolicUserFunction(mbs, springForceUserFunction, 'springForceUserFunction', co)
    mbs.SetObjectParameter(co, 'springForceUserFunction', symbolicFunc)
    
    symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, 'loadUserFunction', load)
    mbs.SetLoadParameter(load, 'loadUserFunction', symFuncLoad)
    
    #check function:
    exu.Print('spring user function:',symbolicFunc)
    exu.Print('load user function:  ',symFuncLoad)


#assemble and solve system for default parameters
mbs.Assemble()

endTime = 50
stepSize = 0.005

simulationSettings = exu.SimulationSettings()
#simulationSettings.solution.file.writePeriod = 0.01
simulationSettings.solution.file.write = False
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True

if not testIsActive:
    SC.renderer.Start()
    # SC.renderer.DoIdleTasks()

import time
ts = time.time()
exu.Print('start simulation')
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
exu.Print('finished: ', time.time()-ts, 'seconds')

if not testIsActive:
    SC.renderer.Stop() #safely close rendering window!

n = mbs.GetObject(oMassPoint)['nodeNumber']
p = mbs.GetNodeOutput(n, exu.OutputVariableType.Position)
u = np.linalg.norm(p)

#every user function can be made symbolic, also one whose signature an earlier item has (#2664): the coordinate
#spring-damper shares it with the spring-damper, the torque and the mass-proportional load with the force vector
def UFcoordinate(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset):
    return stiffness*(displacement-offset) + damping*velocity
def UFvector(mbs, t, loadVector):
    return [loadVector[0]*t, loadVector[1], loadVector[2]]
symbolicCoordinate = CreateSymbolicUserFunction(mbs, UFcoordinate, 'springForceUserFunction',
                                                itemTypeName='ObjectConnectorCoordinateSpringDamper')
uErrors = abs(symbolicCoordinate.Evaluate(mbs, 0., 0, 0.1, 0.2, 100., 2., 0.05) - UFcoordinate(mbs, 0., 0, 0.1, 0.2, 100., 2., 0.05))
for itemTypeName in ['LoadTorqueVector', 'LoadMassProportional']:
    symbolicVector = CreateSymbolicUserFunction(mbs, UFvector, 'loadVectorUserFunction', itemTypeName=itemTypeName)
    uErrors += np.linalg.norm(np.array(symbolicVector.Evaluate(mbs, 2., [1., 2., 3.])) - UFvector(mbs, 2., [1., 2., 3.]))
exu.Print('symbolic user functions with a shared signature, errors:', uErrors)
u += uErrors

exu.Print('u=',u)
exu.Print('solution of symbolicUserFunctionTest=',u)

# result for 10000 steps; identical for both UF cases
exu.sys['testResult'] = u

#++++++++++++++++++++++++++++++
#i7-1390, boost
#results for ExplicitMidpoint, 1e6 steps, best of three, exudynFast=True:
#C++:                    1.71s  #1710ns/step
#Python user function:  13.56s  #
#Symbolic user function: 2.28s  #570ns overhead for 2 x user function
# => speedup of user function part 20.8

#++++++++++++++++++++++++++++++
#OLD/original timings, one user function spring-damper
#timings for 2e6 steps
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

