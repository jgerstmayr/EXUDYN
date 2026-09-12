#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test sensor with user function; 
#           This test includes two cases:
#           1) symbolic user function which allows to use regular graphics multithreading and is faster
#           2) Python user function, which is convenient to be used, but requires to set useMultiThreadedRendering=False in visualizationSettings.general
#
# Author:   Johannes Gerstmayr
# Date:     2021-02-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.basicUtilities import NormL2
from exudyn.utilities import CreateSymbolicUserFunction
from math import pi

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#useGraphics = False

esym = exu.symbolic
import numpy as np

result = 0
cases = ['PythonUserFunction', 'SymbolicUserFunction']
for case in cases:
    exu.Print('CASE: '+case)
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    
    useSymbolicUF = case == 'SymbolicUserFunction'
    
    #variable can be used to switch behavior
    #esym.variables.Set('flag',1)
    
    if useSymbolicUF:
        #scaling load with time
        def UFload(mbs, t, loadVector):
            return esym.cos(2*(2*pi)*t) * loadVector
    else:
        #general load changing over time (rotating)
        def UFload(mbs, t, loadVector):
            return [np.sin(2*(2*pi)*t) * loadVector[0], 
                    np.cos(2*(2*pi)*t) * loadVector[0], 
                    0]
    
    oGround = mbs.CreateGround()
    
    oMassPoint = mbs.CreateMassPoint(referencePosition=[1.+0.05,0,0], physicsMass=1, drawSize=0.1)
    co = mbs.CreateSpringDamper(bodyNumbers=[oGround, oMassPoint],
                                referenceLength = 1, stiffness = 100, damping = 1)
    sMass = mbs.AddSensor(SensorBody(bodyNumber=oMassPoint, 
                                     outputVariableType=exu.OutputVariableType.Position))
    
    load = mbs.CreateForce(bodyNumber=oMassPoint, loadVector=[10,0,0], 
                           loadVectorUserFunction=UFload)
    
    if useSymbolicUF:
        #these steps have to be done to inject the symbolic user function into exudyn as C-function
        symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, 'loadVectorUserFunction', load)
        mbs.SetLoadParameter(load, 'loadVectorUserFunction', symFuncLoad)
    
        #check function:
        exu.Print('load user function:  ',symFuncLoad)
    
    
    #assemble and solve system for default parameters
    mbs.Assemble()
    
    endTime = 10
    stepSize = 0.005
    
    SC.visualizationSettings.loads.drawSimplified = False
    SC.visualizationSettings.loads.drawWithUserFunction = True
    SC.visualizationSettings.loads.fixedLoadSize = False #otherwise only sign will be shown
    SC.visualizationSettings.loads.loadSizeFactor = 0.025
    SC.visualizationSettings.general.useMultiThreadedRendering = useSymbolicUF
    SC.visualizationSettings.nodes.drawNodesAsPoint = False
    SC.visualizationSettings.nodes.tiling = 32
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
    SC.visualizationSettings.general.renderWindowString = 'Test case: '+case
    
    simulationSettings = exu.SimulationSettings()
    #simulationSettings.solutionSettings.solutionWritePeriod = 0.01
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.timeIntegration.simulateInRealtime = useGraphics #for visualization to be viewed by user!
    
    simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    
    if useGraphics:
        SC.renderer.Start()
        SC.renderer.DoIdleTasks()
    
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
    
    if useGraphics:
        SC.renderer.Stop() #safely close rendering window!
    
    result += NormL2(mbs.GetSensorValues(sMass))



#evaluate final (=current) output values
exu.Print('result of loadUserFunctionTest=',result)

exudynTestGlobals.testResult = result  #1.8051173706570725


