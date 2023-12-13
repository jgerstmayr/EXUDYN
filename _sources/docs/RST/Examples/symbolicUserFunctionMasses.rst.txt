
.. _examples-symbolicuserfunctionmasses:

*****************************
symbolicUserFunctionMasses.py
*****************************

You can view and download this file on Github: `symbolicUserFunctionMasses.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/symbolicUserFunctionMasses.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  car with wheels modeled by ObjectConnectorRollingDiscPenalty
   #           formulation is still under development and needs more testing
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
   #from math import pi, sin, cos
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   useSymbolicUF = True
   if useSymbolicUF:
       mysym = esym #np or esym
       myabs = esym.abs
       myReal = esym.Real
   else: #regular mode with numpy / Python functions
       mysym = np #np or esym
       myabs = abs
       myReal = float
   
   def springForceUserFunction(mbs: exu.MainSystem, t: float, itemNumber: int, 
                               deltaL: float, deltaL_t: float, stiffness: float,
                               damping: float, force: float):
       #f0 = damping*deltaL_t + stiffness*deltaL + force
       f0 = (damping*deltaL_t + stiffness*deltaL)*(mysym.sign(-deltaL+myReal(0.005))+1.)*0.5
       #f0 = (damping*deltaL_t + stiffness*deltaL)*esym.IfThenElse(deltaL > 0.01,myReal(0),myReal(1))
       return f0
   
   
   #**function: internal function to convert a Python user function into a symbolic representation
   def ConvertFunctionToSymbolic(mbs, function, verbose=0):
       fnName = function.__name__
       fnArgs = function.__code__.co_varnames
       fnAnnotations = function.__annotations__
       if verbose:
           print("Annotations:", fnAnnotations)
           print("Function Name:", fnName)
           print("Docstring:", function.__doc__)
           print("Number of Arguments:", function.__code__.co_argcount)
           print("Argument Names:", fnArgs)
   
       recStored = exu.symbolic.GetRecording()
       exu.symbolic.SetRecording(True)
   
       #create args as dict
       fnDict = {}
       argList = []
       for arg in fnArgs[:function.__code__.co_argcount]: #fnArgs contains also other vars inside function code
           varType = fnAnnotations[arg]
           # print('arg:',arg,', varType:',varType)
           var = None
           if varType == float:
               var = exu.symbolic.Real(0.,arg)
               # print(arg,"=",var.ToString())
               #var = float(0.)
           elif varType == int:
               var = exu.symbolic.Real(0,arg)
               #var = int(0)
           elif varType == exu.MainSystem:
               var = mbs
       
           argList += [var] #float, int, MainSystem, Vector3D, etc.
           fnDict[arg] = var
   
       if verbose:
           print('\nargList=', argList)
           print('\nfnDict=', fnDict)
   
       #now we record the function:
       returnValue = function(**fnDict)
       exu.symbolic.SetRecording(recStored)
   
       if verbose:
           print('return value=', returnValue)
   
       returnList = []
       if type(returnValue) is list:
           #requires to store a list of symbolic functions
           returnList = returnValue
       else:
           returnList = [returnValue]
   
       fnDict = {'functionName': fnName, 
                 'argList': argList, 
                 'returnList': returnList}
       return fnDict
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   Lx = 2
   Ly= 0.2
   Lz = 0.4
   aFact = 2
   
   ff = 2
   nx = 25*ff
   ny = 4*ff
   # nx = 15
   # ny = 3
   nz = 2*ny
   
   g = [0,-9.81,0]
   m = 1 / (nx*ny*4)
   stiffness = 200
   damping = 2e-3*stiffness
   
   drawSize = 0.2/ff
   showSprings = False
   
   bList = np.zeros((nx,ny,nz),dtype=exu.ObjectIndex)
   cList = []
   for i in range(nx):
       for j in range(ny):
           for k in range(nz):
               x = i/nx
               
               fact = 3*aFact**(-x*6)+1
               aLy = Ly*fact
               aLz = Lz #*fact
               p = np.array([Lx*x, -0.5*aLy+aLy*j/(ny-1), -0.5*aLz+aLz*k/(nz-1)])
               if i == 0:
                   b = mbs.AddObject(ObjectGround(referencePosition=p))
               else:
                   b = mbs.CreateMassPoint(referencePosition=p, physicsMass=m, gravity=g, drawSize = drawSize)
                       
               # print('b=',b,': i',i,': j',j,': k',k)
               bList[i,j,k] = b
   
                   
   for i in range(nx):
       for j in range(ny):
           for k in range(nz):
               if i > 0:
                   rr = 1. - np.random.rand()*0.05
                   cList += [mbs.CreateSpringDamper(    bodyList=[bList[i,j,k], bList[i-1,j,k]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                   if j>0:
                       cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i-1,j-1,k]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                       cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i  ,j-1,k]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                       if k>0:
                           cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i,j-1,k-1]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                   if j<ny-1:
                       cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i-1,j+1,k]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                   if k>0:
                       cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i-1,j,k-1]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                       cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i  ,j,k-1]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                   if k<nz-1:
                       cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i-1,j,k+1]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
                       if j>0:
                           cList += [mbs.CreateSpringDamper(bodyList=[bList[i,j,k], bList[i,j-1,k+1]], stiffness=stiffness*rr, damping=damping,drawSize = 0, show=showSprings)]
   
   def CreateSymbolicUserFunction(mbs, function, itemIndex, userFunctionName, verbose=0):
       fnDict = ConvertFunctionToSymbolic(mbs, function)
       symbolicFunc = exu.symbolic.SymbolicUserFunction()
       symbolicFunc.SetUserFunctionFromDict(mbs, fnDict, itemIndex, userFunctionName)
       return symbolicFunc
       
   if useSymbolicUF:
       symbolicFunc = CreateSymbolicUserFunction(mbs, springForceUserFunction, cList[0], 'springForceUserFunction')
       # x = symbolicFunc.Evaluate(mbs, 0., 0, 1.1, 0.,  100., 0., 13.)
       # print('x=', x)
   
   
   # CAUTION: this only works, if every object has its own user function!!!
   if useSymbolicUF: #do assemble before adding user functions => then, they will be parallelized
       mbs.Assemble()
   
   if True:
       for cc in cList:
           if useSymbolicUF:
               symbolicFunc.TransferUserFunction2Item(mbs, cc, 'springForceUserFunction')    
           else:
               mbs.SetObjectParameter(cc, 'springForceUserFunction', springForceUserFunction)
   
   
   #assemble and solve system for default parameters
   if not useSymbolicUF: #do assemble before adding user functions => then, they will be parallelized
       mbs.Assemble()
   
   endTime = 0.01#10
   stepSize = 1e-4
   if ff==1: stepSize = 5e-4
   if ff>2: stepSize = 0.5e-5
   
   # endTime = 0.01
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   # simulationSettings.solutionSettings.writeSolutionToFile = False
   
   simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
   simulationSettings.timeIntegration.endTime = endTime
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.parallel.numberOfThreads = 8
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   #simulationSettings.displayComputationTime = True
   
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.tiling = 6
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.contour.outputVariableComponent = -1
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
   SC.visualizationSettings.window.renderWindowSize = [2000,1600]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   import time
   exu.StartRenderer()
   ts = time.time()
   print('start simulation')
   mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)
   print('finished: ', time.time()-ts, 'seconds')
   mbs.WaitForUserToContinue()
   exu.StopRenderer()
   # mbs.SolutionViewer()
   #i7-1390, boost
   #results for ExplicitMidpoint, 100 steps, 6272 nodes, RK44, exudynFast=True:
   #8 threads:
   #C++:                   2.62s
   #Symbolic user function:4.74s 
   #1 thread:
   #C++:                   9.83s
   #Symbolic user function:11.04s 
   #Python user function:  81.8s => speedup = 38
   
   #i9:
   #results for ExplicitMidpoint, 100 steps, 6272 nodes, RK44, exudynFast=True:
   #8 threads:
   #C++:                   3.27s
   #Symbolic user function:6.16s 
   #Python user function:  62.72s => speedup = 20.57


