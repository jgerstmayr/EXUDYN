
.. _testmodels-comparefullmodifiednewton:

****************************
compareFullModifiedNewton.py
****************************

You can view and download this file on Github: `compareFullModifiedNewton.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/compareFullModifiedNewton.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  pendulum example: check difference of full and modified newton
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-06-19
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   
   import numpy as np
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #rigid pendulum:
   rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
   background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
   a = 0.5     #x-dim of pendulum
   b = 0.05    #y-dim of pendulum
   massRigid = 12
   inertiaRigid = massRigid/12*(2*a)**2
   g = 9.81    # gravity
   
   graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
   omega0 = 5*0 #inconsistent initial conditions lead to integration problems!
   nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[a,0,0], initialVelocities=[0,omega0*a,omega0]));
   oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))
   
   mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-a,0.,0.])) #support point
   mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ a,0.,0.])) #end point
   
   mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0.]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))
   
   mbs.AddLoad(Force(markerNumber = mR2, loadVector = [0, -massRigid*g, 0]))
   
   mbs.Assemble()
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = 100 #100
   simulationSettings.timeIntegration.endTime = 2
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-4
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   #simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   #simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   simulationSettings.displayStatistics = True
   simulationSettings.solutionSettings.solutionWritePeriod = 1e-4
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/modifiedNewton.txt"
   mbs.SolveDynamic(simulationSettings)#, experimentalNewSolver=False)
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/fullNewton.txt"
   mbs.SolveDynamic(simulationSettings)#, experimentalNewSolver=False)
   
   
   #%%*****************************************
   #post processing for mass point system
   
   dataM = np.loadtxt('solution/modifiedNewton.txt', comments='#', delimiter=',')
   dataF = np.loadtxt('solution/fullNewton.txt', comments='#', delimiter=',')
   
   # exu.Print("solFullNewton     =",sum(abs(dataF[:,5])))
   # exu.Print("solModifiedNewton =",sum(abs(dataM[:,5])))
   
   u=sum(abs(dataM[:,5]-dataF[:,5]))
   exu.Print("compareFullModifiedNewton u=",u)
   
   exudynTestGlobals.testError = u - (0.0001583478719999567 ) #2020-12-18: 0.0001583478719999567 
   exudynTestGlobals.testResult = u
   
   import os
   os.remove('solution/modifiedNewton.txt')
   os.remove('solution/fullNewton.txt')
   
   if useGraphics:
       # plt.plot(dataM[:,0], dataM[:,3+2], 'b-') #plot column i over column 0 (time)
       # plt.plot(dataF[:,0], dataF[:,3+2], 'r-') #plot column i over column 0 (time)
       plt.plot(dataF[:,0], dataF[:,5]-dataM[:,5], 'r-') 
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) #use maximum of 8 ticks on y-axis
       plt.tight_layout()
       plt.show() 
   
   


