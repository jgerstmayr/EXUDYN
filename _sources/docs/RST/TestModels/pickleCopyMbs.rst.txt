
.. _testmodels-picklecopymbs:

****************
pickleCopyMbs.py
****************

You can view and download this file on Github: `pickleCopyMbs.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/pickleCopyMbs.py>`_

.. code-block:: python
   :linenos:

   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Two rigid bodies, where the first body moves on a straight rail (fixed on ground) and the second is mounted with a revolute joint on the first body;
   #           the particular reason for this test example is the copying of the model as well as load save;
   #           shows reading and writing mbs with dictionaries and export by HDF5 files
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-10-13
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from math import sin, cos
   import copy 
   
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
   
   
   #%%+++++++++++++++++++++++++++++++++++
   L = 0.5
   w = 0.05
   ground = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0,0,-0.1*L], size=10)])
   
   b0size = [0.5*L,L,0.2*L]
   p0 = np.array([0.5,1,0])
   b0 = mbs.CreateRigidBody(referencePosition=p0,
                            initialVelocity=[0,1,0],
                            inertia=InertiaCuboid(500, b0size),
                            graphicsDataList=[graphics.Brick(size=b0size,
                                                             color=graphics.color.dodgerblue)])
   b1size=[w,L,w]
   p1 = p0 + [0,0.5*L,0.5*b0size[2]+w]
   b1 = mbs.CreateRigidBody(referencePosition=p1,
                            initialVelocity=[0,1,0],
                            initialAngularVelocity=[0,0,4*pi],
                            inertia=InertiaCuboid(1000, b1size),
                            graphicsDataList=[graphics.Brick(size=b1size,
                                                             color=graphics.color.red)])
   
   mbs.CreateRevoluteJoint(bodyNumbers=[b0,b1],
                           position=p1-[0,0.5*L,0.5*w], 
                           axis = [0,0,1],
                           axisRadius=0.5*w, axisLength=2.2*w)
   
   mbs.CreatePrismaticJoint(bodyNumbers=[ground,b0],
                            position=p0, 
                            axis = [0,1,0],
                            axisRadius=0.5*w, axisLength=5*L)
   
   # UFtorque = 0
   def UFtorque(mbs, t, loadVector):
       f = np.heaviside(t - 1,1)
       return [0,0,-f*2*cos(pi*2*t)]
   
   load = mbs.CreateTorque(bodyNumber=b1, loadVector=[0,0,0],
                           loadVectorUserFunction=UFtorque)
   
   sPos = mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Position))
   
   
   #del mbs
   
   #%%+++++++++++++++++++++++++++++++++++
   #use dictionaries and pickle to load/save MBS
   import pickle
   
   mbsDict = mbs.GetDictionary()
   
   #save MUST be done before solving, as solver structures and settings cannot be saved!
   #erase variables and system variables as they may contain some solver stuff (if solved once)
   #  that should not / can not be pickled
   mbsDict['variables'] = {} 
   mbsDict['systemVariables'] = {}
   
   #save mbs data (could also use dill)
   with open('solution/mbs.pkl', 'wb') as f:
       pickle.dump(mbsDict, f) #, pickle.HIGHEST_PROTOCOL)
   
   #load mbs data
   with open('solution/mbs.pkl', 'rb') as f:
       mbsCopy = pickle.load(f)
   
   SC = exu.SystemContainer()
   mbs2 = SC.AddSystem()
   mbs2.SetDictionary(mbsDict)
   
   #+++++++++++++++++++++++
   
   hasH5py = False
   try:
       import h5py
       hasH5py = True
   except ImportError:
       exu.Print('h5py not available; skipping HDF5 test')
       
   if hasH5py:
       from exudyn.advancedUtilities import SaveDictToHDF5, LoadDictFromHDF5
       #use HDF5 load / save may not work for all cases, as some types are not implemented
       #save MUST be done before solving, as solver structures and settings cannot be saved!
       mbsDict = mbs.GetDictionary()
       mbsDict['variables'] = {} 
       mbsDict['systemVariables'] = {}
       SaveDictToHDF5('solution/mbs.h5', mbsDict) #problems with Python functions as sub-dicts
       mbsCopy2 = LoadDictFromHDF5('solution/mbs.h5', globals())
       # print('loaded_data:\n', mbsCopy2)
       
       SC = exu.SystemContainer()
       mbs2 = SC.AddSystem()
       mbs2.SetDictionary(mbsCopy)
   
   #%%+++++++++++++++++++++++++++++++++++
   #ALTERNATIVE: work with copy of mbs:
   #we have to create a new Systemcontainer:
   #mbs2 = copy.copy(mbs) #alternative way to directly copy
   # SC = exu.SystemContainer()
   # SC.AppendSystem(mbs2)
   
   mbs2.Assemble()
   
   #check mbs2:
   #print(mbs2)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 1
   h = 1e-3
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   # simulationSettings.timeIntegration.simulateInRealtime = True
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
   # simulationSettings.displayComputationTime = True
   
   simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
   SC.visualizationSettings.nodes.defaultSize = 0.05
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.window.renderWindowSize = [2000,1600]
   
   if useGraphics:
       SC.renderer.Start()
       mbs2.WaitForUserToContinue()
   
   mbs2.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   uTotal = 0.1 * sum(mbs2.GetSensorValues(sPos)) #sPos is still the correct index
   exu.Print('uTotal=',uTotal)
   
   exudynTestGlobals.testResult = uTotal
   #+++++++++++++++++++++++++++++++++++++++++++++
   
   


