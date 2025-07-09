
.. _examples-stiffflyballgovernorkt:

*************************
stiffFlyballGovernorKT.py
*************************

You can view and download this file on Github: `stiffFlyballGovernorKT.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/stiffFlyballGovernorKT.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Stiff flyball governor built with kinematic tree (IFToMM benchmark problem);
   #           Ref.: https://www.iftomm-multibody.org/benchmark/problem/Stiff_flyball_governor/
   #
   # Model:    Flyball governor with kinematic tree
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-8-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import libaries
   import sys
   sys.exudynFast=True
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   from numpy import linalg as LA
   
   ## set up MainSystem mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   useGraphics=True
   
   color = [0.1,0.1,0.8,1]
   r = 0.2 #radius
   L = 1   #length
   
   
   #%%%%%%%%%
   
   background0 = GraphicsDataRectangle(-L,-L,L,L,color)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## body dimensions according to reference in m
   
   # shaft
   lengthShaft = 1     #z
   widthShaft  = 0.01  #=height
   
   # rod
   lengthRod = 1
   widthRod  = 0.01    #=height
   
   # slider
   dimSlider = 0.1 #x=y=z
   sSlider = 0.5
   
   # scalar distance between point A and B
   xAB = 0.1   
   beta0 = np.deg2rad(30) 
   initAngleRod = np.deg2rad(60)
   
   # initial angular velocity of shaft and slider
   omega0 = [0., 0., 2*np.pi]
   
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## body masses according to reference in kg
   
   density = 3000
   
   mShaft        = 0.3
   mRod          = 0.3
   mSlider       = 3
   mMassPoint    = 5
   mRodMassPoint = mRod + mMassPoint
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## define gravity vector
   g = [0,0,-9.81]
   
   ## setup rod inertia along x-direction
   iRod = InertiaCuboid(density=density, sideLengths=[lengthRod,widthRod,0.01]).Translated([lengthRod/2,0,0])
   iMass = InertiaMassPoint(mass=mMassPoint).Translated([lengthRod,0,0])
   iRodSum = iRod+iMass
   
   # #compute reference point of rod (midpoint)
   # refRod = -iRodSum.com
   # iRodSum = iRodSum.Translated(refRod)
   # exu.Print("iRodSum=", iRodSum)
   
   nRigidBodyNodes = 4
   
   ## set ub shaft and slider inertias w.r.t. center of mass
   inertiaList=[InertiaCuboid(density=density, sideLengths=[widthShaft,widthShaft,lengthShaft]),
                InertiaCuboid(density=density, sideLengths=[dimSlider,dimSlider,dimSlider]),
                iRodSum, iRodSum]
   
   ## set up graphics objects (blocks) for 4 bodies
   graphicsShaft  = graphics.BrickXYZ(-widthShaft/2,-widthShaft/2,-lengthShaft/2, widthShaft/2,widthShaft/2,lengthShaft/2, [0.1,0.1,0.8,1])
   graphicsSlider = graphics.BrickXYZ(-dimSlider/2,-dimSlider/2,-dimSlider/2, dimSlider/2,dimSlider/2,dimSlider/2, [0.1,0.1,0.8,1])
   graphicsRodAC  = graphics.Brick([0.5*lengthRod, 0, 0], [lengthRod,widthRod,widthRod], graphics.color.red)
   graphicsRodBD  = graphics.Brick([0.5*lengthRod, 0, 0], [lengthRod,widthRod,widthRod], graphics.color.dodgerblue)
   
   ## lists for 4 bodies: [shaft, slider, rodAC, rodBD]
   graphicsList=[[graphicsShaft], [graphicsSlider], [graphicsRodAC], [graphicsRodBD]]
   
   
   
   ## create kinematic tree for 4 links [shaft, slider, rodAC, rodBD]
   ### create generic node for unknowns of KinematicTree
   nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.]*nRigidBodyNodes,
                                          initialCoordinates=[0.]*nRigidBodyNodes,
                                          initialCoordinates_t=[omega0[2],0,0,0], #initial angular velocity
                                          numberOfODE2Coordinates=nRigidBodyNodes))
   
   ### create position vectors for links in kinematic tree
   refPosList=[[0,0,lengthShaft*0.5],        # shaft
               [0,0,sSlider-lengthShaft*0.5],              # slider
               [ xAB/2, 0, lengthShaft*0.5],   # rodAC
               [-xAB/2, 0, lengthShaft*0.5]]   # rodBD
   
   ### set up list of joint types, masses, COMs, inertias, and transformations for kinematic tree
   jointTypes = [exu.JointType.RevoluteZ, exu.JointType.PrismaticZ, exu.JointType.RevoluteY, exu.JointType.RevoluteY]
   linkMasses = []
   linkCOMs = exu.Vector3DList()
   linkInertiasCOM=exu.Matrix3DList()
   
   jointTransformations=exu.Matrix3DList()
   jointOffsets = exu.Vector3DList()
   
   ### transform quantities for kinematic tree
   for i in range(nRigidBodyNodes):    
       inertia = inertiaList[i]
       linkMasses += [inertia.Mass()]
       linkCOMs.Append(inertia.COM())
       linkInertiasCOM.Append(inertia.InertiaCOM())
       
       A = np.eye(3)
       if i == 2:
           A = RotationMatrixY(beta0)
       if i == 3:
           A = RotationMatrixY((pi-beta0))
           
       
       jointTransformations.Append(A)
       jointOffsets.Append(refPosList[i])
       
   
   ## create kinematic tree object 'KinematicTree' with links [shaft, slider, rodAC, rodBD]
   oKT=mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=jointTypes, 
                                     linkParents=[-1,0,0,0],
                                     jointTransformations=jointTransformations, jointOffsets=jointOffsets, 
                                     linkInertiasCOM=linkInertiasCOM, linkCOMs=linkCOMs, linkMasses=linkMasses, 
                                     baseOffset = [0.,0.,0.], gravity=g, 
                                     visualization=VObjectKinematicTree(graphicsDataList = graphicsList)
                                     ))
       
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## add spring-damper parameters for connecting the rods with the slider
   
   # spring
   k  = 8.e5 # spring stiffness in N/m
   l0 = 0.5  # relaxed spring length in m
   c = 4.e4  # damping coefficient Ns/m
   
   ## add markers for joints
   markerRodACSlider = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=2,
                                                              localPosition=[lengthRod/2,0,0]))
   markerSliderPointE = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=1,
                                                              localPosition=[dimSlider/2,0,0]))
   
   markerRodBDSlider = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=3,
                                                              localPosition=[lengthRod/2,0,0]))
   markerSliderPointF = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=1,
                                                              localPosition=[-dimSlider/2,0,0]))
   
   ## add spring-dampers for compliant mechanism
   mbs.AddObject(SpringDamper(markerNumbers=[markerSliderPointE, markerRodACSlider], stiffness=k, damping=c, referenceLength=l0))
   mbs.AddObject(SpringDamper(markerNumbers=[markerSliderPointF, markerRodBDSlider], stiffness=k, damping=c, referenceLength=l0))
   
   ## add sensor to measure slider position
   sPos = mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=1,
                                            localPosition=[0,0,0],storeInternal=True,
                                            outputVariableType=exu.OutputVariableType.Position))
   
   
   ## assemble system 
   mbs.Assemble()
   
   if useGraphics: #only start graphics once, but after background is set
       ## start renderer
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
       
   tEnd = 10
   # h = 2e-5 #RK44
   h = 5e-4*1
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.displayComputationTime = False
   simulationSettings.timeIntegration.verboseMode = 1
   
   ## use optimized simulation settings for performance
   simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/100
   simulationSettings.solutionSettings.writeSolutionToFile = False
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 2
   simulationSettings.timeIntegration.newton.numericalDifferentiation.jacobianConnectorDerivative = False
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
   simulationSettings.timeIntegration.verboseMode = 1
   # simulationSettings.displayComputationTime = True
   simulationSettings.displayStatistics = True
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.7
       
   simulationSettings.timeIntegration.absoluteTolerance = 1e-6
   simulationSettings.timeIntegration.relativeTolerance = simulationSettings.timeIntegration.absoluteTolerance
   
   SC.visualizationSettings.markers.show = True
   
   # dynamicSolver.SolveSystem(mbs, simulationSettings)
   solverType = exu.DynamicSolverType.TrapezoidalIndex2 #same as generalized alpha
   # solverType = exu.DynamicSolverType.GeneralizedAlpha 
   # solverType = exu.DynamicSolverType.ODE23 #0.8 seconds for h=0.05 and aTol=rTol=1e-5
   
   #tests:
   #Python 3.7, fast, TrapezoidalIndex2, numDiff systemWide, maxModNewtonIts=2: 0.6701 seconds
   #Python 3.8 Linux, fast, TrapezoidalIndex2, numDiff systemWide, maxModNewtonIts=2: 0.5259 seconds
   
   ## start solver
   mbs.SolveDynamic(simulationSettings, 
                     solverType=solverType,
                    )
   
   
   if useGraphics: #only start graphics once, but after background is set
       ## wait for user to quit, then stop visualization
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   ## print relevant results
   # result = mbs.GetNodeOutput(2,exu.OutputVariableType.Velocity)[1] #y-velocity of bar
   # exu.Print('solution of stiffFlyballGovernor=',result)
   resultSlider = mbs.GetNodeOutput(nGeneric,exu.OutputVariableType.Coordinates_t)[1] #z-velocity of slider
   exu.Print('velocity of slider=',resultSlider)
   
   posSlider = mbs.GetNodeOutput(nGeneric,exu.OutputVariableType.Coordinates)[1]+0.5 #z-velocity of slider
   exu.Print('position of slider=', posSlider)
   
   if useGraphics:
       ## plot results
       mbs.PlotSensor(sPos, components=[2], closeAll=True)
   
   
   
   
   
   
   
   
   
   
   
   
     

