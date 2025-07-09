
.. _testmodels-contactspherespheretesteapm:

******************************
contactSphereSphereTestEAPM.py
******************************

You can view and download this file on Github: `contactSphereSphereTestEAPM.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/contactSphereSphereTestEAPM.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test of the SphereSphere contact module
   #
   # Author:   Johannes Gerstmayr and Sebastian Weyrer
   # Date:     2025-01-07
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
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
   exu.Print('EXUDYN version = ' + exu.config.Version())
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # define behaviour of script
   T = 1                           # [s] simulation time
   stepSize = 2e-5                 # [s]
   impactModel = 0                 # 0 = Adhesive Elasto-Plastic Model; 1 = Hunt-Crossley Model; 2 = mixed Gonthier/EtAl-Carvalho/Martins Model
   restitutionCoefficient = 0.9    # used restitution coefficient if impactModel != 0
   use2Contacts = True             # only applies if impactModel != 0, specifies numbers of used contacts for chosen impact model
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # define parameters of the speheres
   radius = 0.1        # [m] radius of spheres
   mass = 1.6          # [kg] mass of each sphere
   # define reference position of the moving sphere and initial velocity
   if impactModel == 0:
       xInit = 0
       yInit = 2 * radius # gap = 0 at beginning of simulation
       vyInit = 0
   else:
       xInit = 0
       yInit = 2 * radius + 0.5 # gap = 0.5m at beginning of simulation 
       vyInit = 10
       
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # create ground (with marker) that is needed for every test case
   mbs.CreateGround(referencePosition=[0, 0, 0], graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.color.red, nTiles=64)])
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates = [0, 0, 0]))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # add mass point (with marker) for moving spehere
   massPoint = mbs.CreateRigidBody(referencePosition=[xInit, yInit, 0],
                                   initialVelocity=[0, vyInit, 0],
                                   inertia=InertiaSphere(mass, radius),
                                   gravity=[0, 0, 0],
                                   graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.color.green, nTiles=64)])
   mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=massPoint))
   nMassPoint = mbs.GetObject(massPoint)['nodeNumber']
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # if impactModel = 0, simulate loading and unloading of (soil) particles
   if impactModel == 0:
       tMove = T/2
       dMove = 0.002
       def UFforce(mbs, t, itemNumber, u, v, k, d, F0):
           if t < tMove:
               setPosition = 2*radius - (dMove/tMove)*t
           else:
               setPosition = 2*radius - dMove + (dMove/tMove)*(t-tMove)
           # exu.Print(setPosition)
           return (u-setPosition)*1e5
               
       mbs.AddObject(SpringDamper(markerNumbers=[mGround, mMass], referenceLength=0, springForceUserFunction=UFforce))
       nData = mbs.AddNode(NodeGenericData(initialCoordinates=[0, 0, 0, 0], numberOfDataCoordinates=4))
       oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround, mMass], # m1 is moving spehere
                                                      nodeNumber=nData,
                                                      spheresRadii=[radius,radius],
                                                      impactModel=impactModel,
                                                      contactStiffness=1e5,
                                                      contactStiffnessExponent=2,
                                                      contactDamping=0.001,
                                                      contactPlasticityRatio=0.6,
                                                      constantPullOffForce=0.01,
                                                      adhesionCoefficient=4e4,
                                                      adhesionExponent=2))
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # if impactModel != 0 do impact simulation of particles
   else:
       nData = mbs.AddNode(NodeGenericData(initialCoordinates=[0, 0, 0, 0], numberOfDataCoordinates=4))
       oSSC = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround, mMass], # m1 is moving spehere
                                                      nodeNumber=nData,
                                                      spheresRadii=[radius,radius],
                                                      contactStiffness=1e6,
                                                      contactDamping=0,
                                                      impactModel=impactModel,
                                                      restitutionCoefficient=restitutionCoefficient,
                                                      minimumImpactVelocity=0))
       if use2Contacts:
           # add this ground object with a gap of 0.5m above the moving spehere
           mbs.CreateGround(referencePosition=[0, 1+4*radius, 0], graphicsDataList=[graphics.Sphere(radius=radius, color=graphics.color.red, nTiles=64)])
           nGround1 = mbs.AddNode(NodePointGround(referenceCoordinates = [0, 1+4*radius, 0]))
           mGround1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround1))
           nData1 = mbs.AddNode(NodeGenericData(initialCoordinates=[0, 0, 0, 0], numberOfDataCoordinates=4))
           oSSC1 = mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround1, mMass], # m1 is moving spehere
                                                          nodeNumber=nData1,
                                                          spheresRadii=[radius,radius],
                                                          contactStiffness=1e6,
                                                          contactDamping=0,
                                                          impactModel=impactModel,
                                                          restitutionCoefficient=restitutionCoefficient,
                                                          minimumImpactVelocity=0))
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   # meassure output variables of moving spehere in any test case
   sPos = mbs.AddSensor(SensorBody(bodyNumber=massPoint, outputVariableType=exu.OutputVariableType.Position, writeToFile=False, storeInternal=True))
   sVel = mbs.AddSensor(SensorBody(bodyNumber=massPoint, outputVariableType=exu.OutputVariableType.Velocity, writeToFile=False, storeInternal=True))
   sGap = mbs.AddSensor(SensorObject(objectNumber=oSSC, outputVariableType=exu.OutputVariableType.DisplacementLocal, writeToFile=False, storeInternal=True))
   sContactForce = mbs.AddSensor(SensorObject(objectNumber=oSSC, outputVariableType=exu.OutputVariableType.Force, writeToFile=False, storeInternal=True))
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   # exu.Print(mbs)
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.numberOfSteps = int(T/stepSize)
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize
   simulationSettings.timeIntegration.endTime = T
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   if useGraphics:
       SC.renderer.Start()                 # start graphics visualization
       SC.renderer.DoIdleTasks()         # wait for pressing SPACE bar to continue
   mbs.SolveDynamic(simulationSettings)
   if useGraphics:
       SC.renderer.DoIdleTasks()    # wait for pressing 'Q' to quit
       SC.renderer.Stop()                  # safely close rendering window!
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++
   uTotal = mbs.GetNodeOutput(nMassPoint, exu.OutputVariableType.CoordinatesTotal)
   exu.Print('uTotal=',uTotal[1])
   
   exudynTestGlobals.testResult = uTotal[1]
   
   if useGraphics:
       #+++++++++++++++++++++++++++++++++++++++++++++
       # plot sensor data for different test cases
       import matplotlib.pyplot as plt
       def setMatplotlibSettings():
           import matplotlib as mpl
           import matplotlib.font_manager as font_manager
           import matplotlib.pyplot as plt
           fsize = 9 # general fontsize
           tsize = 9 # legend
           # setting ticks, fontsize
           tdir = 'out' # 'in' or 'out': direction of ticks
           major = 5.0 # length major ticks
           minor = 3.0 # length minor ticks
           lwidth = 0.8 # width of the frame
           lhandle = 2 # length legend handle
           plt.style.use('default')
           # set font to the locally saved computermodern type
           plt.rcParams['font.family']='serif'
           cmfont = font_manager.FontProperties(fname=mpl.get_data_path() + '/fonts/ttf/cmr10.ttf')
           plt.rcParams['font.serif']=cmfont.get_name()
           plt.rcParams['mathtext.fontset']='cm'
           plt.rcParams['axes.unicode_minus']=False
           plt.rcParams['font.size'] = fsize
           plt.rcParams['legend.fontsize'] = tsize
           plt.rcParams['xtick.direction'] = tdir
           plt.rcParams['ytick.direction'] = tdir
           plt.rcParams['xtick.major.size'] = major
           plt.rcParams['xtick.minor.size'] = minor
           plt.rcParams['ytick.major.size'] = major
           plt.rcParams['ytick.minor.size'] = minor
           plt.rcParams['axes.linewidth'] = lwidth
           plt.rcParams['axes.formatter.use_mathtext'] = True
           plt.rcParams['legend.handlelength'] = lhandle
           plt.rcParams['lines.linewidth'] = 1.2
           return
       setMatplotlibSettings()
       mbs.PlotSensor([sPos,sVel], components=[1, 1], closeAll=False)
       plt.show(block=True) # for figures to stay open at end of plot routines
       if impactModel == 0:
           def add_arrow(line, position=None, direction='right', size=15, color=None):
               if color is None:
                   color = line.get_color()
               xdata = line.get_xdata()
               ydata = line.get_ydata()
               if position is None:
                   position = xdata.mean()
               start_ind = np.argmin(np.absolute(xdata - position))
               if direction == 'right':
                   end_ind = start_ind + 1
               else:
                   end_ind = start_ind - 1
               line.axes.annotate('', xytext=(xdata[start_ind], ydata[start_ind]),
                                  xy=(xdata[end_ind], ydata[end_ind]),
                                  arrowprops=dict(arrowstyle="->", color=color), size=size)
           fig, (ax1, ax2) = plt.subplots(1, 2, figsize=[8, 4])
           t = mbs.GetSensorStoredData(sGap)[:, 0]
           penetration = -mbs.GetSensorStoredData(sGap)[:, 1]
           globalContactForce = mbs.GetSensorStoredData(sContactForce)[:, 2]
           repulsiveForce = -globalContactForce
           indexFirstPositivePenetration = np.where(penetration > 0)[0][0]
           indexLastPositivePenetration = np.where(penetration > 0)[0][-1]
           line1 = ax1.plot(penetration[indexFirstPositivePenetration:indexLastPositivePenetration+1], repulsiveForce[indexFirstPositivePenetration:indexLastPositivePenetration+1])[0]
           add_arrow(line1, position=0.001)
           add_arrow(line1, position=0.0016)
           ax1.grid()
           ax1.set_xlabel('negative gap in m')
           ax1.set_ylabel('force (m1) in N')
           line2 = ax2.plot(penetration[indexFirstPositivePenetration:indexLastPositivePenetration+1], globalContactForce[indexFirstPositivePenetration:indexLastPositivePenetration+1])[0]
           add_arrow(line2, position=0.001)
           add_arrow(line2, position=0.0016)
           ax2.grid()
           ax2.set_xlabel('negative gap in m')
           ax2.set_ylabel('force (m0) in N (= global force)')
           plt.tight_layout()
       if impactModel != 0:
           def MaximaAfterCrossings(y):
               # Identify where the signal crosses from (-) to (+)
               crossings = np.where((y[:-1] < 0) & (y[1:] > 0))[0] + 1  # Indices where crossing occurs
               # Add the end of the signal as the last segment
               crossings = np.concatenate(([0], crossings, [len(y)]))
               # Calculate the maximum of each segment
               maxima = [np.max(y[crossings[i]:crossings[i + 1]]) for i in range(len(crossings) - 1)]
               minima = [np.min(y[crossings[i]:crossings[i + 1]]) for i in range(len(crossings) - 1)]
               return [maxima[0],-minima[0],maxima[1],-minima[1]]
               # return [maxima[0],-minima[0]]
           y = mbs.GetSensorStoredData(sVel)[:,2]
           maxima = MaximaAfterCrossings(y) #max 4 crossings
           exu.Print("Maxima after each crossing:", maxima)
           exu.Print('relations=',maxima[1]/maxima[0],maxima[2]/maxima[1],maxima[3]/maxima[2])
               
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Results:
   #e=0.5, stepSize=1e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 5.001784818335925, 2.501696294255996, 1.251250609056101]
   #e=0.5, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 5.001975126052718, 2.501837721858895, 1.2513458250009686]
   #e=0.5, stepSize=5e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 5.003239017129689, 2.501566602043766, 1.2509080365758343]
   
   #e=0.9, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 9.000168963897542, 8.100761764779794, 7.290837898880879]
   #e=0.8, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 8.000159389915021, 6.400886105172352, 5.120771413440074]
   #e=0.3, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 3.0423710530759065, 0.9256019663437189]
   #e=0.1, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 1.0098986986792768, 0.10198953720292783]
   #e=0.025, stepSize=2e-5, Gonthier-CarvalhoMartins:
   #Maxima after each crossing: [10.0, 0.25015634771732226]
   
   #e=0.9, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 9.090298887177969, 8.263743088547175, 7.512142692359256]
   #e=0.8, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 8.32903493005253, 6.937608834244499, 5.778355621679387]
   #e=0.5, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 6.629838391096419, 4.3951552519078065, 2.913688143667944]
   #e=0.3, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 5.813176318310211, 3.3794625180566293, 1.964542613454227]
   #e=0.1, stepSize=2e-5, Hunt-Crossley:
   #Maxima after each crossing: [10.0, 5.158573889338283, 2.6613482605395395, 1.3728789060291537]
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++


