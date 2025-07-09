
.. _examples-simulateinteractively:

************************
simulateInteractively.py
************************

You can view and download this file on Github: `simulateInteractively.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/simulateInteractively.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Nonlinear oscillations interactive simulation
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-01-16
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.graphicsDataUtilities import *
   import matplotlib.pyplot as plt
   from exudyn.interactive import InteractiveDialog
   
   import numpy as np
   from math import sin, cos, pi
   
   import time #for sleep()
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create model for linear and nonlinear oscillator:
   L=0.5
   mass = 1.6          #mass in kg
   spring = 4000       #stiffness of spring-damper in N/m
   damper = 20         #damping constant in N/(m/s)
   load0 = 80
   
   omega0=np.sqrt(spring/mass)
   f0 = 0.*omega0/(2*np.pi)
   f1 = 1.*omega0/(2*np.pi)
   
   print('resonance frequency = '+str(omega0/(2*pi)))
   tEnd = 200     #end time of simulation
   steps = 20000  #number of steps
   
   omegaInit = omega0*0.5
   mbs.variables['mode'] = 0           #0=linear, 1=cubic nonlinear
   mbs.variables['frequency'] = omegaInit/(2*pi)  #excitation frequency changed by user
   #mbs.variables['omega'] = omegaInit  #excitation, changed in simFunction
   mbs.variables['phi'] = 0            #excitation phase, used to get smooth excitations
   mbs.variables['stiffness'] = spring
   mbs.variables['damping'] = damper
   
   
   
   #user function for spring force
   def springForce(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       k=mbs.variables['stiffness']
       d=mbs.variables['damping']
       if mbs.variables['mode'] == 0:
           return k*u + v*d
       else:
           #return 0.1*k*u+k*u**3+v*d
           return k*u+1000*k*u**3+v*d #breaks down at 13.40Hz
   
   mode = 0 #0...forward, 1...backward
   
   # #linear frequency sweep in time interval [0, t1] and frequency interval [f0,f1];
   # def Sweep(t, t1, f0, f1):
   #     k = (f1-f0)/t1
   #     return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!
   
   # #user function for load
   # def userLoad(mbs, t, load):
   #     #return load*np.sin(0.5*omega0*t) #gives resonance
   #     #print(t)
   #     if mode==0:
   #         return load*Sweep(t, tEnd, f0, f1)
   #     else:
   #         return load*Sweep(t, tEnd, f1, f0) #backward sweep
   
   #user function for load
   def userLoad(mbs, t, load):
       #return load*sin(t*mbs.variables['frequency']*2*pi+mbs.variables['phi'])
       return load*sin(mbs.GetSensorValues(mbs.variables['sensorPhi']))
   
   #dummy user function for frequency
   def userFrequency(mbs, t, load):
       return mbs.variables['frequency']
   
   #user function used in GenericODE2 to integrate current omega
   def UFintegrateOmega(mbs, t, itemIndex, q, q_t):
       return [mbs.variables['frequency']*(2*pi)] #current frequency*2*pi is integrated into phi, return vector!
   
   #node for 3D mass point:
   nMass=mbs.AddNode(Point(referenceCoordinates = [L,0,0]))
   
   #ground node
   nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   a=L
   z=-0.1*L
   background = graphics.Quad([[-0,-a,z],[ 2*a,-a,z],[ 2*a, a,z],[0, a,z]], 
                                 color=graphics.color.lightgrey, alternatingColor=graphics.color.white)
   oGround=mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[background])))
   
   #add mass point (this is a 3D object with 3 coordinates):
   gCube = graphics.Brick([0.1*L,0,0], [0.2*L]*3, graphics.color.steelblue)
   massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = nMass,
                                       visualization=VMassPoint(graphicsData=[gCube])))
   
   #marker for ground (=fixed):
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   #marker for springDamper for first (x-)coordinate:
   nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
   
   #Spring-Damper between two marker coordinates
   mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                        stiffness = spring, damping = damper, 
                                        springForceUserFunction = springForce,
                                        visualization=VCoordinateSpringDamper(drawSize=0.05))) 
   
   #add load:
   mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                              load = load0, loadUserFunction=userLoad))
   
   #dummy load applied to ground marker, just to record/integrate frequency
   lFreq = mbs.AddLoad(LoadCoordinate(markerNumber = groundMarker, 
                              load = load0, loadUserFunction=userFrequency))
   
   sensPos = mbs.AddSensor(SensorNode(nodeNumber=nMass, fileName='solution/nonlinearPos.txt',
                                      outputVariableType=exu.OutputVariableType.Displacement))
   sensVel = mbs.AddSensor(SensorNode(nodeNumber=nMass, fileName='solution/nonlinearVel.txt',
                                      outputVariableType=exu.OutputVariableType.Velocity))
   sensFreq = mbs.AddSensor(SensorLoad(loadNumber=lFreq, fileName='solution/nonlinearFreq.txt', 
                                       visualization=VSensorLoad(show=False)))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #node used to integrate omega into phi for excitation function
   nODE2=mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0],initialCoordinates_t=[0],
                                     numberOfODE2Coordinates=1))
   
   oODE2=mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nODE2],massMatrix=np.diag([1]),
                                         forceUserFunction=UFintegrateOmega,
                                         visualization=VObjectGenericODE2(show=False)))
   #improved version, using integration of omega:
   mbs.variables['sensorPhi'] = mbs.AddSensor(SensorNode(nodeNumber=nODE2, fileName='solution/nonlinearPhi.txt', 
                                       outputVariableType = exu.OutputVariableType.Coordinates_t,
                                       visualization=VSensorNode(show=False)))
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   mbs.Assemble()
   
   
   SC.visualizationSettings.general.textSize = 16
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   #SC.visualizationSettings.window.renderWindowSize=[1024,900]
   SC.visualizationSettings.window.renderWindowSize=[1200,1080]
   SC.visualizationSettings.general.showSolverInformation = False
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #this is an exemplary simulation function, which adjusts some values for simulation
   def SimulationUF(mbs, dialog):
       #next two commands to zoom all ...:
       if mbs.variables['mode'] == 1:
           dialog.plots['limitsY'][0] = (-0.055,0.055)
       else:
           dialog.plots['limitsY'][0] = (-0.1,0.1)
   
   
   
   SC.visualizationSettings.general.autoFitScene = False #otherwise, renderState not accepted for zoom
   SC.renderer.Start()
   
   SC.renderer.SetState({'centerPoint': [0.500249445438385, -0.02912527695298195, 0.0],
    'maxSceneSize': 0.5,
    'zoom': 0.428807526826858,
    'currentWindowSize': [1400, 1200],
    'modelRotation': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]})
   time.sleep(0.5) #allow window to adjust view
   
   h = 1e-3      #step size of solver
   deltaT = 0.01 #time period to be simulated between every update
   
   #++++++++++++++++++++++++++++
   #define items for dialog
   dialogItems = [{'type':'label', 'text':'Nonlinear oscillation simulator', 'grid':(0,0,2), 'options':['L']},
                  {'type':'radio', 'textValueList':[('linear',0),('nonlinear (f=k*u+1000*k*u**3+d*v)',1)], 'value':0, 'variable':'mode', 'grid': [(2,0),(2,1)]},
                  {'type':'label', 'text':'excitation frequency (Hz):', 'grid':(5,0)},
                  {'type':'slider', 'range':(3*f1/800, 2.2*f1), 'value':omegaInit/(2*pi), 'steps':600, 'variable':'frequency', 'grid':(5,1)},
                  {'type':'label', 'text':'damping:', 'grid':(6,0)},
                  {'type':'slider', 'range': (0, 40), 'value':damper, 'steps':600, 'variable':'damping', 'grid':(6,1)},
                  {'type':'label', 'text':'stiffness:', 'grid':(7,0)},
                  {'type':'slider', 'range':(0, 10000), 'value':spring, 'steps':600, 'variable':'stiffness', 'grid':(7,1)}]
   
   #++++++++++++++++++++++++++++++++++++++++
   #specify subplots to be shown interactively
   plt.close('all')
   
   if False: #with phase
       deltaT*=0.5 #higher resolution for phase
       plots={'fontSize':16,'sizeInches':(12,12),'nPoints':200, 
              'subplots':(2,2), 'sensors':[[(sensPos,0),(sensPos,1),'time','mass position'], 
                                           [(sensFreq,0),(sensFreq,1),'time','excitation frequency'],
                                           [(sensPos,1),(sensVel,1),'position (phase space)','velocity (phase space)']
                                           ],
              'limitsX':[(),(),()], #omit if time auto-range
              'limitsY':[(-0.1,0.1),(0,2.2*f1*1.01),()]}
   else:
       plots={'fontSize':16,'sizeInches':(12,12),'nPoints':400, 
              'subplots':(2,1), 'sensors':[[(sensPos,0),(sensPos,1),'time','mass position'], 
                                           [(sensFreq,0),(sensFreq,1),'time','excitation frequency']],
              'limitsX':[(),()], #omit if time auto-range
              'limitsY':[(-0.1,0.1),(0,2.2*f1*1.01)]}
   
   #++++++++++++++++++++++++++++++++++++++++
   #setup simulation settings and run interactive dialog:
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.1 #data not used
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.1 #data not used
   simulationSettings.solutionSettings.solutionInformation = 'Nonlinear oscillations: compare linear / nonlinear case'
   simulationSettings.timeIntegration.verboseMode = 0 #turn off, because of lots of output
   
   simulationSettings.timeIntegration.numberOfSteps = int(deltaT/h)
   simulationSettings.timeIntegration.endTime = deltaT
   
   InteractiveDialog(mbs=mbs, simulationSettings=simulationSettings,
                     simulationFunction=SimulationUF, title='Interactive window',
                     dialogItems=dialogItems, period=deltaT, realtimeFactor=10,
                     plots=plots, fontSize=12)
   
   # #stop solver and close render window
   SC.renderer.Stop() #safely close rendering window!
   
   
   


