
.. _examples-nmassoscillatorinteractive:

*****************************
nMassOscillatorInteractive.py
*****************************

You can view and download this file on Github: `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_

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
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import matplotlib.pyplot as plt
   from exudyn.interactive import InteractiveDialog
   
   import numpy as np
   from math import sin, cos, pi, sqrt
   
   import time #for sleep()
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #
   N = 12;                     #number of masses
   spring = 800;               #stiffness [800]
   mass = 1;                   #mass
   damper = 2;               #old:0.1; damping parameter
   force = 1;                 #force amplitude
   
   d0 = damper*spring/(2*sqrt(mass*spring))  #dimensionless damping for single mass
   
   
   caseHarmonic = 1
   #damper=2
   #mode1:force=0.52
   #mode2:force=2
   #mode3:force=4
   #mode12: force=100, damping=0.05, period=0.005
   
   caseStep = 2
   #damper=1
   #force=20
   
   
   mbs.variables['loadCase'] = caseHarmonic
   mbs.variables['resetMotion'] = 0 #run
   mbs.variables['forceAmplitude'] = force
   eigenMode = 1
   
   h = 0.002            #step size
   deltaT = 0.01 #time period to be simulated between every update
   
   
   
   # if (mode < 2) h=h*2; F=0.4*F; end
   # if (mode < 5) h=h*2; F=0.4*F; end
   # if (mode < 6) h=h*2.5; end
   # if (mode == 6) F=F*2; end
   # if (mode == 3) h=h*0.5; end
   
   # if (mode > 16) F=2*F; end
   # if (mode > 10) F=2*F; end
   # if (N < 11) h=h/2; l_mass = 2*l_mass; r_mass=2*r_mass; F=0.5*F; end
   # if (N < 6) h=h/2; l_mass = 2*l_mass; r_mass=2*r_mass; end
   
   # om=sqrt(diag(ew))
   
   
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create model for linear and nonlinear oscillator:
   # L=0.5
   # load0 = 80
   
   # omega0=np.sqrt(spring/mass)
   # f0 = 0.*omega0/(2*np.pi)
   # f1 = 1.*omega0/(2*np.pi)
   
   # tEnd = 200     #end time of simulation
   # steps = 20000  #number of steps
   
   omegaInit = 3.55
   omegaMax = 40 #for plots
   mbs.variables['mode'] = 0           #0=linear, 1=cubic nonlinear
   mbs.variables['omega'] = omegaInit  #excitation frequency changed by user
   #mbs.variables['omega'] = omegaInit #excitation, changed in simFunction
   mbs.variables['phi'] = 0            #excitation phase, used to get smooth excitations
   mbs.variables['stiffness'] = spring
   mbs.variables['damping'] = damper
   mbs.variables['dampingPrev'] = damper
   
   
   # #user function for spring force
   # def springForce(mbs, t, itemIndex, u, v, k, d): #changed 2023-01-21:, mu, muPropZone):
   #     k=mbs.variables['stiffness']
   #     d=mbs.variables['damping']
   #     if mbs.variables['mode'] == 0:
   #         return k*u + v*d
   #     else:
   #         #return 0.1*k*u+k*u**3+v*d
   #         return k*u+1000*k*u**3+v*d #breaks down at 13.40Hz
   
   # mode = 0 #0...forward, 1...backward
   
   #user function for load
   def userLoad(mbs, t, load):
       f = mbs.variables['forceAmplitude']
       fact = 1
       if mbs.variables['loadCase']==caseHarmonic:
           fact = sin(mbs.GetSensorValues(mbs.variables['sensorPhi']))
       return f*fact
   
   def userLoad3D(mbs,t, load):
       f = mbs.variables['forceAmplitude']
       fact = 1
       if mbs.variables['loadCase']==caseHarmonic:
           fact = sin(mbs.GetSensorValues(mbs.variables['sensorPhi']))
       mbs.SetLoadParameter(0,'loadVector',[fact,0,0])
       return [f*fact,0,0]
       
   #dummy user function for frequency
   def userFrequency(mbs, t, load):
       return mbs.variables['omega']
   
   #user function used in GenericODE2 to integrate current omega
   def UFintegrateOmega(mbs, t, itemIndex, q, q_t):
       return [mbs.variables['omega']] #current frequency*2*pi is integrated into phi, return vector!
   
   #ground node
   nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
   #drawing parameters:
   l_mass = 0.2          #spring length
   r_mass = 0.030*2       #radius of mass
   r_spring = r_mass*1.2
   L0 = l_mass*1
   L = N * l_mass + 4*l_mass
   z=-r_mass-0.1
   hy=0.25*L
   hy1=2*hy - 4*r_mass
   hy0=-4*r_mass
   maxAmp0 = 0.1
   maxAmpN = 0.1*N
   
   background = [graphics.Quad([[-L0,hy0,z],[ L,hy0,z],[ L,hy1,z],[-L0,hy1,z]], 
                                 color=graphics.color.lightgrey)]
   offCircleY = 1*hy
   for i in range(N):
       t=r_mass*0.5
       ox = l_mass*(i+1)
       oy = offCircleY
       line0 = {'type':'Line', 'data':[ox-t,oy+0,0, ox+t,oy+0,0], 'color':graphics.color.grey}
       line1 = {'type':'Line', 'data':[ox+0,oy-t,0, ox+0,oy+t,0], 'color':graphics.color.grey}
       background += [line0, line1]
       
   oGround=mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=background)))
   #marker for ground (=fixed):
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   prevMarker = groundMarker
   nMass = []
   mbs.variables['springDamperList'] = []
   
   for i in range(N):
       #node for 3D mass point:
       col = graphics.color.steelblue
       if i==0:
           col = graphics.color.green
       elif i==N-1:
           col = graphics.color.lightred
   
       gSphere = graphics.Sphere(point=[0,0,0], radius=r_mass, color=col, nTiles=16)
       node = mbs.AddNode(Node1D(referenceCoordinates = [l_mass*(1+len(nMass))],
                                 initialCoordinates=[0.],
                                 initialVelocities=[0.]))
       massPoint = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=mass,
                                        referencePosition=[0,0,0],
                                        visualization=VMass1D(graphicsData=[gSphere])))
   
       gCircle = {'type':'Circle','position':[0,0,0],'radius':0.5*r_mass, 'color':col}
       massPoint2 = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=0,
                                        referencePosition=[l_mass*(len(nMass)+1),offCircleY-l_mass*(len(nMass)+1),0], 
                                        referenceRotation=[[0,1,0],[1,0,0],[0,0,1]],
                                        visualization=VMass1D(graphicsData=[gCircle])))
   
   
       # node = mbs.AddNode(Point(referenceCoordinates = [l_mass*(1+len(nMass)),0,0]))    
      
       # massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = node,
       #                                     visualization=VMassPoint(graphicsData=[gSphere])))
       
       nMass += [node]
       #marker for springDamper for first (x-)coordinate:
       nodeMarker =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= node, coordinate = 0))
       
       #Spring-Damper between two marker coordinates
       sd = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [prevMarker, nodeMarker], 
                                            stiffness = spring, damping = damper, 
                                            #springForceUserFunction = springForce,
                                            visualization=VCoordinateSpringDamper(drawSize=r_spring))) 
       mbs.variables['springDamperList'] += [sd]
       prevMarker = nodeMarker
       
   #add load to last mass:
   if False: #scalar load
       mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                  load = 0, loadUserFunction=userLoad)) #load set in user function
   else:
       mMassN = mbs.AddMarker(MarkerBodyPosition(bodyNumber= massPoint, localPosition=[0,0,0]))
       mbs.AddLoad(Force(markerNumber=mMassN, loadVector=[1,0,0],
                         loadVectorUserFunction=userLoad3D))
   
   # #dummy load applied to ground marker, just to record/integrate frequency
   lFreq = mbs.AddLoad(LoadCoordinate(markerNumber = groundMarker, 
                                      load = 0, loadUserFunction=userFrequency))
   
   sensPos0 = mbs.AddSensor(SensorNode(nodeNumber=nMass[0], fileName='solution/nMassPos0.txt',
                                       outputVariableType=exu.OutputVariableType.Coordinates))
   sensPosN = mbs.AddSensor(SensorNode(nodeNumber=nMass[-1], fileName='solution/nMassPosN.txt',
                                       outputVariableType=exu.OutputVariableType.Coordinates))
   sensFreq = mbs.AddSensor(SensorLoad(loadNumber=lFreq, fileName='solution/nMassFreq.txt', 
                                       visualization=VSensorLoad(show=False)))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute eigenvalues
   from exudyn.solver import ComputeODE2Eigenvalues
   mbs.Assemble()
   [values, vectors] = ComputeODE2Eigenvalues(mbs)
   print('omegas (rad/s)=', np.sqrt(values))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #integrate omega: node used to integrate omega into phi for excitation function
   nODE2=mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0],initialCoordinates_t=[0],
                                     numberOfODE2Coordinates=1))
   
   oODE2=mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nODE2],massMatrix=np.eye(1),
                                         forceUserFunction=UFintegrateOmega,
                                         visualization=VObjectGenericODE2(show=False)))
   #improved version, using integration of omega:
   mbs.variables['sensorPhi'] = mbs.AddSensor(SensorNode(nodeNumber=nODE2, fileName='solution/nonlinearPhi.txt', 
                                       outputVariableType = exu.OutputVariableType.Coordinates_t,
                                       visualization=VSensorNode(show=False)))
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #finalize model and settings
   mbs.Assemble()
   
   
   SC.visualizationSettings.general.textSize = 12
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.005
   #SC.visualizationSettings.window.renderWindowSize=[1024,900]
   SC.visualizationSettings.window.renderWindowSize=[1600,1000]
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.general.drawCoordinateSystem = False
   
   SC.visualizationSettings.loads.fixedLoadSize=0
   SC.visualizationSettings.loads.loadSizeFactor=0.5
   SC.visualizationSettings.loads.drawSimplified=False
   SC.visualizationSettings.loads.defaultSize=1
   SC.visualizationSettings.loads.defaultRadius=0.01
   
   SC.visualizationSettings.general.autoFitScene = True #otherwise, renderState not accepted for zoom
   SC.renderer.Start()
   
   #++++++++++++++++++++++++++++++++++++++++
   #setup simulation settings and run interactive dialog:
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.1 #data not used
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.1 #data not used
   simulationSettings.solutionSettings.solutionInformation = 'n-mass-oscillatior'
   simulationSettings.timeIntegration.verboseMode = 0 #turn off, because of lots of output
   
   simulationSettings.timeIntegration.numberOfSteps = int(deltaT/h)
   simulationSettings.timeIntegration.endTime = deltaT
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.displayComputationTime = True
   simulationSettings.parallel.numberOfThreads = 2
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #set up interactive window
   
   #interactive function
   def SimulationUF(mbs, dialog):
       if mbs.variables['resetMotion']:
           u = mbs.systemData.GetODE2Coordinates()
           u *= 0
           mbs.systemData.SetODE2Coordinates(u, configuration=exu.ConfigurationType.Initial)
           v = mbs.systemData.GetODE2Coordinates_t()
           v *= 0
           mbs.systemData.SetODE2Coordinates_t(v, configuration=exu.ConfigurationType.Initial)
           #clear plot data:
           nPoints = dialog.plots['nPoints']
           t = mbs.systemData.GetTime()
           p = mbs.variables['period']
           for j in range(len(dialog.plots['sensors'])):
               #dialog.plots['data'][j][:,1] *= 0
               # tEnd = dialog.plots['data'][j][:,0]
               dialog.plots['data'][j] = np.zeros((nPoints,2))
               dialog.plots['data'][j][:,0] = p*np.arange(0,nPoints) + ( t-2*p*(nPoints))
           dialog.plots['currentIndex'] = nPoints-1
   
       if mbs.variables['dampingPrev'] != mbs.variables['damping']:
           #print('here')
           for sd in mbs.variables['springDamperList']:
               mbs.SetObjectParameter(sd,'damping',mbs.variables['damping'])
           mbs.variables['dampingPrev'] = mbs.variables['damping']
   
       dialog.period = mbs.variables['period']
   
   #++++++++++++++++++++++++++++
   #define items for dialog
   dialogItems = [{'type':'label', 'text':'n-mass oscillatior simulator', 'grid':(0,0,2), 'options':['L']},
                  {'type':'radio', 'textValueList':[('harmonic',caseHarmonic),('step',caseStep)], 'value': mbs.variables['loadCase'], 'variable':'loadCase', 'grid': [(2,0),(2,1)]},
                  {'type':'label', 'text':'excitation frequency (rad/s):', 'grid':(5,0)},
                  {'type':'slider', 'range':(0, 60), 'value':omegaInit, 'steps':601, 'variable':'omega', 'grid':(5,1)},
                  {'type':'radio', 'textValueList':[('run',0),('reset motion',1)], 'value': mbs.variables['resetMotion'], 'variable':'resetMotion', 'grid': [(6,0),(6,1)]},
                  {'type':'label', 'text':'force amplitude:', 'grid':(7,0)},
                  {'type':'slider', 'range': (0, 100), 'value':mbs.variables['forceAmplitude'], 'steps':1001, 'variable':'forceAmplitude', 'grid':(7,1)},
                  {'type':'label', 'text':'damping:', 'grid':(8,0)},
                  {'type':'slider', 'range': (0, 40), 'value':damper, 'steps':801, 'variable':'damping', 'grid':(8,1)},
                  {'type':'label', 'text':'period:', 'grid':(9,0)},
                  {'type':'slider', 'range': (0.001, 0.1), 'value':deltaT, 'steps':100, 'variable':'period', 'grid':(9,1)},
                  # {'type':'label', 'text':'stiffness:', 'grid':(7,0)},
                  # {'type':'slider', 'range':(0, 1000), 'value':spring, 'steps':500, 'variable':'stiffness', 'grid':(7,1)}
                  ]
   
   #++++++++++++++++++++++++++++++++++++++++
   #specify subplots to be shown interactively
   plt.close('all')
   
   plots={'fontSize':12,'sizeInches':(16,12),'nPoints':400, 
          'subplots':(2,1), 
          'sensors':[[(sensPos0,0),(sensPos0,1),'time','mass0 position'], 
                     [(sensPosN,0),(sensPosN,1),'time','massN position'], 
                     #[(sensFreq,0),(sensFreq,1),'time','excitation freq.(rad/s)']
                     ],
          'subplots':False,
          'lineStyles':['g-','r-'],
          'limitsX':[(),()], #omit if time auto-range
          'limitsY':[(),()]}#,(0,omegaMax)]}
          # 'limitsY':[(-maxAmp0,maxAmp0),(-maxAmpN,maxAmpN)]}#,(0,omegaMax)]}
   
   
   InteractiveDialog(mbs=mbs, simulationSettings=simulationSettings,
                     simulationFunction=SimulationUF, title='Interactive window',
                     dialogItems=dialogItems, period=deltaT, 
                     #realtimeFactor=1,
                     plots=plots, 
                     fontSize=12)
   
   # #stop solver and close render window
   SC.renderer.Stop() #safely close rendering window!
   
   
   


