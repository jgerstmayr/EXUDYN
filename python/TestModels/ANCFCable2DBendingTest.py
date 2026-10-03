#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A cantilever of `ANCFCable2D` elements, clamped at one end and pulled down by a tip
#           force. Solved dynamically once and statically twice, the second time with the Newton
#           tolerances at 1e-14 so that it converges onto the MATLAB reference it is compared with.
#           The model compares against a reference value written into it, so its
#           result is that difference and its reference solution is 0 (#2632).
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01, reworked 2026-09-24
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)
exu.sys['testTolerance'] = 4e-13 #the tolerance RunAllModelUnitTests used for these ten

SC = exu.SystemContainer()
mbs = SC.AddSystem()


totalError = 0 #total error computed in test

L=2                    # length of ANCF element in m
Em=2.07e11             # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.1                  # width of rectangular ANCF element in m
h=0.1                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
f=3*Em*I/L**2           # tip load applied to ANCF element in N

EI = Em*I
rhoA = rho*A
EA = Em*A


nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))

nElements = 1
lElem = L / nElements

for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    elem = mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rhoA, 
                                 physicsBendingStiffness=EI, physicsAxialStiffness=EA, 
                                 nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))

#tip node / force
mANCFnode = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast))
mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, -f, 0]))


#ground node / coordinate
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action; coordinate number does not matter

#constraints
mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3)) #3
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0],velocityLevel=True))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1],velocityLevel=True))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2],velocityLevel=True))

#Assemble and Solve:
mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.solution.file.name = "solution/ANCFCable2D_bending_test.txt"
simulationSettings.solution.file.write=False
simulationSettings.timeIntegration.numberOfSteps = 1000
#simulationSettings.solution.file.writePeriod = simulationSettings.timeIntegration.endTime/1000
simulationSettings.timeIntegration.endTime = 0.1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.show.statistics = True

if not testIsActive: 
    SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)

sol = mbs.systemData.GetODE2Coordinates(); n = len(sol)
#tip displacements:
u = sol[n-4];v = sol[n-3] #15.12.2019:(-1.040678127615946 -1.444419986874761); 20.10.2019: (-1.0406781266430292 -1.4444199866881322); 17.10.2019: sol= -1.040678126647053 -1.4444199866858678; #28.7.2009: -1.040678126643273, -1.444419986688082
if True:
    totalError += u+v - (-1.0611305415798655 -1.4396907464589017)  #2021-09-27: new JacobianODE2RHS
    #totalError += u+v -(-1.0611305415779122 -1.4396907464583975)  #2021-02-04: (-1.0611305415779122 -1.4396907464583975)
else:
    totalError += u+v - (-1.0391620828192676 -1.4443521331339881)  #2019-12-26: (-1.0391620828192676 -1.4443521331339881) old (before correct initial accelerations): (-1.040678127615946 -1.444419986874761)
exu.Print('sol dynamic=',u,v)
#exu.Print('time integration error =',totalError)

mbs.SolveStatic(simulationSettings)

sol = mbs.systemData.GetODE2Coordinates(); n = len(sol)
u = sol[n-4]; v = sol[n-3]; #20.10.2019: -0.3622447299987188 -0.9941447593196007; 17.10.2019: sol= -0.3622447299990847 -0.9941447593206921; #28.7.2019: -0.3622447300008477, -0.994144759326213
exu.Print('sol static (standardTol)=',u,v)
totalError += u+v - (-0.36224473018839665 -0.9941447595447153) #2021-09-27: new JacobianODE2RHS
# totalError += u+v-(-0.3622447299987188 -0.9941447593196007)

simulationSettings.staticSolver.newton.relativeTolerance = 1e-14 #in order to converge to MATLAB results
simulationSettings.staticSolver.newton.absoluteTolerance = 1e-14

mbs.SolveStatic(simulationSettings)

sol = mbs.systemData.GetODE2Coordinates(); n = len(sol)
#tip displacements: paper GerstmIschrik2008: 1Element: u=-0.362244729891,  v=-0.994144758725; 4 Elements: 0.507428715119 1.205533702233
u = sol[n-4]; v = sol[n-3];                 #2019-12-17: -0.3622447298904951 -0.9941447587249616
exu.Print('sol static (tol=1e-14)=',u,v)
totalError += u+v - (-0.36224472989050654 -0.9941447587249747) #2021-09-27: new JacobianODE2RHS
# totalError += u+v-(-0.3622447298904951 -0.9941447587249616)


#totalError -= -1.3563894893270607-2.4850981133313548 #reference solution with one element and standard settings except: gen-alpha=0.6, useModifiedNewton = False

if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

testResult = totalError
exu.Print('solution of ANCFCable2DBendingTest=', testResult)
exu.sys['testResult'] = testResult
