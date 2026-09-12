#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for computation of eigenvalues using utility eigensolver functionality based on scipy.linalg
#
# Author:   Johannes Gerstmayr
# Date:     2020-12-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
import numpy as np

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


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2.                   # length of ANCF element in m
#L=mypi                 # length of ANCF element in m
E=2.07e11              # Young's modulus of ANCF element in N/m^2
rho=7800               # density of ANCF element in kg/m^3
b=0.01                  # width of rectangular ANCF element in m
h=0.01                  # height of rectangular ANCF element in m
A=b*h                  # cross sectional area of ANCF element in m^2
I=b*h**3/12            # second moment of area of ANCF element in m^4
EI = E*I
rhoA = rho*A

exu.Print("EI="+str(EI))
exu.Print("rhoA="+str(rhoA))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]



nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
nElements = 32 #32
lElem = L / nElements
for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    elem=mbs.AddObject(Cable2D(physicsLength=lElem, 
                               physicsMassPerLength=rho*A, 
                               physicsBendingStiffness=E*I, 
                               physicsAxialStiffness=E*A*0.1, 
                               nodeNumbers=[int(nc0)+i,int(nc0)+i+1], 
                               useReducedOrderIntegration=True))
    cableList+=[elem]


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.staticSolver.verboseMode = 1

nEig = 3
[values, vectors] = mbs.ComputeODE2Eigenvalues(simulationSettings, 
                                               numberOfEigenvalues = nEig+3)    #3 eigenvalues + 3 rigid body zero eigenvalues


omegaNumerical = np.sqrt(values[3:nEig+3])
exu.Print("eigenvalues=",omegaNumerical) #exclude 3 rigid body modes
#[ 83.17966459 229.28844645 449.50021798] 

#analytical: bending eigenfrequency of free-free beam:
#4.7300, 7.8532, 10.9956, 14.1371, 17.2787 (cosh(beta) * cos(beta) = 1)
#find roots beta:
#from mpmath import *
#mp.dps = 16 #digits
#for i in range(10): exu.Print(findroot(lambda x: cosh(x) * cos(x) - 1, 3*i+4.7))
beta = [4.730040744862704, 7.853204624095838, 10.99560783800167, 14.13716549125746, 17.27875965739948, 20.42035224562606, 23.56194490204046, 26.70353755550819, 29.84513020910325]
omega = np.zeros(nEig)
for i in range(nEig):
    omega[i] = ((beta[i]/L)**4 * (EI/rhoA))**0.5

exu.Print('omega analytical =',omega)
u = omega[0]-omegaNumerical[0]
exu.Print('omega difference=',u)

exudynTestGlobals.testError = 1e-6*(u - (-2.7613614363986017e-05)) #2021-01-04: added factor 1e-6, because of larger errors/differences in 32/64bit eigenvalue solvers; 2020-12-18: (nElements=32) -2.7613614363986017e-05
exudynTestGlobals.testResult = 1e-6*u




