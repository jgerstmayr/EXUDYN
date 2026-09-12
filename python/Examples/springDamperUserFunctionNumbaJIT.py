#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test with user-defined load function and user-defined spring-damper function (Duffing oscillator)
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.exudynFast = True

from exudyn.utilities import ClearWorkspace
ClearWorkspace()

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#NUMBA PART; mainly, we need to register MainSystem mbs in numba to get user functions work
#import numba jit for compilation of functions:
# from numba import jit

#create identity operator for replacement of jit:
try: 
    from numba import jit
    print('running WITH JIT')
except: #define replacement operator
    print('running WITHOUT JIT')
    def jit(ob):
        return ob

# from numba import jit, cfunc, types, njit
# from numba.types import float64, void, int64 #for signatures of user functions!
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


# @jit
# def myfunc():
#     print("my function")

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


useGraphics = False #without test


SC = exu.SystemContainer()
mbs = SC.AddSystem()
exu.Print('EXUDYN version='+exu.config.Version())

L=0.5
mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 4          #damping constant in N/(m/s)
load0 = 80

omega0=np.sqrt(spring/mass)
f0 = 0.*omega0/(2*np.pi)
f1 = 1.*omega0/(2*np.pi)

exu.Print('resonance frequency = '+str(omega0))
tEnd = 50     #end time of simulation
steps = 1000000  #number of steps

#first test without JIT:

def sf(u,v,k,d):
    return 0.1*k*u+k*u**3 + 1e-3*k*u**5 + 1e-6*k*u**7+v*d

def springForce(mbs2, t, itemIndex, u, v, k, d, offset):
    return sf(u,v,k,d)
    # x=test(mbs.systemData.GetTime()) #5 microseconds
    # q=mbs.systemData.GetODE2Coordinates() #5 microseconds
    # return 0.1*k*u+k*u**3+v*d

#linear frequency sweep in time interval [0, t1] and frequency interval [f0,f1];
def Sweep(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#user function for load; void replaces mbs, which then may not be used!!!
#most time lost due to pybind11 std::function capturing; no simple way to overcome problem at this point (avoid many function calls!)
#@cfunc(float64(void, float64, float64)) #possible, but does not lead to speed up
#@jit #not possible because of mbs not recognized by numba
def userLoad(mbs, t, load):
    #x=mbs.systemData.GetTime() #call to systemData function takes around 5us ! Cannot be optimized!
    #global tEnd, f0, f1 #global does not change performance
    return load*Sweep(t, tEnd, f0, f1) #global variable does not seem to make problems!

#node for 3D mass point:
n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0]))

#ground node
nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))

#add mass point (this is a 3D object with 3 coordinates):
massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

#marker for ground (=fixed):
groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
#marker for springDamper for first (x-)coordinate:
nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))

#Spring-Damper between two marker coordinates
oSD=mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                     stiffness = spring, damping = damper, 
                                     springForceUserFunction = springForce,
                                     )) 

#add load:
loadC = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                           load = load0, 
                           loadUserFunction=userLoad,
                           ))

mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.numberOfSteps = steps
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.newton.useModifiedNewton=True

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1

simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

#start solver:
mbs.SolveDynamic(simulationSettings)

#evaluate final (=current) output values
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
exu.Print('displacement=',u[0])


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#run again with JIT included:

#use jit for every time-consuming parts
#the more complex it gets, the speedup will be larger!
#however, this part can only contain simple structures (no mbs, no exudyn functions [but you could @jit them!])
@jit
def sf2(u,v,k,d):
    return 0.1*k*u+k*u**3 + 1e-3*k*u**5 + 1e-6*k*u**7+v*d

def springForce2(mbs2, t, itemIndex, u, v, k, d, offset):
    return sf2(u,v,k,d)

# jit for both sub-functions of user functions:
mbs.SetObjectParameter(oSD, 'springForceUserFunction', springForce2)

#jit gives us speedup and works out of the box:
@jit
def Sweep2(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#user function for load; void replaces mbs, which then may not be used!!!
# @cfunc(float64(void, float64, float64), nopython=True, fastmath=True) #possible, but does not lead to speed up
def userLoad2(mbs, t, load):
    return load*Sweep2(t, tEnd, f0, f1) #global variable does not seem to make problems!

mbs.SetLoadParameter(loadC,'loadUserFunction', userLoad2)

mbs.SolveDynamic(simulationSettings)

#evaluate final (=current) output values
u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
exu.Print('JIT, displacement=',u[0])


#performance:
#1e6 time steps
# no user functions:
# tCPU=1.15 seconds

# regular, Python user function for spring-damper and load:
# tCPU=16.7 seconds
    
# jit, Python user function for spring-damper and load:
# tCPU=5.58 seconds (on average)
#==>speedup of user function part: 16.7/(5.58-1.15)=4.43
#speedup will be much larger if Python functions are larger!
#approx. 400.000 Python function calls/second!
