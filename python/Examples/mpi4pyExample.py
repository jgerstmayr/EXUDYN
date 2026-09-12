#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This is an example for mpi4py
#
# on linux/WSL run with: 
#           mpiexec -n 9 python3 -m mpi4py.futures mpi4pyExample.py
#           n represents 8 workers and 1 for running main script
#           on 4core/8threads optimum reached with n=9 (1 core running on 15%, all other cores around 95%)
#
# troubleshoot: you need to install mpi4py with conda; if your code starts n times, deinstall 
#               all mpi4py versions (also if installed with pip, remove it with python -m pip uninstall)
#               MAY NOT run with virtual environments (best results with conda base, Python 3.9 under linux/WSL)
#
# Author:   Johannes Gerstmayr 
# Date:     2023-03-17
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.processing import *
import time

import numpy as np
import sys

useMPI = True #True requires mpi4py to be installed




#function, which creates and runs model; executed in parallel!        
def TestExudyn(parameterDict):

    #create an environment for mini example
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    x=1
    y=1000
    computationIndex = 0
    x = parameterDict['mass']
    y = parameterDict['stiffness']
    
    oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

    node = mbs.AddNode(Node1D(referenceCoordinates = [0], 
                              initialCoordinates=[(x-0.5)**2],
                              initialVelocities=[(y-0.2)**2]))
    mass = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    #mbs.SolveDynamic(exu.SimulationSettings())

    h=1e-3
    tEnd = 100 #nominal: 10
    #tEnd = 1000
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.solutionSettings.writeSolutionToFile = False #no concurrent writing to files ...!
    #SC.renderer.Start() #don't do this in parallelization: will crash
    mbs.SolveDynamic(simulationSettings)
    #SC.renderer.Stop() #don't do this in parallelization: will crash

    #check result, get current mass position at local position [0,0,0]
    result = mbs.GetObjectOutputBody(mass, exu.OutputVariableType.Position, [0,0,0])[0]
    #print("result ",x, "=",result)
    
    del mbs #dont forget to delete variables, otherwise memory may leak significantly
    del SC
    return result
    #final x-coordinate of position shall be 2

#now run parallelized parameter variation; 
#make sure that this only runs in main process:
if __name__ == '__main__':
    n=640
    start_time = time.time()
    print('parameter variation '+'with MPI'*useMPI)
    [p,v]=ParameterVariation(parameterFunction = TestExudyn,
                             parameters = {'mass':(1.,1.,1), 'stiffness':(1000,2000,n)}, 
                             # debugMode=True,
                             addComputationIndex = True,
                             useMultiProcessing = True, 
                             showProgress = True,
                             #numberOfThreads=8, #automatically determined by mpi4py routines in ParameterVariationList(...)
                             resultsFile='solution/resultsMPI.txt',
                             useMPI = useMPI,
                             ) 
    print("--- %s seconds ---" % (time.time() - start_time))
    #print("values=",v)
    print('sum=',np.array(v).sum()) #gives sum= 14931163024.24202 with default values

        
# old, manual implementation of parameter variation with mpi
# if useMPI:
#     import mpi4py
#     from mpi4py import MPI
#    
#     comm = MPI.COMM_WORLD
#     nprocs = comm.Get_size()
#     rank   = comm.Get_rank() 
#     print('rank=', rank, ', size=', nprocs)
#    
#     from mpi4py.futures import MPIPoolExecutor
#
#
# if __name__ == '__main__' and useMPI:
#     #MPI.Init()      # manual initialization of the MPI environment
#     print('mpi4py test program\n')
#     x=[]
#     y=np.arange(1,10)
#     #executor = MPIPoolExecutor(max_workers=8)
#     executor = MPIPoolExecutor()
#     #for result in executor.map(fmpi, [1,2,3,4]):
#     for i in range(n):
#         x+=[{'mass':1,
#              'stiffness':1000+1000*i/(n-1),
#              'computationIndex':i}]
#     #print('x=',x)
#     v=[]
#     if False:
#         start_time = time.time()
#         for result in executor.map(TestExudyn, x):
#             v.append(result)
#         print("--- %s seconds ---" % (time.time() - start_time))
#     else:
#         nVariations=n
#         import tqdm #progress bar
#         try: #_instances only available after first run!
#             tqdm.tqdm._instances.clear() #if open instances of tqdm, which leads to nasty newline
#         except:
#             pass
#         useTQDM = True
    
#         start_time = time.time()
#         #for v in (tqdm.tqdm(p.imap(parameterFunction, vInput), total=nVariations)):
#         for result in (tqdm.tqdm(executor.map(TestExudyn, x), total=nVariations)):
#             v.append(result)
#         print("--- %s seconds ---" % (time.time() - start_time))
    
#     #print('rank=',rank)
#     print('sum=',np.array(v).sum())
#     #MPI.Finalize()  # manual finalization of the MPI environment


