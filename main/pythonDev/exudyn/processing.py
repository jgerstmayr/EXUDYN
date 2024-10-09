#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The processing module supports multiple execution of EXUDYN models.
#           It includes parameter variation and (genetic) optimization functionality.
#
# Author:   Johannes Gerstmayr, Stefan Holzinger
# Date:     2020-11-17 (2022-02-04 modified by Stefan Holzinger)
# Notes:    Parallel processing, which requires multiprocessing library, can lead to considerable speedup (measured speedup factor > 50 on 80 core machine). The progess bar during multiprocessing requires the library tqdm.
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import numpy as np
import sys
import time
from copy import deepcopy #, copy 
import os
from exudyn.advancedUtilities import IsInteger, IsEmptyList

#%%+++++++++++++++++++++++++++++++++++++++++++
#**function: internal function to return Exudyn version string, which allows to identify how results have been obtained
#            writes something like 'Exudyn version = 1.2.33.dev1; Python3.9.11; Windows AVX2 FLOAT64; Windows10 V10.0.19044; AMD64; Intel64 Family 6 Model 142 Stepping 10, GenuineIntel'
#**notes: If exudyn C++ module is not available, it outputs the Python version
def GetVersionPlatformString():
    sReturn = ''
    try:
        import exudyn
        sReturn += 'Exudyn version = '
        sReturn += exudyn.GetVersionString(True)
    except:
        import sys
        #the micro version may be different!
        sReturn += '(no exudyn) Python'+str(sys.version_info.major)+'.'+str(sys.version_info.minor)+'.'+str(sys.version_info.micro)
        
    try: #put into try except block to avoid problems with new machines ...
        import platform
        sReturn += '; ' + platform.uname().system + platform.uname().release
        if platform.uname().version != '':
            sReturn += ' V' + platform.uname().version
        sReturn += '; ' + platform.uname().machine + '; ' + platform.uname().processor
    except:
        pass

    return sReturn

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#*function: internal function; convert single index i into subindices based on given list of ranges in subIndexRanges; return list of subindices for sub ranges; ordering is according to way computed in ParameterVariation
# try following to see effects: for i in range(100): print(SingleIndex2SubIndices(i,[10,2,3,2]))
def SingleIndex2SubIndices(i, subIndexRanges):
    nr = len(subIndexRanges)
    iRanges=[]
    for k in range(nr):
        rem = 1
        for j in range(k+1,nr):
            rem *= subIndexRanges[j]
        # print('k=',k,'rem=',rem)
        kk = int(i/rem)%subIndexRanges[k]
        iRanges += [kk]
    # print('iRanges=',iRanges)
    return iRanges
    

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#*function: internal function; add computation Index and parameterFunctionData to dictionry entry
# 
def AddComputationIndexAndFunctionData(ind, cnt, addComputationIndex, parameterFunctionData): 
    if addComputationIndex:
        ind['computationIndex'] = cnt
    if bool(parameterFunctionData): # if dict is not empty
        ind['functionData'] = parameterFunctionData
    return


#%%+++++++++++++++++++++++++++++++++++++++++++
#function: internal output function for ParameterVariation and GeneticOptimization
# write header or values to output file and increase counter
def WriteToFile(resultsFile, parameters, currentGeneration, values, globalCnt, writeHeader = False, fileType='genetic optimization', multiProcessingMode=''):
    if resultsFile != '':
        #print('write to file')
        if writeHeader:
            try:
                #check only added in case of writeHeader to reduce overheads; may fail, if no header is written (but why...?)
                os.makedirs(os.path.dirname(resultsFile), exist_ok=True)
            except:
                pass #makedirs may fail on some systems, but we keep going

            file = open(resultsFile, 'w')
            file.write('#EXUDYN '+fileType+' results file:'+resultsFile+'\n')
            file.write('#results stored columnwise for every parameter and individual\n')
            sVersion = GetVersionPlatformString()
            if multiProcessingMode != '':
                sVersion += '(processing='+multiProcessingMode+')'
            file.write('#'+sVersion+'\n')
            if 'functionData' in parameters:
                file.write('functionData:'+str(parameters['functionData'])+'\n')
            else:
                file.write('#\n')
            file.write('#columns:\n') #'globalIndex, parameters, computationIndex:\n')
            s = '#globalIndex,value'
            for (key,value) in parameters.items():
                if key != 'functionData':
                    s += ',' + key
            s += ',computationIndex'
            file.write(s+'\n')
            file.write('#parameter ranges [format: (begin, end, numberOfVariations) or list, parameters separated with ";"]:\n')
            sep = ''
            s = '#'
            for (key,value) in parameters.items():
                if key != 'functionData':
                    s += sep + str(value)
                    sep = ';'
            file.write(s+'\n')
            file.close()
        
        file = open(resultsFile, 'a')
        for i in range(len(values)):
            s = ''
            s += str(globalCnt) + ', '
            s += str(values[i])
            for (key,value) in currentGeneration[i].items():
                #print(currentGeneration[i])
                if key != 'functionData':
                    s += ', ' + str(value)
            file.write(s+'\n')
            globalCnt += 1 #for every line of values
            
        file.close()
        #print('... done')
    return globalCnt
    


#**function: processes parameterFunction for given parameters in parameterList, see ParameterVariation
#**input:
#    parameterFunction: function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
#    parameterList: list of parameter sets (as dictionaries) which are fed into the parameter variation, see example 
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#    resultsFile: if provided, output is immediately written to resultsFile during processing
#    clusterHostNames: list of hostnames, e.g. clusterHostNames=['123.124.125.126','123.124.125.127'] providing a list of strings with IP addresses or host names, see dispy documentation. If list is non-empty and useMultiProcessing==True and dispy is installed, cluster computation is used; NOTE that cluster computation speedup factors shown are not fully true, as they include a significant overhead; thus, only for computations which take longer than 1-5 seconds and for sufficient network bandwith, the speedup is roughly true
#    useDispyWebMonitor: if given in **kwargs, a web browser is startet in case of cluster computation to manage the cluster during computation
#    useMPI: if given in **kwargs and set True, and if Python package mpi4py is installed, mpi parallelization is used; for hints see parameterVariationExample.py
#**output: returns values containing the results according to parameterList
#**notes: options are passed from Parametervariation
#**example:
#def PF(parameterSet):
#    #in reality, value will be result of a complex exudyn simulation:
#    value = sin(parameterSet['mass']) * parameterSet['stiffness']
#    return value
#
#values=ProcessParameterList(parameterFunction=PF, 
#                            parameterList=[{'m':1, 's':100},
#                                          {'m':2, 's':100},
#                                          {'m':3, 's':100},
#                                          {'m':1, 's':200},
#                                          {'m':2, 's':250},
#                                          {'m':3, 's':300},
#                                          ], useMultiProcessing=False )
def ProcessParameterList(parameterFunction, parameterList, useMultiProcessing, clusterHostNames=[], **kwargs):
    values = [] #create empty list
    nVariations = len(parameterList)
    #print("pl=",parameterList)
    showProgress = False
    if 'showProgress' in kwargs: 
        showProgress = kwargs['showProgress']

    resultsFile = ''
    if 'resultsFile' in kwargs: 
        resultsFile = kwargs['resultsFile']

    parameters = {}
    if 'parameters' in kwargs: 
        parameters = kwargs['parameters']

    useCluster = (clusterHostNames != []) and useMultiProcessing
    if useCluster:
        try:
            import dispy
        except:
            print('WARNING: ProcessParameterList: dispy is not installed (try: pip install dispy); switching to multiprocessing mode instead')
            useCluster = False

    useMPI = False
    if 'useMPI' in kwargs and useMultiProcessing:
        useMPI = kwargs['useMPI']
        if useMPI:
            try:
                from mpi4py.futures import MPIPoolExecutor
            except:
                print('WARNING: ProcessParameterList: mpi4py is not installed or mpi4py.futures not available (try: conda install mpi4py); switching to multiprocessing mode instead')
                useMPI = False

    useTQDM = False
    if showProgress and useMultiProcessing and not useCluster:
        try:
            import tqdm #progress bar
            try: #_instances only available after first run!
                tqdm.tqdm._instances.clear() #if open instances of tqdm, which leads to nasty newline
            except:
                pass
            useTQDM = True
        except:
            pass
            #print("module 'tqdm' not available (use pip to install); progress bar not shown")

    resultsFileCnt = 0 #counter for results file; used in several if branches

    if not useCluster and not useMPI:
        if not useMultiProcessing:
            for i in range(nVariations):
                currentParameters = parameterList[i]
                v = parameterFunction(currentParameters)
                values += [v]
                if showProgress:
                    printStr = ''
                    if (type(v) == float) or (type(v) == int):
                        printStr = ', value = '+str(v)
                    if 'functionData' in currentParameters: #functionData may be large, DO not print!
                        copyParameters = {}
                        for key, value in currentParameters.items():
                            if key != 'functionData':
                                copyParameters[key] = value
                        printStr += ": parameters=" + str(copyParameters)
                    else:
                        printStr += ": parameters=" + str(currentParameters)
                    
                    print("\rrun ", i+1, "/", nVariations, printStr, '                ', end='', flush=True)
                if resultsFile != '':
                    resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                              [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                              fileType='parameter variation',
                                              multiProcessingMode='serial')
            if showProgress:
                print("", flush=True) #newline after tqdm progress bar output....
        else:
            from multiprocessing import Pool, cpu_count #parallelization of computation
           
            numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
            if 'numberOfThreads' in kwargs: 
                numberOfThreads = kwargs['numberOfThreads']
            
            vInput = np.array(parameterList)
                
            if useTQDM:
                with Pool(processes=numberOfThreads) as p:
                    #values = list(tqdm.tqdm(p.imap(parameterFunction, vInput), total=nVariations))
                    for v in (tqdm.tqdm(p.imap(parameterFunction, vInput), total=nVariations)):
                        values+=[v]
                        if resultsFile != '':
                            resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                                      [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                                      fileType='parameter variation',
                                                      multiProcessingMode='multiprocessing.Pool, numberOfThreads='+str(numberOfThreads))
                print("", flush=True) #newline after tqdm progress bar output....
            else:
                #simpler approach without tqdm:
                # with Pool(processes=numberOfThreads) as p:
                #     values = p.map(parameterFunction, vInput)
                with Pool(processes=numberOfThreads) as p:
                    for v in p.imap(parameterFunction, vInput):
                        values+=[v]
                        if resultsFile != '':
                            resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                                      [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                                      fileType='parameter variation',
                                                      multiProcessingMode='multiprocessing.Pool, numberOfThreads='+str(numberOfThreads))
                            #print("value=",i)
    elif useCluster: 
        
        # form cluster and submit jobs
        cluster = dispy.JobCluster(parameterFunction, nodes=clusterHostNames, host=clusterHostNames, cleanup=True, dispy_port=9700) 
        
        # import dispy's httpd module, create http server for this cluster
        # monitor allows to monitor and manage clusters with a web browser; it works with common web browsers, including in iOS and Android devices.
        http_server = None
        if 'useDispyWebMonitor' in kwargs: #showProgress: #True:
            if showProgress:
                print('open http monitor')
            import dispy.httpd
            import socket
            clientHostname = socket.gethostname()
            http_server = dispy.httpd.DispyHTTPServer(cluster, host=clientHostname, port=8181, poll_sec=5) 
            import webbrowser
            clientIPv4Address = socket.gethostbyname(clientHostname)
            webbrowser.open("http://"+clientIPv4Address+":8181/monitor.html")
        
        # perform computations
        nVariations = len(parameterList)
        jobs = [None]*nVariations
        for i in range(nVariations):
            
            # submit job to cluster
            job = cluster.submit(parameterList[i]) #, refSol=referenceSolution) 
            jobs[i] = job
            job.id = i+1

        #cluster.wait() # waits until all jobs finish
        
        # collect return value of each job      
        values = [None]*len(jobs)  
        
        jobCtr = 0        
        for job in jobs:     
            #host, val = job()
            val = job()
            values[jobCtr] = val
            jobCtr += 1
            if showProgress:
                #show return values only in case that they are of float type
                if (type(job.result) == float) or (type(job.result) == int):
                    print('executed job '+str(job.id) + ' / ' + str(nVariations) +
                          ' with return value ' + str(job.result))
                else:
                    print('executed job ' + str(job.id) + ' / ' + str(nVariations))
        
        # close cluster
        cluster.wait()
        if showProgress:
            cluster.print_status()
        
        if http_server != None:
            http_server.shutdown() # this waits until browser gets all updates
        cluster.close()
    elif useMPI:
        #numberOfThreads not used in this case; given by mpiexec call

        vInput = np.array(parameterList)

        from mpi4py import MPI
        
        comm = MPI.COMM_WORLD
        nprocs = comm.Get_size()
        # rank   = comm.Get_rank() 
        
        if useTQDM:

            with MPIPoolExecutor() as p:
            # with Pool(processes=numberOfThreads) as p:
                for v in (tqdm.tqdm(p.map(parameterFunction, vInput), total=nVariations)):
                    values+=[v]
                    if resultsFile != '':
                        resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                                  [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                                  fileType='parameter variation',
                                                  multiProcessingMode='mpi4py, numberOfWorkers='+str(nprocs))
            print("", flush=True) #newline after tqdm progress bar output....
        else:
            #simpler approach without tqdm:
            # with Pool(processes=numberOfThreads) as p:
            #     values = p.map(parameterFunction, vInput)
            with MPIPoolExecutor() as p:
                for v in p.map(parameterFunction, vInput):
                    values+=[v]
                    if resultsFile != '':
                        resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                                  [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                                  fileType='parameter variation',
                                                  multiProcessingMode='mpi4py, numberOfWorkers='+str(nprocs))

            
    return values



#**function: calls successively the function parameterFunction(parameterDict) with variation of parameters in given range; parameterDict is a dictionary, containing the current values of parameters,
#  e.g., parameterDict=['mass':13, 'stiffness':12000] to be computed and returns a value or a list of values which is then stored for each parameter
#**input:
#    parameterFunction: function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
#    parameters: given as a dictionary, consist of name and tuple of (begin, end, numberOfValues) same as in np.linspace(...), e.g. 'mass':(10,50,10), for a mass varied from 10 to 50, using 10 steps OR a list of values [v0, v1, v2, ...], e.g. 'mass':[10,15,25,50]
#    useLogSpace: (optional) if True, the parameters are varied at a logarithmic scale, e.g., [1, 10, 100] instead linear [1, 50.5, 100]
#    debugMode: if True, additional print out is done
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    showProgress: if True, shows for every iteration the progress bar (requires tqdm library)
#    resultsFile: if provided, output is immediately written to resultsFile during processing
#    numberOfThreads: default(None): same as number of cpus (threads); used for multiprocessing lib;
#    parameterFunctionData: dictionary containing additional data passed to the parameterFunction inside the parameters with dict key 'functionData'; use this e.g. for passing solver parameters or other settings
#    clusterHostNames: list of hostnames, e.g. clusterHostNames=['123.124.125.126','123.124.125.127'] providing a list of strings with IP addresses or host names, see dispy documentation. If list is non-empty and useMultiProcessing==True and dispy is installed, cluster computation is used; NOTE that cluster computation speedup factors shown are not fully true, as they include a significant overhead; thus, only for computations which take longer than 1-5 seconds and for sufficient network bandwith, the speedup is roughly true
#    useDispyWebMonitor: if given in **kwargs, a web browser is started in case of cluster computation to manage the cluster during computation
#    useMPI: if given in **kwargs and set True, and if Python package mpi4py is installed, mpi parallelization is used; for hints see parameterVariationExample.py
#**output:
#    returns [parameterList, values], containing, e.g., parameterList=\{'mass':[1,1,1,2,2,2,3,3,3], 'stiffness':[4,5,6, 4,5,6, 4,5,6]\} and the result values of the parameter variation accoring to the parameterList, 
#           values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
#**example:
#if __name__ == '__main__':
#    ParameterVariation(parameterFunction=Test, 
#                       parameters={'mass':(1,10,10), 'stiffness':(1000,10000,10)}, 
#                       useMultiProcessing=True)
def ParameterVariation(parameterFunction, parameters, 
                       useLogSpace=False, debugMode=False, addComputationIndex=False,
                       useMultiProcessing=False, showProgress = True, parameterFunctionData={}, clusterHostNames=[],
                       numberOfThreads=None, resultsFile='', **kwargs):
    
    if 'multiprocessing' in sys.modules:
        from multiprocessing import cpu_count
        if numberOfThreads == None:
            numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
        if debugMode:
            if IsEmptyList(clusterHostNames):
                print("using", numberOfThreads, "cpus")
            else:
                print("using cluster")
    if numberOfThreads == None:
        numberOfThreads = 8 #just some default, if no other value available

    #generate list of parameters to iterate
    dim = len(parameters)       #dimensionality (dimension) of problem
    nParams = np.zeros(dim, dtype=int)     #number of variations in each dimension
    cnt = 0
    for (key,value) in parameters.items(): 
        if isinstance(value, tuple): #then it is a range (start, end, numberOfValues)
            nParams[cnt] = value[2] #last value is the number of variations
        elif isinstance(value, list): #then it contains list of values, e.g., [1,2,4,8]
            nParams[cnt] = len(value) #last value is the number of variations
        else:
            raise ValueError('ParameterVariation: parameters must contain tuple with range (begin, end, numberOfValues) or list of values [v0, v1, v2, ...]')
        cnt+=1 #counts the dimensionality
        
        
    nVariations = np.array(nParams).prod() #product of all ranges gives total count
    if nVariations == 0:
        print("WARNING: number of variations =", nVariations)
        return []

    if debugMode:
        print("number of variations =", nVariations)

    cnt = 0
    isIntType = [False] * dim # true if corresponding parameter ranges/list is all int type
    parameterDict = {} #dictionary of parameter lists
    for (key,value) in parameters.items():
        if isinstance(value, tuple): #then it is a range (start, end, numberOfValues)
            pStart = value[0]
            pEnd = value[1]
            numberOfValues = value[2]

            if numberOfValues == 0:
                raise ValueError('ParameterVariation: in tuples (begin, end, numberOfValues), numberOfValues may not be zero')
            if int(numberOfValues) != numberOfValues:
                raise ValueError('ParameterVariation: in tuples (begin, end, numberOfValues), numberOfValues must be integer')
            if pStart > pEnd:
                raise ValueError('ParameterVariation: in tuples (begin, end, numberOfValues), there must be begin <= end')
        
            step = (pEnd-pStart+1)/numberOfValues # is for check if numberOfValues is even
            if IsInteger(pStart) and IsInteger(pEnd) and step == int(step): # is true, if ranges/list are all int and step is integer
                isIntType[cnt] = True 

            #now create list of parameters, using duplicates according to dimensionality
            if useLogSpace:
                space = np.logspace(np.log10(pStart),np.log10(pEnd),numberOfValues)
            else:
                space = np.linspace(pStart,pEnd,numberOfValues)
        else: #already checked above: if isinstance(value, list): #then it contains list of values, e.g., [1,2,4,8]
            space = value
            isIntType[cnt] = True
            for val in value:
                if not IsInteger(val):
                    isIntType[cnt] = False
            
        range1 = nParams[0:cnt].prod()
        range2 = nParams[cnt+1:dim+1].prod()
        if range1 == 0:
            range1 = 1 #otherwise kronecker product won't work
        if range2 == 0:
            range2 = 1 #otherwise kronecker product won't work
            
        #print("space=",space)
        
        parameterDict[key] = np.kron(np.kron([1]*range1, space), [1]*range2)
        cnt+=1 #counts the dimensionality

    if debugMode:
        print("parameterDict =", parameterDict)

    if addComputationIndex:
        parameterDict['computationIndex'] = np.linspace(0,nVariations-1,nVariations,dtype=int)
        
    #finally convert parameter dictinary to list of dictionaries:
    parameterList = [] #list of parameter dictionaries
    for i in range(nVariations):
        parameterSet = {}
        cnt = 0
        for (key,value) in parameterDict.items(): 
            if key == 'computationIndex':
                v = int(value[i]) #make integers, which follow type(v)==int
            elif isIntType[cnt]: # check if this variable is an integer type
                v = int(value[i]) #make integers, which follow type(v)==int
            else:
                v = float(value[i]) #make floats, which follow type(v)==float
            parameterSet[key] = v
            cnt += 1
        if parameterFunctionData != {}:
            parameterSet['functionData'] = parameterFunctionData
            #not needed: parameterSet['functionData'] = copy.deepcopy(parameterFunctionData)
            
        parameterList += [parameterSet]

    values = ProcessParameterList(parameterFunction, parameterList, useMultiProcessing, 
                                  showProgress = showProgress, numberOfThreads=numberOfThreads,
                                  resultsFile = resultsFile, parameters=parameters, 
                                  clusterHostNames=clusterHostNames, **kwargs)
    
    return [parameterDict, values]
    


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute minimum of given objectiveFunction
#**input:
#    objectiveFunction: function, which takes the form parameterFunction(parameterDict) and which returns a value or list (or numpy array) which reflects the size of the objective to be minimized
#    parameters: given as a dictionary, consist of name and tuple containing the search range for this parameter (begin, end), e.g. 'mass':(10,50)
#
#    populationSize: individuals in every generation
#    initialPopulationSize: number of random initial individuals; default: population size
#    numberOfGenerations: number of generations; NOTE: it is required that elitistRatio*populationSize >= 1
#    elitistRatio: the number of surviving individuals in every generation is equal to the previous population times the elitistRatio
#    crossoverProbability: if > 0: children are generated from two (randomly selected) parents by gene-crossover; if 0, no crossover is used
#    crossoverAmount: if crossoverProbability > 0, then this amount is the probability of genes to cross; 0.1: small amount of genes cross, 0.5: 50\% of genes cross
#    rangeReductionFactor: reduction of mutation range (boundary) relative to range of last generation; helps algorithm to converge to more accurate values
#    distanceFactor: children only survive at a certain relative distance of the current range; must be small enough (< 0.5) to allow individuals to survive; ignored if distanceFactor=0; as a rule of thumb, the distanceFactor should be zero in case that there is only one significant minimum, but if there are many local minima, the distanceFactor should be used to search at several different local minima
#    childDistribution: string with name of distribution for producing childs: "normal" (Gaussian, with sigma defining range), "uniform" (exactly in range of childs)
#    distanceFactorGenerations: number of generations (populations) at which the distance factor is active; the distance factor is used to find several local minima; finally, convergence is speed up without the distance factor
#    parameterFunctionData: dictionary containing additional data passed to the objectiveFunction inside the parameters with dict key 'functionData'; use this e.g. for passing solver parameters or other settings
#    randomizerInitialization: initialize randomizer at beginning of optimization in order to get reproducible results, provide any integer in the range between 0 and 2**32 - 1 (default: no initialization)
#
#    debugMode: if True, additional print out is done
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    showProgress: if True, shows for every iteration the progress bar (requires tqdm library)
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#    resultsFile: if provided, the results are stored columnwise into the given file and written after every generation; use resultsMonitor.py to track results in realtime
#    clusterHostNames: list of hostnames, e.g. clusterHostNames=['123.124.125.126','123.124.125.127'] providing a list of strings with IP addresses or host names, see dispy documentation. If list is non-empty and useMultiProcessing==True and dispy is installed, cluster computation is used; NOTE that cluster computation speedup factors shown are not fully true, as they include a significant overhead; thus, only for computations which take longer than 1-5 seconds and for sufficient network bandwith, the speedup is roughly true
#    useDispyWebMonitor: if given in **kwargs, a web browser is startet in case of cluster computation to manage the cluster during computation
#**output:
#    returns [optimumParameter, optimumValue, parameterList, valueList], containing the optimum parameter set 'optimumParameter', optimum value 'optimumValue', the whole list of parameters parameterList with according objective values 'valueList'
#           values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
#**notes: This function is still under development and shows an experimental state! 
#**example:
#GeneticOptimization(objectiveFunction = fOpt, parameters={'mass':(1,10), 'stiffness':(1000,10000)})
def GeneticOptimization(objectiveFunction, parameters, 
                        populationSize=100,
                        numberOfGenerations=10,
                        elitistRatio = 0.1,
                        crossoverProbability=0.25,
                        crossoverAmount=0.5,
                        rangeReductionFactor=0.7,
                        distanceFactor=0.1,
                        childDistribution="uniform",  
                        distanceFactorGenerations=-1,
                        debugMode=False, 
                        addComputationIndex=False,
                        useMultiProcessing=False, 
                        showProgress = True,
                        clusterHostNames = [],
                        parameterFunctionData = {}, 
                        **kwargs):

    def RandomNumber(distribution, rangeBegin, rangeEnd, vMin, vMax):
        a = rangeBegin
        b = rangeEnd
        value = vMax + 1
        while (value < vMin or value > vMax):
            if distribution == 'uniform':
                value = np.random.uniform(a, b)
            elif distribution == 'normal':
                value = np.random.normal(loc = 0.5*(a+b), scale = 0.5*(b-a))
            else:
                raise ValueError('GeneticOptimization: invalid childDistribution "'+childDistribution+'"')
        
        return value
        
    
    #get number of threads:
    if 'multiprocessing' in sys.modules:
        from multiprocessing import cpu_count
        numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
    else:
        numberOfThreads = 8
    if 'numberOfThreads' in kwargs: 
        numberOfThreads = kwargs['numberOfThreads']

    if useMultiProcessing:
        if clusterHostNames==[]:
            print("number of threads used =", numberOfThreads,flush=True) #very useful information
        else:
            print("using cluster",flush=True) 
            

    useDispyWebMonitor = False
    if 'useDispyWebMonitor' in kwargs: 
        useDispyWebMonitor = kwargs['useDispyWebMonitor']

    initialPopulationSize = populationSize
    if 'initialPopulationSize' in kwargs: 
        initialPopulationSize = kwargs['initialPopulationSize']

    resultsFile = ''
    if 'resultsFile' in kwargs: 
        resultsFile = kwargs['resultsFile']

    #+++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++
    #delete this in future:
    if 'numberOfChildren' in kwargs: 
        print("GeneticOptimization: deprecated and unused parameter; use population size a and elitistRatio instead\n")
    
    #old value: survivingIndividuals=8
    survivingIndividuals = int(elitistRatio*populationSize)
    if 'survivingIndividuals' in kwargs: 
        survivingIndividuals = kwargs['survivingIndividuals']
        print("GeneticOptimization: survivingIndividuals: deprecated parameter; use population size a and elitistRatio instead\n")


    if 'randomizerInitialization' in kwargs: 
        randomizerInitialization = kwargs['randomizerInitialization']
        if not isinstance(randomizerInitialization,int):
            raise ValueError("GeneticOptimization: ERROR: randomizerInitialization must be positive 32 bit integer")
        np.random.seed(randomizerInitialization)
    #+++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++

    #+++++++++++++++++++++++++++++++++++++++++++++++
    #check that there are surviving individuals at all (otherwise hangs...)
    if survivingIndividuals < 1:
        elitistRatio = 1/populationSize
        survivingIndividuals = 1
        print("WARNING: elitistRatio*populationSize < 1. Setting elitistRatio=", elitistRatio,'\n')

    if distanceFactor >= 1:
        distanceFactor = 0.5
        print("WARNING: distanceFactor >= 1, setting distanceFactor = 0.5\n")

    if distanceFactorGenerations < 0:
        distanceFactorGenerations = numberOfGenerations+1 #will be never active

    dim = 0
    ranges = []                 #list containing the ranges of each dimension
    rangesDict = {} #dict containing only the ranges
    for (key,value) in parameters.items():
        dim += 1 #count dimensions of parameters
        r = value[1]-value[0]
        if r <= 0:
            raise ValueError("GeneticOptimization: ERROR: range of component "+str(dim-1)+" has negative or zero range")
        ranges += [r]
        rangesDict[key] = r
    
    #+++++++++++++++++++++++++++++++++++++++++++++++
    #generate first generation:
    currentGeneration = []
    for i in range(initialPopulationSize):
        ind = {} #dictionary for individual
        for (key,value) in parameters.items():
            pBegin = value[0]
            pEnd = value[1]
            value = np.random.uniform(pBegin, pEnd)
            ind[key] = value
        AddComputationIndexAndFunctionData(ind, i, addComputationIndex, parameterFunctionData)
        currentGeneration += [ind]
    #+++++++++++++++++++++++++++++++++++++++++++++++

    if debugMode:
        if initialPopulationSize <= 50:
            print("initial population =", currentGeneration)
        print("rangesDict =", rangesDict)

    parametersAll = []
    valueList = []
    newGeneration = []      #surviving individuals, not re-computed!
    newGenerationValues = []#surviving individuals' values, not re-computed!
    
    totalEvaluations = 0
    resultsFileCnt = 0 #counter for output file

    for popCnt in range(numberOfGenerations):
        if debugMode:
            print("===============\nevaluate population", popCnt, ":")

        totalEvaluations += len(currentGeneration)
        values = ProcessParameterList(objectiveFunction, currentGeneration, useMultiProcessing, showProgress = showProgress, numberOfThreads=numberOfThreads,
                                      clusterHostNames=clusterHostNames, useDispyWebMonitor=useDispyWebMonitor)

        if (showProgress and useMultiProcessing and popCnt < numberOfGenerations-1): print("            #"+str(popCnt+1), end='')
        #print("values=",values)
        multiProcessingMode = ''
        if useMultiProcessing:
            multiProcessingMode = 'multiprocessing.Pool, numberOfThreads='+str(numberOfThreads)
        resultsFileCnt = WriteToFile(resultsFile, parameters, currentGeneration, values, resultsFileCnt, 
                                     writeHeader = (popCnt == 0),
                                     multiProcessingMode=multiProcessingMode)

        #remove computationIndex and parameterfunctiondata from new generation
        for item in currentGeneration:
            if 'computationIndex' in item:
                del item['computationIndex']
            if 'functionData' in item: 
                del item['functionData']

        #store all values
        parametersAll += currentGeneration.copy()
        valueList += values.copy()

        #add best individuals from previous parents:
        currentGeneration += newGeneration.copy()
        values += newGenerationValues.copy()
        
        #compute norm and minimum values:
        scalarValues = [(0,0)]*len(values)
        for i in range(len(values)):
            item = values[i]
            if isinstance(item, float):
                scalarValues[i] = (item,i)
            else: #must be list of values ==> compute norm!
                scalarValues[i] = (np.sqrt(np.dot(item,item)),i)

        valuesDtype = [('value', float), ('index', int)]
        scalarValues = np.array(scalarValues, dtype = valuesDtype)

        sortedValues = np.sort(scalarValues, order='value') #sort for item values
        #print("scalarValues=",scalarValues)
        #print("sortedValues=",sortedValues)

        if popCnt < numberOfGenerations-1: #go on for next population
            relativeRange = rangeReductionFactor**(popCnt+1) #this is the relative range for the next population

            if debugMode:
                print("Child ranges in population",popCnt)
                for (key,value) in parameters.items():
                    r = value[1]-value[0]
                    r *= relativeRange #reduce range
                    print('  '+key+':', r)
            
            #selection: chose best surviving individuals
            newGeneration = []
            newGenerationValues = []
            cnt = 0

            if distanceFactor == 0 or popCnt >= distanceFactorGenerations: #distance not important
                for i in range(min(survivingIndividuals,len(sortedValues))):
                    ind = currentGeneration[int(sortedValues[i][1])] #dictionary for individual
                    if addComputationIndex:
                        ind['computationIndex'] = cnt #unique index for one set of computations
                    newGeneration += [ind]
                    newGenerationValues += [values[int(sortedValues[i][1])]]
                    cnt += 1
            else:
                nSurviving = min(survivingIndividuals,len(sortedValues))
                nGen = len(currentGeneration)
                j = 0 #index counter
                i = 0 #counter for surviving individuals to be found
                distanceList = [1]*nGen #initialize with one, saying that all distances large enough #relativeRange*np.reshape(ranges*dim,(nGen, dim)) #these are the maximum distances
                
                #print("nSurviving=",nSurviving)
                #print("len(sortedValues)=",len(sortedValues))
                #find indices which have sallest objective function value, but obey distanceFactor
                while i < nSurviving and j < len(sortedValues):
                    iInd = int(sortedValues[j][1]) #index for individual in currentGeneration
                    ind = currentGeneration[iInd] #dictionary for individual
                    j += 1
                    if distanceList[iInd] > distanceFactor:
                        if addComputationIndex:
                            ind['computationIndex'] = cnt #unique index for one set of computations
                        newGeneration += [ind]
                        newGenerationValues += [values[iInd]]
                        cnt += 1 #computation index counter
                        i += 1   #counts the surviving individuals
                        #print("\nadd individual", ind)
                        #print("currentGeneration", currentGeneration)
                        
                        #update distances for added individual:
                        for k in range(nGen):
                            d = 0
                            for (key,value) in ind.items():
                                if key != 'computationIndex' and key in parameters.keys(): # only manipulate values from given parameters
                                    d += (ind[key] - currentGeneration[k][key])**2/(rangesDict[key])**2
                            d = np.sqrt(d/dim) #number of parameters shall not influence distanceFactor
                            #print("d=",d,":",ind,"-",currentGeneration[k])
                            if d < distanceList[k]:
                                distanceList[k] = d
                    # else:
                    #     print("\nindiv.",ind," ignored due to small distance:", distanceList[iInd])
                        
                    #print("distanceList=",distanceList)
            
            #print("survivingIndividuals=",newGeneration)
            if debugMode:
                print("\nbest values=",sortedValues[0:min(survivingIndividuals,4)])
            
            #prolongate best individuals:
            currentGeneration = []
            # for item in newGeneration: #removed, in order to save number of evaluations
            #     currentGeneration += [item]
    
            #modification of parents
            genSize = len(newGeneration)
            if genSize == 0:
                print("WARNING: size of generation = 0, terminating (check your optimization parameters...)\n")
                
            p = 0 #current population size
            while p < populationSize and genSize != 0:
                for item in newGeneration:
                    indList = [{}]      #dictionary for individual
                    parents = [item]    #list of parents

                    #do gene crossing fro parents:
                    if crossoverProbability > 0:
                        r = np.random.random()
                        if r < crossoverProbability: #do cross-over only for a smaller portion of parents!
                            indList += [{}] #use two new individuals with gene-crossover
                            rand0 = np.random.randint(genSize)
                            rand1 = np.random.randint(genSize) #may be same as rand1
                            p0 = newGeneration[rand0].copy() #parent1 for crossover
                            p1 = newGeneration[rand1].copy() #parent2 for crossover
                            parents = [p0, p1]
                            for (key,value) in parameters.items():
                                r = np.random.random()
                                if r < crossoverAmount: #usually 50% gene cross over
                                    indList[0][key] = p1[key] #uniform gene crossing
                                    indList[1][key] = p0[key]
                            # print('indList=',indList)

                    #one or two individuals
                    for pi in range(len(parents)):
                        item = parents[pi]
                        for (key,value) in parameters.items():
                            r = value[1]-value[0]
                            r *= relativeRange #reduce range
                            #print("range=",r)
                            pBegin = item[key]-0.5*r #minimum value
                            pEnd = item[key]+0.5*r 
                            #obej ranges: ==> this may give more values at boundary
                            if pBegin < value[0]: pBegin = value[0]
                            if pEnd > value[1]: pEnd = value[1]
    
                            #print("new range=",pBegin,pEnd)
                            #value = np.random.uniform(pBegin, pEnd)
                            newValue = RandomNumber(childDistribution, 
                                                 pBegin, pEnd, 
                                                 value[0], value[1])
                            indList[pi][key] = newValue
    
                        if p < populationSize:
                            newGen = {}
                            for (key,value) in parameters.items(): #sort according to parameters, for output file!
                                newGen[key] = indList[pi][key]
                            if addComputationIndex:
                                newGen['computationIndex'] = cnt #unique index for one set of computations
                            currentGeneration += [newGen]
                            #currentGeneration += [indList[pi]] #gives wrong sorting in dict ..., destroys output file order
                            p += 1
                        indList[-1]['functionData'] = parameterFunctionData # add function data
                        cnt += 1 #computation count ... for every parameter variation within one generation
            # print("pop", popCnt, ": currentGeneration=\n",currentGeneration)
        else:
            #select final best individual
            optimumParameter = currentGeneration[int(sortedValues[0][1])]
            optimumValue = sortedValues[0][0]
            if debugMode:
                print("opt par=", optimumParameter, ", opt val=", optimumValue)

    if debugMode:
        print("===============\ntotal evaluations=", totalEvaluations)

    #now make dict of parameter lists instead list of dicts
    parameterList = {}
    n = len(parametersAll)
    if n != 0:
        for key in parametersAll[0]:
            if key != 'computationIndex' and key in parameters.keys(): # do not output function data
                parameterData = np.zeros(n)
                #extract parameter list from list of dictionaries:
                for i in range(n):
                    parameterData[i] = parametersAll[i][key]
    
                #add parameter list to final dictionary
                parameterList[key] = parameterData


    return [optimumParameter, optimumValue, parameterList, valueList]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Compute minimum of given objectiveFunction. This function is based on scipy.optimize.minimize() and it provides the same interface as GeneticOptimization(). Note that in special cases, you should copy this function and adapt to your needs.
#**input:
#    objectiveFunction: function, which takes the form parameterFunction(parameterDict) and which returns a value or list (or numpy array) which reflects the size of the objective to be minimized
#    parameters: given as a dictionary, consist of name and tuple containing the search range for this parameter (begin, end), e.g. 'mass':(10,50)
#    storeFunctionValues: if True, objectiveFunction values are computed (additional costs!) and stored in every iteration into valueList
#    initialGuess: initial guess. Array of real elements of size (n,), where 'n' is the number of independent variables. If not provided by the user, initialGuess is computed from bounds provided in parameterDict.
#    method: solver that should be used, e.g. 'Nelder-Mead', 'Powell', 'CG' etc. A list of available solvers can be found in the documentation of scipy.optimize.minimize().
#    tol: tolerance for termination. When tol is specified, the selected minimization algorithm sets some relevant solver-specific tolerance(s) equal to tol (but this is usually not the tolerance for loss or parameters1). For detailed control, use solver-specific options using the 'options' variable.
#    options: dictionary of solver options. Can be used to set absolute and relative error tolerances. Detailed information can be found in the documentation of scipy.optimize.minimize().
#    enforceBounds: if True, ensures that only parameters within the bounds specified in ParameterDict are used for minimization; this may help to avoid, e.g., negative values, but may lead to non-convergence 
#    verbose: prints solver information into console, e.g. number of iterations 'nit', number of funcion evaluations 'nfev', status etc.
#    showProgress: if True, shows for every iteration objective function value, current iteration number, time needed for current iteration, maximum number of iterations and loss (current value of objective function)
#    addComputationIndex: if True, key 'computationIndex' is added for consistency reasons with GeneticOptimizaiton to every parameterDict in the call to parameterFunction(); however, the value is always 0, because no multi threading is used in Minimize(...)
#    resultsFile: if provided, the results are stored columnwise into the given file and written after every generation; use resultsMonitor.py to track results in realtime
#    useScipyBounds: if True, use scipy.optimize.minimize() option 'bounds' to apply bounds on variable specified in ParameterDict. Note, this option is only used by some specific methods of scipy.optimize.minimize()! method='Nelder-Mead' ignores this option for example! if False, option 'enforceBounds' will be set to False!
#    args: extra arguments passed to the objective function and its derivatives (fun, jac and hess functions). 
#**author: Stefan Holzinger, Johannes Gerstmayr
#**output: returns [optimumParameter, optimumValue, parameterList, valueList], containing the optimum parameter set 'optimumParameter', optimum value 'optimumValue', the whole list of parameters parameterList with according objective values 'valueList'
#**notes: This function is still under development and shows an experimental state! There are currently unused arguments of scipy.optimize.minimize(): Detailed information can be found in the documentation of scipy.optimize.minimize().
def Minimize(objectiveFunction, parameters, initialGuess=[], method='Nelder-Mead', tol=1e-4, options={}, 
             enforceBounds=True, debugMode=False, showProgress=True, addComputationIndex=False,
             storeFunctionValues=True, **kwargs):
    from scipy import optimize #for minimize
    # from functools import partial #add extra args to function
    
    # get parameter names
    parKeyLst = list(parameters.keys())
    initialGuessCopy = list(initialGuess)
    
    # number of parameters
    nParameters = len(parKeyLst)
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    resultsFile = '' # if resultsFile name is provided, write solution to resultsFile
    if 'resultsFile' in kwargs: 
        resultsFile = kwargs['resultsFile']
    
    maxiter = 200*nParameters # maximum iterations used by th solver (default value of method='Nelder-Mead')
    if 'maxiter' in options:
        maxiter = options['maxiter']
    
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # get boundaries ((min, max), ... ,(min, max))
    bounds = [None]*nParameters 
    for i in range(nParameters):
        bounds[i] = parameters[parKeyLst[i]]    
    bounds = tuple(bounds) # type cast: list --> tuple
    
    useScipyBounds = False
    scipyMinimizeBounds = None # 'bounds' option of scipy.optimize.minimize() will not be used!
    if 'useScipyBounds' in kwargs: 
        if kwargs['useScipyBounds']:
            useScipyBounds = True
            scipyMinimizeBounds = bounds # use option 'bounds'
        
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # process initial guess
    if initialGuessCopy == []: # initial guess has been provided by the user
        initialGuessCopy = [None]*nParameters
        for i in range(nParameters):
            initialGuessCopy[i] = np.mean(bounds[i])


    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #create lists for output
    parameterValueLst = [None]*nParameters # type=list(list()); contains parameter values for each iteration
    for i in range(nParameters):
        parameterValueLst[i] = []
    valueList = [] # objective function value for initial guess parameters


    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # map initialGuess list --> dict 
    def ParameterListToParameterDict(initialGuess, *args):
        paraDict = {}
        for i in range(nParameters):
            paraDict[parKeyLst[i]] = initialGuess[i]

        if addComputationIndex: # add computation index
            paraDict['computationIndex'] = 0 #itCtr[0]
        
        return paraDict
         
    
    # count number of iterations
    itCtr = [0]
    resultsFileCnt = [0] # counter for output file

    # this function is a inerface to exudyn.processing.WriteToFile()
    def WriteToFileMinimize(resFileName, parDictInit, pDict, objFunVal, resFileCnt):       
        resultsFileCntTemp = WriteToFile(resFileName, 
                                         parDictInit, # parameter dict with value range supplied by user
                                         [pDict], # parameter values 
                                         [objFunVal], # objective function values
                                         resFileCnt, # line counter --> write data to file at this line
                                         writeHeader = (resFileCnt == 0), 
                                         fileType='optimization using scipy.optimize.minimize(method='+method+')',
                                         )
        return resultsFileCntTemp

    startTime = time.time()  #for calculating time to go
    timePrintLast = [startTime-10]

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # callback: needed to get parameters after each iteration for file writing and post processing
    def StoreParameterFunctionValues(parametersAtIteration, convergence=0, lastTime=False):
        #global timePrintLast
        # print(parametersAtIteration2)
        for i in range(nParameters):
            parameterValueLst[i].append(parametersAtIteration[i])
        
        # map initial guess (type list) to dict
        pDict = ParameterListToParameterDict(parametersAtIteration)

        if storeFunctionValues or resultsFile != '':
            # compute objective function value        
            valuesAtIteration = objectiveFunction(pDict)  #additional costs!
            valueList.append(valuesAtIteration) # add value to value list
            # write parameters and objective function at current iteration to file
            if resultsFile != '':
                resultsFileCnt[0] = WriteToFileMinimize(resultsFile, parameters, pDict, valuesAtIteration, resultsFileCnt[0])
        
        # increase iteration counter 
        itCtr[0] += 1 
        
        # time needed for iteration     
        iterationsToGo = (maxiter-itCtr[0]) 
        timeToGo = 0
        currentTime = time.time()
        timeSpent = currentTime - startTime
        if itCtr[0] != 0:
            timeToGo = timeSpent/itCtr[0] * iterationsToGo

        # print progess to console 
        if showProgress: 
            if currentTime - timePrintLast[0] > 1 or lastTime:
                timePrintLast[0] = currentTime
                print('iteration ' + str(itCtr[0]) + ' / ' + str(maxiter), end='')
                print(', time = '+str(round(timeSpent,2)), 's / ',end='')
                print(str(round(timeToGo,2)) + 's',end='')
                print(', loss:', ("{:.0"+str(3)+"g}").format(valuesAtIteration))
            



    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # local parameterFunction used for minimization
    def ParameterFunctionMinimize(parametersAtIteration):
    
        if not useScipyBounds: # if useScipyBounds=True, use scipy option 'bounds', else apply manual enforcement of bounds. 
            if enforceBounds: # enforce bounds by mapping initial guess to allowed parameter range
                for i in range(nParameters):
                    pVal = parametersAtIteration[i]
                    lowerBound = min(bounds[i])
                    upperBound = max(bounds[i])
                    if pVal < lowerBound:
                        parametersAtIteration[i] = lowerBound
                    if pVal > upperBound:
                        parametersAtIteration[i] = upperBound

        return objectiveFunction(ParameterListToParameterDict(parametersAtIteration))
    
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # perform optimizaion
    optimizeResult = optimize.minimize(ParameterFunctionMinimize, 
                                       initialGuessCopy, 
                                        method=method, 
                                        bounds=scipyMinimizeBounds, 
                                        # callback=partial(StoreParameterFunctionValues, convergence=0, callbackArgs=callbackArgs), 
                                        callback=StoreParameterFunctionValues, 
                                        tol=tol,
                                        options=options,
                                        )
    
    # if showProgress: # print progress data for final iteration to console
    StoreParameterFunctionValues(optimizeResult['x'], lastTime=True)
    
    if debugMode: # show solver informations (e.g. number of function evaluations etc.)
        print('---------------------------------------\n')
        print('solver output:\n')
        print(optimizeResult, '\n')
        print('---------------------------------------\n')

    # iteration ctr of optimize.minimize is one-based! --> add first (0th) iteration to itCtr
    itCtr = itCtr[0] + 1  

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # optimization results; generate same data for return as in GeneticOptimization()
    optimizedParameter = optimizeResult['x']
    optimumValue = optimizeResult['fun']
    parameterList = {}
    optimumParameter = {}
    for i in range(nParameters):
        parameterList[parKeyLst[i]] = parameterValueLst[i]
        optimumParameter[parKeyLst[i]] = optimizedParameter[i]
        
    return [optimumParameter, optimumValue, parameterList, valueList]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
#**function: Perform a sensitivity analysis by successively calling the function parameterFunction(parameterList[i]) with a one at a time variation of parameters in the defined increments. 
#  e.g., parameterList[0] =['mass':13, 'stiffness':12000] to be computed and returns a value or a list of values which is then stored for each parameter
#**input:
#    parameterFunction: function, which takes the form parameterFunction(parameterDict) and which returns one or more output values for which the sensitivity is calculated
#    parameters: given as a dictionary, consist of name and tuple of (begin, Variation steps, numberOfValues) e.g. 'mass':(10,0.01,5), for a reference mass of 10, incremented by 0.01*10 and using 5 steps in negative and positive, doing 10 steps in total
#    scaledByReference: if true multiplies the sensitivities with the corresponding reference parameters, so that the sensitivity resembles a change relative to the reference value
#    debugMode: if True, additional information is shown
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter etc. 
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    showProgress: if True, shows for every iteration the progress bar (requires tqdm library)
#    resultsFile: if provided, output is immediately written to resultsFile during processing
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#    parameterFunctionData: dictionary containing additional data passed to the parameterFunction inside the parameters with dict key 'functionData'; use this e.g. for passing solver parameters or other settings
#**output:
#    returns [parameterList, valRef, valuesSorted, sensitivity], parameterList containing the list of dictionaries processed. valRef is the Solution for the reference values paramList[0], valuesSorted contains the results sorted by the dictionary key that was varied in the simulation. The sensitivity contains the calculated sensitivity, where the rows are the corresponding outputparameters, while the columns are the input parameters, thereby the index sensitivity[1,0] is the sensitivity of output parameter 1 with respect to the input parameter 0. 
#**author: Peter Manzl
#**example:
#   ComputeSensitivities(parameterFunction=ParameterFunction, parameters = {'mass': (mRef, 0.01, 3), 'spring': (1000,0.01, 10),}, multiprocessing=True)
def ComputeSensitivities(parameterFunction, parameters, scaledByReference=False, 
                       debugMode=False, addComputationIndex=False, useMultiProcessing=False, 
                       showProgress = True, parameterFunctionData=dict(),**kwargs):
    
    
    if 'multiprocessing' in sys.modules:
        from multiprocessing import cpu_count
        numberOfThreads = cpu_count() #cpu_count gives number of threads
        if debugMode:
            print("using", numberOfThreads, "cpus")
    else:
        numberOfThreads = 8
    if 'numberOfThreads' in kwargs: 
        numberOfThreads = kwargs['numberOfThreads']

    resultsFile = ''
    if 'resultsFile' in kwargs: 
        resultsFile = kwargs['resultsFile']
        
    paramKeys = list(parameters.keys())
    for i in range(len(parameters)): 
        iKey = paramKeys[i]
        if not(hasattr(parameters[iKey], '__iter__')): # 
            parameters[iKey] = (parameters[iKey], 1e-3) # only a scalar value for reference value is given
        if len(parameters[iKey]) == 2: 
            parameters[iKey] = tuple(list(parameters[iKey]) + [0])
    # create Reference parameters [0]
    parameterList = [] #list of parameter dictionaries
    
    parameterList += [{paramKeys[0]: parameters[paramKeys[0]][0]}]
    nVar = [parameters[paramKeys[0]][2]]
    for i in range(1, len(paramKeys)): 
        parameterList[0][paramKeys[i]] = parameters[paramKeys[i]][0]
        nVar += [parameters[paramKeys[i]][2]]
    
    
    
    for i in range(len(nVar)):
        iKey =paramKeys[i] 

        fVal = parameters[iKey][1]
        # except 
        if debugMode: 
            print('Variate {} by {} each in {} steps'.format(iKey, fVal, nVar[i]))
        
        if nVar[i] == 0: # use forward difference
            parameterList += [deepcopy(parameterList[0])]
            parameterList[-1][iKey] = parameterList[-1][iKey] * (1+fVal)
        else:         
            for j in range(nVar[i]*2): 
                if j < nVar[i]: 
                    jVar = j -  nVar[i]
                else: 
                    jVar = j  - nVar[i] +1
                parameterList += [deepcopy(parameterList[0])]
                valVar = max(jVar*fVal, 1e-6) # if fVal == 0! 
                parameterList[-1][iKey] = parameterList[-1][iKey] *(1+jVar*fVal)
                if parameterList[-1][iKey] == 0: 
                    parameterList[-1][iKey] += max(fVal, 1e-6)
    
    if addComputationIndex: 
        for cnt in range(1, len(parameterList)): 
            parameterList[cnt]['computationIndex'] = cnt
        
    if parameterFunctionData != {}:
        for cnt in range(0, len(parameterList)): 
            parameterList[cnt]['functionData'] = parameterFunctionData

    values = ProcessParameterList(parameterFunction, parameterList, useMultiProcessing, 
                                  showProgress = showProgress, numberOfThreads=numberOfThreads,
                                  resultsFile = resultsFile, parameters=parameters)
   
    if not(hasattr(values[0], '__iter__')): 
        # if sensitivity is only calculated regarding to one output, reshape
        # calculated values in 2D array to support same indexing
        values = np.reshape(values, [-1,1])

    # calculate sensitivity of the parameter by forward/central difference
    sensitivity = np.zeros([len(nVar), len(values[0])])
    valuesSorted = {paramKeys[0]: []}
    iForward = nVar[0] + 1
    iBackward = nVar[0]
    for i in range(sensitivity.shape[0]): # iterate over keys from input
        iKey = paramKeys[i]
        for j in range(sensitivity.shape[1]): # iterate over outputvalues 
            if nVar[i] == 0: # use forward difference
                sensitivity[i][j] = (values[iForward][j] - values[0][j])/(parameterList[iForward][iKey] - parameterList[0][iKey])
            else:
                sensitivity[i][j] = (values[iForward][j] - values[iBackward][j])/(parameterList[iForward][iKey] - parameterList[iBackward][iKey])
            
            if scaledByReference: # multiply with the reference value so the sensitivity shows the influence by change of percent of the input
                sensitivity[i][j] = sensitivity[i][j] * parameterList[0][iKey]
        if nVar[i] == 0: 
            valuesSorted[iKey] = np.array([values[iForward]])
            iForward +=1
            iBackward = iForward - 1
        else: 
            valuesSorted[iKey] = np.array(values[iBackward-nVar[i]+1:iForward+nVar[i]])
            try: 
                iForward += nVar[i] + nVar[i+1]
            except: 
                continue
            iBackward = iForward -1
    valRef = values[0]
    return [parameterList, valRef, valuesSorted, sensitivity]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: visualize results of optimization for every parameter (2D plots)
#**input: 
#   parameterList: taken from output parameterList of \texttt{GeneticOptimization}, containing a dictinary with lists of parameters
#   valueList: taken from output valueList of \texttt{GeneticOptimization}; containing a list of floats that result from the objective function
#   xLogScale: use log scale for x-axis
#   yLogScale: use log scale for y-axis
#**output: return [figList, axList] containing the corresponding handles; creates a figure for every parameter in parameterList
def PlotOptimizationResults2D(parameterList, valueList, xLogScale=False, yLogScale=False):
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")

    n = len(valueList) #length of data
    if  n == 0:
        print('WARNING: PlotOptimizationResults: parameterList has zero length and therefore terminates!')
    
    figList = []
    axList = []
    for key in parameterList:
        fig = plt.figure()
        figList += [fig]
        ax=fig.gca() # get current axes
        axList+=[ax]

        parameterData = parameterList[key]
        if n != len(parameterData):
            raise ValueError('PlotOptimizationResults: length of parameterList is different from length of valueList')
        
        ax.plot(parameterData, valueList, 'b.', label=key) 
        ax.set_ylabel('value (objective function)')
        ax.set_xlabel(key)


        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 

        if xLogScale:
            ax.set_xscale('log')
        if yLogScale:
            ax.set_yscale('log')
            

        plt.tight_layout()
        plt.legend()

    plt.show() 
    return [figList, axList]




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: visualize results of Sensitivityanalyis for every parameter (2D plots)
#**input: 
#   valRef: The output values of the reference solution
#   valuesSorted: The output values of the analysed function sorted by the parameter which was varied
#   sensitivity: The sensitivity Matrix calculated by the function \texttt{ComputeSensitivities()}
#   fVar: The list of variation stepsizes. It is assumed to be 1e-3 if not defined.
#   strYAxis: A list of strings to label the plots yAxis
#   
#**output: return [fig, axs] containing the corresponding handles; creates a subplot for every row in the sensitivity matrix
#**author: Peter Manzl
def PlotSensitivityResults(valRef, valuesSorted, sensitivity, fVar=None, strYAxis = None):
    import matplotlib.pyplot as plt
    if strYAxis == None: 
        strYAxis = ['']*sensitivity.shape[1]
    if fVar == None: 
        fVar = [1e-3] * sensitivity.shape[1]
    if type(fVar) != list: # if one scalar varaible is passed it is assumed it is the same for each parameter
        fVar = [fVar]*sensitivity.shape[1]
    # the rows of the sensitivity are the different outputparameters, while the columns are the input parameters
    n = []
    fig, axs = plt.subplots(sensitivity.shape[1]) 
    if sensitivity.shape[1] == 1: 
        axs = [axs] # enable indexing for loop also for a single value
    for i in range(sensitivity.shape[1]): # each type of values gets a subplot 
        for j in range(len(list(valuesSorted.keys()))): # iterate over the variated parameters/keys
            iKey = list(valuesSorted.keys())[j]
            n += [int(len(valuesSorted[iKey])/2)] # there are +- n variations = 2*2 in the list
            iVar = np.linspace(-n[j]*fVar[i]*100, n[j]*fVar[i]*100, 2*n[j]+1) # variations 
            if len(iVar) != 1: 
                iPlt = iVar # 2*n+1, contains the reference value
                
                pltValues = np.r_[valuesSorted[iKey][:int(len(iVar)/2),i], valRef[i], valuesSorted[iKey][int(len(iVar)/2):,i]] 
            else: 
                pltValues = [valRef[i], valuesSorted[iKey][0,i]]
                iVar = np.r_[iVar, fVar[i]*100]
                iPlt = iVar
            iVar = np.delete(iVar, n[j]) # without the reference value
            axs[i].plot(iPlt, pltValues, '--', color=np.ones(3)*0.8) # connect all values including reference values
            if sensitivity[j,i] != 0: 
                sDigits = int(-np.log10(abs(sensitivity[j,i])) + 4)
            else: 
                sDigits = 1
            axs[i].plot(iVar, valuesSorted[iKey][:,i], 'o', label='{}, s={}'.format(iKey, np.round(sensitivity[j,i], sDigits))) # plot variation as points
        axs[i].plot(0, valRef[i], 'd', label='Ref') # reference value in the center of plot
        axs[i].set(ylabel=strYAxis[i])
        axs[i].legend()
        axs[i].grid()
    axs[-1].set( xlabel='Variation in $\\%$') # in % because spacing is fVar*100 on x-Axis
    plt.tight_layout()
    return [fig, axs]

