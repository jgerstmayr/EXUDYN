#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The processing module supports multiple execution of EXUDYN models
#           it includes parameter variation and optimization functionality
#
# Author:   Johannes Gerstmayr 
# Date:     2020-11-17
# Notes:    This module is still under construction and for testing purposes only!
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys


import numpy as np
import sys

#**function: processes parameterFunction for given parameters in parameterList, see ParameterVariation
#**input:
#    parameterFunction: function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
#    parameterList: list of parameter sets (as dictionaries) which are fed into the parameter variation, e.g., [{'mass': 10}, {'mass':20}, ...]
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#**output: returns values containing the results according to parameterList
#**notes: options are passed from Parametervariation
def ProcessParameterList(parameterFunction, parameterList, addComputationIndex, useMultiProcessing, **kwargs):
    values = [] #create empty list
    nVariations = len(parameterList)
    showProgress = False
    if 'showProgress' in kwargs: 
        showProgress = kwargs['showProgress']

    if not useMultiProcessing:
        for i in range(nVariations):
            parameters = parameterList[i]
            v = parameterFunction(parameters)
            values += [v]
            if showProgress:
                print("\rrun ", i+1, "/", nVariations, ": parameters=", parameters, "value =",v, end='', flush=True)
        if showProgress:
            print("", flush=True) #newline after tqdm progress bar output....
    else:
        from multiprocessing import Pool, cpu_count #parallelization of computation
       
        numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
        if 'numberOfThreads' in kwargs: 
            numberOfThreads = kwargs['numberOfThreads']
        
        vInput = np.array(parameterList)

        useTQDM = False
        if showProgress:
            try:
                import tqdm #progress bar
                tqdm.tqdm._instances.clear() #if open instances of tqdm, which leads to nasty newline
                useTQDM = True
            except:
                dummy=1
                #print("module 'tqdm' not available (use pip to install); progress bar not shown")
        
        if useTQDM:
            with Pool(processes=numberOfThreads) as p:
                #values = list(tqdm.tqdm(p.imap_unordered(parameterFunction, vInput), total=nVariations))
                values = list(tqdm.tqdm(p.imap(parameterFunction, vInput), total=nVariations))
            print("", flush=True) #newline after tqdm progress bar output....
        else:
            #simpler approach without tqdm:
            with Pool(numberOfThreads) as p:
                values = p.map(parameterFunction, vInput)
    return values

#**function: calls successively the function parameterFunction(parameterDict) with variation of parameters in given range; parameterDict is a dictionary, containing the current values of parameters,
#  e.g., parameterDict=['mass':13, 'stiffness':12000] to be computed and returns a value or a list of values which is then stored for each parameter
#**input:
#    parameterFunction: function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
#    parameters: given as a dictionary, consist of name and triple of (begin, end and steps) same as in np.linspace(...), e.g. 'mass':(10,50,10) for a mass varied from 10 to 50, using 10 steps
#    useLogSpace: (optional) if True, the parameters are varied at a logarithmic scale, e.g., [1, 10, 100] instead linear [1, 50.5, 100]
#    debugMode: if True, additional print out is done
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#**output:
#    returns [parameterList, values], containing, e.g., parameterList={'mass':[1,1,1,2,2,2,3,3,3], 'stiffness':[4,5,6, 4,5,6, 4,5,6]} and the result values of the parameter variation accoring to the parameterList, 
#           values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
#**example:
#   ParameterVariation(parameters={'mass':(1,10,10), 'stiffness':(1000,10000,10)}, parameterFunction=Test, useMultiProcessing=True)
def ParameterVariation(parameterFunction, parameters, **kwargs):
    
    debugMode = False
    if 'debugMode' in kwargs:
        debugMode = kwargs['debugMode']

    useLogSpace = False
    if 'useLogSpace' in kwargs and kwargs['useLogSpace']==True:
        useLogSpace = True
    
    addComputationIndex = False
    if 'addComputationIndex' in kwargs and kwargs['addComputationIndex']==True:
        addComputationIndex = True
    
    useMultiProcessing = False
    if 'useMultiProcessing' in kwargs and kwargs['useMultiProcessing']==True:
        useMultiProcessing = True
        if debugMode:
            print("using multiprocessing")

    if 'multiprocessing' in sys.modules:
        from multiprocessing import cpu_count
        numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
        if debugMode:
            print("using", numberOfThreads, "cpus")
    else:
        numberOfThreads = 8
    if 'numberOfThreads' in kwargs: 
        numberOfThreads = kwargs['numberOfThreads']

    #generate list of parameters to iterate
    dim = len(parameters)       #dimensionality (dimension) of problem
    nParams = np.zeros(dim, dtype=int)     #number of variations in each dimension
    cnt = 0
    for (key,value) in parameters.items(): 
        nParams[cnt] = parameters[key][2] #last value is the number of variations
        cnt+=1 #counts the dimensionality
        
        
    nVariations = np.array(nParams).prod() #product of all ranges gives total count
    if nVariations == 0:
        print("WARNING: number of variations =", nVariations)
        return []

    if debugMode:
        print("number of variations =", nVariations)

    cnt = 0
    parameterDict = {} #dictionary of parameter lists
    for (key,value) in parameters.items(): 
        pStart = parameters[key][0]
        pEnd = parameters[key][1]
        pRange = parameters[key][2]
        
        range1 = nParams[0:cnt].sum()
        range2 = nParams[cnt+1:dim+1].sum()
        if range1 == 0:
            range1 = 1 #otherwise kronecker product won't work
        if range2 == 0:
            range2 = 1 #otherwise kronecker product won't work

        # if debugMode:
        #     print("parameter", cnt, ":")
        #     print("  range1 =", range1)
        #     print("  range2 =", range2)
            
        #now create list of parameters, using duplicates according to dimensionality
        if useLogSpace:
            space = np.logspace(np.log10(pStart),np.log10(pEnd),pRange)
        else:
            space = np.linspace(pStart,pEnd,pRange)
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
        for (key,value) in parameterDict.items(): 
            parameterSet[key] = value[i]
        parameterList += [parameterSet]

    # if debugMode:
    #     print("parameterList =", parameterList)

    values = ProcessParameterList(parameterFunction, parameterList, addComputationIndex, useMultiProcessing, showProgress = True, numberOfThreads=numberOfThreads)


    # if debugMode:
    #print("values =", values)
    
    return [parameterDict, values]
    
#**function: compute minimum of given objectiveFunction
#**input:
#    objectiveFunction: function, which takes the form parameterFunction(parameterDict) and which returns a value or list (or numpy array) which reflects the size of the objective to be minimized
#    parameters: given as a dictionary, consist of name and tuple of (begin, end), e.g. 'mass':(10,50)
#
#    initialPopulationSize: number of random initial individuals (default: 100)
#    numberOfGenerations: number of generations (default: 10)
#    numberOfChildren: number childrens of surviving population (default: 8)
#    useGenCrossing: if True, the children are generated from parents by gen-crossover (default: True)
#    survivingIndividuals: number of surviving individuals after children are born (default: 8)
#    rangeReductionFactor: (not implemented yet!) reduction of mutation range relative to ranges of last step (default: 0.7)
#    randomizerInitialization: (not implemented yet!) initialize randomizer at beginning in order to get reproducible results (Default: True)
#    distanceFactor: (not implemented yet!) children only survive at a certain distance of the current range (Default: 0.1)
#
#    debugMode: if True, additional print out is done
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#**output:
#    returns [optimumParameter, optimumValue, parametersAll, valuesAll], containing the optimum parameter set 'optimumParameter', optimum value 'optimumValue', the whole list of parameters parametersAll with according objective values 'valuesAll'
#           values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
#**notes: This function is still under development and shows an experimental state! 
#**example:
#   GeneticOptimization(objectiveFunction = fOpt, parameters={'mass':(1,10), 'stiffness':(1000,10000)})
def GeneticOptimization(objectiveFunction, parameters, **kwargs):
    debugMode = False
    if 'debugMode' in kwargs:
        debugMode = kwargs['debugMode']

    initialPopulationSize = 100
    if 'initialPopulationSize' in kwargs:
        initialPopulationSize = kwargs['initialPopulationSize']

    numberOfGenerations = 10
    if 'numberOfGenerations' in kwargs:
        numberOfGenerations = kwargs['numberOfGenerations']

    numberOfChildren = 8
    if 'numberOfChildren' in kwargs:
        numberOfChildren = kwargs['numberOfChildren']

    survivingIndividuals = 8
    if 'survivingIndividuals' in kwargs:
        survivingIndividuals = kwargs['survivingIndividuals']

    rangeReductionFactor = 0.7
    if 'rangeReductionFactor' in kwargs:
        rangeReductionFactor = kwargs['rangeReductionFactor']

    addComputationIndex = False
    if 'addComputationIndex' in kwargs and kwargs['addComputationIndex']==True:
        addComputationIndex = True
    
    useMultiProcessing = False
    if 'useMultiProcessing' in kwargs and kwargs['useMultiProcessing']==True:
        useMultiProcessing = True
        if debugMode:
            print("using multiprocessing")

    #get number of threads:
    if 'multiprocessing' in sys.modules:
        from multiprocessing import cpu_count
        numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
    else:
        numberOfThreads = 8
    if 'numberOfThreads' in kwargs: 
        numberOfThreads = kwargs['numberOfThreads']
    
    #generate first generation:
    currentGeneration = []
    for i in range(initialPopulationSize):
        ind = {} #dictionary for individual
        for (key,value) in parameters.items():
            pBegin = value[0]
            pEnd = value[1]
            value = np.random.uniform(pBegin, pEnd)
            ind[key] = value
            if addComputationIndex:
                ind['computationIndex'] = i #unique index for one set of computations

        currentGeneration += [ind]

    if debugMode:
        print("initial population =", currentGeneration)

    #TODO: store all values for all generations
    parametersAll = []
    valuesAll = []

    for popCnt in range(numberOfGenerations):
        if debugMode:
            print("===============\nevaluate population", popCnt, ":")

        values = ProcessParameterList(objectiveFunction, currentGeneration, addComputationIndex, useMultiProcessing, showProgress = True, numberOfThreads=numberOfThreads)
        #print("values=",values)

        #remove computationIndex from new generation
        for item in currentGeneration:
            if 'computationIndex' in item:
                del item['computationIndex']

        #store all values
        parametersAll += currentGeneration
        valuesAll += values
        
        #compute norm and minimum values:
        scalarValues = [(0,0)]*len(values)
        for i in range(len(values)):
            item = values[i]
            if isinstance(item, float):
                scalarValues[i] = (item,i)
            else: #must be list of values ==> compute norm!
                scalarValues[i] = (np.sqrt(np.dot(item,item)),i)

        valuesDtype = [('value', float), ('index', int)]
        scalarValues = np.array(scalarValues,         dtype = valuesDtype)

        sortedValues = np.sort(scalarValues, order='value') #sort for item values
        #print("scalarValues=",scalarValues)
        #print("sortedValues=",sortedValues)

        if popCnt < numberOfGenerations-1: #go on for next population
            #selection: chose best surviving individuals
            #TODO: consider minimum distance between individuals!
            
            newGeneration = []
            cnt = 0
            for i in range(min(survivingIndividuals,len(sortedValues))):
                ind = currentGeneration[int(sortedValues[i][1])] #dictionary for individual
                if addComputationIndex:
                    ind['computationIndex'] = cnt #unique index for one set of computations
                newGeneration += [ind]
            
            #print("survivingIndividuals=",newGeneration)
            if debugMode:
                print("best values=",sortedValues[0:min(survivingIndividuals,4)])
            
            #prolongate best individuals:
            currentGeneration = []
            for item in newGeneration:
                currentGeneration += [item]
    
            #modification
            #TODO: add gene mutation
            for item in newGeneration:
                for j in range(numberOfChildren):
                    ind = {} #dictionary for individual
    
                    for (key,value) in parameters.items():
                        r = value[1]-value[0]
                        r *= rangeReductionFactor**(popCnt+1) #reduce range
                        #print("range=",r)
                        pBegin = item[key]-0.5*r #minimum value
                        pEnd = item[key]+0.5*r 
                        #obej ranges: ==> this may give more values at boundary
                        if pBegin < value[0]: pBegin = value[0]
                        if pEnd > value[1]: pEnd = value[1]

                        #print("new range=",pBegin,pEnd)
                        
                        value = np.random.uniform(pBegin, pEnd)
                        ind[key] = value
                        if addComputationIndex:
                            ind['computationIndex'] = cnt #unique index for one set of computations
                        cnt += 1
    
                    currentGeneration += [ind]
            #print("pop", popCnt, ": currentGeneration=\n",currentGeneration)
        else:
            #select final best individual
            optimumParameter = currentGeneration[int(sortedValues[0][1])]
            optimumValue = sortedValues[0][0]
            print("opt par=", optimumParameter, ", opt val=", optimumValue)

        
    return [optimumParameter, optimumValue, parametersAll, valuesAll]

#**function: visualize results of optimization
#**input: 
#   parameterList: taken from output parametersAll of \texttt{GeneticOptimization}, containing a list of parameter set dictinaries
#   valueList: taken from output valuesAll of \texttt{GeneticOptimization}; containing a list of floats that result from the objective function
#**output: creates a figure for every parameter in parameterList
def PlotOptimizationResults(parameterList, valueList):
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    plt.close("all")
    ax=plt.gca() # get current axes

    n = len(parameterList) #length of data
    if  n == 0:
        print('WARNING: PlotOptimizationResults: parameterList has zero length and therefore terminates!')

    if n != len(valueList):
        raise ValueError('PlotOptimizationResults: length of parameterList is different from length of valueList')
        
    for key in parameterList[0]:
        plt.figure()
        parameterData = np.zeros(n)
        #extract parameter list from list of dictionaries:
        for i in range(n):
            parameterData[i] = parameterList[i][key]
        
        plt.plot(parameterData, valueList, 'b.', label=key) 
        plt.ylabel('value (objective function)')
        plt.xlabel(key)
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        plt.tight_layout()
        plt.legend()

    plt.show() 
    


