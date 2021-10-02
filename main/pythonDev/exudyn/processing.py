#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The processing module supports multiple execution of EXUDYN models.
#           It includes parameter variation and (genetic) optimization functionality.
#
# Author:   Johannes Gerstmayr 
# Date:     2020-11-17
# Notes:    Parallel processing, which requires multiprocessing library, can lead to considerable speedup (measured speedup factor > 50 on 80 core machine). The progess bar during multiprocessing requires the library tqdm.
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import numpy as np
import sys

#function: internal output function for ParameterVariation and GeneticOptimization
# write header or values to output file and increase counter
def WriteToFile(resultsFile, parameters, currentGeneration, values, globalCnt, writeHeader = False, fileType='genetic optimization'):
    if resultsFile != '':
        #print('write to file')
        if writeHeader:
            file = open(resultsFile, 'w')
            file.write('#EXUDYN '+fileType+' results file:'+resultsFile+'\n')
            file.write('#results stored columnwise for every parameter and individual\n')
            file.write('#\n')
            file.write('#\n')
            file.write('#columns:\n') #'globalIndex, parameters, computationIndex:\n')
            s = '#globalIndex,value'
            for (key,value) in parameters.items():
                s += ',' + key
            s += ',computationIndex'
            file.write(s+'\n')
            file.write('#parameter ranges [format: (begin, end, numberOfVariations) or list, parameters separated with ";"]:\n')
            sep = ''
            s = '#'
            for (key,value) in parameters.items():
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
                s += ', ' + str(value)
            file.write(s+'\n')
            globalCnt += 1 #for every line of values
            
        file.close()
        #print('... done')
    return globalCnt
    


#**function: processes parameterFunction for given parameters in parameterList, see ParameterVariation
#**input:
#    parameterFunction: function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
#    parameterList: list of parameter sets (as dictionaries) which are fed into the parameter variation, e.g., [{'mass': 10}, {'mass':20}, ...]
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#    resultsFile: if provided, output is immediately written to resultsFile during processing
#**output: returns values containing the results according to parameterList
#**notes: options are passed from Parametervariation
def ProcessParameterList(parameterFunction, parameterList, addComputationIndex, useMultiProcessing, **kwargs):
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

    resultsFileCnt = 0 #counter for results file
    if not useMultiProcessing:
        for i in range(nVariations):
            parameters = parameterList[i]
            v = parameterFunction(parameters)
            values += [v]
            if showProgress:
                print("\rrun ", i+1, "/", nVariations, ": parameters=", parameters, "value =",v, end='', flush=True)
            if resultsFile != '':
                resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                          [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                          fileType='parameter variation')
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
                try: #_instances only available after first run!
                    tqdm.tqdm._instances.clear() #if open instances of tqdm, which leads to nasty newline
                except:
                    pass
                useTQDM = True
            except:
                pass
                #print("module 'tqdm' not available (use pip to install); progress bar not shown")
        
        if useTQDM:
            with Pool(processes=numberOfThreads) as p:
                #values = list(tqdm.tqdm(p.imap(parameterFunction, vInput), total=nVariations))
                for v in (tqdm.tqdm(p.imap(parameterFunction, vInput), total=nVariations)):
                    values+=[v]
                    if resultsFile != '':
                        resultsFileCnt = WriteToFile(resultsFile, parameters, [parameterList[resultsFileCnt]], 
                                                  [v], resultsFileCnt, writeHeader = (resultsFileCnt == 0), 
                                                  fileType='parameter variation')
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
                                                  fileType='parameter variation')
                        #print("value=",i)
                
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
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#**output:
#    returns [parameterList, values], containing, e.g., parameterList={'mass':[1,1,1,2,2,2,3,3,3], 'stiffness':[4,5,6, 4,5,6, 4,5,6]} and the result values of the parameter variation accoring to the parameterList, 
#           values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
#**example:
#   ParameterVariation(parameters={'mass':(1,10,10), 'stiffness':(1000,10000,10)}, parameterFunction=Test, useMultiProcessing=True)
def ParameterVariation(parameterFunction, parameters, 
                       useLogSpace=False, debugMode=False, addComputationIndex=False,
                       useMultiProcessing=False, showProgress = True,
                       **kwargs):
    
    # debugMode = False
    # if 'debugMode' in kwargs:
    #     debugMode = kwargs['debugMode']

    # useLogSpace = False
    # if 'useLogSpace' in kwargs and kwargs['useLogSpace']==True:
    #     useLogSpace = True
    
    # addComputationIndex = False
    # if 'addComputationIndex' in kwargs and kwargs['addComputationIndex']==True:
    #     addComputationIndex = True
    
    # useMultiProcessing = False
    # if 'useMultiProcessing' in kwargs and kwargs['useMultiProcessing']==True:
    #     useMultiProcessing = True

    # showProgress = True #for larger variations very nice to have
    # if 'showProgress' in kwargs: 
    #     showProgress = kwargs['showProgress']

    if 'multiprocessing' in sys.modules:
        from multiprocessing import cpu_count
        numberOfThreads = cpu_count() #cpu_count in fact gives number of threads ...
        if debugMode:
            print("using", numberOfThreads, "cpus")
    else:
        numberOfThreads = 8
    if 'numberOfThreads' in kwargs: 
        numberOfThreads = kwargs['numberOfThreads']

    resultsFile = ''
    if 'resultsFile' in kwargs: 
        resultsFile = kwargs['resultsFile']

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
    parameterDict = {} #dictionary of parameter lists
    for (key,value) in parameters.items():
        if isinstance(value, tuple): #then it is a range (start, end, numberOfValues)
            pStart = value[0]
            pEnd = value[1]
            pRange = value[2]
            
            #now create list of parameters, using duplicates according to dimensionality
            if useLogSpace:
                space = np.logspace(np.log10(pStart),np.log10(pEnd),pRange)
            else:
                space = np.linspace(pStart,pEnd,pRange)
        else: #already checked above: if isinstance(value, list): #then it contains list of values, e.g., [1,2,4,8]
            space = value
            
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
        for (key,value) in parameterDict.items(): 
            parameterSet[key] = value[i]
        parameterList += [parameterSet]

    values = ProcessParameterList(parameterFunction, parameterList, addComputationIndex, useMultiProcessing, 
                                  showProgress = showProgress, numberOfThreads=numberOfThreads,
                                  resultsFile = resultsFile, parameters=parameters)


    # if debugMode:
    #print("values =", values)
    
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
#    randomizerInitialization: initialize randomizer at beginning of optimization in order to get reproducible results, provide any integer in the range between 0 and 2**32 - 1 (default: no initialization)
#
#    debugMode: if True, additional print out is done
#    addComputationIndex: if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
#    useMultiProcessing: if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
#    showProgress: if True, shows for every iteration the progress bar (requires tqdm library)
#    numberOfThreads: default: same as number of cpus (threads); used for multiprocessing lib;
#    resultsFile: if provided, the results are stored columnwise into the given file and written after every generation; use resultsMonitor.py to track results in realtime
#    numberOfChildren: (DEPRECATED, UNUSED) number childrens of surviving population
#    survivingIndividuals: (DEPRECATED) number of surviving individuals after children are born
#**output:
#    returns [optimumParameter, optimumValue, parameterList, valueList], containing the optimum parameter set 'optimumParameter', optimum value 'optimumValue', the whole list of parameters parameterList with according objective values 'valueList'
#           values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
#**notes: This function is still under development and shows an experimental state! 
#**example:
#   GeneticOptimization(objectiveFunction = fOpt, parameters={'mass':(1,10), 'stiffness':(1000,10000)})
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
        print("number of threads used =", numberOfThreads,flush=True) #very useful information

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
        if addComputationIndex:
            ind['computationIndex'] = i #unique index for one set of computations

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
        values = ProcessParameterList(objectiveFunction, currentGeneration, addComputationIndex, useMultiProcessing, showProgress = showProgress, numberOfThreads=numberOfThreads)
        if (showProgress and useMultiProcessing and popCnt < numberOfGenerations-1): print("            #"+str(popCnt+1), end='')
        #print("values=",values)
        resultsFileCnt = WriteToFile(resultsFile, parameters, currentGeneration, values, resultsFileCnt, writeHeader = (popCnt == 0))

        #remove computationIndex from new generation
        for item in currentGeneration:
            if 'computationIndex' in item:
                del item['computationIndex']

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
                                if key != 'computationIndex':
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
                            value = RandomNumber(childDistribution, 
                                                 pBegin, pEnd, 
                                                 value[0], value[1])
                            indList[pi][key] = value
    
                        if addComputationIndex:
                            indList[pi]['computationIndex'] = cnt #unique index for one set of computations
                        cnt += 1
                        
                        if p < populationSize:
                            currentGeneration += [indList[pi]]
                            p += 1
            #print("pop", popCnt, ": currentGeneration=\n",currentGeneration)
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
            if key != 'computationIndex':
                parameterData = np.zeros(n)
                #extract parameter list from list of dictionaries:
                for i in range(n):
                    parameterData[i] = parametersAll[i][key]
    
                #add parameter list to final dictionary
                parameterList[key] = parameterData


    return [optimumParameter, optimumValue, parameterList, valueList]




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


