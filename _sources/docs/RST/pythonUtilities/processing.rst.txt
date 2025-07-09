
.. _sec-module-processing:

Module: processing
==================

The processing module supports multiple execution of EXUDYN models.
It includes parameter variation and (genetic) optimization functionality.

- Author:    Johannes Gerstmayr, Stefan Holzinger 
- Date:      2020-11-17 (2022-02-04 modified by Stefan Holzinger) 
- Notes:     Parallel processing, which requires multiprocessing library, can lead to considerable speedup (measured speedup factor > 50 on 80 core machine). The progess bar during multiprocessing requires the library tqdm. 


.. _sec-processing-getversionplatformstring:

Function: GetVersionPlatformString
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetVersionPlatformString <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L29>`__\ ()

- | \ *function description*\ :
  | internal function to return Exudyn version string, which allows to identify how results have been obtained
  | writes something like 'Exudyn version = 1.2.33.dev1; Python3.9.11; Windows AVX2 FLOAT64; Windows10 V10.0.19044; AMD64; Intel64 Family 6 Model 142 Stepping 10, GenuineIntel'
- | \ *notes*\ :
  | If exudyn C++ module is not available, it outputs the Python version



----


.. _sec-processing-processparameterlist:

Function: ProcessParameterList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ProcessParameterList <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L162>`__\ (\ ``parameterFunction``\ , \ ``parameterList``\ , \ ``useMultiProcessing``\ , \ ``clusterHostNames = []``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | processes parameterFunction for given parameters in parameterList, see ParameterVariation
- | \ *input*\ :
  | \ ``parameterFunction``\ : function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
  | \ ``parameterList``\ : list of parameter sets (as dictionaries) which are fed into the parameter variation, see example
  | \ ``useMultiProcessing``\ : if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
  | \ ``numberOfThreads``\ : default: same as number of cpus (threads); used for multiprocessing lib;
  | \ ``resultsFile``\ : if provided, output is immediately written to resultsFile during processing
  | \ ``clusterHostNames``\ : list of hostnames, e.g. clusterHostNames=['123.124.125.126','123.124.125.127'] providing a list of strings with IP addresses or host names, see dispy documentation. If list is non-empty and useMultiProcessing==True and dispy is installed, cluster computation is used; NOTE that cluster computation speedup factors shown are not fully true, as they include a significant overhead; thus, only for computations which take longer than 1-5 seconds and for sufficient network bandwith, the speedup is roughly true
  | \ ``useDispyWebMonitor``\ : if given in \*\*kwargs, a web browser is startet in case of cluster computation to manage the cluster during computation
  | \ ``useMPI``\ : if given in \*\*kwargs and set True, and if Python package mpi4py is installed, mpi parallelization is used; for hints see parameterVariationExample.py
- | \ *output*\ :
  | returns values containing the results according to parameterList
- | \ *notes*\ :
  | options are passed from Parametervariation
- | \ *example*\ :

.. code-block:: python

  def PF(parameterSet):
      #in reality, value will be result of a complex exudyn simulation:
      value = sin(parameterSet['mass']) * parameterSet['stiffness']
      return value
  values=ProcessParameterList(parameterFunction=PF,
                              parameterList=[{'m':1, 's':100},
                                            {'m':2, 's':100},
                                            {'m':3, 's':100},
                                            {'m':1, 's':200},
                                            {'m':2, 's':250},
                                            {'m':3, 's':300},
                                            ], useMultiProcessing=False )




----


.. _sec-processing-parametervariation:

Function: ParameterVariation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ParameterVariation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L391>`__\ (\ ``parameterFunction``\ , \ ``parameters``\ , \ ``useLogSpace = False``\ , \ ``debugMode = False``\ , \ ``addComputationIndex = False``\ , \ ``useMultiProcessing = False``\ , \ ``showProgress = True``\ , \ ``parameterFunctionData = {}``\ , \ ``clusterHostNames = []``\ , \ ``numberOfThreads = None``\ , \ ``resultsFile = ''``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | calls successively the function parameterFunction(parameterDict) with variation of parameters in given range; parameterDict is a dictionary, containing the current values of parameters,
  | e.g., parameterDict=['mass':13, 'stiffness':12000] to be computed and returns a value or a list of values which is then stored for each parameter
- | \ *input*\ :
  | \ ``parameterFunction``\ : function, which takes the form parameterFunction(parameterDict) and which returns any values that can be stored in a list (e.g., a floating point number)
  | \ ``parameters``\ : given as a dictionary, consist of name and tuple of (begin, end, numberOfValues) same as in np.linspace(...), e.g. 'mass':(10,50,10), for a mass varied from 10 to 50, using 10 steps OR a list of values [v0, v1, v2, ...], e.g. 'mass':[10,15,25,50]
  | \ ``useLogSpace``\ : (optional) if True, the parameters are varied at a logarithmic scale, e.g., [1, 10, 100] instead linear [1, 50.5, 100]
  | \ ``debugMode``\ : if True, additional print out is done
  | \ ``addComputationIndex``\ : if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
  | \ ``useMultiProcessing``\ : if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
  | \ ``showProgress``\ : if True, shows for every iteration the progress bar (requires tqdm library)
  | \ ``resultsFile``\ : if provided, output is immediately written to resultsFile during processing
  | \ ``numberOfThreads``\ : default(None): same as number of cpus (threads); used for multiprocessing lib;
  | \ ``parameterFunctionData``\ : dictionary containing additional data passed to the parameterFunction inside the parameters with dict key 'functionData'; use this e.g. for passing solver parameters or other settings
  | \ ``clusterHostNames``\ : list of hostnames, e.g. clusterHostNames=['123.124.125.126','123.124.125.127'] providing a list of strings with IP addresses or host names, see dispy documentation. If list is non-empty and useMultiProcessing==True and dispy is installed, cluster computation is used; NOTE that cluster computation speedup factors shown are not fully true, as they include a significant overhead; thus, only for computations which take longer than 1-5 seconds and for sufficient network bandwith, the speedup is roughly true
  | \ ``useDispyWebMonitor``\ : if given in \*\*kwargs, a web browser is started in case of cluster computation to manage the cluster during computation
  | \ ``useMPI``\ : if given in \*\*kwargs and set True, and if Python package mpi4py is installed, mpi parallelization is used; for hints see parameterVariationExample.py
- | \ *output*\ :
  | returns [parameterList, values], containing, e.g., parameterList=\{'mass':[1,1,1,2,2,2,3,3,3], 'stiffness':[4,5,6, 4,5,6, 4,5,6]\} and the result values of the parameter variation accoring to the parameterList,
  | values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
- | \ *example*\ :

.. code-block:: python

  if __name__ == '__main__':
      ParameterVariation(parameterFunction=Test,
                         parameters={'mass':(1,10,10), 'stiffness':(1000,10000,10)},
                         useMultiProcessing=True)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `dispyParameterVariationExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/dispyParameterVariationExample.py>`_\  (Ex), \ `mpi4pyExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mpi4pyExample.py>`_\  (Ex), \ `multiprocessingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiprocessingTest.py>`_\  (Ex), \ `parameterVariationExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/parameterVariationExample.py>`_\  (Ex), \ `geneticOptimizationTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geneticOptimizationTest.py>`_\  (TM)



----


.. _sec-processing-geneticoptimization:

Function: GeneticOptimization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GeneticOptimization <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L540>`__\ (\ ``objectiveFunction``\ , \ ``parameters``\ , \ ``populationSize = 100``\ , \ ``numberOfGenerations = 10``\ , \ ``elitistRatio = 0.1``\ , \ ``crossoverProbability = 0.25``\ , \ ``crossoverAmount = 0.5``\ , \ ``rangeReductionFactor = 0.7``\ , \ ``distanceFactor = 0.1``\ , \ ``childDistribution = "uniform"``\ , \ ``distanceFactorGenerations = -1``\ , \ ``debugMode = False``\ , \ ``addComputationIndex = False``\ , \ ``useMultiProcessing = False``\ , \ ``showProgress = True``\ , \ ``clusterHostNames = []``\ , \ ``parameterFunctionData = {}``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | compute minimum of given objectiveFunction
- | \ *input*\ :
  | \ ``objectiveFunction``\ : function, which takes the form parameterFunction(parameterDict) and which returns a value or list (or numpy array) which reflects the size of the objective to be minimized
  | \ ``parameters``\ : given as a dictionary, consist of name and tuple containing the search range for this parameter (begin, end), e.g. 'mass':(10,50)
  | \ ``populationSize``\ : individuals in every generation
  | \ ``initialPopulationSize``\ : number of random initial individuals; default: population size
  | \ ``numberOfGenerations``\ : number of generations; NOTE: it is required that elitistRatio\*populationSize >= 1
  | \ ``elitistRatio``\ : the number of surviving individuals in every generation is equal to the previous population times the elitistRatio
  | \ ``crossoverProbability``\ : if > 0: children are generated from two (randomly selected) parents by gene-crossover; if 0, no crossover is used
  | \ ``crossoverAmount``\ : if crossoverProbability > 0, then this amount is the probability of genes to cross; 0.1: small amount of genes cross, 0.5: 50\% of genes cross
  | \ ``rangeReductionFactor``\ : reduction of mutation range (boundary) relative to range of last generation; helps algorithm to converge to more accurate values
  | \ ``distanceFactor``\ : children only survive at a certain relative distance of the current range; must be small enough (< 0.5) to allow individuals to survive; ignored if distanceFactor=0; as a rule of thumb, the distanceFactor should be zero in case that there is only one significant minimum, but if there are many local minima, the distanceFactor should be used to search at several different local minima
  | \ ``childDistribution``\ : string with name of distribution for producing childs: "normal" (Gaussian, with sigma defining range), "uniform" (exactly in range of childs)
  | \ ``distanceFactorGenerations``\ : number of generations (populations) at which the distance factor is active; the distance factor is used to find several local minima; finally, convergence is speed up without the distance factor
  | \ ``parameterFunctionData``\ : dictionary containing additional data passed to the objectiveFunction inside the parameters with dict key 'functionData'; use this e.g. for passing solver parameters or other settings
  | \ ``randomizerInitialization``\ : initialize randomizer at beginning of optimization in order to get reproducible results, provide any integer in the range between 0 and 2\*\*32 - 1 (default: no initialization)
  | \ ``debugMode``\ : if True, additional print out is done
  | \ ``addComputationIndex``\ : if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter, etc.
  | \ ``useMultiProcessing``\ : if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
  | \ ``showProgress``\ : if True, shows for every iteration the progress bar (requires tqdm library)
  | \ ``numberOfThreads``\ : default: same as number of cpus (threads); used for multiprocessing lib;
  | \ ``resultsFile``\ : if provided, the results are stored columnwise into the given file and written after every generation; use resultsMonitor.py to track results in realtime
  | \ ``clusterHostNames``\ : list of hostnames, e.g. clusterHostNames=['123.124.125.126','123.124.125.127'] providing a list of strings with IP addresses or host names, see dispy documentation. If list is non-empty and useMultiProcessing==True and dispy is installed, cluster computation is used; NOTE that cluster computation speedup factors shown are not fully true, as they include a significant overhead; thus, only for computations which take longer than 1-5 seconds and for sufficient network bandwith, the speedup is roughly true
  | \ ``useDispyWebMonitor``\ : if given in \*\*kwargs, a web browser is startet in case of cluster computation to manage the cluster during computation
- | \ *output*\ :
  | returns [optimumParameter, optimumValue, parameterList, valueList], containing the optimum parameter set 'optimumParameter', optimum value 'optimumValue', the whole list of parameters parameterList with according objective values 'valueList'
  | values=[7,8,9 ,3,4,5, 6,7,8] (depends on solution of problem ..., can also contain tuples, etc.)
- | \ *notes*\ :
  | This function is still under development and shows an experimental state!
- | \ *example*\ :

.. code-block:: python

  GeneticOptimization(objectiveFunction = fOpt, parameters={'mass':(1,10), 'stiffness':(1000,10000)})


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Ex), \ `shapeOptimization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/shapeOptimization.py>`_\  (Ex), \ `geneticOptimizationTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geneticOptimizationTest.py>`_\  (TM)



----


.. _sec-processing-minimize:

Function: Minimize
^^^^^^^^^^^^^^^^^^
`Minimize <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L886>`__\ (\ ``objectiveFunction``\ , \ ``parameters``\ , \ ``initialGuess = []``\ , \ ``method = 'Nelder-Mead'``\ , \ ``tol = 1e-4``\ , \ ``options = {}``\ , \ ``enforceBounds = True``\ , \ ``debugMode = False``\ , \ ``showProgress = True``\ , \ ``addComputationIndex = False``\ , \ ``storeFunctionValues = True``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | Compute minimum of given objectiveFunction. This function is based on scipy.optimize.minimize() and it provides the same interface as GeneticOptimization(). Note that in special cases, you should copy this function and adapt to your needs.
- | \ *input*\ :
  | \ ``objectiveFunction``\ : function, which takes the form parameterFunction(parameterDict) and which returns a value or list (or numpy array) which reflects the size of the objective to be minimized
  | \ ``parameters``\ : given as a dictionary, consist of name and tuple containing the search range for this parameter (begin, end), e.g. 'mass':(10,50)
  | \ ``storeFunctionValues``\ : if True, objectiveFunction values are computed (additional costs!) and stored in every iteration into valueList
  | \ ``initialGuess``\ : initial guess. Array of real elements of size (n,), where 'n' is the number of independent variables. If not provided by the user, initialGuess is computed from bounds provided in parameterDict.
  | \ ``method``\ : solver that should be used, e.g. 'Nelder-Mead', 'Powell', 'CG' etc. A list of available solvers can be found in the documentation of scipy.optimize.minimize().
  | \ ``tol``\ : tolerance for termination. When tol is specified, the selected minimization algorithm sets some relevant solver-specific tolerance(s) equal to tol (but this is usually not the tolerance for loss or parameters1). For detailed control, use solver-specific options using the 'options' variable.
  | \ ``options``\ : dictionary of solver options. Can be used to set absolute and relative error tolerances. Detailed information can be found in the documentation of scipy.optimize.minimize().
  | \ ``enforceBounds``\ : if True, ensures that only parameters within the bounds specified in ParameterDict are used for minimization; this may help to avoid, e.g., negative values, but may lead to non-convergence
  | \ ``verbose``\ : prints solver information into console, e.g. number of iterations 'nit', number of funcion evaluations 'nfev', status etc.
  | \ ``showProgress``\ : if True, shows for every iteration objective function value, current iteration number, time needed for current iteration, maximum number of iterations and loss (current value of objective function)
  | \ ``addComputationIndex``\ : if True, key 'computationIndex' is added for consistency reasons with GeneticOptimizaiton to every parameterDict in the call to parameterFunction(); however, the value is always 0, because no multi threading is used in Minimize(...)
  | \ ``resultsFile``\ : if provided, the results are stored columnwise into the given file and written after every generation; use resultsMonitor.py to track results in realtime
  | \ ``useScipyBounds``\ : if True, use scipy.optimize.minimize() option 'bounds' to apply bounds on variable specified in ParameterDict. Note, this option is only used by some specific methods of scipy.optimize.minimize()! method='Nelder-Mead' ignores this option for example! if False, option 'enforceBounds' will be set to False!
  | \ ``args``\ : extra arguments passed to the objective function and its derivatives (fun, jac and hess functions).
- | \ *output*\ :
  | returns [optimumParameter, optimumValue, parameterList, valueList], containing the optimum parameter set 'optimumParameter', optimum value 'optimumValue', the whole list of parameters parameterList with according objective values 'valueList'
- | \ *author*\ :
  | Stefan Holzinger, Johannes Gerstmayr
- | \ *notes*\ :
  | This function is still under development and shows an experimental state! There are currently unused arguments of scipy.optimize.minimize(): Detailed information can be found in the documentation of scipy.optimize.minimize().

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `minimizeExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/minimizeExample.py>`_\  (Ex), \ `shapeOptimization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/shapeOptimization.py>`_\  (Ex)



----


.. _sec-processing-computesensitivities:

Function: ComputeSensitivities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeSensitivities <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L1087>`__\ (\ ``parameterFunction``\ , \ ``parameters``\ , \ ``scaledByReference = False``\ , \ ``debugMode = False``\ , \ ``addComputationIndex = False``\ , \ ``useMultiProcessing = False``\ , \ ``showProgress = True``\ , \ ``parameterFunctionData = dict()``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | Perform a sensitivity analysis by successively calling the function parameterFunction(parameterList[i]) with a one at a time variation of parameters in the defined increments.
  | e.g., parameterList[0] =['mass':13, 'stiffness':12000] to be computed and returns a value or a list of values which is then stored for each parameter
- | \ *input*\ :
  | \ ``parameterFunction``\ : function, which takes the form parameterFunction(parameterDict) and which returns one or more output values for which the sensitivity is calculated
  | \ ``parameters``\ : given as a dictionary, consist of name and tuple of (begin, Variation steps, numberOfValues) e.g. 'mass':(10,0.01,5), for a reference mass of 10, incremented by 0.01\*10 and using 5 steps in negative and positive, doing 10 steps in total
  | \ ``scaledByReference``\ : if true multiplies the sensitivities with the corresponding reference parameters, so that the sensitivity resembles a change relative to the reference value
  | \ ``debugMode``\ : if True, additional information is shown
  | \ ``addComputationIndex``\ : if True, key 'computationIndex' is added to every parameterDict in the call to parameterFunction(), which allows to generate independent output files for every parameter etc.
  | \ ``useMultiProcessing``\ : if True, the multiprocessing lib is used for parallelized computation; WARNING: be aware that the function does not check if your function runs independently; DO NOT use GRAPHICS and DO NOT write to same output files, etc.!
  | \ ``showProgress``\ : if True, shows for every iteration the progress bar (requires tqdm library)
  | \ ``resultsFile``\ : if provided, output is immediately written to resultsFile during processing
  | \ ``numberOfThreads``\ : default: same as number of cpus (threads); used for multiprocessing lib;
  | \ ``parameterFunctionData``\ : dictionary containing additional data passed to the parameterFunction inside the parameters with dict key 'functionData'; use this e.g. for passing solver parameters or other settings
- | \ *output*\ :
  | returns [parameterList, valRef, valuesSorted, sensitivity], parameterList containing the list of dictionaries processed. valRef is the Solution for the reference values paramList[0], valuesSorted contains the results sorted by the dictionary key that was varied in the simulation. The sensitivity contains the calculated sensitivity, where the rows are the corresponding outputparameters, while the columns are the input parameters, thereby the index sensitivity[1,0] is the sensitivity of output parameter 1 with respect to the input parameter 0.
- | \ *author*\ :
  | Peter Manzl
- | \ *example*\ :

.. code-block:: python

  ComputeSensitivities(parameterFunction=ParameterFunction, parameters = {'mass': (mRef, 0.01, 3), 'spring': (1000,0.01, 10),}, multiprocessing=True)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_\  (Ex)



----


.. _sec-processing-plotoptimizationresults2d:

Function: PlotOptimizationResults2D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`PlotOptimizationResults2D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L1201>`__\ (\ ``parameterList``\ , \ ``valueList``\ , \ ``xLogScale = False``\ , \ ``yLogScale = False``\ )

- | \ *function description*\ :
  | visualize results of optimization for every parameter (2D plots)
- | \ *input*\ :
  | \ ``parameterList``\ : taken from output parameterList of \ ``GeneticOptimization``\ , containing a dictinary with lists of parameters
  | \ ``valueList``\ : taken from output valueList of \ ``GeneticOptimization``\ ; containing a list of floats that result from the objective function
  | \ ``xLogScale``\ : use log scale for x-axis
  | \ ``yLogScale``\ : use log scale for y-axis
- | \ *output*\ :
  | return [figList, axList] containing the corresponding handles; creates a figure for every parameter in parameterList

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Ex), \ `minimizeExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/minimizeExample.py>`_\  (Ex), \ `shapeOptimization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/shapeOptimization.py>`_\  (Ex), \ `geneticOptimizationTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geneticOptimizationTest.py>`_\  (TM)



----


.. _sec-processing-plotsensitivityresults:

Function: PlotSensitivityResults
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`PlotSensitivityResults <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/processing.py\#L1257>`__\ (\ ``valRef``\ , \ ``valuesSorted``\ , \ ``sensitivity``\ , \ ``fVar = None``\ , \ ``strYAxis = None``\ )

- | \ *function description*\ :
  | visualize results of Sensitivityanalyis for every parameter (2D plots)
- | \ *input*\ :
  | \ ``valRef``\ : The output values of the reference solution
  | \ ``valuesSorted``\ : The output values of the analysed function sorted by the parameter which was varied
  | \ ``sensitivity``\ : The sensitivity Matrix calculated by the function \ ``ComputeSensitivities()``\ 
  | \ ``fVar``\ : The list of variation stepsizes. It is assumed to be 1e-3 if not defined.
  | \ ``strYAxis``\ : A list of strings to label the plots yAxis
- | \ *output*\ :
  | return [fig, axs] containing the corresponding handles; creates a subplot for every row in the sensitivity matrix
- | \ *author*\ :
  | Peter Manzl

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_\  (Ex)

