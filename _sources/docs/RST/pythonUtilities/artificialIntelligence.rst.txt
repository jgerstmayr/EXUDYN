
.. _sec-module-artificialintelligence:

Module: artificialIntelligence
==============================

This library collects interfaces and functionality for artificial intelligence
This library is under construction (2022-05);
To make use of this libraries, you need to install openAI gym with 'pip install gym';
For standard machine learning algorithms, install e.g. stable_baselines3 using 'pip install stable_baselines3'

- Author:    Johannes Gerstmayr 
- Date:      2022-05-21 (created) 


.. _sec-module-artificialintelligence-class-openaigyminterfaceenv(env):

CLASS OpenAIGymInterfaceEnv(Env) (in module artificialIntelligence)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    interface class to set up Exudyn model which can be used as model in open AI gym;
    see specific class functions which contain 'OVERRIDE' to integrate your model;
    in general, set up a model with CreateMBS(), map state to initial values, initial values to state and action to mbs;


.. _sec-artificialintelligence-openaigyminterfaceenv(env)---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L43>`__\ (\ ``self``\ , \ ``**kwargs``\ )

- | \ *classFunction*\ :
  | internal function to initialize model; store self.mbs and self.simulationSettings; special arguments \*\*kwargs are passed to CreateMBS

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-creatembs:

Class function: CreateMBS
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateMBS <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L77>`__\ (\ ``self``\ , \ ``SC``\ , \ ``mbs``\ , \ ``simulationSettings``\ , \ ``**kwargs``\ )

- | \ *classFunction*\ :
  | OVERRIDE this function to create multibody system mbs and setup simulationSettings; call Assemble() at the end!
  | you may also change SC.visualizationSettings() individually; kwargs may be used for special setup

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-setupspaces:

Class function: SetupSpaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SetupSpaces <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L81>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | OVERRIDE this function to set up self.action_space and self.observation_space

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-mapaction2mbs:

Class function: MapAction2MBS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`MapAction2MBS <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L86>`__\ (\ ``self``\ , \ ``action``\ )

- | \ *classFunction*\ :
  | OVERRIDE this function to map the action given by learning algorithm to the multibody system, e.g. as a load parameter

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-output2stateanddone:

Class function: Output2StateAndDone
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Output2StateAndDone <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L91>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | OVERRIDE this function to collect output of simulation and map to self.state tuple
- | \ *output*\ :
  | return bool done which contains information if system state is outside valid range

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-state2initialvalues:

Class function: State2InitialValues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`State2InitialValues <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L97>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | OVERRIDE this function to maps the current state to mbs initial values
- | \ *output*\ :
  | return [initialValues, initialValues_t] where initialValues[_t] are ODE2 vectors of coordinates[_t] for the mbs

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-testmodel:

Class function: TestModel
^^^^^^^^^^^^^^^^^^^^^^^^^
`TestModel <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L115>`__\ (\ ``self``\ , \ ``numberOfSteps = 500``\ , \ ``seed = 0``\ , \ ``model = None``\ , \ ``solutionFileName = None``\ , \ ``useRenderer = True``\ , \ ``sleepTime = 0.01``\ , \ ``stopIfDone = False``\ , \ ``showTimeSpent = True``\ , \ ``**kwargs``\ )

- | \ *classFunction*\ :
  | test model by running in simulation environment having several options
- | \ *input*\ :
  | \ ``numberOfSteps``\ : number of steps to test MBS and model (with or without learned model); with renderer, press 'Q' in render window to stop simulation
  | \ ``seed``\ : seed value for reset function; this value initializes the randomizer; use e.g. time to obtain non-reproducible results
  | \ ``model``\ : either None to just test the MBS model without learned model, or containing a learned model, e.g., with A2C; use A2C.save(...) and A2C.load(...) for storing and retrieving models
  | \ ``solutionFileName``\ : if given, the MBS internal states are written to the file with given name, which can be loaded with solution viewer and visualized; solution is written every period given in simulationSettings.solutionSettings.solutionWritePeriod
  | \ ``useRenderer``\ : if set True, the internal renderer is used and model updates are shown in visualization of Exudyn
  | \ ``return_info``\ : internal value in reset function
  | \ ``sleepTime``\ : sleep time between time steps to obtain certain frame rate for visualization
  | \ ``stopIfDone``\ : if set to True, the simulation will reset as soon as the defined observation limits are reached and done is set True
  | \ ``showTimeSpent``\ : if True, the total time spent is measured; this helps to check the performance of the model (e.g. how many steps can be computed per second)

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-setsolver:

Class function: SetSolver
^^^^^^^^^^^^^^^^^^^^^^^^^
`SetSolver <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L154>`__\ (\ ``self``\ , \ ``solverType``\ )

- | \ *classFunction*\ :
  | use solverType = exudyn.DynamicSolverType.[...] to define solver (choose between implicit and explicit solvers!)

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-preinitializesolver:

Class function: PreInitializeSolver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`PreInitializeSolver <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L180>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | internal function which initializes dynamic solver; adapt in special cases; this function has some overhead and should not be called during reset() or step()

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-integratestep:

Class function: IntegrateStep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`IntegrateStep <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L187>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | internal function which is called to solve for one step

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-step:

Class function: step
^^^^^^^^^^^^^^^^^^^^
`step <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L205>`__\ (\ ``self``\ , \ ``action``\ )

- | \ *classFunction*\ :
  | openAI gym interface function which is called to compute one step

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-reset:

Class function: reset
^^^^^^^^^^^^^^^^^^^^^
`reset <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L252>`__\ (\ ``self``\ , \ ``*``\ , \ ``seed: Optional[int] = None``\ , \ ``return_info: bool = False``\ , \ ``options: Optional[dict] = None``\ )

- | \ *classFunction*\ :
  | openAI gym function which resets the system

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-render:

Class function: render
^^^^^^^^^^^^^^^^^^^^^^
`render <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L293>`__\ (\ ``self``\ , \ ``mode = "human"``\ )

- | \ *classFunction*\ :
  | openAI gym interface function to render the system

----

.. _sec-artificialintelligence-openaigyminterfaceenv(env)-close:

Class function: close
^^^^^^^^^^^^^^^^^^^^^
`close <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/artificialIntelligence.py\#L299>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | openAI gym interface function to close system after learning or simulation

