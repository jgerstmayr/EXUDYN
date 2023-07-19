

.. _sec-item-objectgenericode1:

ObjectGenericODE1
=================

A system of \ :math:`n`\  \ :ref:`ODE1 <ODE1>`\ , having a system matrix, a rhs vector, but mostly it will use a user function to describe special \ :ref:`ODE1 <ODE1>`\  systems. It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions \ :math:`n`\  or \ :math:`(n \times n)`\ , or they must be empty \ :math:`(0 \times 0)`\ , using [] in Python.

\ **Additional information for ObjectGenericODE1**\ :

* | This \ ``Object``\  has/provides the following types = \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type: read detailed information of item


The item \ **ObjectGenericODE1**\  with type = 'GenericODE1' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [\ :math:`\mathbf{n}_n = [n_0,\,\ldots,\,n_n]\tp`\ , type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list)
* | **systemMatrix** [\ :math:`{\mathbf{A}} \in \Rcal^{n \times n}`\ , type = NumpyMatrix, default = Matrix[]]:
  | system matrix (state space matrix) of first order ODE
* | **rhsVector** [\ :math:`{\mathbf{f}} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | a constant rhs vector (e.g., for constant input)
* | **rhsUserFunction** [\ :math:`{\mathbf{f}}_{user} \in \Rcal^{n}`\ , type = PyFunctionVectorMbsScalarIndexVector, default =  0]:
  | A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **tempCoordinates** [\ :math:`{\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [\ :math:`\dot {\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates
* | **visualization** [type = VObjectGenericODE1]:
  | parameters for visualization of item



The item VObjectGenericODE1 has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-objectgenericode1:

DESCRIPTION of ObjectGenericODE1
--------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : 
  | all \ :ref:`ODE1 <ODE1>`\  coordinates
* | ``Coordinates\_t``\ : 
  | all \ :ref:`ODE1 <ODE1>`\  velocity coordinates



Equations of motion
-------------------

An object with node numbers \ :math:`[n_0,\,\ldots,\,n_n]`\  and according numbers of nodal coordinates \ :math:`[n_{c_0},\,\ldots,\,n_{c_n}]`\ , the total number of equations (=coordinates) of the object is

.. math::

   n = \sum_{i} n_{c_i},


which is used throughout the description of this object.

Equations of motion
-------------------


.. math::
   :label: eq-objectgenericode1-eom

   \dot {\mathbf{q}} = {\mathbf{f}} + {\mathbf{f}}_{user}(mbs, t, i_N, {\mathbf{q}})


Note that the user function \ :math:`{\mathbf{f}}_{user}(mbs, t, i_N, {\mathbf{q}})`\  may be empty (=0), and that \ ``iN``\  represents the itemNumber (=objectNumber). 

CoordinateLoads are added for the respective \ :ref:`ODE1 <ODE1>`\  coordinate on the RHS of the latter equation.

--------

\ **Userfunction**\ : ``rhsUserFunction(mbs, t, itemNumber, q)`` 


A user function, which computes a RHS vector depending on current time and states of the object. 
Can be used to create any kind of first order system, especially state space equations (inputs are added via CoordinateLoads to every node).
Note that itemNumber represents the index of the ObjectGenericODE1 object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``q``\ 
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object coordinates (composed from \ :ref:`ODE1 <ODE1>`\  nodal coordinates) in current configuration, without reference values
   * - | \returnValue
     - | Vector \ :math:`\in \Rcal^{n}`\ 
     - | returns force vector for object


--------

\ **User function example**\ :



.. code-block:: python

    A = numpy.diag([200,100])
    #simple linear user function returning A*q + const
    def UFrhs(mbs, t, itemNumber, q): 
        return np.dot(A, q) + np.array([0,2])
        
    nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                        initialCoordinates=[1,0], numberOfODE1Coordinates=2))

    #now add object instead of object in mini-example:
    oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1], 
                       rhsUserFunction=UFrhs))
                                 





.. _miniexample-objectgenericode1:

MINI EXAMPLE for ObjectGenericODE1
----------------------------------


.. code-block:: python
   :linenos:

   #set up a 2-DOF system
   nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                       initialCoordinates=[1,0],
                                       numberOfODE1Coordinates=2))
   
   #build system matrix and force vector
   #undamped mechanical system with m=1, K=100, f=1
   A = np.array([[0,1],
                 [-100,0]])
   b = np.array([0,1])
   
   oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1], 
                                                  systemMatrix=A, 
                                                  rhsVector=b))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   sims=exu.SimulationSettings()
   solverType = exu.DynamicSolverType.RK44
   mbs.SolveDynamic(solverType=solverType, simulationSettings=sims)
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nODE1, exu.OutputVariableType.Coordinates)[0]

Relevant Examples and TestModels with weblink:

    \ `HydraulicsUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicsUserFunction.py>`_\  (Examples/), \ `lugreFrictionODE1.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionODE1.py>`_\  (Examples/), \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Examples/), \ `solverExplicitODE1ODE2test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/solverExplicitODE1ODE2test.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


