

.. _sec-item-objectconnectorcoordinatevector:

ObjectConnectorCoordinateVector
===============================

A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies. The marker uses the objects \ :ref:`LTG <LTG>`\ -lists to build the according coordinate mappings.

\ **Additional information for ObjectConnectorCoordinateVector**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Coordinate``\ 
* | \ **Short name**\  for Python = \ ``CoordinateVectorConstraint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCoordinateVectorConstraint``\ 


The item \ **ObjectConnectorCoordinateVector**\  with type = 'ConnectorCoordinateVector' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **scalingMarker0** [\ :math:`{\mathbf{X}}_{m0} \in \Rcal^{n_{ae} \times n_{q_{m0}}}`\ , type = NumpyMatrix, default = Matrix[]]:
  | linear scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format
* | **scalingMarker1** [\ :math:`{\mathbf{X}}_{m1} \in \Rcal^{n_{ae} \times n_{q_{m1}}}`\ , type = NumpyMatrix, default = Matrix[]]:
  | linear scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format
* | **quadraticTermMarker0** [\ :math:`{\mathbf{Y}}_{m0} \in \Rcal^{n_{ae} \times n_{q_{m0}}}`\ , type = NumpyMatrix, default = Matrix[]]:
  | quadratic scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format
* | **quadraticTermMarker1** [\ :math:`{\mathbf{Y}}_{m0} \in \Rcal^{n_{ae} \times n_{q_{m0}}}`\ , type = NumpyMatrix, default = Matrix[]]:
  | quadratic scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format
* | **offset** [\ :math:`{\mathbf{v}}_\mathrm{off} \in \Rcal^{n_{ae}}`\ , type = NumpyVector, default = []]:
  | offset added to constraint equation; only active, if no userFunction is defined
* | **velocityLevel** [type = Bool, default = False]:
  | If true: connector constrains velocities (only works for \ :ref:`ODE2 <ODE2>`\  coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored
* | **constraintUserFunction** [\ :math:`{\mathbf{c}}_{user} \in \Rcal^{n_{ae}}`\ , type = PyFunctionVectorMbsScalarIndex2VectorBool, default =  0]:
  | A Python user function which computes the constraint equations; to define the number of algebraic equations, set scalingMarker0 as a numpy.zeros((nAE,1)) array with nAE being the number algebraic equations; see description below
* | **jacobianUserFunction** [\ :math:`{\mathbf{J}}_{user} \in \Rcal^{(n_{q_{m0}}+n_{q_{m1}}) \times n_{ae}}`\ , type = PyFunctionMatrixContainerMbsScalarIndex2VectorBool, default =  0]:
  | A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t.\ the coordinates (times \ :math:`f_{ODE2}`\ ) and w.r.t.\ the velocities (times \ :math:`f_{ODE2_t}`\ ). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorCoordinateVector]:
  | parameters for visualization of item



The item VObjectConnectorCoordinateVector has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorcoordinatevector:

DESCRIPTION of ObjectConnectorCoordinateVector
----------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta {\mathbf{q}}`\ 
  | relative scalar displacement of marker coordinates, not including scaling matrices
* | ``Velocity``\ : \ :math:`\Delta {\mathbf{v}}`\ 
  | difference of scalar marker velocity coordinates, not including scaling matrices
* | ``ConstraintEquation``\ : \ :math:`{\mathbf{c}}`\ 
  | (residuum of) constraint equations
* | ``Force``\ : \ :math:`\tlambda`\ 
  | constraint force vector (vector of Lagrange multipliers), resulting from action of constraint equations



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 coordinate vector
     - | \ :math:`{\mathbf{q}}_{m0} \in \Rcal^{n_{q_{m0}}}`\ 
     - | coordinate vector provided by marker \ :math:`m0`\ ; depending on the marker, the coordinates may or may not include reference coordinates
   * - | marker m1 coordinate vector
     - | \ :math:`{\mathbf{q}}_{m1} \in \Rcal^{n_{q_{m1}}}`\ 
     - | coordinate vector provided by marker \ :math:`m1`\ ; depending on the marker, the coordinates may or may not include reference coordinates
   * - | marker m0 velocity coordinate vector
     - | \ :math:`\dot {\mathbf{q}}_{m0} \in \Rcal^{n_{q_{m0}}}`\ 
     - | velocity coordinate vector provided by marker \ :math:`m0`\ 
   * - | marker m1 velocity coordinate vector
     - | \ :math:`\dot {\mathbf{q}}_{m1} \in \Rcal^{n_{q_{m1}}}`\ 
     - | velocity coordinate vector provided by marker \ :math:`m1`\ 
   * - | number of algebraic equations
     - | \ :math:`n_{ae}`\ 
     - | number of algebraic equations must be same as number of rows in \ :math:`{\mathbf{X}}_{m0}`\  and \ :math:`{\mathbf{X}}_{m1}`\ 
   * - | difference of coordinates
     - | \ :math:`\Delta {\mathbf{q}} = {\mathbf{q}}_{m1} - {\mathbf{q}}_{m0}`\ 
     - | Displacement between marker m0 to marker m1 coordinates
   * - | difference of velocity coordinates
     - | \ :math:`\Delta {\mathbf{v}}= \dot {\mathbf{q}}_{m1} - \dot {\mathbf{q}}_{m0}`\ 
     - | 


Remarks
-------

The number of algebraic equations depends on the maximum number of rows in \ :math:`{\mathbf{X}}_{m0}`\ , \ :math:`{\mathbf{Y}}_{m0}`\ , \ :math:`{\mathbf{X}}_{m1}`\  and \ :math:`{\mathbf{Y}}_{m1}`\ . 
The number of rows of the latter matrices must either be zero or the maximum of these rows.

The number of columns in \ :math:`{\mathbf{X}}_{m0}`\  (or \ :math:`{\mathbf{Y}}_{m0}`\ ) must agree with the length of the coordinate vector
\ :math:`{\mathbf{q}}_{m0}`\  and the number of columns in \ :math:`{\mathbf{X}}_{m1}`\  (or \ :math:`{\mathbf{Y}}_{m1}`\ ) must agree with the length of the coordinate vector
\ :math:`{\mathbf{q}}_{m1}`\ , if these matrices are not empty matrices. 
If one marker \ :math:`k`\  is a ground marker (node/object), the length of \ :math:`{\mathbf{q}}_{m,k}`\  is zero and also the according matrices
\ :math:`{\mathbf{X}}_{m,k}`\ , \ :math:`{\mathbf{Y}}_{m,k}`\   have zero size and will not be considered in the computation of the constraint equations.



Connector constraint equations
------------------------------

If \ ``activeConnector = True``\ , the index 3 algebraic equations

.. math::

   {\mathbf{c}}({\mathbf{q}}_{m0}, {\mathbf{q}}_{m1}) = {\mathbf{X}}_{m1} \cdot {\mathbf{q}}_{m1} + {\mathbf{Y}}_{m1} \cdot {\mathbf{q}}^2_{m1} - {\mathbf{X}}_{m0} \cdot{\mathbf{q}}_{m0} -{\mathbf{Y}}_{m0} \cdot{\mathbf{q}}^2_{m0} - {\mathbf{v}}_\mathrm{off} = 0


Note that the squared coordinates are understood as \ :math:`{\mathbf{q}}^2_{m0} = [q^2_{0,m0}, \; q^2_{1,m0}, \; \ldots]\tp`\ , same for \ :math:`{\mathbf{q}}^2_{m1}`\ .

If the offsetUserFunction \ :math:`\mathrm{UF}`\  is defined, \ :math:`{\mathbf{c}}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   {\mathbf{c}}({\mathbf{q}}_{m0}, {\mathbf{q}}_{m1}) = {\mathbf{X}}_{m1} \cdot {\mathbf{q}}_{m1} + {\mathbf{Y}}_{m1} \cdot {\mathbf{q}}^2_{m1} - {\mathbf{X}}_{m0} \cdot{\mathbf{q}}_{m0} -{\mathbf{Y}}_{m0} \cdot{\mathbf{q}}^2_{m0} -  \mathrm{UF}(mbs, t,{\mathbf{v}}_\mathrm{off}) = 0


The \ ``activeConnector = True``\ , index 2 (velocity level) algebraic equation reads

.. math::

   \dot {\mathbf{c}}(\dot {\mathbf{q}}_{m0}, \dot {\mathbf{q}}_{m1}) = {\mathbf{X}}_{m1} \cdot \dot {\mathbf{q}}_{m1} + {\mathbf{Y}}_{m1} \cdot \dot {\mathbf{q}}^2_{m1} -{\mathbf{X}}_{m0} \cdot \dot {\mathbf{q}}_{m0} - {\mathbf{Y}}_{m0} \cdot \dot {\mathbf{q}}^2_{m0} - {\mathbf{d}}_\mathrm{off} = 0


The vector \ :math:`dv`\  in velocity level equations is zero, except if parameters.velocityLevel = True, then \ :math:`{\mathbf{d}}={\mathbf{v}}_\mathrm{off}`\ .

If velocity level constraints are active and the velocity level \ ``offsetUserFunction_t``\  \ :math:`\mathrm{UF}_t`\  is defined, 
\ :math:`\dot {\mathbf{c}}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   \dot {\mathbf{c}}(\dot {\mathbf{q}}_{m0}, \dot {\mathbf{q}}_{m1}) = {\mathbf{X}}_{m1} \cdot \dot {\mathbf{q}}_{m1} + {\mathbf{Y}}_{m1} \cdot \dot {\mathbf{q}}^2_{m1} -{\mathbf{X}}_{m0} \cdot \dot {\mathbf{q}}_{m0} - {\mathbf{Y}}_{m0} \cdot \dot {\mathbf{q}}^2_{m0} - \mathrm{UF}_t(mbs, t,{\mathbf{v}}_\mathrm{off}) = 0


Note that the index 2 equations are used, if the solver uses index 2 formulation OR if the flag parameters.velocityLevel = True (or both).
The user functions include dependency on time \ :math:`t`\ , but this time dependency is not respected in the computation of initial accelerations. Therefore,
it is recommended that \ :math:`\mathrm{UF}`\  and \ :math:`\mathrm{UF}_t`\  does not include initial accelerations.

If \ ``activeConnector = False``\ , the (index 1) algebraic equation reads for ALL cases:

.. math::

   {\mathbf{c}}(\tlambda) = \tlambda = 0





--------

\ **Userfunction**\ : ``constraintUserFunction(mbs, t, itemNumber, q, q_t, velocityLevel)`` 


A user function, which computes algebraic equations for the connector based on the marker coordinates stored in \ ``q``\  and \ ``q_t``\ .
Depending on \ ``velocityLevel``\ , the user function needs to compute either the position-level (\ ``velocityLevel=False``\ ) or
the velocity level (\ ``velocityLevel=True``\ ) constraint equations.
Note that for Index 2 solvers, the \ ``constraintUserFunction``\  may be called with \ ``velocityLevel=True``\  but \ ``jacobianUserFunction``\  
is called with \ ``velocityLevel=False``\ .
To define the number of algebraic equations, set \ ``scalingMarker0``\  as a \ ``numpy.zeros((nAE,1))``\  array with \ ``nAE``\  being the number algebraic equations. 
The returned vector of \ ``constraintUserFunction``\  must have size \ ``nAE``\ .

Note that itemNumber represents the index of the ObjectGenericODE2 object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs to
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``q``\ 
     - | Vector \ :math:`\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}`\ 
     - | connector coordinates, subsequently for marker \ :math:`m0`\  and marker \ :math:`m1`\ , in current configuration
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}`\ 
     - | connector velocity coordinates in current configuration
   * - | \ ``velocityLevel``\ 
     - | Bool
     - | velocityLevel as currently stored in connector
   * - | \returnValue
     - | Vector \ :math:`\in \Rcal^{n_{ae}}`\ 
     - | returns vector (numpy array or list) of evaluated constraint equations for connector



--------

\ **Userfunction**\ : ``jacobianUserFunction(mbs, t, itemNumber, q, q_t, velocityLevel)`` 


A user function, which computes the jacobian of the algebraic equations w.r.t. the ODE2 coordiantes (ODE2_t velocity coordinates if \ ``velocityLevel=True``\ ).
The jacobian needs to exactly represent the derivative of the constraintUserFunction.
The returned matrix of \ ``jacobianUserFunction``\  must have \ ``nAE``\  rows and \ ``len(q)``\  columns.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs to
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``q``\ 
     - | Vector \ :math:`\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}`\ 
     - | connector coordinates, subsequently for marker \ :math:`m0`\  and marker \ :math:`m1`\ , in current configuration
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}`\ 
     - | connector velocity coordinates in current configuration
   * - | \ ``velocityLevel``\ 
     - | Bool
     - | velocityLevel as currently stored in connector
   * - | \returnValue
     - | MatrixContainer \ :math:`\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})\times n_{ae}}`\ 
     - | returns special jacobian for connector, as exu.MatrixContainer, 
                          numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations;
                          sparse triplets MAY NOT contain zero values!



Relevant Examples and TestModels with weblink:

    \ `coordinateVectorConstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraint.py>`_\  (TestModels/), \ `coordinateVectorConstraintGenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraintGenericODE2.py>`_\  (TestModels/), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


