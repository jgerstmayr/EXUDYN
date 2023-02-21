

.. _sec-item-objectconnectorcoordinatevector:

ObjectConnectorCoordinateVector
===============================

A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies. The marker uses the objects LTG-lists to build the according coordinate mappings.

\ **Additional information for ObjectConnectorCoordinateVector**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Coordinate``\ 
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
  | If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored
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



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta {\mathbf{q}}`\ 
  | relative scalar displacement of marker coordinates, not including scaling matrices
* | ``Velocity``\ : \ :math:`\Delta {\mathbf{v}}`\ 
  | difference of scalar marker velocity coordinates, not including scaling matrices
* | ``ConstraintEquation``\ : \ :math:`{\mathbf{c}}`\ 
  | (residuum of) constraint equations
* | ``Force``\ : \ :math:`\tlambda`\ 
  | constraint force vector (vector of Lagrange multipliers), resulting from action of constraint equations




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


