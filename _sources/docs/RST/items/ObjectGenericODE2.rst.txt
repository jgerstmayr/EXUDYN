

.. _sec-item-objectgenericode2:

ObjectGenericODE2
=================

A system of \ :math:`n`\  second order ordinary differential equations (\ :ref:`ODE2 <ODE2>`\ ), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces. It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE that all matrices, vectors, etc. must have the same dimensions \ :math:`n`\  or \ :math:`(n \times n)`\ , or they must be empty \ :math:`(0 \times 0)`\ , except for the mass matrix which always needs to have dimensions \ :math:`(n \times n)`\ .

\ **Additional information for ObjectGenericODE2**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ , \ ``SuperElement``\ 
* | Requested \ ``Node``\  type: read detailed information of item


The item \ **ObjectGenericODE2**\  with type = 'GenericODE2' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [\ :math:`\mathbf{n}_n = [n_0,\,\ldots,\,n_n]\tp`\ , type = ArrayNodeIndex, default = []]:
  | node numbers which provide the coordinates for the object (consecutively as provided in this list)
* | **massMatrix** [\ :math:`{\mathbf{M}} \in \Rcal^{n \times n}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | mass matrix of object as MatrixContainer (or numpy array / list of lists)
* | **stiffnessMatrix** [\ :math:`{\mathbf{K}} \in \Rcal^{n \times n}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | stiffness matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with dampingMatrix and jacobianUserFunction
* | **dampingMatrix** [\ :math:`{\mathbf{D}} \in \Rcal^{n \times n}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | damping matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with stiffnessMatrix and jacobianUserFunction
* | **forceVector** [\ :math:`{\mathbf{f}} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | generalized force vector added to RHS
* | **forceUserFunction** [\ :math:`{\mathbf{f}}_{user} \in \Rcal^{n}`\ , type = PyFunctionVectorMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the generalized user force vector for the \ :ref:`ODE2 <ODE2>`\  equations; see description below
* | **massMatrixUserFunction** [\ :math:`{\mathbf{M}}_{user} \in \Rcal^{n\times n}`\ , type = PyFunctionMatrixContainerMbsScalarIndex2Vector, default =  0]:
  | A Python user function which computes the mass matrix instead of the constant mass matrix given in \ :math:`{\mathbf{M}}`\ ; return numpy array or MatrixContainer; see description below
* | **jacobianUserFunction** [\ :math:`{\mathbf{J}}_{user} \in \Rcal^{n\times n}`\ , type = PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar, default =  0]:
  | A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t.\ the coordinates (times \ :math:`f_{ODE2}`\ ) and w.r.t.\ the velocities (times \ :math:`f_{ODE2_t}`\ ). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below
* | **coordinateIndexPerNode** [type = ArrayIndex, default = []]:
  | this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
* | **tempCoordinates** [\ :math:`{\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing coordinates
* | **tempCoordinates_t** [\ :math:`\dot {\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing velocity coordinates
* | **tempCoordinates_tt** [\ :math:`\ddot {\mathbf{c}}_{temp} \in \Rcal^{n}`\ , type = NumpyVector, default = []]:
  | temporary vector containing acceleration coordinates
* | **visualization** [type = VObjectGenericODE2]:
  | parameters for visualization of item



The item VObjectGenericODE2 has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used
* | **triangleMesh** [type = NumpyMatrixI, default = MatrixI[]]:
  | a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!
* | **showNodes** [type = Bool, default = False]:
  | set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization


----------

.. _description-objectgenericode2:

DESCRIPTION of ObjectGenericODE2
--------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``CoordinatesTotal``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  displacement plus reference coordinates of object
* | ``Coordinates``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  (displacement) coordinates
* | ``Coordinates\_t``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  velocity coordinates
* | ``Coordinates\_tt``\ : 
  | all \ :ref:`ODE2 <ODE2>`\  acceleration coordinates
* | ``Force``\ : 
  | generalized forces for all coordinates (residual of all forces except mass*accleration; corresponds to ComputeODE2LHS)



Additional output variables for superelement node access
--------------------------------------------------------

Functions like \ ``GetObjectOutputSuperElement(...)``\ , see Section :ref:`sec-mainsystem-object`\ , 
or \ ``SensorSuperElement``\ , see Section :ref:`sec-mainsystem-sensor`\ , directly access special output variables
(\ ``OutputVariableType``\ ) of the mesh nodes of the superelement.
Additionally, the contour drawing of the object can make use the \ ``OutputVariableType``\  of the meshnodes.

For this object, all nodes of \ ``ObjectGenericODE2``\  map their \ ``OutputVariableType``\  to the meshnode \ :math:`\ra`\ 
see at the according node for the list of \ ``OutputVariableType``\ .

Equations of motion
-------------------

An object with node numbers \ :math:`[n_0,\,\ldots,\,n_n]`\  and according numbers of nodal coordinates \ :math:`[n_{c_0},\,\ldots,\,n_{c_n}]`\ , the total number of equations (=coordinates) of the object is

.. math::

   n = \sum_{i} n_{c_i},


which is used throughout the description of this object.

Equations of motion
-------------------

The equations of motion read,

.. math::
   :label: eq-objectgenericode2-eom

   {\mathbf{M}} \ddot {\mathbf{q}} + {\mathbf{D}} \dot {\mathbf{q}} + {\mathbf{K}} {\mathbf{q}} = {\mathbf{f}} + {\mathbf{f}}_{user}(mbs, t, i_N,{\mathbf{q}},\dot {\mathbf{q}})


Note that the user function \ :math:`{\mathbf{f}}_{user}(mbs, t, i_N,{\mathbf{q}},\dot {\mathbf{q}})`\  may be empty (=0), and \ ``iN``\  represents the itemNumber (=objectNumber). 

In case that a user mass matrix is specified, Eq. :eq:`eq-objectgenericode2-eom`\  is replaced with

.. math::

   {\mathbf{M}}_{user}(mbs, t, i_N, {\mathbf{q}},\dot {\mathbf{q}}) \ddot {\mathbf{q}} + {\mathbf{D}} \dot {\mathbf{q}} + {\mathbf{K}} {\mathbf{q}} = {\mathbf{f}} + {\mathbf{f}}_{user}(mbs, t, i_N, {\mathbf{q}},\dot {\mathbf{q}})



The (internal) Jacobian \ :math:`{\mathbf{J}}`\  of Eq. :eq:`eq-objectgenericode2-eom`\  (assuming \ :math:`{\mathbf{f}}`\  to be constant!) reads

.. math::

   {\mathbf{J}} = f_{ODE2}   \left({\mathbf{K}} - \frac{\partial {\mathbf{f}}_{user}(mbs, t, i_N,{\mathbf{q}},\dot {\mathbf{q}})}{\partial {\mathbf{q}}}\right) + f_{ODE2_t} \left({\mathbf{D}} - \frac{\partial {\mathbf{f}}_{user}(mbs, t, i_N,{\mathbf{q}},\dot {\mathbf{q}})}{\partial \dot {\mathbf{q}}} \right) +


Chosing \ :math:`f_{ODE2} = 1`\  and \ :math:`f_{ODE2_t}=0`\  would immediately give the jacobian of position quantities.

If no \ ``jacobianUserFunction``\  is specified, the jacobian is -- as with many objects in Exudyn -- computed 
by means of numerical differentiation.
In case that a \ ``jacobianUserFunction``\  is specified, it must represent the jacobian of the \ :ref:`LHS <LHS>`\  of Eq. :eq:`eq-objectgenericode2-eom`\  
without \ :math:`{\mathbf{K}}`\  and \ :math:`{\mathbf{D}}`\  (these matrices are added internally),

.. math::
   :label: eq-objectgenericode2-jac

   {\mathbf{J}}_{user}(mbs, t, i_N, {\mathbf{q}}, \dot {\mathbf{q}}, f_{ODE2}, f_{ODE2_t}) = -f_{ODE2}   \left(\frac{\partial {\mathbf{f}}_{user}(mbs, t, i_N,{\mathbf{q}},\dot {\mathbf{q}})}{\partial {\mathbf{q}}} \right) - f_{ODE2_t} \left(\frac{\partial {\mathbf{f}}_{user}(mbs, t, i_N,{\mathbf{q}},\dot {\mathbf{q}})}{\partial \dot {\mathbf{q}}} \right)


For clarification also see the \ **example**\  in \ ``TestModels/linearFEMgenericODE2.py``\ .

CoordinateLoads are added for the respective \ :ref:`ODE2 <ODE2>`\  coordinate on the RHS of the latter equation.

--------

\ **Userfunction**\ : ``forceUserFunction(mbs, t, itemNumber, q, q_t)`` 


A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
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
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``q``\ 
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object velocity coordinates (time derivative of \ ``q``\ ) in current configuration
   * - | \returnValue
     - | Vector \ :math:`\in \Rcal^{n}`\ 
     - | returns force vector for object


--------

\ **Userfunction**\ : ``massMatrixUserFunction(mbs, t, itemNumber, q, q_t)`` 


A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.

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
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object velocity coordinates (time derivative of \ ``q``\ ) in current configuration
   * - | \returnValue
     - | MatrixContainer \ :math:`\in \Rcal^{n \times n}`\ 
     - | returns mass matrix for object, as exu.MatrixContainer, 
                          numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations.



--------

\ **Userfunction**\ : ``jacobianUserFunction(mbs, t, itemNumber, q, q_t, fODE2, fODE2_t)`` 


A user function, which computes the jacobian of the \ :ref:`LHS <LHS>`\  of the equations of motion, depending on current time, states of object and two
factors which are used to distinguish between position level and velocity level derivatives. 
Can be used to create any kind of mechanical system by using the object states.

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
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values
   * - | \ ``q_t``\ 
     - | Vector \ :math:`\in \Rcal^n`\ 
     - | object velocity coordinates (time derivative of \ ``q``\ ) in current configuration
   * - | \ ``fODE2``\ 
     - | Real
     - | factor to be multiplied with the position level jacobian, see Eq. :eq:`eq-objectgenericode2-jac`\ 
   * - | \ ``fODE2_t``\ 
     - | Real
     - | factor to be multiplied with the velocity level jacobian, see Eq. :eq:`eq-objectgenericode2-jac`\ 
   * - | \returnValue
     - | MatrixContainer \ :math:`\in \Rcal^{n \times n}`\ 
     - | returns special jacobian for object, as exu.MatrixContainer, 
                          numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations;
                          NOTE that the format of returnValue must AGREE with (dense/sparse triplet) format of stiffnessMatrix and dampingMatrix;
                          sparse triplets MAY NOT contain zero values!



--------

\ **Userfunction**\ : ``graphicsDataUserFunction(mbs, itemNumber)`` 


A user function, which is called by the visualization thread in order to draw user-defined objects.
The function can be used to generate any \ ``BodyGraphicsData``\ , see Section  :ref:`sec-graphicsdata`\ .
Use \ ``exudyn.graphics``\  functions, see Section  :ref:`sec-module-graphics`\ , to create more complicated objects. 
Note that \ ``graphicsDataUserFunction``\  needs to copy lots of data and is therefore
inefficient and only designed to enable simpler tests, but not large scale problems.

For an example for \ ``graphicsDataUserFunction``\  see ObjectGround, Section :ref:`sec-item-objectground`\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides reference to mbs, which can be used in user function to access all data of the object
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access
   * - | \returnValue
     - | BodyGraphicsData
     - | list of \ ``GraphicsData``\  dictionaries, see Section  :ref:`sec-graphicsdata`\ 


--------

\ **User function example**\ :



.. code-block:: python

    #user function, using variables M, K, ... from mini example, replacing ObjectGenericODE2(...)
    KD = numpy.diag([200,100])
    #nonlinear force example; this force is added to right-hand-side ==> negative sign!
    def UFforce(mbs, t, itemNumber, q, q_t): 
        return -np.dot(KD, q_t*q) #add nonlinear term for q_t and q, q_t*q gives vector
    
    #non-constant mass matrix:
    def UFmass(mbs, t, itemNumber, q, q_t): 
        return (q[0]+1)*M #uses mass matrix from mini example
    
    #non-constant mass matrix:
    def UFgraphics(mbs, itemNumber):
        t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization) #get time if needed
        p = mbs.GetObjectOutputSuperElement(objectNumber=itemNumber, variableType = exu.OutputVariableType.Position,
                                            meshNodeNumber = 0, #get first node's position 
                                            configuration = exu.ConfigurationType.Visualization)
        graphics1=graphics.Sphere(point=p,radius=0.1, color=graphics.color.red)
            graphics2 = {'type':'Line', 'data': list(p)+[0,0,0], 'color':graphics.color.blue}
        return [graphics1, graphics2] 

    #now add object instead of object in mini-example:
    oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nMass0,nMass1], 
                       massMatrix=M, stiffnessMatrix=K, dampingMatrix=D,
                       forceUserFunction=UFforce, massMatrixUserFunction=UFmass,
                       visualization=VObjectGenericODE2(graphicsDataUserFunction=UFgraphics)))





.. _miniexample-objectgenericode2:

MINI EXAMPLE for ObjectGenericODE2
----------------------------------


.. code-block:: python
   :linenos:

   #set up a mechanical system with two nodes; it has the structure: |~~M0~~M1
   nMass0 = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
   nMass1 = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
   
   mass = 0.5 * np.eye(3)      #mass of nodes
   stif = 5000 * np.eye(3)     #stiffness of nodes
   damp = 50 * np.eye(3)      #damping of nodes
   Z = 0. * np.eye(3)          #matrix with zeros
   #build mass, stiffness and damping matrices (:
   M = np.block([[mass,         0.*np.eye(3)],
                 [0.*np.eye(3), mass        ] ])
   K = np.block([[2*stif, -stif],
                 [ -stif,  stif] ])
   D = np.block([[2*damp, -damp],
                 [ -damp,  damp] ])
   
   oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nMass0,nMass1], 
                                                  massMatrix=M, 
                                                  stiffnessMatrix=K,
                                                  dampingMatrix=D))
   
   mNode1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass1))
   mbs.AddLoad(Force(markerNumber = mNode1, loadVector = [10, 0, 0])) #static solution=10*(1/5000+1/5000)=0.0004
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   mbs.SolveDynamic(solverType = exudyn.DynamicSolverType.TrapezoidalIndex2)
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.Position)[0]

Relevant Examples and TestModels with weblink:

    \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Examples/), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Examples/), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Examples/), \ `simulateInteractively.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simulateInteractively.py>`_\  (Examples/), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TestModels/), \ `coordinateVectorConstraintGenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraintGenericODE2.py>`_\  (TestModels/), \ `genericODE2test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericODE2test.py>`_\  (TestModels/), \ `linearFEMgenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/linearFEMgenericODE2.py>`_\  (TestModels/), \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TestModels/), \ `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_\  (TestModels/), \ `rigidBodyAsUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyAsUserFunctionTest.py>`_\  (TestModels/), \ `solverExplicitODE1ODE2test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/solverExplicitODE1ODE2test.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


