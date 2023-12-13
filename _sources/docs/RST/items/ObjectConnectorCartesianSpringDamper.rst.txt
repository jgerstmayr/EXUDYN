

.. _sec-item-objectconnectorcartesianspringdamper:

ObjectConnectorCartesianSpringDamper
====================================

An 3D spring-damper element, providing springs and dampers in three (global) directions (x,y,z); the connector can be attached to position-based markers.

\ **Additional information for ObjectConnectorCartesianSpringDamper**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``CartesianSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCartesianSpringDamper``\ 


The item \ **ObjectConnectorCartesianSpringDamper**\  with type = 'ConnectorCartesianSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **stiffness** [\ :math:`{\mathbf{k}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction
* | **damping** [\ :math:`{\mathbf{d}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction
* | **offset** [\ :math:`{\mathbf{v}}_{\mathrm{off}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | offset between two springs
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^3`\ , type = PyFunctionVector3DmbsScalarIndexScalar4Vector3D, default =  0]:
  | A Python function which computes the 3D force vector between the two marker points, if activeConnector=True; see description below
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorCartesianSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorCartesianSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorcartesianspringdamper:

DESCRIPTION of ObjectConnectorCartesianSpringDamper
---------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{p}}} = \LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
  | relative displacement in global coordinates
* | ``Distance``\ : \ :math:`L=|\Delta\! \LU{0}{{\mathbf{p}}}|`\ 
  | scalar distance between both marker points
* | ``Velocity``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{v}}} = \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
  | relative translational velocity in global coordinates
* | ``Force``\ : \ :math:`{\mathbf{f}}_{SD}`\ 
  | joint force in global coordinates, see equations



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position which is provided by marker m0
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | 
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | 


Connector forces
----------------

Connector forces are based on relative displacements and relative veolocities in global coordinates.
Relative displacement between marker m0 to marker m1 positions is given by

.. math::
   :label: eq-objectcartesianspringdamper-deltapos

   \Delta\! \LU{0}{{\mathbf{p}}}= \LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0} ,


and relative velocity reads

.. math::

   \Delta\! \LU{0}{{\mathbf{v}}}= \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0} .


If \ ``activeConnector = True``\ , the spring force vector is computed as

.. math::

   \LU{0}{{\mathbf{f}}_{SD}} = \diag({\mathbf{k}})\cdot(\Delta\! \LU{0}{{\mathbf{p}}}-\LU{0}{{\mathbf{v}}_{\mathrm{off}}}) + \diag({\mathbf{d}}) \cdot \Delta\! \LU{0}{{\mathbf{v}}} .


If the springForceUserFunction \ :math:`\mathrm{UF}`\  is defined, \ :math:`{\mathbf{f}}_{SD}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   \LU{0}{{\mathbf{f}}_{SD}} = \mathrm{UF}(mbs, t, i_N, \Delta\! \LU{0}{{\mathbf{p}}}, \Delta\! \LU{0}{{\mathbf{v}}}, {\mathbf{k}}, {\mathbf{d}}, {\mathbf{v}}_{\mathrm{off}}) ,


and \ ``iN``\  represents the itemNumber (=objectNumber).
If \ ``activeConnector = False``\ , \ :math:`{\mathbf{f}}_{SD}`\  is set to zero.

The force \ :math:`{\mathbf{f}}_{SD}`\  acts via the markers' position jacobians \ :math:`{\mathbf{J}}_{pos,m0}`\  and \ :math:`{\mathbf{J}}_{pos,m1}`\ .
The generalized forces added to the \ :ref:`LHS <LHS>`\  equations read for marker \ :math:`m0`\ ,

.. math::

   {\mathbf{f}}_{LHS,m0} = -\LU{0}{{\mathbf{J}}_{pos,m0}\tp} \LU{0}{{\mathbf{f}}_{SD}} ,


and for marker \ :math:`m1`\ ,

.. math::

   {\mathbf{f}}_{LHS,m1} =  \LU{0}{{\mathbf{J}}_{pos,m1}\tp} \LU{0}{{\mathbf{f}}_{SD}} .


The \ :ref:`LHS <LHS>`\  equation parts are added accordingly using the \ :ref:`LTG <LTG>`\  mapping.
Note that the different signs result from the signs in Eq. :eq:`eq-objectcartesianspringdamper-deltapos`\ .

The connector also provides an analytic jacobian, which is used if \ ``newton.numericalDifferentiation.forODE2 = False``\  
and if there is no springForceUserFunction (otherwise numerical differentiation is used).

The anayltic jacobian for the coupled equation parts \ :math:`{\mathbf{f}}_{LHS,m0}`\  and \ :math:`{\mathbf{f}}_{LHS,m1}`\  is based on the local jacobians

.. math::

   {\mathbf{J}}_{loc0} &=& f_{ODE2}\frac{\partial \LU{0}{{\mathbf{f}}_{SD}}}{\partial \LU{0}{{\mathbf{p}}}_{m0}} + f_{ODE2_t}\frac{\partial \LU{0}{{\mathbf{f}}_{SD}}}{\partial \LU{0}{{\mathbf{v}}}_{m0}} = -f_{ODE2} \cdot \diag({\mathbf{k}}) - f_{ODE2_t} \cdot \diag({\mathbf{d}}) , \nonumber \\
   {\mathbf{J}}_{loc1} &=& f_{ODE2}\frac{\partial \LU{0}{{\mathbf{f}}_{SD}}}{\partial \LU{0}{{\mathbf{p}}}_{m1}} + f_{ODE2_t}\frac{\partial \LU{0}{{\mathbf{f}}_{SD}}}{\partial \LU{0}{{\mathbf{v}}}_{m1}} =  f_{ODE2} \cdot \diag({\mathbf{k}}) + f_{ODE2_t} \cdot \diag({\mathbf{d}}) .


Here, \ :math:`f_{ODE2}`\  is the factor for the position derivative and \ :math:`f_{ODE2_t}`\  is the factor for the velocity derivative, 
which allows a computation of the computation for both the position as well as the velocity part at the same time.

The complete jacobian for the \ :ref:`LHS <LHS>`\  equations then reads,

.. math::

   {\mathbf{J}}_{CSD}&=&\mp{\displaystyle \frac{\partial {\mathbf{f}}_{LHS,m0}}{\partial  {\mathbf{q}}_{m0}}} {\displaystyle \frac{\partial {\mathbf{f}}_{LHS,m0}}{\partial  {\mathbf{q}}_{m1}}} {\displaystyle \frac{\partial {\mathbf{f}}_{LHS,m1}}{\partial  {\mathbf{q}}_{m0}}} {\displaystyle \frac{\partial {\mathbf{f}}_{LHS,m1}}{\partial  {\mathbf{q}}_{m1}}} + {\mathbf{J}}_{CSD'} \nonumber \\
   &=& \mp{-\LU{0}{{\mathbf{J}}_{pos,m0}\tp} {\mathbf{J}}_{loc0} {\mathbf{J}}_{pos,m0}} {-\LU{0}{{\mathbf{J}}_{pos,m0}\tp} {\mathbf{J}}_{loc1} {\mathbf{J}}_{pos,m1}} {\LU{0}{{\mathbf{J}}_{pos,m1}\tp} {\mathbf{J}}_{loc0} {\mathbf{J}}_{pos,m0}} {\LU{0}{{\mathbf{J}}_{pos,m1}\tp} {\mathbf{J}}_{loc1} {\mathbf{J}}_{pos,m1}} + {\mathbf{J}}_{CSD'} \nonumber \\
   &=& \mp{\LU{0}{{\mathbf{J}}_{pos,m0}\tp} {\mathbf{J}}_{loc1} {\mathbf{J}}_{pos,m0}} {-\LU{0}{{\mathbf{J}}_{pos,m0}\tp} {\mathbf{J}}_{loc1} {\mathbf{J}}_{pos,m1}} {-\LU{0}{{\mathbf{J}}_{pos,m1}\tp} {\mathbf{J}}_{loc1} {\mathbf{J}}_{pos,m0}} {\LU{0}{{\mathbf{J}}_{pos,m1}\tp} {\mathbf{J}}_{loc1} {\mathbf{J}}_{pos,m1}} + {\mathbf{J}}_{CSD'}


Here, \ :math:`{\mathbf{q}}_{m0}`\  are the coordinates associated with marker \ :math:`m0`\  and \ :math:`{\mathbf{q}}_{m1}`\  of marker \ :math:`m1`\ .

The second term \ :math:`{\mathbf{J}}_{CSD'}`\  is only non-zero if \ :math:`\frac{\partial \LU{0}{{\mathbf{J}}_{pos,i}\tp}}{\partial {\mathbf{q}}_{i}}`\  is non-zero, using \ :math:`i \in \{m0, \, m1\}`\ .
As the latter terms would require to compute a 3-dimensional array, the second jacobian term is computed as 

.. math::
   :label: eq-objectcartesianspringdamper-jacderiv

   {\mathbf{J}}_{CSD'} = \mp{-f_{ODE2}\frac{\partial \left(\LU{0}{{\mathbf{J}}_{pos,m0}\tp} {\mathbf{f}}' \right)}{\partial {\mathbf{q}}_{m0}}}{\Null}{\Null} { f_{ODE2}\frac{\partial \left(\LU{0}{{\mathbf{J}}_{pos,m1}\tp} {\mathbf{f}}' \right)}{\partial {\mathbf{q}}_{m1}}}


in which we set \ :math:`{\mathbf{f}}' = \LU{0}{{\mathbf{f}}_{SD}}`\ , but the derivatives in Eq. :eq:`eq-objectcartesianspringdamper-jacderiv`\  are evaluated by setting \ :math:`{\mathbf{f}}' = const`\ .


--------

\ **Userfunction**\ : ``springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)`` 


A user function, which computes the 3D spring force vector depending on time, object variables (deltaL, deltaL_t) and object parameters 
(stiffness, damping, force).
The object variables are provided to the function using the current values of the SpringDamper object.
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``displacement``\ 
     - | Vector3D
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{p}}}`\ 
   * - | \ ``velocity``\ 
     - | Vector3D
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{v}}}`\ 
   * - | \ ``stiffness``\ 
     - | Vector3D
     - | copied from object
   * - | \ ``damping``\ 
     - | Vector3D
     - | copied from object
   * - | \ ``offset``\ 
     - | Vector3D
     - | copied from object
   * - | \returnValue
     - | Vector3D
     - | list or numpy array of computed spring force


--------

\ **User function example**\ :



.. code-block:: python

    #define simple force for spring-damper:
    def UFforce(mbs, t, itemNumber, u, v, k, d, offset): 
        return [u[0]*k[0],u[1]*k[1],u[2]*k[2]]
    
    #markerNumbers and parameters taken from mini example
    mbs.AddObject(CartesianSpringDamper(markerNumbers = [mGround, mMass], 
                                        stiffness = [k,k,k], 
                                        damping = [0,k*0.05,0], offset = [0,0,0],
                                        springForceUserFunction = UFforce))





.. _miniexample-objectconnectorcartesianspringdamper:

MINI EXAMPLE for ObjectConnectorCartesianSpringDamper
-----------------------------------------------------


.. code-block:: python
   :linenos:

   #example with mass at [1,1,0], 5kg under load 5N in -y direction
   k=5000
   nMass = mbs.AddNode(NodePoint(referenceCoordinates=[1,1,0]))
   oMass = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
   
   mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
   mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [1,1,0]))
   mbs.AddObject(CartesianSpringDamper(markerNumbers = [mGround, mMass], 
                                       stiffness = [k,k,k], 
                                       damping = [0,k*0.05,0], offset = [0,0,0]))
   mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -5, 0])) #static solution=-5/5000=-0.001m
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[1]

Relevant Examples and TestModels with weblink:

    \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Examples/), \ `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigid3Dexample.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `flexibleRotor3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/flexibleRotor3Dtest.py>`_\  (Examples/), \ `lavalRotor2Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lavalRotor2Dtest.py>`_\  (Examples/), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Examples/), \ `NGsolvePostProcessingStresses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePostProcessingStresses.py>`_\  (Examples/), \ `objectFFRFreducedOrderNetgen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/objectFFRFreducedOrderNetgen.py>`_\  (Examples/), \ `particlesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesTest.py>`_\  (Examples/), \ `scissorPrismaticRevolute2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/scissorPrismaticRevolute2D.py>`_\  (TestModels/), \ `sphericalJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sphericalJointTest.py>`_\  (TestModels/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


