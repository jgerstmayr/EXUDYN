

.. _sec-item-objectconnectorgravity:

ObjectConnectorGravity
======================

A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems; DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.

\ **Additional information for ObjectConnectorGravity**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``ConnectorGravity``\ 
* | \ **Short name**\  for Python visualization object = \ ``VConnectorGravity``\ 


The item \ **ObjectConnectorGravity**\  with type = 'ConnectorGravity' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **gravitationalConstant** [\ :math:`G`\ , type = Real, default = 6.67430e-11]:
  | gravitational constant [SI:m\ :math:`^3`\ kg\ :math:`^{-1}`\ s\ :math:`^{-2}`\ )]; while not recommended, a negative constant gan represent a repulsive force
* | **mass0** [\ :math:`mass_0`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of object attached to marker \ :math:`m0`\ 
* | **mass1** [\ :math:`mass_1`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of object attached to marker \ :math:`m1`\ 
* | **minDistanceRegularization** [\ :math:`d_{min}`\ , type = UReal, default = 0.]:
  | distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorGravity]:
  | parameters for visualization of item



The item VObjectConnectorGravity has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorgravity:

DESCRIPTION of ObjectConnectorGravity
-------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : \ :math:`L`\ 
  | distance between both points
* | ``Displacement``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{p}}}`\ 
  | relative displacement between both points
* | ``Force``\ : \ :math:`{\mathbf{f}}`\ 
  | gravity force vector, pointing from marker \ :math:`m0`\  to marker \ :math:`m1`\ 



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


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | output variables
     - | symbol
     - | formula
   * - | Displacement
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{p}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
   * - | Velocity
     - | \ :math:`\Delta\! \LU{0}{{\mathbf{v}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
   * - | Distance
     - | \ :math:`L`\ 
     - | \ :math:`|\Delta\! \LU{0}{{\mathbf{p}}}|`\ 
   * - | Force
     - | \ :math:`{\mathbf{f}}`\ 
     - | see below


Connector forces
----------------

The unit vector in force direction reads (if \ :math:`L=0`\ , singularity can be avoided using regularization),

.. math::

   {\mathbf{v}}_{f} = \frac{1}{L} \Delta\! \LU{0}{{\mathbf{p}}}


If \ ``activeConnector = True``\ , and \ :math:`L>=d_{min}`\  the gravitational force is computed as

.. math::

   f_G = - G \frac{mass_0 \cdot mass_1}{L^2}


If \ ``activeConnector = True``\ , and \ :math:`L<d_{min}`\  the gravitational force is computed as

.. math::

   f_G = - G \frac{mass_0 \cdot mass_1}{L^2+(L-d_{min})^2}


which results in a regularization for small distances, which is helpful if there are no restrictions in objects to keep apart.
If \ :math:`d_{min}=0`\  and \ :math:`L=0`\ , there a system error is raised.

The vector of the gravitational force applied at both markers, pointing from marker \ :math:`m0`\  to marker \ :math:`m1`\ , finally reads

.. math::

   {\mathbf{f}} = f_G {\mathbf{v}}_{f}


The virtual work of the connector force is computed from the virtual displacement 

.. math::

   \delta \Delta\! \LU{0}{{\mathbf{p}}} = \delta \LU{0}{{\mathbf{p}}}_{m1} - \delta \LU{0}{{\mathbf{p}}}_{m0} ,


and the virtual work (not the transposed version here, because the resulting generalized forces shall be a column vector,

.. math::

   \delta W_G = {\mathbf{f}} \delta \Delta\! \LU{0}{{\mathbf{p}}} = -\left( - G \frac{mass_0 \cdot mass_1}{L^2} \right) \left(\delta \LU{0}{{\mathbf{p}}}_{m1} - \delta \LU{0}{{\mathbf{p}}}_{m0} \right)\tp {\mathbf{v}}_{f} .


The generalized (elastic) forces thus result from

.. math::

   {\mathbf{Q}}_G = \frac{\partial \LU{0}{{\mathbf{p}}}}{\partial {\mathbf{q}}_G\tp} {\mathbf{f}} ,


and read for the markers \ :math:`m0`\  and \ :math:`m1`\ ,

.. math::

   {\mathbf{Q}}_{G, m0} = -\left( - G \frac{mass_0 \cdot mass_1}{L^2} \right) {\mathbf{J}}_{pos,m0}\tp {\mathbf{v}}_{f} , \quad {\mathbf{Q}}_{G, m1} = \left( - G \frac{mass_0 \cdot mass_1}{L^2} \right) {\mathbf{J}}_{pos,m1}\tp {\mathbf{v}}_{f} ,


where \ :math:`{\mathbf{J}}_{pos,m1}`\  represents the derivative of marker \ :math:`m1`\  w.r.t.\ its associated coordinates \ :math:`{\mathbf{q}}_{m1}`\ , analogously \ :math:`{\mathbf{J}}_{pos,m0}`\ .



.. _miniexample-objectconnectorgravity:

MINI EXAMPLE for ObjectConnectorGravity
---------------------------------------


.. code-block:: python
   :linenos:

   mass0 = 1e25
   mass1 = 1e3
   r = 1e5
   G = 6.6743e-11
   vInit = np.sqrt(G*mass0/r)
   tEnd = (r*0.5*np.pi)/vInit #quarter period
   node0 = mbs.AddNode(NodePoint(referenceCoordinates = [0,0,0])) #star
   node1 = mbs.AddNode(NodePoint(referenceCoordinates = [r,0,0], 
                                 initialVelocities=[0,vInit,0])) #satellite
   oMassPoint0 = mbs.AddObject(MassPoint(nodeNumber = node0, physicsMass=mass0))
   oMassPoint1 = mbs.AddObject(MassPoint(nodeNumber = node1, physicsMass=mass1))
   
   m0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node0))
   m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node1))
   
   mbs.AddObject(ObjectConnectorGravity(markerNumbers=[m0,m1],
                                        mass0 = mass0, mass1=mass1))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   sims = exu.SimulationSettings()
   sims.timeIntegration.endTime = tEnd
   mbs.SolveDynamic(sims, solverType=exu.DynamicSolverType.RK67)
   
   #check result at default integration time
   #expect y=x after one period of orbiting (got: 100000.00000000479)
   exudynTestGlobals.testResult = mbs.GetNodeOutput(node1, exu.OutputVariableType.Position)[1]/100000

Relevant Examples and TestModels with weblink:

    \ `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorGravityTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


