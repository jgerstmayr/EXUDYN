

.. _sec-item-markersuperelementposition:

MarkerSuperElementPosition
==========================

A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is in its current implementation inefficient for large number of meshNodeNumbers). The marker acts on the mesh (interface) nodes, not on the underlying nodes of the object.

\ **Additional information for MarkerSuperElementPosition**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ 


The item \ **MarkerSuperElementPosition**\  with type = 'SuperElementPosition' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [\ :math:`n_b`\ , type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **meshNodeNumbers** [\ :math:`[k_0,\,\ldots,\,k_{n_m-1}]\tp`\ , type = ArrayIndex, default = []]:
  | a list of \ :math:`n_m`\  mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
* | **weightingFactors** [\ :math:`[w_{0},\,\ldots,\,w_{n_m-1}]\tp`\ , type = Vector, default = []]:
  | a list of \ :math:`n_m`\  weighting factors per node to compute the final local position; the sum of these weights shall be 1, such that a summation of all nodal positions times weights gives the average position of the marker
* | **visualization** [type = VMarkerSuperElementPosition]:
  | parameters for visualization of item



The item VMarkerSuperElementPosition has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **showMarkerNodes** [type = Bool, default = True]:
  | set true, if all nodes are shown (similar to marker, but with less intensity)


----------

.. _description-markersuperelementposition:

DESCRIPTION of MarkerSuperElementPosition
-----------------------------------------
\ **Definition of marker quantities**\ :

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | number of mesh nodes
     - | \ :math:`n_m`\ 
     - | size of \ ``meshNodeNumbers``\  and \ ``weightingFactors``\  which are marked; this must not be the number of mesh nodes in the marked object
   * - | mesh node number
     - | \ :math:`i = k_i`\ 
     - | abbreviation
   * - | mesh node points
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{i}`\ 
     - | position of mesh node \ :math:`k_i`\  in object \ :math:`n_b`\ 
   * - | mesh node velocities
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{i}`\ 
     - | velocity of mesh node \ :math:`i`\  in object \ :math:`n_b`\ 
   * - | marker position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m} = \sum_i w_i \cdot \LU{0}{{\mathbf{p}}_i}`\ 
     - | current global position which is provided by marker
   * - | marker velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m} = \sum_i w_i \cdot \LU{0}{{\mathbf{v}}_i}`\ 
     - | current global velocity which is provided by marker



Marker quantities
-----------------

The marker provides a 'position' jacobian, which is the derivative of the marker velocity w.r.t.\ the 
object velocity coordinates \ :math:`\dot {\mathbf{q}}_{n_b}`\ ,

.. math::

   {\mathbf{J}}_{m,pos} = \frac{\partial \LU{0}{{\mathbf{v}}}_{m}}{\partial \dot {\mathbf{q}}_{n_b}} = \sum_i w_i \cdot {\mathbf{J}}_{i,pos}


in which \ :math:`{\mathbf{J}}_{i,pos}`\  denotes the position jacobian of mesh node \ :math:`i`\ ,

.. math::

   {\mathbf{J}}_{i,pos} = \frac{\partial \LU{0}{{\mathbf{v}}}_{i}}{\partial \dot {\mathbf{q}}_{n_b}}


The jacobian \ :math:`{\mathbf{J}}_{i,pos}`\  usually contains mostly zeros for \ ``ObjectGenericODE2``\ , because the jacobian only affects one single node.
In \ ``ObjectFFRFreducedOrder``\ , the jacobian may affect all reduced coordinates.

Note that \ :math:`{\mathbf{J}}_{m,pos}`\  is actually computed by the
\ ``ObjectSuperElement``\  within the function \ ``GetAccessFunctionSuperElement``\ .



.. _miniexample-markersuperelementposition:

MINI EXAMPLE for MarkerSuperElementPosition
-------------------------------------------


.. code-block:: python
   :linenos:

   #set up a mechanical system with two nodes; it has the structure: |~~M0~~M1
   #==>further examples see objectGenericODE2Test.py, objectFFRFTest2.py, etc.
   nMass0 = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
   nMass1 = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
   mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [1,0,0]))
   
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
   
   #EXAMPLE for single node marker on super element body, mesh node 1; compare results to ObjectGenericODE2 example!!! 
   mSuperElement = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oGenericODE2, meshNodeNumbers=[1], weightingFactors=[1]))
   mbs.AddLoad(Force(markerNumber = mSuperElement, loadVector = [10, 0, 0])) 
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   mbs.SolveDynamic(solverType = exudyn.DynamicSolverType.TrapezoidalIndex2)
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.Position)[0]

Relevant Examples and TestModels with weblink:

    \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Examples/), \ `NGsolvePostProcessingStresses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePostProcessingStresses.py>`_\  (Examples/), \ `objectFFRFreducedOrderNetgen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/objectFFRFreducedOrderNetgen.py>`_\  (Examples/), \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TestModels/), \ `objectFFRFreducedOrderStressModesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderStressModesTest.py>`_\  (TestModels/), \ `objectFFRFreducedOrderTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderTest.py>`_\  (TestModels/), \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TestModels/), \ `objectFFRFTest2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest2.py>`_\  (TestModels/), \ `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_\  (TestModels/), \ `perfObjectFFRFreducedOrder.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perfObjectFFRFreducedOrder.py>`_\  (TestModels/), \ `superElementRigidJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/superElementRigidJointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


