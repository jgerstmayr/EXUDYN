

.. _sec-item-loadmassproportional:

LoadMassProportional
====================

Load attached to MarkerBodyMass marker, applying a 3D vector load (e.g. the vector [0,-g,0] is used to apply gravitational loading of size g in negative y-direction).

\ **Additional information for LoadMassProportional**\ :

* | Requested \ ``Marker``\  type = \ ``Body``\  + \ ``BodyMass``\ 
* | \ **Short name**\  for Python = \ ``Gravity``\ 
* | \ **Short name**\  for Python visualization object = \ ``VGravity``\ 


The item \ **LoadMassProportional**\  with type = 'MassProportional' has the following parameters:

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **loadVector** [\ :math:`{\mathbf{b}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | vector-valued load [SI:N/kg = m/s\ :math:`^2`\ ]; typically, this will be the gravity vector in global coordinates; in case of a user function, this v is ignored
* | **loadVectorUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^3`\ , type = PyFunctionVector3DmbsScalarVector3D, default =  0]:
  | A Python function which defines the time-dependent load; see description below; see also notes on loadFactor and drawing in LoadForceVector!
* | **visualization** [type = VLoadMassProportional]:
  | parameters for visualization of item



The item VLoadMassProportional has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-loadmassproportional:

DESCRIPTION of LoadMassProportional
-----------------------------------

Details
-------

The load applies a (translational) and distributed load proportional to the distributed body's density.
The marker of type \ ``MarkerBodyMass``\  transforms the loadVector via an according jacobian matrix to object coordinates.

--------

\ **Userfunction**\ : ``loadVectorUserFunction(mbs, t, loadVector)`` 


A user function, which computes the mass proporitional load vector depending on time and object parameters, which is hereafter applied to object or node.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which load belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``loadVector``\ 
     - | Vector3D
     - | \ :math:`{\mathbf{b}}`\  copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
   * - | \returnValue
     - | Vector3D
     - | computed load vector

Example of user function: functionality same as in \ ``LoadForceVector``\ 



.. _miniexample-loadmassproportional:

MINI EXAMPLE for LoadMassProportional
-------------------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(NodePoint(referenceCoordinates = [1,0,0]))
   body = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=2))
   mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=body))
   mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=[0,0,-9.81]))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result
   exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]
   #final z-coordinate of position shall be -g/2 due to constant acceleration with g=-9.81
   #result independent of mass

Relevant Examples and TestModels with weblink:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Examples/), \ `NGsolvePostProcessingStresses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePostProcessingStresses.py>`_\  (Examples/), \ `ObjectFFRFconvergenceTestBeam.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestBeam.py>`_\  (Examples/), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Examples/), \ `pendulumGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumGeomExactBeam2D.py>`_\  (Examples/), \ `pendulumVerify.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumVerify.py>`_\  (Examples/), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_\  (TestModels/), \ `genericJointUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericJointUserFunctionTest.py>`_\  (TestModels/), \ `modelUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/modelUnitTests.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


