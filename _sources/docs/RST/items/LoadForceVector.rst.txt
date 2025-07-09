

.. _sec-item-loadforcevector:

LoadForceVector
===============

Load with (3D) force vector; attached to position-based marker.

\ **Additional information for LoadForceVector**\ :

* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``Force``\ 
* | \ **Short name**\  for Python visualization object = \ ``VForce``\ 


The item \ **LoadForceVector**\  with type = 'ForceVector' has the following parameters:

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **loadVector** [\ :math:`{\mathbf{f}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | vector-valued load [SI:N]; in case of a user function, this vector is ignored
* | **bodyFixed** [type = Bool, default = False]:
  | if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower force; if false: global coordinates are used
* | **loadVectorUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^3`\ , type = PyFunctionVector3DmbsScalarVector3D, default =  0]:
  | A Python function which defines the time-dependent load and replaces loadVector; see description below; NOTE that in static computations, the loadFactor is always 1 for forces computed by user functions (this means for the static computation, that a user function returning [t*5,t*1,0] corresponds to loadVector=[5,1,0] without a user function); NOTE that forces are drawn using the value of loadVector; thus the current values according to the user function are NOT shown in the render window; however, a sensor (SensorLoad) returns the user function force which is applied to the object; to draw forces with current user function values, use a graphicsDataUserFunction of a ground object
* | **visualization** [type = VLoadForceVector]:
  | parameters for visualization of item



The item VLoadForceVector has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-loadforcevector:

DESCRIPTION of LoadForceVector
------------------------------

Details
-------

The load vector acts on a body or node via the local (\ ``bodyFixed = True``\ ) or global coordinates of a body or at a node. 
The marker transforms the (translational) force via the according jacobian matrix of the object (or node) to object (or node) coordinates.

--------

\ **Userfunction**\ : ``loadVectorUserFunction(mbs, t, loadVector)`` 


A user function, which computes the force vector depending on time and object parameters, which is hereafter applied to object or node.

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
     - | \ :math:`{\mathbf{f}}`\  copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
   * - | \returnValue
     - | Vector3D
     - | computed force vector


--------

\ **User function example**\ :



.. code-block:: python

    from math import sin, cos, pi
    def UFforce(mbs, t, loadVector): 
        return [loadVector[0]*sin(t*10*2*pi),0,0]




Relevant Examples and TestModels with weblink:

    \ `interactiveTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/interactiveTutorial.py>`_\  (Examples/), \ `NGsolveModalAnalysis.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveModalAnalysis.py>`_\  (Examples/), \ `pendulumVerify.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumVerify.py>`_\  (Examples/), \ `ROSMassPoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMassPoint.py>`_\  (Examples/), \ `solutionViewerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/solutionViewerTest.py>`_\  (Examples/), \ `SpringDamperMassUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SpringDamperMassUserFunction.py>`_\  (Examples/), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Examples/), \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Examples/), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Examples/), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Examples/), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Examples/), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Examples/), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TestModels/), \ `plotSensorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/plotSensorTest.py>`_\  (TestModels/), \ `revoluteJointPrismaticJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/revoluteJointPrismaticJointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


