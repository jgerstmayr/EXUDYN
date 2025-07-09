

.. _sec-item-loadtorquevector:

LoadTorqueVector
================

Load with (3D) torque vector; attached to rigidbody-based marker.

\ **Additional information for LoadTorqueVector**\ :

* | Requested \ ``Marker``\  type = \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``Torque``\ 
* | \ **Short name**\  for Python visualization object = \ ``VTorque``\ 


The item \ **LoadTorqueVector**\  with type = 'TorqueVector' has the following parameters:

* | **name** [type = String, default = '']:
  | load's unique name
* | **markerNumber** [type = MarkerIndex, default = invalid (-1)]:
  | marker's number to which load is applied
* | **loadVector** [\ :math:`\ttau`\ , type = Vector3D, default = [0.,0.,0.]]:
  | vector-valued load [SI:N]; in case of a user function, this vector is ignored
* | **bodyFixed** [type = Bool, default = False]:
  | if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used
* | **loadVectorUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^3`\ , type = PyFunctionVector3DmbsScalarVector3D, default =  0]:
  | A Python function which defines the time-dependent load and replaces loadVector; see description below; see also notes on loadFactor and drawing in LoadForceVector! Example for Python function: def f(mbs, t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]
* | **visualization** [type = VLoadTorqueVector]:
  | parameters for visualization of item



The item VLoadTorqueVector has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-loadtorquevector:

DESCRIPTION of LoadTorqueVector
-------------------------------

Details
-------

The torque vector acts on a body or node via the local (\ ``bodyFixed = True``\ ) or global coordinates of a body or at a node. 
The marker transforms the torque via the according jacobian matrix of the object (or node) to object (or node) coordinates.

--------

\ **Userfunction**\ : ``loadVectorUserFunction(mbs, t, loadVector)`` 


A user function, which computes the torque vector depending on time and object parameters, which is hereafter applied to object or node.

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
     - | \ :math:`\ttau`\  copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
   * - | \returnValue
     - | Vector3D
     - | computed torque vector


--------

\ **User function example**\ :



.. code-block:: python

    from math import sin, cos, pi
    def UFforce(mbs, t, loadVector): 
        return [loadVector[0]*sin(t*10*2*pi),0,0]




Relevant Examples and TestModels with weblink:

    \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Examples/), \ `reevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystem.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Examples/), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Examples/), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Examples/), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFtests2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtests2.py>`_\  (Examples/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TestModels/), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


