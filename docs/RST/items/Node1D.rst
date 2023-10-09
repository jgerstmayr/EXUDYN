

.. _sec-item-node1d:

Node1D
======

A node with one \ :ref:`ODE2 <ODE2>`\  coordinate for one dimensional (1D) problems; use e.g. for scalar dynamic equations (Mass1D) and mass-spring-damper mechanisms, representing either translational or rotational degrees of freedom: in most cases, Node1D is equivalent to NodeGenericODE2 using one coordinate, however, it offers a transformation to 3D translational or rotational motion and allows to couple this node to 2D or 3D bodies.

\ **Additional information for Node1D**\ :

* | This \ ``Node``\  has/provides the following types = \ ``GenericODE2``\ 


The item \ **Node1D**\  with type = '1D' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`[q_0]\tp\cRef`\ , type = Vector, default = [0.]]:
  | reference coordinate of node (in vector form)
* | **initialCoordinates** [\ :math:`[q_0]\tp\cIni`\ , type = Vector, default = [0.]]:
  | initial displacement coordinate (in vector form)
* | **initialVelocities** [\ :math:`[\dot q_0]\tp\cIni`\ , type = Vector, default = [0.]]:
  | initial velocity coordinate (in vector form)
* | **visualization** [type = VNode1D]:
  | parameters for visualization of item



The item VNode1D has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown; The node1D is represented as reference position and displacement along the global x-axis, which must not agree with the representation in the object using the Node1D


----------

.. _description-node1d:

DESCRIPTION of Node1D
---------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`{\mathbf{q}}\cConfig = [q_0]\tp\cConfig`\ 
  | \ :ref:`ODE2 <ODE2>`\  coordinate of node (in vector form)
* | ``Coordinates\_t``\ : \ :math:`\dot {\mathbf{q}}\cConfig = [\dot q_0]\tp\cConfig`\ 
  | \ :ref:`ODE2 <ODE2>`\  velocity coordinate of node (in vector form)
* | ``Coordinates\_tt``\ : \ :math:`\ddot {\mathbf{q}}\cConfig = [\ddot q_0]\tp\cConfig`\ 
  | \ :ref:`ODE2 <ODE2>`\  acceleration coordinate of node (in vector form)



\ **Detailed information:** 
The current position/rotation coordinate of the 1D node is computed from

.. math::

   p_0 = {q_0}\cRef + {q_0}\cCur


The coordinate leads to one second order differential equation.
The graphical representation and the (internal) position of the node is

.. math::

   p\cConfig= \vr{{p_0}\cConfig}{0}{0}


The (internal) velocity vector is \ :math:`[{p_0}\cConfig,\,0,\,0]\tp`\ .


Relevant Examples and TestModels with weblink:

    \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Examples/), \ `mpi4pyExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mpi4pyExample.py>`_\  (Examples/), \ `multiprocessingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiprocessingTest.py>`_\  (Examples/), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Examples/), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Examples/), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Examples/), \ `coordinateSpringDamperExt.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateSpringDamperExt.py>`_\  (TestModels/), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TestModels/), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


