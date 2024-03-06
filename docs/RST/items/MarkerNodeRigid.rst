

.. _sec-item-markernoderigid:

MarkerNodeRigid
===============

A rigid-body (position+orientation) node-marker attached to a rigid-body node. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.

\ **Additional information for MarkerNodeRigid**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Node``\ , \ ``Position``\ , \ ``Orientation``\ 


The item \ **MarkerNodeRigid**\  with type = 'NodeRigid' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number to which marker is attached to
* | **visualization** [type = VMarkerNodeRigid]:
  | parameters for visualization of item



The item VMarkerNodeRigid has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markernoderigid:

DESCRIPTION of MarkerNodeRigid
------------------------------
The node rigid body marker provides an interface to a node which contains a position and an orientation
(\ ``NodeRigidBodyEP``\ , \ ``NodeRigidBody2D``\ , ...)
and provides access to kinematic quantities such as \ **position**\ , \ **velocity**\ , \ **orientation**\  (rotation matrix),
\ **angular velocity**\ . It also provides the \ **position jacobian**\  and the \ **rotation jacobian**\ .
The kinematic quantities are computed according to the definition of output variables in the respective nodes.

The position jacobian represents the derivative of the node position \ :math:`{\mathbf{p}}_\mathrm{n}`\  with all nodal coordinates,

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}} = \frac{\partial \LU{0}{{\mathbf{p}}_\mathrm{n}}}{\partial {\mathbf{q}}_\mathrm{n}}


and it is usually computed as the derivative of the (global) translational velocity w.r.t.\ velocity coordinates,

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}} = \frac{\partial \LU{0}{{\mathbf{v}}_\mathrm{n}}}{\partial \dot {\mathbf{q}}_\mathrm{n}}


The rotation jacobian is computed as the derivative of the (global) angular velocity w.r.t.\ velocity coordinates,

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{rot}} = \frac{\partial \LU{0}{\tomega_\mathrm{n}}}{\partial \dot {\mathbf{q}}_\mathrm{n}}


This usually results in the velocity transformation matrix.
For details, see the respective definition of the node and the C++ implementation.


Relevant Examples and TestModels with weblink:

    \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFtests2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtests2.py>`_\  (Examples/), \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Examples/), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Examples/), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Examples/), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TestModels/), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


