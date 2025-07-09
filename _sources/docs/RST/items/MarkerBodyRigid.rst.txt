

.. _sec-item-markerbodyrigid:

MarkerBodyRigid
===============

A rigid-body (position+orientation) body-marker attached to a local (body-fixed) position \ :math:`\pLocB = [b_0,\; b_1,\; b_2]`\  (\ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  coordinates) of the body. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.

\ **Additional information for MarkerBodyRigid**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ , \ ``Orientation``\ 


The item \ **MarkerBodyRigid**\  with type = 'BodyRigid' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **localPosition** [\ :math:`\pLocB`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local body position of marker; e.g. local (body-fixed) position where force is applied to
* | **visualization** [type = VMarkerBodyRigid]:
  | parameters for visualization of item



The item VMarkerBodyRigid has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markerbodyrigid:

DESCRIPTION of MarkerBodyRigid
------------------------------

Relevant Examples and TestModels with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Examples/), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFtests2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtests2.py>`_\  (Examples/), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Examples/), \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Examples/), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TestModels/), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


