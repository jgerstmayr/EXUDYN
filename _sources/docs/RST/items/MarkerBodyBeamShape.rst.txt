

.. _sec-item-markerbodybeamshape:

MarkerBodyBeamShape
===================

A special Marker attached to a 3D beam finite element which provides at least position and tangent to the beam axis.

\ **Additional information for MarkerBodyBeamShape**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Object``\ , \ ``Body``\ 


The item \ **MarkerBodyBeamShape**\  with type = 'BodyBeamShape' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to (beam type)
* | **visualization** [type = VMarkerBodyBeamShape]:
  | parameters for visualization of item



The item VMarkerBodyBeamShape has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markerbodybeamshape:

DESCRIPTION of MarkerBodyBeamShape
----------------------------------

Relevant Examples and TestModels with weblink:

    \ `ANCFslidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint.py>`_\  (Examples/), \ `NGsolveFFRFSlidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRFSlidingJoint.py>`_\  (Examples/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


