

.. _sec-item-objectmass1d:

ObjectMass1D
============

A 1D (translational) mass which is attached to Node1D. Note, that the mass does not need to have the interpretation as a translational mass.
 



The item \ **ObjectMass1D**\  with type = 'Mass1D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [type = UReal, default = 0.]:
  | mass [SI:kg] of mass
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for Node1D
* | **referencePosition** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | a reference position, used to transform the 1D coordinate to a position
* | **referenceRotation** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | the constant body rotation matrix, which transforms body-fixed (b) to global (0) coordinates



The item VObjectMass1D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


