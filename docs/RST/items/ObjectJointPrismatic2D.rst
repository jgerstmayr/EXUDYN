

.. _sec-item-objectjointprismatic2d:

ObjectJointPrismatic2D
======================

A prismatic joint in 2D; allows the relative motion of two bodies, using two RigidMarkers; the vector \ :math:`{\mathbf{t}}_0`\  = axisMarker0 is given in local coordinates of the first marker's (body) frame and defines the prismatic axis; the vector \ :math:`\mathbf{n}_1`\  = normalMarker1 is given in the second marker's (body) frame and is the normal vector to the prismatic axis; using the global position vector \ :math:`{\mathbf{p}}_0`\  and rotation matrix \ :math:`{\mathbf{A}}_0`\  of marker0 and the global position vector \ :math:`{\mathbf{p}}_1`\  rotation matrix \ :math:`{\mathbf{A}}_1`\  of marker1, the equations for the prismatic joint follow as 
.. math::

   ({\mathbf{p}}_1-{\mathbf{p}}_0)^T\cdot {\mathbf{A}}_1 \cdot \mathbf{n}_1 = 0

  
.. math::

   ({\mathbf{A}}_0 \cdot {\mathbf{t}}_0)^T \cdot {\mathbf{A}}_1 \cdot \mathbf{n}_1 = 0

 The lagrange multipliers follow for these two equations \ :math:`[\lambda_0,\lambda_1]`\ , in which \ :math:`\lambda_0`\  is the transverse force and \ :math:`\lambda_1`\  is the torque in the joint.

\ **Additional information for ObjectJointPrismatic2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``PrismaticJoint2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VPrismaticJoint2D``\ 


The item \ **ObjectJointPrismatic2D**\  with type = 'JointPrismatic2D' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **axisMarker0** [type = Vector3D, default = [1.,0.,0.]]:
  | direction of prismatic axis, given as a 3D vector in Marker0 frame
* | **normalMarker1** [type = Vector3D, default = [0.,1.,0.]]:
  | direction of normal to prismatic axis, given as a 3D vector in Marker1 frame
* | **constrainRotation** [type = Bool, default = True]:
  | flag, which determines, if the connector also constrains the relative rotation of the two objects; if set to false, the constraint will keep an algebraic equation set equal zero
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointPrismatic2D]:
  | parameters for visualization of item



The item VObjectJointPrismatic2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = radius of revolute joint; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointprismatic2d:

DESCRIPTION of ObjectJointPrismatic2D
-------------------------------------

Relevant Examples and TestModels with weblink:

    \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `PARTS_ATEs_moving.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/PARTS_ATEs_moving.py>`_\  (TestModels/), \ `scissorPrismaticRevolute2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/scissorPrismaticRevolute2D.py>`_\  (TestModels/), \ `sliderCrankFloatingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrankFloatingTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


