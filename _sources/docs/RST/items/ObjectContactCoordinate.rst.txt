

.. _sec-item-objectcontactcoordinate:

ObjectContactCoordinate
=======================

A penalty-based contact condition for one coordinate; the contact gap \ :math:`g`\  is defined as \ :math:`g=marker.value[1]- marker.value[0] - offset`\ ; the contact force \ :math:`f_c`\  is zero for \ :math:`gap>0`\  and otherwise computed from \ :math:`f_c = g*contactStiffness + \dot g*contactDamping`\ ; during Newton iterations, the contact force is actived only, if \ :math:`dataCoordinate[0] <= 0`\ ; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.

\ **Additional information for ObjectContactCoordinate**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Coordinate``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 


The item \ **ObjectContactCoordinate**\  with type = 'ContactCoordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | markers define contact gap
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData for 1 dataCoordinate (used for active set strategy ==> holds the gap of the last discontinuous iteration)
* | **contactStiffness** [type = UReal, default = 0.]:
  | contact (penalty) stiffness [SI:N/m]; acts only upon penetration
* | **contactDamping** [type = UReal, default = 0.]:
  | contact damping [SI:N/(m s)]; acts only upon penetration
* | **offset** [type = Real, default = 0.]:
  | offset [SI:m] of contact
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactCoordinate]:
  | parameters for visualization of item



The item VObjectContactCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectcontactcoordinate:

DESCRIPTION of ObjectContactCoordinate
--------------------------------------

Relevant Examples and TestModels with weblink:

    \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `contactCoordinateTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCoordinateTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


