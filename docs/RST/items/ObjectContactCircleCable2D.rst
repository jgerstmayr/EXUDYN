

.. _sec-item-objectcontactcirclecable2d:

ObjectContactCircleCable2D
==========================

A very specialized penalty-based contact condition between a 2D circle (=marker0, any Position-marker) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane; a node NodeGenericData is required with the number of cordinates according to the number of contact segments; the contact gap \ :math:`g`\  is integrated (piecewise linear) along the cable and circle; the contact force \ :math:`f_c`\  is zero for \ :math:`gap>0`\  and otherwise computed from \ :math:`f_c = g*contactStiffness + \dot g*contactDamping`\ ; during Newton iterations, the contact force is actived only, if \ :math:`dataCoordinate[0] <= 0`\ ; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.

\ **Additional information for ObjectContactCircleCable2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``_None``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 


The item \ **ObjectContactCircleCable2D**\  with type = 'ContactCircleCable2D' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | markers define contact gap
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData for nSegments dataCoordinates (used for active set strategy ==> hold the gap of the last discontinuous iteration and the friction state)
* | **numberOfContactSegments** [type = Index, default = 3]:
  | number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker
* | **contactStiffness** [type = UReal, default = 0.]:
  | contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) \ :math:`f_N`\  act in contact normal direction only upon penetration
* | **contactDamping** [type = UReal, default = 0.]:
  | contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration
* | **circleRadius** [type = UReal, default = 0.]:
  | radius [SI:m] of contact circle
* | **offset** [type = Real, default = 0.]:
  | offset [SI:m] of contact, e.g. to include thickness of cable element
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactCircleCable2D]:
  | parameters for visualization of item



The item VObjectContactCircleCable2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **showContactCircle** [type = Bool, default = True]:
  | if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectcontactcirclecable2d:

DESCRIPTION of ObjectContactCircleCable2D
-----------------------------------------

Relevant Examples and TestModels with weblink:

    \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TestModels/), \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


