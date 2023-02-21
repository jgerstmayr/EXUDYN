

.. _sec-item-objectcontactfrictioncirclecable2d:

ObjectContactFrictionCircleCable2D
==================================

A very specialized penalty-based contact/friction condition between a 2D circle in the local x/y plane (=marker0, a RigidBody Marker, from node or object) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane; a node NodeGenericData is required with 3\ :math:`\times`\ (number of contact segments) -- containing per segment: [contact gap, stick/slip (stick=0, slip=+-1, undefined=-2), last friction position]. The connector works with Cable2D and ALECable2D, HOWEVER, due to conceptual differences the (tangential) frictionStiffness cannot be used with ALECable2D; if using, it gives wrong tangential stresses, even though it may work in general.

\ **Additional information for ObjectContactFrictionCircleCable2D**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``_None``\ 
* | Requested node type = \ ``GenericData``\ 


The item \ **ObjectContactFrictionCircleCable2D**\  with type = 'ContactFrictionCircleCable2D' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | a marker \ :math:`m0`\  with position and orientation and a marker \ :math:`m1`\  of type BodyCable2DShape; together defining the contact geometry
* | **nodeNumber** [\ :math:`n_g`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData with 3 \ :math:`\times n_{cs}`\   dataCoordinates (used for active set strategy \ :math:`\ra`\  hold the gap of the last discontinuous iteration, friction state (+-1=slip, 0=stick, -2=undefined) and the last sticking position; initialize coordinates with list [0.1]*\ :math:`n_{cs}`\ +[-2]*\ :math:`n_{cs}`\ +[0.]*\ :math:`n_{cs}`\ , meaning that there is no initial contact with undefined slip/stick
* | **numberOfContactSegments** [\ :math:`n_{cs}`\ , type = PInt, default = 3]:
  | number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker
* | **contactStiffness** [\ :math:`k_c`\ , type = UReal, default = 0.]:
  | contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) \ :math:`f_n`\  act in contact normal direction only upon penetration
* | **contactDamping** [\ :math:`d_c`\ , type = UReal, default = 0.]:
  | contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration
* | **frictionVelocityPenalty** [\ :math:`\mu_v`\ , type = UReal, default = 0.]:
  | tangential velocity dependent penalty coefficient for friction [SI:N/(m s)/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential velocities in the contact area
* | **frictionStiffness** [\ :math:`\mu_k`\ , type = UReal, default = 0.]:
  | tangential displacement dependent penalty/stiffness coefficient for friction [SI:N/m/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential displacements in the contact area
* | **frictionCoefficient** [\ :math:`\mu`\ , type = UReal, default = 0.]:
  | friction coefficient [SI: 1]; tangential specific friction forces (per length) \ :math:`f_t`\  must fulfill the condition \ :math:`f_t \le \mu f_n`\ 
* | **circleRadius** [\ :math:`r`\ , type = UReal, default = 0.]:
  | radius [SI:m] of contact circle
* | **useSegmentNormals** [type = Bool, default = True]:
  | True: use normal and tangent according to linear segment; this is appropriate for very long (compared to circle) segments; False: use normals at segment points according to vector to circle center; this is more consistent for short segments, as forces are only applied in beam tangent and normal direction
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactFrictionCircleCable2D]:
  | parameters for visualization of item



The item VObjectContactFrictionCircleCable2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set True, if item is shown in visualization and false if it is not shown; note that only normal contact forces can be  drawn, which are approximated by \ :math:`k_c \cdot g`\  (neglecting damping term)
* | **showContactCircle** [type = Bool, default = True]:
  | if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`[u_{t,0},\, g_0,\, u_{t,1},\, g_1,\, \ldots,\, u_{t,n_{cs}},\, g_{n_{cs}}]\tp`\ 
  | local (relative) displacement in tangential (\ :math:`{\mathbf{t}}`\ ) and normal (\ :math:`{\mathbf{n}}`\ ) direction per segment (\ :math:`n_{cs}`\ ); values are only provided in case of contact, otherwise zero; tangential displacement is only non-zero in case of sticking!
* | ``Coordinates\_t``\ : \ :math:`[v_{t,0},\, v_{n,0},\, v_{t,1},\, v_{n,1},\, \ldots,\, v_{t,n_{cs}},\, v_{n,n_{cs}}]\tp`\ 
  | local (relative) velocity in tangential (\ :math:`{\mathbf{t}}`\ ) and normal (\ :math:`{\mathbf{n}}`\ ) direction per segment (\ :math:`n_{cs}`\ ); values are only provided in case of contact, otherwise zero
* | ``ForceLocal``\ : \ :math:`[f_{t,0},\, f_{n,0},\, f_{t,1},\, f_{n,1},\, \ldots,\, f_{t,n_{cs}},\, f_{n,n_{cs}}]\tp`\ 
  | local contact forces in tangential (\ :math:`{\mathbf{t}}`\ ) and normal (\ :math:`{\mathbf{n}}`\ ) direction per segment (\ :math:`n_{cs}`\ )




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


