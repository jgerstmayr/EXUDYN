

.. _sec-item-objectconnectorrollingdiscpenalty:

ObjectConnectorRollingDiscPenalty
=================================

A (flexible) connector representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body, not moving) in global \ :math:`x`\ -\ :math:`y`\  plane. The connector is based on a penalty formulation and adds friction and slipping. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. Parameters may need to be adjusted for better convergence (e.g., dryFrictionProportionalZone). The formulation for the arbitrary disc axis is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.

\ **Additional information for ObjectConnectorRollingDiscPenalty**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested node type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``RollingDiscPenalty``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRollingDiscPenalty``\ 


The item \ **ObjectConnectorRollingDiscPenalty**\  with type = 'ConnectorRollingDiscPenalty' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m0`\  represents a point at the plane surface (normal of surface plane defined by planeNormal); the ground can also be a moving rigid body; \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
* | **discRadius** [type = PReal, default = 0.]:
  | defines the disc radius
* | **discAxis** [\ :math:`\LU{m1}{{\mathbf{w}}_{1}}, \;\; |\LU{m1}{{\mathbf{w}}_{1}}| = 1`\ , type = Vector3D, default = [1,0,0]]:
  | axis of disc defined in marker \ :math:`m1`\  frame
* | **planeNormal** [\ :math:`\LU{m0}{{\mathbf{v}}_{PN}}, \;\; |\LU{m0}{{\mathbf{v}}_{PN}}| = 1`\ , type = Vector3D, default = [0,0,1]]:
  | normal to the contact / rolling plane (ground); note that the plane reference point can be arbitrarily chosen by the location of the marker \ :math:`m0`\ 
* | **dryFrictionAngle** [\ :math:`\alpha_t`\ , type = Real, default = 0.]:
  | angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
* | **contactStiffness** [\ :math:`k_c`\ , type = UReal, default = 0.]:
  | normal contact stiffness [SI:N/m]
* | **contactDamping** [\ :math:`d_c`\ , type = UReal, default = 0.]:
  | normal contact damping [SI:N/(m s)]
* | **dryFriction** [\ :math:`[\mu_x,\mu_y]\tp`\ , type = Vector2D, default = [0,0]]:
  | dry friction coefficients [SI:1] in local marker 1 joint \ :math:`J1`\  coordinates; if \ :math:`\alpha_t==0`\ , lateral direction \ :math:`l=x`\  and forward direction \ :math:`f=y`\ ; assuming a normal force \ :math:`f_n`\ , the local friction force can be computed as \ :math:`\LU{J1}{\vp{f_{t,x}}{f_{t,y}}} = \vp{\mu_x f_n}{\mu_y f_n}`\ 
* | **dryFrictionProportionalZone** [\ :math:`v_\mu`\ , type = Real, default = 0.]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
* | **viscousFriction** [\ :math:`[d_x, d_y]\tp`\ , type = Vector2D, default = [0,0]]:
  | viscous friction coefficients [SI:1/(m/s)] in local marker 1 joint \ :math:`J1`\  coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity
* | **rollingFrictionViscous** [\ :math:`\mu_r`\ , type = Real, default = 0.]:
  | rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; currently, only implemented for disc axis parallel to ground!
* | **useLinearProportionalZone** [type = Bool, default = False]:
  | if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorRollingDiscPenalty]:
  | parameters for visualization of item



The item VObjectConnectorRollingDiscPenalty has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **discWidth** [type = float, default = 0.1]:
  | width of disc for drawing
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{G}`\ 
  | current global position of contact point between rolling disc and ground
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{trail}`\ 
  | current velocity of the trail (according to motion of the contact point along the trail!) in global coordinates; this is not the velocity of the contact point!
* | ``VelocityLocal``\ : \ :math:`\LU{J1}{{\mathbf{v}}}`\ 
  | relative slip velocity at contact point in special \ :math:`J1`\  joint coordinates
* | ``ForceLocal``\ : \ :math:`\LU{J1}{{\mathbf{f}}} = \LU{0}{[f_{t,x},\, f_{t,y},\, f_{n}]\tp}`\ 
  | contact forces acting on disc, in special \ :math:`J1`\  joint coordinates, see section Connector Forces, \ :math:`f_{t,x}`\  being the lateral force (parallel to ground plane), \ :math:`f_{t,y}`\  being the longitudinal force and \ :math:`f_{n}`\  being the contact normal force
* | ``RotationMatrix``\ : \ :math:`\LU{0,J1}{{\mathbf{A}}} = [\LU{0}{{\mathbf{w}}_{lat}},\, \LU{0}{{\mathbf{w}}}_2,\, \LU{0}{{\mathbf{v}}_{PN}}]`\ 
  | transformation matrix of special joint coordinates \ :math:`J1`\  to global coordinates




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


