

.. _sec-item-objectconnectorrollingdiscpenalty:

ObjectConnectorRollingDiscPenalty
=================================

A (flexible) connector representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body, not moving) in global \ :math:`x`\ -\ :math:`y`\  plane. The connector is based on a penalty formulation and adds friction and slipping. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. Parameters may need to be adjusted for better convergence (e.g., dryFrictionProportionalZone). The formulation for the arbitrary disc axis is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.
 



The item \ **ObjectConnectorRollingDiscPenalty**\  with type = 'ConnectorRollingDiscPenalty' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ], size =  2]:
  | list of markers used in connector; \ :math:`m0`\  represents a point at the plane surface (normal of surface plane defined by planeNormal); the ground can also be a moving rigid body; \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
* | **discRadius** [type = PReal, default = 0.]:
  | defines the disc radius
* | **discAxis** [type = Vector3D, default = [1,0,0]]:
  | axis of disc defined in marker \ :math:`m1`\  frame
* | **planeNormal** [type = Vector3D, default = [0,0,1]]:
  | normal to the contact / rolling plane (ground); note that the plane reference point can be arbitrarily chosen by the location of the marker \ :math:`m0`\ 
* | **dryFrictionAngle** [type = Real, default = 0.]:
  | angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
* | **contactStiffness** [type = UReal, default = 0.]:
  | normal contact stiffness [SI:N/m]
* | **contactDamping** [type = UReal, default = 0.]:
  | normal contact damping [SI:N/(m s)]
* | **dryFriction** [type = Vector2D, default = [0,0]]:
  | dry friction coefficients [SI:1] in local marker 1 joint \ :math:`J1`\  coordinates; if \ :math:`\alpha_t==0`\ , lateral direction \ :math:`l=x`\  and forward direction \ :math:`f=y`\ ; assuming a normal force \ :math:`f_n`\ , the local friction force can be computed as \ :math:`\LUJ1\vpf_t,xf_t,y = \vp\mu_x f_n\mu_y f_n`\ 
* | **dryFrictionProportionalZone** [type = Real, default = 0.]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
* | **viscousFriction** [type = Vector2D, default = [0,0]]:
  | viscous friction coefficients [SI:1/(m/s)] in local marker 1 joint \ :math:`J1`\  coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity
* | **rollingFrictionViscous** [type = Real, default = 0.]:
  | rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; currently, only implemented for disc axis parallel to ground!
* | **useLinearProportionalZone** [type = Bool, default = False]:
  | if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint



The item VObjectConnectorRollingDiscPenalty has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **discWidth** [type = float, default = 0.1]:
  | width of disc for drawing
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


