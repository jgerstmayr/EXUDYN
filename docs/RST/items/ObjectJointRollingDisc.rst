

.. _sec-item-objectjointrollingdisc:

ObjectJointRollingDisc
======================

A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global \ :math:`x`\ -\ :math:`y`\  plane. The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the \ :math:`x`\ -\ :math:`y`\ -plane at \ :math:`z=0`\ . Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than \ :math:`z`\ -direction, wheel axis other than \ :math:`x`\ -axis and moving ground body needs to be tested further, check your results!

\ **Additional information for ObjectJointRollingDisc**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``RollingDiscJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRollingDiscJoint``\ 


The item \ **ObjectJointRollingDisc**\  with type = 'JointRollingDisc' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m0`\  represents the ground and \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
* | **constrainedAxes** [\ :math:`{\mathbf{j}}=[j_0,\,\ldots,\,j_2]`\ , type = ArrayIndex, size = 3, default = [1,1,1]]:
  | flag, which determines which constraints are active, in which \ :math:`j_0,j_1`\  represent the tangential motion and \ :math:`j_2`\  represents the normal (contact) direction; currently the constraints are given in global coordinates, but will be changed into local \ :math:`J1`\  coordinates in future
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **discRadius** [type = PReal, default = 0]:
  | defines the disc radius
* | **discAxis** [\ :math:`\LU{m1}{{\mathbf{w}}_{1}}, \;\; |\LU{m1}{{\mathbf{w}}_{1}}| = 1`\ , type = Vector3D, default = [1,0,0]]:
  | axis of disc defined in marker \ :math:`m1`\  frame
* | **planeNormal** [\ :math:`\LU{m0}{{\mathbf{v}}_{PN}}`\ , type = Vector3D, default = [0,0,1]]:
  | normal to the contact / rolling plane defined in marker \ :math:`m0`\  coordinates
* | **visualization** [type = VObjectJointRollingDisc]:
  | parameters for visualization of item



The item VObjectJointRollingDisc has the following parameters:

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
  | current velocity of the trail (according to motion of the contact point along the trail!) in global coordinates; this is not the velocity of the contact point; needs further testing for general case of relative moving bodies
* | ``ForceLocal``\ : \ :math:`\LU{J1}{{\mathbf{f}}} = \LU{0}{[f_0,\, f_1,\, f_2]\tp}= [-{\mathbf{z}}^T \LU{0}{{\mathbf{w}}_{lat}}, \, -{\mathbf{z}}^T \LU{0}{{\mathbf{w}}_2}, \, -{\mathbf{z}}^T \LU{0}{{\mathbf{v}}_{PN}}]\tp`\ 
  | contact forces acting on disc, in special \ :math:`J1`\  joint coordinates, \ :math:`f_0`\  being the lateral force (parallel to ground plane), \ :math:`f_1`\  being the longitudinal force and \ :math:`f_2`\  being the normal force
* | ``RotationMatrix``\ : \ :math:`\LU{0,J1}{{\mathbf{A}}} = [\LU{0}{{\mathbf{w}}_{lat}},\, \LU{0}{{\mathbf{w}}}_2,\, \LU{0}{{\mathbf{v}}_{PN}}]`\ 
  | transformation matrix of special joint coordinates \ :math:`J1`\  to global coordinates




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


