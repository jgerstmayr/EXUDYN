

.. _sec-item-objectcontactcurvecircles:

ObjectContactCurveCircles
=========================

[UNDER CONSTRUCTION] A contact model between a curve defined by piecewise segments and a set of circles. The 2D curve may corotate in 3D with the underlying marker and also defines the plane of action for the circles. Note that there is a limit of 100 circle markes above which computation becomes slower as it requires memory allocation.

\ **Additional information for ObjectContactCurveCircles**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``CamFollower``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCamFollower``\ 


The item \ **ObjectContactCurveCircles**\  with type = 'ContactCurveCircles' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m_{c0},m_{c1},\ldots]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of \ :math:`n_c+1`\  markers; marker \ :math:`m0`\  represents the marker carrying the curve; all other markers represent centers of \ :math:`n_c`\  circles, used in connector
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData with nDataVariablesPerSegment dataCoordinates per segment, needed for discontinuous iteration; data variables contain values from last PostNewton iteration: data[0+3*i] is the circle number, data[1+3*i] is the gap, data[2+3*i] is the tangential velocity (and thus contains information if it is stick or slip)
* | **circlesRadii** [\ :math:`[r_{c0},r_{c1}, \ldots]\tp \in \Rcal^{n_c}`\ , type = NumpyVector, default = []]:
  | Vector containing radii of \ :math:`n_c`\  circles [SI:m]; number according to size of markerNumbers-1
* | **segmentsData** [\ :math:`{\mathbf{D}} \in \Rcal^{n_s \times 4}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | matrix containing a set of two planar point coordinates in each row, representing segments attached to marker \ :math:`m0`\  and undergoing contact with the circles; for segment \ :math:`s0`\  row 0 reads \ :math:`[p_{0x,s0},\,p_{0y,s0},\,p_{1x,s0},\,p_{1y,s0}]`\ ; note that the segments must be ordered such that going from \ :math:`{\mathbf{p}}_0`\  to \ :math:`{\mathbf{p}}_1`\ , the exterior lies on the right (positive) side. MatrixContainer has to be provided in dense mode!
* | **polynomialData** [\ :math:`{\mathbf{P}} \in \Rcal^{n_s \times n_p}`\ , type = PyMatrixContainer, default = PyMatrixContainer[]]:
  | matrix containing coefficients for polynomial enhancements of the linear segments; each row contains polynomial coefficients for the according segment; the polynomial coefficients may contain quadratic, cubic, etc. coefficients, while constant and linear coefficients are automatically selected such that the end points of the polynomial match the segment's endpoints; the local coordinate \ :math:`x`\  of the polynomial runs from 0 to 1 and positive values represent concave geometries (enlarging the curve). MatrixContainer has to be provided in dense mode!
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 0; used to rotate marker coordinates such that the curve lies in the \ :math:`x-y`\ -plane
* | **dynamicFriction** [\ :math:`\mu_d`\ , type = UReal, default = 0.]:
  | dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **frictionProportionalZone** [\ :math:`v_{reg}`\ , type = UReal, default = 1e-3]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section :ref:`sec-module-physics`\ 
* | **contactStiffness** [\ :math:`k_c`\ , type = Real, default = 0.]:
  | normal contact stiffness [SI:N/m] (units in case that \ :math:`n_\mathrm{exp}=1`\ )
* | **contactDamping** [\ :math:`d_c`\ , type = Real, default = 0.]:
  | linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
* | **contactModel** [\ :math:`m_\mathrm{contact}`\ , type = UInt, default = 0]:
  | number of contact model: 0) linear model for stiffness and damping, only proportional to penetration; 1) model taking contact geometry, in particular curvature of circle and curve into account, giving nonlinear normal force model
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactCurveCircles]:
  | parameters for visualization of item



The item VObjectContactCurveCircles has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; draws curve and circles with given radii; uses visualizationSettings circleTiling for circles and circleTiling/2 for tiling of non-straight segments
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectcontactcurvecircles:

DESCRIPTION of ObjectContactCurveCircles
----------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``DisplacementLocal``\ : 
  | vector containing the minimum distance to segments per circle midpoint (< 0 in case of contact, and -1 if not computed: if not in according vicinity in search tree)
* | ``VelocityLocal``\ : 
  | vector containing relative (normal) velocity per circle midpoint (or NaN if not computed)
* | ``ForceLocal``\ : 
  | pairs of normal and tangential forces per circle or (Nan,Nan) if not computed



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | global position of sphere 0 center as provided by marker m0
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m0 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m0}`\ 
     - | current angular velocity vector provided by marker m0
   * - | data coordinates
     - | \ :math:`{\mathbf{x}}=[x_0,\,x_1,\, \ldots]\tp`\ 
     - | data coordinates per number of circle markers


Geometric relations
-------------------

tbd


Relevant Examples and TestModels with weblink:

    \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Examples/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


