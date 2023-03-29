.. _sec-generalnotation:


********
Notation
********

The notation is used to explain:

+  typical symbols in equations and formulas (e.g., \ :math:`q`\ )
+  common types used as parameters in items (e.g., \ ``PInt``\ )
+  typical annotation of equations (or parts of it) and symbols (e.g., \ ``ODE2``\ )



.. _sec-typesdescriptions:


Common types
------------

Common types are especially used in the definition of items. 
These types indicate how they need to be set (e.g., a \ ``Vector3D``\  is set as a list of 3 floats or as a numpy array with 3 floats), 
and they usually include some range or size check (e.g., \ ``PReal``\  is checked for being positive and non-zero):

+  \ ``float``\  \ :math:`\ldots`\  a single-precision floating point number (note: in Python, '\ ``float``\ ' is used also for double precision numbers; in EXUDYN, internally floats are single precision numbers especially for graphics objects and OpenGL)
+  \ ``Real``\  \ :math:`\ldots`\  a double-precision floating point number (note: in Python this is also of type '\ ``float``\ ')
+  \ ``UReal``\  \ :math:`\ldots`\  same as \ ``Real``\ , but may not be negative
+  \ ``PReal``\  \ :math:`\ldots`\  same as \ ``Real``\ , but must be positive, non-zero (e.g., step size may never be zero)
+  \ ``Index``\  \ :math:`\ldots`\  deprecated, represents unsined integer, \ ``UInt``\ 
+  \ ``Int``\  \ :math:`\ldots`\  a (signed) integer number, which converts to '\ ``int``\ ' in Python, '\ ``int``\ ' in C++
+  \ ``UInt``\  \ :math:`\ldots`\  an unsigned integer number, which converts to '\ ``int``\ ' in Python
+  \ ``PInt``\  \ :math:`\ldots`\  an positive integer number (> 0), which converts to '\ ``int``\ ' in Python
+  \ ``NodeIndex, MarkerIndex, ...``\  \ :math:`\ldots`\  a special (non-negative) integer type to represent indices of nodes, markers, ...; specifically, an unintentional conversion from one index type to the other is not possible (e.g., to convert \ ``NodeIndex``\  to \ ``MarkerIndex``\ ); see Section :ref:`sec-itemindex`\ 
+  \ ``String``\  \ :math:`\ldots`\  a string
+  \ ``ArrayIndex``\  \ :math:`\ldots`\  a list of integer numbers (either list or in some cases \ ``numpy``\  arrays may be allowed)
+  \ ``ArrayNodeIndex``\  \ :math:`\ldots`\  a list of node indices
+  \ ``Bool``\  \ :math:`\ldots`\  a boolean parameter: either \ ``True``\  or \ ``False``\  ('\ ``bool``\ ' in Python)
+  \ ``VObjectMassPoint``\ , \ ``VObjectRigidBody``\ , \ ``VObjectGround``\ , etc.  \ :math:`\ldots`\  represents the visualization object of the underlying object; 'V' is put in front of object name
+  \ ``BodyGraphicsData``\  \ :math:`\ldots`\  see Section :ref:`sec-graphicsdata`\ 
+  \ ``Vector2D``\  \ :math:`\ldots`\  a list or \ ``numpy``\  array of 2 real numbers
+  \ ``Vector3D``\  \ :math:`\ldots`\  a list or \ ``numpy``\  array of 3 real numbers
+  \ ``Vector'X'D``\  \ :math:`\ldots`\  a list or \ ``numpy``\  array of 'X' real numbers
+  \ ``Float4``\  \ :math:`\ldots`\  a list of 4 float numbers
+  \ ``Vector``\  \ :math:`\ldots`\  a list or \ ``numpy``\  array of real numbers (length given by according object)
+  \ ``NumpyVector``\  \ :math:`\ldots`\  a 1D \ ``numpy``\  array with real numbers (size given by according object); similar as Vector, but not accepting list
+  \ ``Matrix3D``\  \ :math:`\ldots`\  a list of lists or \ ``numpy``\  array with \ :math:`3 \times 3`\  real numbers
+  \ ``Matrix6D``\  \ :math:`\ldots`\  a list of lists or \ ``numpy``\  array with \ :math:`6 \times 6`\  real numbers
+  \ ``NumpyMatrix``\  \ :math:`\ldots`\  a 2D \ ``numpy``\  array (matrix) with real numbers (size given by according object)
+  \ ``NumpyMatrixI``\  \ :math:`\ldots`\  a 2D \ ``numpy``\  array (matrix) with integer numbers (size given by according object)
+  \ ``MatrixContainer``\  \ :math:`\ldots`\  a versatile representation for dense and sparse matrices, see Section :ref:`sec-matrixcontainer`\ 
+  \ ``PyFunctionGraphicsData``\  \ :math:`\ldots`\  a user function providing GraphicsData, see the user function description of the according object
+  \ ``PyFunctionMbsScalar...``\  \ :math:`\ldots`\  a user function for the according object; the name is chosen according to the interface (arguments containing scalars, vectors, etc.) and is only used internally for code generation; see the according user function description



States and coordinate attributes
--------------------------------

The following subscripts are used to define configurations of a quantity, e.g., for a vector of displacement coordinates \ :math:`{\mathbf{q}}`\ :

+  \ :math:`{\mathbf{q}}\cConfig \ldots`\  \ :math:`{\mathbf{q}}`\  in any configuration
+  \ :math:`{\mathbf{q}}\cRef \ldots`\  \ :math:`{\mathbf{q}}`\  in reference configuration, e.g., reference coordinates: \ :math:`{\mathbf{c}}\cRef`\ 
+  \ :math:`{\mathbf{q}}\cIni \ldots`\  \ :math:`{\mathbf{q}}`\  in initial configuration, e.g., initial displacements: \ :math:`{\mathbf{u}}\cIni`\ 
+  \ :math:`{\mathbf{q}}\cCur \ldots`\  \ :math:`{\mathbf{q}}`\  in current configuration
+  \ :math:`{\mathbf{q}}\cVis \ldots`\  \ :math:`{\mathbf{q}}`\  in visualization configuration
+  \ :math:`{\mathbf{q}}\cSOS \ldots`\  \ :math:`{\mathbf{q}}`\  in start of step configuration

As written in the introduction, the coordinates are attributed to certain types of equations and therefore, the following attributes are used (usually as subscript, e.g., \ :math:`{\mathbf{q}}_{ODE2}`\ ):



 + \ :ref:`ODE2 <ODE2>`\ , \ :ref:`ODE1 <ODE1>`\ , \ :ref:`AE <AE>`\ , Data : These attributes refer to these types of equations or coordinates (click to see explanation)


Time is usually defined as 'time' or \ :math:`t`\ .
The cross product or vector product '\ :math:`\times`\ ' is often replaced by the skew symmetric matrix using the tilde '\ :math:`\tilde{\;\;}`\ ' symbol,

.. math::

   {\mathbf{a}} \times {\mathbf{b}} = \tilde {\mathbf{a}} \, {\mathbf{b}} = -\tilde {\mathbf{b}} \, {\mathbf{a}} ,


in which \ :math:`\tilde{\;\;}`\  transforms a vector \ :math:`{\mathbf{a}}`\  into a skew-symmetric matrix \ :math:`\tilde {\mathbf{a}}`\ .
If the components of \ :math:`{\mathbf{a}}`\  are defined as \ :math:`{\mathbf{a}} = \vrRow{a_0}{a_1}{a_2}\tp`\ , then the skew-symmetric matrix reads

.. math::

   \tilde {\mathbf{a}} = \mr{0}{-a_2}{a_1} {a_2}{0}{-a_0} {-a_1}{a_0}{0} .


The inverse operation is denoted as \ :math:`\vec`\ , resulting in \ :math:`\vec(\tilde {\mathbf{a}}) = {\mathbf{a}}`\ .

For the length of a vector we often use the abbreviation 

.. math::
   :label: eq-definition-length

   \Vert {\mathbf{a}} \Vert = \sqrt{{\mathbf{a}}^T {\mathbf{a}}} .



A vector \ :math:`{\mathbf{a}}=[x,\, y,\, z]\tp`\  can be transformed into a diagonal matrix, e.g.,

.. math::

   {\mathbf{A}} = \diag({\mathbf{a}}) = \mr{x}{0}{0} {0}{y}{0} {0}{0}{z}



.. _sec-symbolsitems:


Symbols in item equations
-------------------------

The following tables contains the common notation
General \ **coordinates**\  are: 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **python name (or description)**\ 
     - | \ **symbol**\ 
     - | \ **description**\  
   * - | displacement coordinates (\ :ref:`ODE2 <ODE2>`\ )
     - | \ :math:`{\mathbf{q}} = [q_0,\, \ldots,\, q_n]\tp`\ 
     - | vector of \ :math:`n`\  displacement based coordinates in any configuration; used for second order differential equations 
   * - | rotation coordinates (\ :ref:`ODE2 <ODE2>`\ )
     - | \ :math:`\tpsi = [\psi_0,\, \ldots,\, \psi_\eta]\tp`\ 
     - | vector of \ :math:`\eta`\  \ **rotation based coordinates**\  in any configuration; these coordinates are added to reference rotation parameters to provide the current rotation parameters; used for second order differential equations 
   * - | coordinates (\ :ref:`ODE1 <ODE1>`\ )
     - | \ :math:`{\mathbf{y}} = [y_0,\, \ldots,\, y_n]\tp`\ 
     - | vector of \ :math:`n`\  coordinates for first order ordinary differential equations (\ :ref:`ODE1 <ODE1>`\ ) in any configuration 
   * - | algebraic coordinates
     - | \ :math:`{\mathbf{z}} = [z_0,\, \ldots,\, z_m]\tp`\ 
     - | vector of \ :math:`m`\  algebraic coordinates if not Lagrange multipliers in any configuration 
   * - | Lagrange multipliers
     - | \ :math:`\tlambda = [\lambda_0,\, \ldots,\, \lambda_m]\tp`\ 
     - | vector of \ :math:`m`\  Lagrange multipliers (=algebraic coordinates) in any configuration 
   * - | data coordinates
     - | \ :math:`{\mathbf{x}} = [x_0,\, \ldots,\, x_l]\tp`\ 
     - | vector of \ :math:`l`\  data coordinates in any configuration 


The following parameters represent possible \ **OutputVariable**\  (list is not complete): 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **python name**\ 
     - | \ **symbol**\ 
     - | \ **description**\  
   * - | Coordinate
     - | \ :math:`{\mathbf{c}} = [c_0,\, \ldots,\, c_n]\tp`\ 
     - | coordinate vector with \ :math:`n`\  generalized coordinates \ :math:`c_i`\  in any configuration; the letter \ :math:`c`\  is used both for \ :ref:`ODE1 <ODE1>`\  and \ :ref:`ODE2 <ODE2>`\  coordinates 
   * - | Coordinate_t
     - | \ :math:`\dot {\mathbf{c}} = [c_0,\, \ldots,\, c_n]\tp`\ 
     - | time derivative of coordinate vector 
   * - | Displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}} = [u_0,\, u_1,\, u_2]\tp`\ 
     - | global displacement vector with 3 displacement coordinates \ :math:`u_i`\  in any configuration; in 1D or 2D objects, some of there coordinates may be zero 
   * - | Rotation
     - | \ :math:`[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig`\ 
     - | vector with 3 components of the Euler angles in xyz-sequence (\ :math:`\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)`\ ), recomputed from rotation matrix 
   * - | Rotation (alt.)
     - | \ :math:`\ttheta = [\theta_0,\, \ldots,\, \theta_n]\tp`\ 
     - | vector of \ **rotation parameters**\  (e.g., Euler parameters, Tait Bryan angles, ...) with \ :math:`n`\  coordinates \ :math:`\theta_i`\  in any configuration 
   * - | Identity matrix
     - | \ :math:`{\mathbf{I}} = \mr{1}{0}{0} {0}{\ddots}{0} {0}{0}{1}`\ 
     - | the identity matrix, very often \ :math:`{\mathbf{I}} = \mathbf{I}_{3 \times 3}`\ , the \ :math:`3 \times 3`\  identity matrix 
   * - | Identity transformation
     - | \ :math:`\LU{0b}{\mathbf{I}_{3 \times 3}} = \mathbf{I}_{3 \times 3}`\ 
     - | converts body-fixed into global coordinates, e.g., \ :math:`\LU{0}{{\mathbf{x}}} = \LU{0b}{\mathbf{I}_{3 \times 3}} \LU{b}{{\mathbf{x}}}`\ , thus resulting in \ :math:`\LU{0}{{\mathbf{x}}} = \LU{b}{{\mathbf{x}}}`\  in this case 
   * - | RotationMatrix
     - | \ :math:`\LU{0b}{\Rot} = \mr{A_{00}}{A_{01}}{A_{02}} {A_{10}}{A_{11}}{A_{12}} {A_{20}}{A_{21}}{A_{22}}`\ 
     - | a 3D rotation matrix, which transforms local (e.g., body \ :math:`b`\ ) to global coordinates (0): \ :math:`\LU{0}{{\mathbf{x}}} = \LU{0b}{\Rot} \LU{b}{{\mathbf{x}}}`\  
   * - | RotationMatrixX
     - | \ :math:`\LU{01}{\Rot_0(\theta_0)} = \mr{1}{0}{0} {0}{\cos(\theta_0)}{-\sin(\theta_0)} {0}{\sin(\theta_0)}{\cos(\theta_0)}`\ 
     - | rotation matrix for rotation around \ :math:`X`\  axis (axis 0), transforming a vector from frame 1 to frame 0 
   * - | RotationMatrixY
     - | \ :math:`\LU{01}{\Rot_1(\theta_1)} = \mr{\cos(\theta_1)}{0}{\sin(\theta_1)} {0}{1}{0} {-\sin(\theta_1)}{0}{\cos(\theta_1)}`\ 
     - | rotation matrix for rotation around \ :math:`Y`\  axis (axis 1), transforming a vector from frame 1 to frame 0 
   * - | RotationMatrixZ
     - | \ :math:`\LU{01}{\Rot_2(\theta_2)} = \mr{\cos(\theta_2)}{-\sin(\theta_2)}{0} {\sin(\theta_2)}{\cos(\theta_2)}{0} {0}{0}{1}`\ 
     - | rotation matrix for rotation around \ :math:`Z`\  axis (axis 2), transforming a vector from frame 1 to frame 0 
   * - | Position
     - | \ :math:`\LU{0}{{\mathbf{p}}} = [p_0,\, p_1,\, p_2]\tp`\ 
     - | global position vector with 3 position coordinates \ :math:`p_i`\  in any configuration 
   * - | Velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}} = \LU{0}{\dot {\mathbf{u}}} = [v_0,\, v_1,\, v_2]\tp`\ 
     - | global velocity vector with 3 displacement coordinates \ :math:`v_i`\  in any configuration 
   * - | AngularVelocity
     - | \ :math:`\LU{0}{\tomega} = [\omega_0,\, \ldots,\, \omega_2]\tp`\ 
     - | global angular velocity vector with \ :math:`3`\  coordinates \ :math:`\omega_i`\  in any configuration 
   * - | Acceleration
     - | \ :math:`\LU{0}{{\mathbf{a}}} = \LU{0}{\ddot {\mathbf{u}}} = [a_0,\, a_1,\, a_2]\tp`\ 
     - | global acceleration vector with 3 displacement coordinates \ :math:`a_i`\  in any configuration 
   * - | AngularAcceleration
     - | \ :math:`\LU{0}{\talpha} = \LU{0}{\dot \tomega} = [\alpha_0,\, \ldots,\, \alpha_2]\tp`\ 
     - | global angular acceleration vector with \ :math:`3`\  coordinates \ :math:`\alpha_i`\  in any configuration 
   * - | VelocityLocal
     - | \ :math:`\LU{b}{{\mathbf{v}}} = [v_0,\, v_1,\, v_2]\tp`\ 
     - | local (body-fixed) velocity vector with 3 displacement coordinates \ :math:`v_i`\  in any configuration 
   * - | AngularVelocityLocal
     - | \ :math:`\LU{b}{\tomega} = [\omega_0,\, \ldots,\, \omega_2]\tp`\ 
     - | local (body-fixed) angular velocity vector with \ :math:`3`\  coordinates \ :math:`\omega_i`\  in any configuration 
   * - | Force
     - | \ :math:`\LU{0}{{\mathbf{f}}} = [f_0,\, \ldots,\, f_2]\tp`\ 
     - | vector of \ :math:`3`\  force components in global coordinates 
   * - | Torque
     - | \ :math:`\LU{0}{\ttau} = [\tau_0,\, \ldots,\, \tau_2]\tp`\ 
     - | vector of \ :math:`3`\  torque components in global coordinates 


The following table collects some typical \ **input parameters**\  for nodes, objects and markers: 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **python name**\ 
     - | \ **symbol**\ 
     - | \ **description**\  
   * - | referenceCoordinates
     - | \ :math:`{\mathbf{c}}\cRef = [c_0,\, \ldots,\, c_n]\cRef\tp = [c_{\mathrm{Ref},0},\, \ldots,\, c_{\mathrm{Ref},n}]\cRef\tp`\ 
     - | \ :math:`n`\  coordinates of reference configuration (can usually be set at initialization of nodes) 
   * - | initialCoordinates
     - | \ :math:`{\mathbf{c}}\cIni`\ 
     - | initial coordinates with generalized or mixed displacement/rotation quantities (can usually be set at initialization of nodes) 
   * - | reference point
     - | \ :math:`\pRefG = [r_0,\, r_1,\, r_2]\tp`\ 
     - | reference point of body, e.g., for rigid bodies or \ :ref:`FFRF <FFRF>`\  bodies, in any configuration; NOTE: for ANCF elements, \ :math:`\pRefG`\  is used for the position vector to the beam centerline 
   * - | localPosition
     - | \ :math:`\pLocB = [\LUR{b}{b}{0},\, \LUR{b}{b}{1},\, \LUR{b}{b}{2}]\tp`\ 
     - | local (body-fixed) position vector with 3 position coordinates \ :math:`b_i`\  in any configuration, measured relative to reference point; NOTE: for rigid bodies, \ :math:`\LU{0}{{\mathbf{p}}} = \pRefG + \LU{0b}{\Rot} \pLocB`\ ; localPosition is used for definition of body-fixed local position of markers, sensors, COM, etc. 



