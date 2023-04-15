


**********************************
Structures for structural elements
**********************************

This section includes data structures for structural elements, such as beams (and plates in future). These classes are used as interface between Python libraries for structural elements and Exudyn internal classes.


.. _sec-pybeamsection:

PyBeamSection
-------------

Data structure for definition of 2D and 3D beam (cross) section mechanical properties. The beam has local coordinates, in which \ :math:`X`\  represents the beam centerline (beam axis) coordinate, being the neutral fiber w.r.t.\ bending; \ :math:`Y`\  and \ :math:`Z`\  are the local cross section coordinates. Note that most elements do not accept all parameters, which results in an error if those parameters (e.g., stiffness parameters) are non-zero.

PyBeamSection has the following items:

* | **dampingMatrix** [type = Matrix6D, default = np.zeros((6,6))]:
  | \ :math:`\LU{c}{{\mathbf{D}}} \in \Rcal^{6 \times 6}\,`\  [SI:Nsm\ :math:`^2`\ , Nsm and Ns (mixed)] sectional linear damping matrix related to \ :math:`\vp{\LU{c}{{\mathbf{n}}}}{\LU{c}{{\mathbf{m}}}} = \LU{c}{{\mathbf{D}}} \vp{\LU{c}{\tepsDot}}{\LU{c}{\tkappaDot}}`\ ; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior. Set with list of lists or numpy array.
* | **inertia** [type = Matrix3D, default = [[0,0,0], [0,0,0], [0,0,0]]]:
  | \ :math:`\LU{c}{{\mathbf{J}}} \in \Rcal^{3 \times 3}\,`\  [SI:kg\ :math:`\,`\ m\ :math:`^2`\ ] sectional inertia for shear-deformable beams. Set with list of lists or numpy array.
* | **massPerLength** [type = UReal, default = 0.]:
  | \ :math:`\rho A\,`\  [SI:kg/m] mass per unit length of the beam
* | **stiffnessMatrix** [type = Matrix6D, default = np.zeros((6,6))]:
  | \ :math:`\LU{c}{{\mathbf{C}}} \in \Rcal^{6 \times 6}\,`\  [SI:Nm\ :math:`^2`\ , Nm and N (mixed)] sectional stiffness matrix related to \ :math:`\vp{\LU{c}{{\mathbf{n}}}}{\LU{c}{{\mathbf{m}}}} = \LU{c}{{\mathbf{C}}} \vp{\LU{c}{\teps}}{\LU{c}{\tkappa}}`\  with sectional normal force \ :math:`\LU{c}{{\mathbf{n}}}`\ , torque \ :math:`\LU{c}{{\mathbf{m}}}`\ , strain \ :math:`\LU{c}{\teps}`\  and curvature \ :math:`\LU{c}{\tkappa}`\ , all quantities expressed in the cross section frame \ :math:`c`\ . Set with list of lists or numpy array.

