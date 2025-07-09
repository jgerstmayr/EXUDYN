
.. _sec-module-robotics-special:

Module: robotics.special
------------------------

additional support functions for robotics;
The library is built on Denavit-Hartenberg Parameters and
Homogeneous Transformations (HT) to describe transformations and coordinate systems

- Author:    Martin Sereinig 
- Date:      2021-22-09 


.. _sec-special-velocitymanipulability:

Function: VelocityManipulability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`VelocityManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L32>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``mode``\ )

- | \ *function description*\ :
  | compute velocity manipulability measure for given pose (homogeneous  transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``HT``\ : actual pose as homogeneous transformaton matrix
  | \ ``mode``\ : rotational or translational part of the movement
- | \ *output*\ :
  | velocity manipulability measure as scalar value, defined as \ :math:`\sqrt(det(JJ^T))`\
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | compute velocity dependent manipulability definded by Yoshikawa, see



----


.. _sec-special-forcemanipulability:

Function: ForceManipulability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ForceManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L62>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``mode``\ , \ ``singularWeight = 100``\ )

- | \ *function description*\ :
  | compute force manipulability measure for given pose (homogeneous  transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``HT``\ : actual pose as hoogenious transformaton matrix
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity, default value=100
  | \ ``mode``\ : rotational or translational part of the movement
- | \ *output*\ :
  | force manipulability measure as scalar value, defined as \ :math:`\sqrt((det(JJ^T))^{-1})`\
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | compute force dependent manipulability definded by Yoshikawa, see



----


.. _sec-special-stiffnessmanipulability:

Function: StiffnessManipulability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`StiffnessManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L106>`__\ (\ ``robot``\ , \ ``JointStiffness``\ , \ ``HT``\ , \ ``mode``\ , \ ``singularWeight = 1000``\ )

- | \ *function description*\ :
  | compute cartesian stiffness measure for given pose (homogeneous transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``JointStiffness``\ : joint stiffness matrix
  | \ ``HT``\ : actual pose as homogeneous transformaton matrix
  | \ ``mode``\ : rotational or translational part of the movement
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity,default value=1000
- | \ *output*\ :
  | stiffness manipulability measure as scalar value, defined as minimum Eigenvalaue of the Cartesian stiffness matrix
  | Cartesian stiffness matrix
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | 
- | \ *status*\ :
  | this function is \ **currently under development**\  and under testing!



----


.. _sec-special-jointjacobian:

Function: JointJacobian
^^^^^^^^^^^^^^^^^^^^^^^
`JointJacobian <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L149>`__\ (\ ``robot``\ , \ ``HTJoint``\ , \ ``HTLink``\ )

- | \ *function description*\ :
  | compute joint jacobian for each frame for given pose (homogeneous transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``HT``\ : actual pose as homogeneous transformaton matrix
- | \ *output*\ :
  | Link(body)-Jacobi matrix JJ: \ :math:`\LU{i}{JJ_i}=[\LU{i}{J_{Ri}},\; \LU{i}{J_{Ti}}]`\  for each link i, seperated in rotational (\ :math:`J_R`\ ) and translational (\ :math:`J_T`\ ) part of Jacobian matrix located in the \ :math:`i^{th}`\  coordiante system, see
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | runs over number of HTs given in HT (may be less than number of links), caclulations in link coordinate system located at the end of each link regarding Standard  Denavid-Hartenberg parameters, see



----


.. _sec-special-massmatrix:

Function: MassMatrix
^^^^^^^^^^^^^^^^^^^^
`MassMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L209>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``jointJacobian``\ )

- | \ *function description*\ :
  | compute mass matrix from jointJacobian
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : actual pose as homogeneous transformaton matrix
  | \ ``jointJacobian``\ : provide list of jacobians as provided by function JointJacobian(...)
- | \ *output*\ :
  | MM: Mass matrix
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | Mass Matrix calculation calculated in joint coordinates regarding (std) DH parameter:
  | \*\*       Dynamic equations in minimal coordinates as described in MehrkÃ¶rpersysteme by Woernle, , p206, eq6.90.
  | \*\*       Caclulations in link coordinate system at the end of each link



----


.. _sec-special-dynamicmanipulability:

Function: DynamicManipulability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`DynamicManipulability <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L243>`__\ (\ ``robot``\ , \ ``HT``\ , \ ``MassMatrix``\ , \ ``Tmax``\ , \ ``mode``\ , \ ``singularWeight = 1000``\ )

- | \ *function description*\ :
  | compute dynamic manipulability measure for given pose (homogeneous transformation)
- | \ *input*\ :
  | \ ``robot``\ : robot structure
  | \ ``HT``\ : actual pose as homogeneous transformaton matrix
  | \ ``Tmax``\ : maximum joint torques
  | \ ``mode``\ : rotational or translational part of the movement
  | \ ``MassMatrix``\ : Mass (inertia) Maxtrix provided by the function MassMatrix
  | \ ``singularWeight``\ : Weighting of singular configurations where the value would be infinity,default value=1000
- | \ *output*\ :
  | dynamic manipulability measure as scalar value, defined as minimum Eigenvalaue of the dynamic manipulability matrix N
  | dynamic manipulability matrix
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | acceleration dependent manipulability definded by Chiacchio, see , eq.32. The eigenvectors and eigenvalues of N ([eigenvec eigenval]=eig(N))gives the direction and value of minimal and maximal accaleration )
- | \ *status*\ :
  | this function is \ **currently under development**\  and under testing!



----


.. _sec-special-calculateallmeasures:

Function: CalculateAllMeasures
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CalculateAllMeasures <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/special.py\#L290>`__\ (\ ``robot``\ , \ ``robotDic``\ , \ ``q``\ , \ ``mode``\ , \ ``flag = [0,0,0,0]``\ )

- | \ *function description*\ :
  | calculation of 4 different manipulability measures using a certain serial robot
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``robotDic``\ : robot dictionary
  | \ ``q``\ : joint position vector
  | \ ``mode``\ : trans or rot, for used parts of the manipulator Jacobi Matrix
  | \ ``Tmax``\ : maximum joint torques
  | \ ``mode``\ : rotational or translational part of the movement
  | \ ``flag``\ : flag vector to swich individual measure on and of [flagmv,flagmf,flagmst,flagma] = [1,1,1,1]
- | \ *output*\ :
  | [mv,mf,mst,mstM,ma,maM]
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | 
- | \ *status*\ :
  | this function is \ **currently under development**\  and under testing!

