#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Exudyn: output variable descriptions shared by several items
#
# Only texts that four or more items use verbatim are named here. Below that threshold a
# constant costs the reader more than the repetition does - naming all 69 repeated texts
# would save 130 lines but force a lookup to read any single description.
#
# Author:   Johannes Gerstmayr
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#used by 15 items
OVDZeroVectorForCompleteness = '$[0,0,0]$ (only for completeness)'

#used by 7 items
OVDCoordinatesTotalNode = 'displacement plus reference coordinates of node'

#used by 6 items
OVDRotationMatrixRowMajor = r'$[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp$vector with 9 components of the rotation matrix $\LU{0b}{\Rot}\cConfig$ in row-major format, in any configuration; the rotation matrix transforms local ($b$) to global (0) coordinates'

#used by 5 items
OVDIdentityMatrixForCompleteness = 'identity matrix (only for completeness)'

#used by 5 items
OVDAngularVelocityNode = r'$\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp$global 3D angular velocity vector of node'

#used by 5 items
OVDAngularVelocityLocalNode = r'$\LU{b}{\tomega}\cConfig = \LU{b}{[\omega_0,\,\omega_1,\,\omega_2]}\cConfig\tp$local (body-fixed)  3D angular velocity vector of node'

#used by 5 items
OVDPositionMarker0 = r'$\LU{0}{\pv}_{m0}$current global position of position marker $m0$'

#used by 5 items
OVDVelocityMarker0 = r'$\LU{0}{\vv}_{m0}$current global velocity of position marker $m0$'

#used by 4 items
OVDAccelerationNode = r'$\LU{0}{\av}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\cConfig\tp$global 3D acceleration vector of node'

#used by 4 items
OVDCoordinatesTotalNodeRotation = 'displacement/rotation coordinates of node including reference configuration'

#used by 4 items
OVDAngularVelocityBody = r'$\LU{0}{\tomega}\cConfig$global 3D angular velocity vector of body'

#used by 4 items
OVDAngularVelocityLocalBody = r'$\LU{b}{\tomega}\cConfig$local (body-fixed) 3D angular velocity vector of body'

#used by 4 items
OVDVelocityCoordinatesODE2 = r'all \hac{ODE2} velocity coordinates'

#used by 4 items
OVDGeneralizedForces = 'generalized forces for all coordinates (residual of all forces except mass*accleration; corresponds to ComputeODE2LHS)'

#used by 4 items
OVDVelocityLocalJoint = r'$\LU{J0}{\Delta\vv}$relative translational velocity in local joint0 coordinates'
