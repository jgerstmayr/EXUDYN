#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Exudyn: the shared declarations of the functions that items override
#
# Details:  HAND-WRITTEN after the first generation. An item does not restate a declaration it
#           shares with other items - it writes ItemFunctionDef(...) and the declaration comes
#           from here. 162 entries stand for 1440 function declarations in the item files.
#
#           An entry is found by the class the function belongs to - its classType and its
#           cParentClass - plus the function name. BOTH of those are None wherever the
#           declaration does not actually depend on them, which is the common case: a
#           parentClass is named only where the parents genuinely disagree, and a classType
#           only where the item types do. Where a name is still not unique - the const and
#           non-const halves of an accessor pair, the two argument lists of
#           ComputeAlgebraicEquations, a name used for both a computation and a visualization
#           function - the use site names the destination, the flags or the args, and a
#           missing one is an error listing the alternatives rather than a silent pick.
#
#           The description lives here too and an item overrides it only where the text is
#           genuinely item-specific. A function description reaches only the generated C++
#           header as a //! AUTO: comment - it appears in no .rst, no .tex and not in
#           itemInterface.py - so unifying the wording is safe.
#
#           NOT derived from the C++ base headers: 1067 of 1507 computational item functions
#           match a base signature exactly, but 169 differ in the PARAMETER NAMES alone
#           (ComputeMassMatrix takes massMatrixC here and massMatrix there), so deriving would
#           change the generated headers. The base header is a CHECK instead - see plan step 32.
#
# Author:   Johannes Gerstmayr
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *


#**************************************************************************************************
#the declarations that are the same for EVERY item type
#**************************************************************************************************
sharedFunctions = [
    ItemFunctionLib(pythonName='CheckPreAssembleConsistency',
        type=TBool, destination=DestMain,
        cFlags=CFConst,
        args='const MainSystem& mainSystem, STDstring& errorString',
        description='Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail'),

    ItemFunctionLib(pythonName='GetAlgebraicEquationsSize',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='number of algebraic equations; independent of node/body coordinates'),

    ItemFunctionLib(pythonName='GetMarkerNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='get according marker number where load is applied'),

    ItemFunctionLib(pythonName='GetRequestedMarkerType',
        type='Marker::Type', destination=DestComp,
        cFlags=CFConst,
        description='provide requested markerType for connector'),

    ItemFunctionLib(pythonName='UpdateGraphics',
        type=Tvoid, destination=DestVisu,
        args='const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber',
        description='Update visualizationSystem -> graphicsData for item; index shows item Number in CData'),

    ]


#**************************************************************************************************
#the declarations for nodes
#**************************************************************************************************
nodeFunctions = [
    ItemFunctionLib(classType='Node', pythonName='CollectCurrentNodeData1',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ConstSizeMatrix<maxRotationCoordinates * nDim3D>& Glocal, Vector3D& angularVelocityLocal',
        description='provide nodal values efficiently for rigid body computation'),

    ItemFunctionLib(classType='Node', pythonName='CollectCurrentNodeMarkerData',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ConstSizeMatrix<maxRotationCoordinates * nDim3D>& Glocal, ConstSizeMatrix<maxRotationCoordinates * nDim3D>& G, Vector3D& pos, Vector3D& vel, Matrix3D& A, Vector3D& angularVelocityLocal',
        description='obtain G matrices, position, velocity, rotation matrix A (local to global), local angular velocity '),

    ItemFunctionLib(classType='Node', pythonName='CompositionRule',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const LinkedDataVector& currentPosition, const LinkedDataVector& currentOrientation, const Vector6D& incrementalMotion, LinkedDataVector& newPosition, LinkedDataVector& newOrientation',
        description='apply composition rule for all nodal coordinates'),

    ItemFunctionLib(classType='Node', pythonName='ComputeAlgebraicEquations',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& algebraicEquations, bool useIndex2 = false',
        description=r"""ONLY for nodes with \hac{AE} / Euler parameters: compute algebraic equations to 'algebraicEquations', which has dimension GetNumberOfAECoordinates();"""),

    ItemFunctionLib(classType='Node', pythonName='ComputeJacobianAE',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, JacobianType::Type& filledJacobians',
        description=r"""ONLY for nodes with \hac{AE} / Euler parameters: compute algebraic equations to 'algebraicEquations', which has dimension GetNumberOfAECoordinates();"""),

    ItemFunctionLib(classType='Node', pythonName='GetAcceleration',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent acceleration of node'),

    ItemFunctionLib(classType='Node', pythonName='GetAngularAcceleration',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent angular acceleration of node'),

    ItemFunctionLib(classType='Node', pythonName='GetAngularVelocity',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),

    ItemFunctionLib(classType='Node', pythonName='GetAngularVelocityLocal',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector'),

    ItemFunctionLib(classType='Node', pythonName='GetG',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current',
        description='Compute G matrix (=diff(angularVelocity, velocityParameters)) for given configuration'),

    ItemFunctionLib(classType='Node', pythonName='GetGTv_q',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& v, ConstSizeMatrix<maxRotationCoordinates * maxRotationCoordinates>& matrix, ConfigurationType configuration = ConfigurationType::Current',
        description='compute d(G^T*v)/dq for any set of parameters; needed for jacobians'),

    ItemFunctionLib(classType='Node', pythonName='GetG_t',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current',
        description='Compute G matrix (=diff(angularVelocity, velocityParameters)) for given configuration'),

    ItemFunctionLib(classType='Node', pythonName='GetGlobalAECoordinateIndex',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='read access function needed by system for algebraic coordinate'),

    ItemFunctionLib(classType='Node', pythonName='GetGlocal',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current',
        description='Compute local G matrix for given configuration'),

    ItemFunctionLib(classType='Node', pythonName='GetGlocalTv_q',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& v, ConstSizeMatrix<maxRotationCoordinates * maxRotationCoordinates>& matrix, ConfigurationType configuration = ConfigurationType::Current',
        description='compute d(Glocal^T*v)/dq for any set of parameters; needed for jacobians'),

    ItemFunctionLib(classType='Node', pythonName='GetGlocal_t',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current',
        description='Compute local G matrix for given configuration'),

    ItemFunctionLib(classType='Node', pythonName='GetInitialCoordinateVector',
        type=TLinkedDataVector, destination=DestMain,
        cFlags=CFConst,
        description='return internally stored initial coordinates (displacements) of node'),

    ItemFunctionLib(classType='Node', pythonName='GetInitialCoordinateVector_t',
        type=TLinkedDataVector, destination=DestMain,
        cFlags=CFConst,
        description='return internally stored initial coordinates (velocities) of node'),

    ItemFunctionLib(classType='Node', pythonName='GetNodeGroup',
        type=TCNodeGroup, destination=DestComp,
        cFlags=CFConst,
        description='return node group, which is special because of algebraic equations'),

    ItemFunctionLib(classType='Node', pythonName='GetNumberOfAECoordinates',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='return number of second order diff. eq. coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetNumberOfDataCoordinates',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='return number of data coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetNumberOfDisplacementCoordinates',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='return number of displacement coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetNumberOfODE1Coordinates',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='return number of second order diff. eq. coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetNumberOfODE2Coordinates',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='return number of second order diff. eq. coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetNumberOfRotationCoordinates',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='return number of rotation coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetOutputVariable',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='OutputVariableType variableType, ConfigurationType configuration, Vector& value',
        description="provide according output variable in 'value'; used e.g. for postprocessing and sensors"),

    ItemFunctionLib(classType='Node', pythonName='GetPosition',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent position of node; returns always a 3D Vector'),

    ItemFunctionLib(classType='Node', pythonName='GetPositionJacobian',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Matrix& value',
        description=r"""provide position jacobian $\Jm_P$ of node; derivative of global 3D position with respect to all nodal coordiantes; action of force: $\Qm_f = \Jm_P^T \fv$; zero-size matrix for ground node (no action)"""),

    ItemFunctionLib(classType='Node', pythonName='GetReferenceCoordinateVector',
        type=TLinkedDataVector, destination=DestComp,
        cFlags=CFConst,
        description='return internally stored reference coordinates of node'),

    ItemFunctionLib(classType='Node', pythonName='GetRotationJacobian',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Matrix& value',
        description=r"""provide 'rotation' jacobian $\Jm_R$ of node; derivative of global 3D angular velocity vector with respect to all velocity coordinates ('G-matrix'); action of torque $\mv$: $\Qm_m = \Jm_R^T \mv$"""),

    ItemFunctionLib(classType='Node', pythonName='GetRotationJacobianTTimesVector_q',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& vector, Matrix& jacobian_q',
        description='provide derivative w.r.t. coordinates of rotation Jacobian times vector; for current configuration'),

    ItemFunctionLib(classType='Node', pythonName='GetRotationMatrix',
        type=TMatrixND(3, 3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent rotation matrix of node; returns always a 3D matrix'),

    ItemFunctionLib(classType='Node', pythonName='GetRotationParameters',
        type='ConstSizeVector<maxRotationCoordinates>', destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='Compute vector to of 4 Euler Parameters from reference and configuration coordinates'),

    ItemFunctionLib(classType='Node', pythonName='GetRotationParameters_t',
        type=TLinkedDataVector, destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='Compute vector to time derivative of 4 Euler Parameters in given configuration'),

    ItemFunctionLib(classType='Node', pythonName='GetVelocity',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent velocity of node; returns always a 3D Vector'),

    ItemFunctionLib(classType='Node', pythonName='SetGlobalAECoordinateIndex',
        type=Tvoid, destination=DestComp,
        args='Index globalIndex',
        description='write access function needed by system for algebraic coordinate'),

    ]


#**************************************************************************************************
#the declarations for objects
#**************************************************************************************************
objectFunctions = [
    ItemFunctionLib(classType='Object', pythonName='AddALEvariation',
        type=Tbool, destination=DestComp,
        cFlags=CFConst,
        description='access to physicsAddALEvariation'),

    ItemFunctionLib(classType='Object', pythonName='CallUserFunction',
        type=Tvoid, destination=DestVisu,
        args='const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber',
        description='user function which is called to update specific object graphics computed in Python functions; this is rather slow, but useful for user elements'),

    ItemFunctionLib(classType='Object', pythonName='ComputeJacobianForce6D',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const MarkerDataStructure& markerData, Index objectNumber, Vector6D& force6D',
        description='compute global 6D force and torque which is used for computation of derivative of jacobian; used only in combination with ComputeJacobianODE2_ODE2'),

    ItemFunctionLib(classType='Object', pythonName='ComputeMassMatrix',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber, bool computeInverse=false',
        description='Computational function: compute mass matrix'),

    ItemFunctionLib(classType='Object', pythonName='ComputeRigidBodyMarkerData',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, bool computeJacobian, MarkerData& markerData',
        description='accelerator function for faster computation of MarkerData for rigid bodies/joints'),

    ItemFunctionLib(classType='Object', pythonName='GetAccessFunction',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='AccessFunctionType accessType, Matrix& value',
        description="provide Jacobian at localPosition in 'value' according to object access"),

    ItemFunctionLib(classType='Object', pythonName='GetAccessFunctionBody',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value',
        description="provide Jacobian at localPosition in 'value' according to object access"),

    ItemFunctionLib(classType='Object', pythonName='GetAccessFunctionSuperElement',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='AccessFunctionType accessType, const Matrix& weightingMatrix, const ArrayIndex& meshNodeNumbers, const Vector3D& localOffset, Matrix& value, const Matrix3D& rotTangentCorrection',
        description='compute Jacobian with weightingMatrix (WM) and/or meshNodeNumbers, which define how the SuperElement mesh nodes or coordinates are transformed to a global position; for details see CObjectSuperElement header file'),

    ItemFunctionLib(classType='Object', pythonName='GetAccessFunctionTypes',
        type=TAccessFunctionType, destination=DestComp,
        cFlags=CFConst,
        description='Flags to determine, which access (forces, moments, connectors, ...) to object are possible'),

    ItemFunctionLib(classType='Object', pythonName='GetAngularVelocity',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),

    ItemFunctionLib(classType='Object', pythonName='GetAngularVelocityLocal',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector'),

    ItemFunctionLib(classType='Object', pythonName='GetAvailableJacobians',
        type='JacobianType::Type', destination=DestComp,
        cFlags=CFConst,
        description='return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags'),

    ItemFunctionLib(classType='Object', pythonName='GetDataVariablesSize',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='data (history) variable simplifies contact switching for implicit time integration and Newton method'),

    ItemFunctionLib(classType='Object', pythonName='GetDisplacement',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
        description="return the (global) displacement of 'localPosition' according to configuration type"),

    ItemFunctionLib(classType='Object', pythonName='GetLength',
        type=TReal, destination=DestComp,
        cFlags=CFConst,
        description='access to individual element paramters for base class functions'),

    ItemFunctionLib(classType='Object', pythonName='GetLocalCenterOfMass',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        description='return the local position of the center of mass, needed for equations of motion and for massProportionalLoad'),

    ItemFunctionLib(classType='Object', pythonName='GetLocalODE2CoordinateIndexPerNode',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        args='Index localNode',
        description='read access to coordinate index array'),

    ItemFunctionLib(classType='Object', pythonName='GetMassPerLength',
        type=TReal, destination=DestComp,
        cFlags=CFConst,
        description='access to individual element paramters for base class functions'),

    ItemFunctionLib(classType='Object', pythonName='GetMaterialParameters',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Real& physicsBendingStiffness, Real& physicsAxialStiffness, Real& physicsBendingDamping, Real& physicsAxialDamping, Real& physicsReferenceAxialStrain, Real& physicsReferenceCurvature, Real& physicsMovingMassFactor',
        description='access to individual element paramters for base class functions'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNode',
        type='CNodeODE2*', destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber',
        description='return the mesh node pointer; for consistency checks'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNodeAcceleration',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current',
        description='return the (global) acceleration of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNodeLocalAcceleration',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current',
        description='return the (local) acceleration of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNodeLocalPosition',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current',
        description='return the (local) position of a mesh node according to configuration type; use Configuration.Reference to access the mesh reference position; meshNodeNumber is the local node number of the (underlying) mesh'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNodeLocalVelocity',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current',
        description='return the (local) velocity of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNodePosition',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current',
        description='return the (global) position of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh'),

    ItemFunctionLib(classType='Object', pythonName='GetMeshNodeVelocity',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current',
        description='return the (global) velocity of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh'),

    ItemFunctionLib(classType='Object', pythonName='GetNodeNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        args='Index localIndex',
        description='Get global node number (with local node index); needed for every object ==> does local mapping'),

    ItemFunctionLib(classType='Object', pythonName='GetNumberOfNodes',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='number of nodes; needed for every object; can depend on the configuration'),

    ItemFunctionLib(classType='Object', pythonName='GetODE1Size',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description=r'number of \hac{ODE1} coordinates; needed for object?'),

    ItemFunctionLib(classType='Object', pythonName='GetODE2Size',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description=r'number of \hac{ODE2} coordinates; needed for object?'),

    ItemFunctionLib(classType='Object', pythonName='GetOutputVariable',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='OutputVariableType variableType, Vector& value, ConfigurationType configuration, Index objectNumber',
        description="provide according output variable in 'value'"),

    ItemFunctionLib(classType='Object', pythonName='GetOutputVariableBody',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber',
        description="provide according output variable in 'value'"),

    ItemFunctionLib(classType='Object', pythonName='GetOutputVariableConnector',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value',
        description="provide according output variable in 'value'"),

    ItemFunctionLib(classType='Object', pythonName='GetOutputVariableSuperElement',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='OutputVariableType variableType, Index meshNodeNumber, ConfigurationType configuration, Vector& value',
        description='get extended output variables for multi-nodal objects with mesh nodes'),

    ItemFunctionLib(classType='Object', pythonName='GetOutputVariableTypes',
        type=TOutputVariableType, destination=DestComp,
        cFlags=CFConst,
        description='Flags to determine, which output variables are available (displacment, velocity, stress, ...)'),

    ItemFunctionLib(classType='Object', pythonName='GetOutputVariableTypesSuperElement',
        type=TOutputVariableType, destination=DestComp,
        cFlags=CFConst,
        args='Index meshNodeNumber',
        description='get extended output variable types for multi-nodal objects with mesh nodes; some objects have meshNode-dependent OutputVariableTypes'),

    ItemFunctionLib(classType='Object', pythonName='GetPosition',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
        description="return the (global) position of 'localPosition' according to configuration type"),

    ItemFunctionLib(classType='Object', pythonName='GetRequestedNodeType',
        type='Node::Type', destination=DestMain,
        cFlags=CFConst,
        description='provide requested nodeType for objects; used for automatic checks in CheckSystemIntegrity(); where no exact type can be given, a generic type is used and the check is done in CheckPreAssembleConsistency(...)'),

    ItemFunctionLib(classType='Object', pythonName='GetRotationMatrix',
        type=TMatrixND(3, 3), destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent rotation matrix of node; returns always a 3D matrix'),

    ItemFunctionLib(classType='Object', pythonName='GetVelocity',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
        description="return the (global) velocity of 'localPosition' according to configuration type"),

    ItemFunctionLib(classType='Object', pythonName='HasConstantMassMatrix',
        type=Tbool, destination=DestComp,
        cFlags=CFConst,
        description='return true if object has time and coordinate independent (=constant) mass matrix'),

    ItemFunctionLib(classType='Object', pythonName='HasDiscontinuousIteration',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='flag to be set for connectors, which use DiscontinuousIteration'),

    ItemFunctionLib(classType='Object', pythonName='HasForceUserFunction',
        type=Tbool, destination=DestComp,
        cFlags=CFConst,
        description='return true if object has force user function'),

    ItemFunctionLib(classType='Object', pythonName='HasReferenceFrame',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        args='Index& localReferenceFrameNode',
        description='return true, if object has reference frame; return according LOCAL node number'),

    ItemFunctionLib(classType='Object', pythonName='HasTorqueUserFunction',
        type=Tbool, destination=DestComp,
        cFlags=CFConst,
        description='return true if object has force user function'),

    ItemFunctionLib(classType='Object', pythonName='IsActive',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='return if connector is active-->speeds up computation'),

    ItemFunctionLib(classType='Object', pythonName='IsConnector',
        type=TBool, destination=DestVisu,
        cFlags=CFConst,
        description='this function is needed to distinguish connector objects from body objects'),

    ItemFunctionLib(classType='Object', pythonName='IsPenaltyConnector',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers'),

    ItemFunctionLib(classType='Object', pythonName='IsTimeDependent',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='connector is time dependent if user functions are defined'),

    ItemFunctionLib(classType='Object', pythonName='ParametersHaveChanged',
        type=Tvoid, destination=DestComp,
        description='This flag is reset upon change of parameters; says that mass matrix (future: other pre-computed values) need to be recomputed'),

    ItemFunctionLib(classType='Object', pythonName='PostAssemble',
        type=Tvoid, destination=DestComp,
        description='operations done after Assemble()'),

    ItemFunctionLib(classType='Object', pythonName='PostDiscontinuousIterationStep',
        type=Tvoid, destination=DestComp,
        description='function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)'),

    ItemFunctionLib(classType='Object', pythonName='PostNewtonStep',
        type=TReal, destination=DestComp,
        args='const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize',
        description='function called after Newton method; returns a residual error (force)'),

    ItemFunctionLib(classType='Object', pythonName='PreComputeMassTerms',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        description='precompute mass terms if it has not been done yet'),

    ItemFunctionLib(classType='Object', pythonName='RequestedNumberOfMarkers',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='number of markers; 0 means no requirements'),

    ItemFunctionLib(classType='Object', pythonName='SetNodeNumber',
        type=Tvoid, destination=DestComp,
        args='Index localIndex, Index nodeNumber',
        description='Get global node number (with local node index); needed for every object ==> does local mapping'),

    ItemFunctionLib(classType='Object', pythonName='StrainIsRelativeToReference',
        type=TReal, destination=DestComp,
        cFlags=CFConst,
        description='access to strainIsRelativeToReference from derived class'),

    ItemFunctionLib(classType='Object', pythonName='UseReducedOrderIntegration',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='access to useReducedOrderIntegration from derived class'),

    ItemFunctionLib(classType='Object', pythonName='UsesVelocityLevel',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='Return true, if constraint currently is formulated at velocity level (e.g. coordinate constraint ==> this information is needed for correct jacobian computation)'),

    ItemFunctionLib(classType='Object', parentClass='CObject', pythonName='ComputeODE1RHS',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& ode1Rhs, Index objectNumber',
        description="Computational function: compute right-hand-side (RHS) of first order ordinary differential equations (ODE) to 'ode1Rhs'"),

    ItemFunctionLib(classType='Object', parentClass='CObject', pythonName='HasUserFunction',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='return true, if object has a computation user function'),

    ItemFunctionLib(classType='Object', parentClass='CObjectANCFCable2DBase', pythonName='ComputeODE2LHS',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& ode2Lhs, Index objectNumber',
        description="Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),

    ItemFunctionLib(classType='Object', parentClass='CObjectBody', pythonName='ComputeAlgebraicEquations',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& algebraicEquations, bool useIndex2 = false',
        description='Compute algebraic equations part of rigid body'),

    ItemFunctionLib(classType='Object', parentClass='CObjectBody', pythonName='ComputeJacobianAE',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE',
        description=r"""Compute jacobians of algebraic equations part of rigid body w.r.t. \hac{ODE2}, \hac{ODE2t}, \hac{ODE1}, \hac{AE}"""),

    ItemFunctionLib(classType='Object', parentClass='CObjectBody', pythonName='ComputeJacobianODE2_ODE2',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg',
        description='Computational function: compute jacobian (dense or sparse mode, see parent CObject function)'),

    ItemFunctionLib(classType='Object', parentClass='CObjectBody', pythonName='ComputeODE2LHS',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& ode2Lhs, Index objectNumber',
        description="Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),

    ItemFunctionLib(classType='Object', parentClass='CObjectBody', pythonName='HasUserFunction',
        type=TBool, destination=DestVisu,
        cFlags=CFConst,
        description='return true, if object has a user function to be called during redraw'),

    ItemFunctionLib(classType='Object', parentClass='CObjectConnector', pythonName='ComputeJacobianODE2_ODE2',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg, const MarkerDataStructure& markerData',
        description=r"""Computational function: compute Jacobian of \hac{ODE2} \ac{LHS} equations w.r.t. ODE2 coordinates and ODE2 velocities; write either dense local jacobian into dense matrix of MatrixContainer or ADD sparse triplets INCLUDING ltg mapping to sparse matrix of MatrixContainer"""),

    ItemFunctionLib(classType='Object', parentClass='CObjectConnector', pythonName='ComputeODE1RHS',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& ode1Rhs, const MarkerDataStructure& markerData, Index objectNumber',
        description="Computational function: compute right-hand-side (RHS) of first order ordinary differential equations (ODE) to 'ode1Rhs'"),

    ItemFunctionLib(classType='Object', parentClass='CObjectConnector', pythonName='ComputeODE2LHS',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber',
        description="Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),

    ItemFunctionLib(classType='Object', parentClass='CObjectConnector', pythonName='GetMarkerNumbers',
        type='ArrayIndex&', destination=DestComp,
        description='default (write) function to return Marker numbers'),

    ItemFunctionLib(classType='Object', parentClass='CObjectConnector', pythonName='GetMarkerNumbers',
        type='const ArrayIndex&', destination=DestComp,
        cFlags=CFConst,
        description='default (read) function to return Marker numbers'),

    ItemFunctionLib(classType='Object', parentClass='CObjectConnector', pythonName='HasUserFunction',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='return true, if object has a computation user function'),

    ItemFunctionLib(classType='Object', parentClass='CObjectConstraint', pythonName='ComputeAlgebraicEquations',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t,  Index itemIndex, bool velocityLevel = false',
        description="Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level"),

    ItemFunctionLib(classType='Object', parentClass='CObjectConstraint', pythonName='ComputeAlgebraicEquations',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false',
        description="Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level"),

    ItemFunctionLib(classType='Object', parentClass='CObjectConstraint', pythonName='ComputeJacobianAE',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex',
        description=r"""compute derivative of algebraic equations w.r.t. \hac{ODE2}, \hac{ODE2t}, \hac{ODE1} and \hac{AE} coordinates in jacobian [flags ODE2_t_AE_function, AE_AE_function, etc. need to be set in GetAvailableJacobians()]; jacobianODE2[_t] has dimension GetAlgebraicEquationsSize() x GetODE2Size() ; q are the system coordinates; markerData provides according marker information to compute jacobians"""),

    ItemFunctionLib(classType='Object', parentClass='CObjectConstraint', pythonName='GetMarkerNumbers',
        type='ArrayIndex&', destination=DestComp,
        description='default (write) function to return Marker numbers'),

    ItemFunctionLib(classType='Object', parentClass='CObjectConstraint', pythonName='GetMarkerNumbers',
        type='const ArrayIndex&', destination=DestComp,
        cFlags=CFConst,
        description='default (read) function to return Marker numbers'),

    ItemFunctionLib(classType='Object', parentClass='CObjectConstraint', pythonName='HasUserFunction',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='return true, if object has a computation user function'),

    ItemFunctionLib(classType='Object', parentClass='CObjectSuperElement', pythonName='ComputeJacobianODE2_ODE2',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg',
        description='Computational function: compute jacobian (dense or sparse mode, see parent CObject function)'),

    ItemFunctionLib(classType='Object', parentClass='CObjectSuperElement', pythonName='ComputeODE2LHS',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='Vector& ode2Lhs, Index objectNumber',
        description="Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),

    ItemFunctionLib(classType='Object', parentClass='CObjectSuperElement', pythonName='HasUserFunction',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='return true, if object has a computation user function'),

    ItemFunctionLib(classType='Object', parentClass='CObjectSuperElement', pythonName='HasUserFunction',
        type=TBool, destination=DestVisu,
        cFlags=CFConst,
        description='return true, if object has a user function to be called during redraw'),

    ]


#**************************************************************************************************
#the declarations for markers
#**************************************************************************************************
markerFunctions = [
    ItemFunctionLib(classType='Marker', pythonName='ComputeMarkerData',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData',
        description='Compute marker data (e.g. position and positionJacobian) for a marker'),

    ItemFunctionLib(classType='Marker', pythonName='ComputeMarkerDataJacobianDerivative',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, const Vector6D& v6D, MarkerData& markerData',
        description='fill in according data for derivative of jacobian times vector v6D, e.g.: d(Jpos.T @ v6D[0:3])/dq; v6D represents 3 force components and 3 torque components in global coordinates!'),

    ItemFunctionLib(classType='Marker', pythonName='GetAngularVelocity',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent angular velocity of node; returns always a 3D Vector'),

    ItemFunctionLib(classType='Marker', pythonName='GetAngularVelocityLocal',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector'),

    ItemFunctionLib(classType='Marker', pythonName='GetCoordinateNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='access to coordinate index'),

    ItemFunctionLib(classType='Marker', pythonName='GetDimension',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData',
        description='return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker'),

    ItemFunctionLib(classType='Marker', pythonName='GetNodeNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='access to node number'),

    ItemFunctionLib(classType='Marker', pythonName='GetNumberOfObjects',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='general access to object number'),

    ItemFunctionLib(classType='Marker', pythonName='GetObjectNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        args='Index localIndex = 0',
        description='general access to object number'),

    ItemFunctionLib(classType='Marker', pythonName='GetPosition',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current',
        description='return position of marker'),

    ItemFunctionLib(classType='Marker', pythonName='GetRotationMatrix',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, Matrix3D& rotationMatrix, ConfigurationType configuration = ConfigurationType::Current',
        description='return configuration dependent rotation matrix of node; returns always a 3D matrix'),

    ItemFunctionLib(classType='Marker', pythonName='GetVelocity',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, Vector3D& velocity, ConfigurationType configuration = ConfigurationType::Current',
        description='return velocity of marker'),

    ItemFunctionLib(classType='Marker', pythonName='PostNewtonStep',
        type=Tvoid, destination=DestComp,
        args='CSystemData& cSystemData, const MarkerData& markerData',
        description='Perform PostNewtonStep for marker (enable continuous rotation)'),

    ItemFunctionLib(classType='Marker', pythonName='SetNodeNumber',
        type=Tvoid, destination=DestComp,
        args='Index nodeNumber',
        description='change bodyNumber'),

    ItemFunctionLib(classType='Marker', parentClass='CMarker', pythonName='SetObjectNumber',
        type=Tvoid, destination=DestComp,
        args='Index bodyNumber, Index localIndex = 0',
        description='change bodyNumber'),

    ItemFunctionLib(classType='Marker', parentClass='CMarker', pythonName='SetObjectNumber',
        type=Tvoid, destination=DestComp,
        args='Index objectNumber, Index localIndex = 0',
        description='change bodyNumber'),

    ]


#**************************************************************************************************
#the declarations for loads
#**************************************************************************************************
loadFunctions = [
    ItemFunctionLib(classType='Load', pythonName='GetLoadValue',
        type=TReal, destination=DestComp,
        cFlags=CFConst,
        args='const MainSystemBase& mbs, Real t',
        description='read access for load value (IsVector=false)'),

    ItemFunctionLib(classType='Load', pythonName='GetLoadVector',
        type=TVectorND(3), destination=DestComp,
        cFlags=CFConst,
        args='const MainSystemBase& mbs, Real t',
        description='read access for load vector; returns user function result in case it is defined'),

    ItemFunctionLib(classType='Load', pythonName='HasUserFunction',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='tells system if loadFactor is used in static computation or if load is time dependent (assumed for any load user function)'),

    ItemFunctionLib(classType='Load', pythonName='IsBodyFixed',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='per default, forces/torques/... are applied in global coordinates; if IsBodyFixed()=true, the marker needs to provide a rotation (orientation) and forces/torques/... are applied in the local coordinate system'),

    ItemFunctionLib(classType='Load', pythonName='IsVector',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='true = load is of vector type'),

    ItemFunctionLib(classType='Load', pythonName='SetMarkerNumber',
        type=Tvoid, destination=DestComp,
        args='Index markerNumberInit',
        description='set according marker number where load is applied'),

    ]


#**************************************************************************************************
#the declarations for sensors
#**************************************************************************************************
sensorFunctions = [
    ItemFunctionLib(classType='Sensor', pythonName='GetFileName',
        type=TSTDstring, destination=DestComp,
        cFlags=CFConst,
        description='get file name'),

    ItemFunctionLib(classType='Sensor', pythonName='GetLoadNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='general access to load number'),

    ItemFunctionLib(classType='Sensor', pythonName='GetNodeNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='general access to node number'),

    ItemFunctionLib(classType='Sensor', pythonName='GetNumberOfSensors',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='total number of dependent sensors'),

    ItemFunctionLib(classType='Sensor', pythonName='GetObjectNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        description='general access to object number'),

    ItemFunctionLib(classType='Sensor', pythonName='GetOutputVariableType',
        type=TOutputVariableType, destination=DestComp,
        cFlags=CFConst,
        description='get OutputVariableType'),

    ItemFunctionLib(classType='Sensor', pythonName='GetSensorNumber',
        type=TIndex, destination=DestComp,
        cFlags=CFConst,
        args='Index localIndex',
        description='general access to sensor number'),

    ItemFunctionLib(classType='Sensor', pythonName='GetSensorValues',
        type=Tvoid, destination=DestComp,
        cFlags=CFConst,
        args='const CSystemData& cSystemData, Vector& values, ConfigurationType configuration = ConfigurationType::Current',
        description='main function to generate sensor output values'),

    ItemFunctionLib(classType='Sensor', pythonName='GetStoreInternalFlag',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='get storeInternal flag'),

    ItemFunctionLib(classType='Sensor', pythonName='GetWriteToFileFlag',
        type=TBool, destination=DestComp,
        cFlags=CFConst,
        description='get writeToFile flag'),

    ItemFunctionLib(classType='Sensor', pythonName='SetLoadNumber',
        type=Tvoid, destination=DestComp,
        args='Index loadNumber',
        description='change loadNumber'),

    ItemFunctionLib(classType='Sensor', pythonName='SetMarkerNumber',
        type=Tvoid, destination=DestComp,
        args='Index markerNumber',
        description='change markerNumber'),

    ItemFunctionLib(classType='Sensor', pythonName='SetNodeNumber',
        type=Tvoid, destination=DestComp,
        args='Index nodeNumber',
        description='change nodeNumber'),

    ItemFunctionLib(classType='Sensor', pythonName='SetSensorNumber',
        type=Tvoid, destination=DestComp,
        args='Index localIndex, Index sensorNumber',
        description='change sensorNumber'),

    ItemFunctionLib(classType='Sensor', parentClass='CSensor', pythonName='SetObjectNumber',
        type=Tvoid, destination=DestComp,
        args='Index bodyNumber',
        description='change bodyNumber'),

    ItemFunctionLib(classType='Sensor', parentClass='CSensor', pythonName='SetObjectNumber',
        type=Tvoid, destination=DestComp,
        args='Index objectNumber',
        description='change objectNumber'),

    ]


#the one list the generators read; the split above is for maintenance only
itemFunctionLibrary = (sharedFunctions
                       + nodeFunctions
                       + objectFunctions
                       + markerFunctions
                       + loadFunctions
                       + sensorFunctions
                       )
