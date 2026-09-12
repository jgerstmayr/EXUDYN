
class MainSystem:
    @overload
    def SolutionViewer(self, solution=None, rowIncrement=1, timeout=0.04, runOnStart=True, runMode=2, fontSize=12, title='', checkRenderEngineStopFlag=True) -> None: 
        """open interactive dialog and visulation (animate) solution loaded with LoadSolutionFile(...); Change slider 'Increment' to change the automatic increment of time frames; Change mode between continuous run, one cycle (fits perfect for animation recording) or 'Static' (to change Solution steps manually with the mouse); update period also lets you change the speed of animation; Press Run / Stop button to start/stop interactive mode (updating of grpahics)."""
    ...

    @overload
    def CreateGround(self, name='', referencePosition=[0.,0.,0.], referenceRotationMatrix=np.eye(3), graphicsDataList=[], graphicsDataUserFunction=0, show=True) -> ObjectIndex: 
        """helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions."""
    ...

    @overload
    def CreateMassPoint(self, name='', referencePosition=[0.,0.,0.], initialDisplacement=[0.,0.,0.], initialVelocity=[0.,0.,0.], physicsMass=0, gravity=[0.,0.,0.], graphicsDataList=[], drawSize=-1, color=[-1.,-1.,-1.,-1.], show=True, create2D=False, returnDict=False) -> Union[dict, ObjectIndex]: 
        """helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint."""
    ...

    @overload
    def CreateRigidBody(self, name='', referencePosition=[0.,0.,0.], referenceRotationMatrix=np.eye(3), initialVelocity=[0.,0.,0.], initialAngularVelocity=[0.,0.,0.], initialDisplacement=None, initialRotationMatrix=None, inertia=None, gravity=[0.,0.,0.], nodeType=exudyn.NodeType.RotationEulerParameters, graphicsDataList=[], graphicsDataUserFunction=0, drawSize=-1, color=[-1.,-1.,-1.,-1.], show=True, create2D=False, returnDict=False) -> Union[dict, ObjectIndex]: 
        """helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.); use this function to easily create a rigid body; graphics can be directly obtained from inertia object, e.g. in case of cylindrical or cuboid shape."""
    ...

    @overload
    def CreateSpringDamper(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], referenceLength=None, stiffness=0., damping=0., force=0., velocityOffset=0., springForceUserFunction=0, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information."""
    ...

    @overload
    def CreateCartesianSpringDamper(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], stiffness=[0.,0.,0.], damping=[0.,0.,0.], offset=[0.,0.,0.], springForceUserFunction=0, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper."""
    ...

    @overload
    def CreateRigidBodySpringDamper(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], stiffness=np.zeros((6,6)), damping=np.zeros((6,6)), offset=[0.,0.,0.,0.,0.,0.], rotationMatrixJoint=np.eye(3), useGlobalFrame=True, intrinsicFormulation=True, springForceTorqueUserFunction=0, postNewtonStepUserFunction=0, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation."""
    ...

    @overload
    def CreateTorsionalSpringDamper(self, name='', bodyNumbers=[None, None], position=[0.,0.,0.], axis=[0.,0.,0.], stiffness=0., damping=0., offset=0., velocityOffset=0., torque=0., useGlobalFrame=True, springTorqueUserFunction=0, unlimitedRotations=True, show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation."""
    ...

    @overload
    def CreateRevoluteJoint(self, name='', bodyNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed."""
    ...

    @overload
    def CreatePrismaticJoint(self, name='', bodyNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed."""
    ...

    @overload
    def CreateSphericalJoint(self, name='', bodyNumbers=[None, None], position=[], constrainedAxes=[1,1,1], useGlobalFrame=True, show=True, jointRadius=0.1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed."""
    ...

    @overload
    def CreateGenericJoint(self, name='', bodyNumbers=[None, None], position=[], rotationMatrixAxes=np.eye(3), constrainedAxes=[1,1,1, 1,1,1], useGlobalFrame=True, offsetUserFunction=0, offsetUserFunction_t=0, show=True, axesRadius=0.1, axesLength=0.4, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed."""
    ...

    @overload
    def CreateDistanceConstraint(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], distance=None, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1., color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed."""
    ...

    @overload
    def CreateCoordinateConstraint(self, name='', bodyNumbers=[None, None], coordinates=[None, None], offset=0., factorValue1=1., velocityLevel=False, offsetUserFunction=0, offsetUserFunction_t=0, show=True, drawSize=-1., color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create coordinate constraint for two bodies, or body on ground; markers and NodePointGround are automatically created when needed."""
    ...

    @overload
    def CreateRollingDisc(self, name='', bodyNumbers=[None, None], axisPosition=[], axisVector=[1,0,0], discRadius=0., planePosition=[0,0,0], planeNormal=[0,0,1], constrainedAxes=[1,1,1], activeConnector=True, show=True, discWidth=0.1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create an ideal rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed; some constraint conditions may be deactivated, e.g. to resolve redundancy of constraints for multi-wheel vehicles."""
    ...

    @overload
    def CreateRollingDiscPenalty(self, name='', bodyNumbers=[None, None], axisPosition=[], axisVector=[1,0,0], discRadius=0., planePosition=[0,0,0], planeNormal=[0,0,1], contactStiffness=0., contactDamping=0., dryFriction=[0,0], dryFrictionAngle=0., dryFrictionProportionalZone=0., viscousFriction=[0,0], rollingFrictionViscous=0., useLinearProportionalZone=False, activeConnector=True, show=True, discWidth=0.1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create penalty-based rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed."""
    ...

    @overload
    def CreateSphereSphereContact(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], spheresRadii=[-1,-1], isHollowSphere1=False, dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0., contactStiffnessExponent=1, constantPullOffForce=0, contactPlasticityRatio=0, adhesionCoefficient=0, adhesionExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0,0,0,0], activeConnector=True, bodyOrNodeList=[None, None], show=False, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create penalty-based sphere-sphere contact between two rigid bodies, mass points (if friction coefficient is zero) or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems."""
    ...

    @overload
    def CreateSphereQuadContact(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], radiusSphere=0, quadPoints=exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]), includeEdges=15, dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0., contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0,0,0,0], activeConnector=True, bodyOrNodeList=[None, None], localPosition1=[0.,0.,0.], show=False, color=exudyn.graphics.color.default) -> dict: 
        """Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems."""
    ...

    @overload
    def CreateSphereTriangleContact(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], radiusSphere=0, trianglePoints=exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]), includeEdges=7, dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0., contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0,0,0,0], activeConnector=True, bodyOrNodeList=[None, None], localPosition1=[0.,0.,0.], show=False, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems."""
    ...

    @overload
    def CreateKinematicTree(self, name='', listOfTreeLinks=[], referenceCoordinates=None, initialCoordinates=None, initialCoordinates_t=None, gravity=[0.,0.,0.], baseOffset=[0.,0.,0.], linkForces=None, linkTorques=None, jointForceVector=None, jointPositionOffsetVector=None, jointVelocityOffsetVector=None, forceUserFunction=0, jointRadius=0.05, jointWidth=0.12, colors=exudyn.graphics.color.default, colorsJoints=exudyn.graphics.color.default, baseGraphicsDataList=None, linkRoundness=0.2, show=True) -> ObjectIndex: 
        """helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint; uses TreeLink as defined in exudyn.rigidBodyUtilities."""
    ...

    @overload
    def CreateFFRFReducedOrderObject(self, name, femInterface, referencePosition=[0., 0., 0.], initialVelocity=[0., 0., 0.], referenceRotationMatrix=np.eye(3), initialAngularVelocity=[0., 0., 0.], massProportionalDamping=0., stiffnessProportionalDamping=0., gravity=[0., 0., 0.], color=exudyn.graphics.color.defaultFFRF, superElementRigidMarkersOffsets=None, showMarkers=True, verbose=False) -> dict: 
        """Create an FFRF reduced order object; the function adds SuperElementRigid markers if boundaries are defined in the given femInterface and thus enables straightforward integration of flexible bodies into a multibody system."""
    ...

    @overload
    def CreateForce(self, name='', bodyNumber=None, loadVector=[0.,0.,0.], localPosition=[0.,0.,0.], bodyFixed=False, loadVectorUserFunction=0, show=True) -> LoadIndex: 
        """helper function to create force applied to given body."""
    ...

    @overload
    def CreateTorque(self, name='', bodyNumber=None, loadVector=[0.,0.,0.], localPosition=[0.,0.,0.], bodyFixed=False, loadVectorUserFunction=0, show=True) -> LoadIndex: 
        """helper function to create torque applied to given body."""
    ...

    @overload
    def PlotSensor(self, sensorNumbers=[], components=0, xLabel='time (s)', yLabel=None, labels=[], colorCodeOffset=0, newFigure=True, closeAll=False, componentsX=[], title='', figureName='', fontSize=16, colors=[], lineStyles=[], lineWidths=[], markerStyles=[], markerSizes=[], markerDensity=0.08, rangeX=[], rangeY=[], majorTicksX=10, majorTicksY=10, offsets=[], factors=[], subPlot=[], sizeInches=[6.4,4.8], fileName='', useXYZcomponents=True, legendArgs=None, **kwargs) -> [Any, Any, Any, Any]: 
        """Helper function for direct and easy visualization of sensor outputs, without need for loading text files, etc.; PlotSensor can be used to simply plot, e.g., the measured x-Position over time in a figure. PlotSensor provides an interface to matplotlib (which needs to be installed). Default values of many function arguments can be changed using the exudyn.plot function PlotSensorDefaults(), see there for usage."""
    ...

    @overload
    def SolveStatic(self, simulationSettings=None, updateInitialValues=False, storeSolver=True, showHints=False, showCausingItems=True, autoAssemble=True) -> bool: 
        """solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; this function is also available in exudyn (using exudyn.SolveStatic(...))."""
    ...

    @overload
    def SolveDynamic(self, simulationSettings=None, solverType=exudyn.DynamicSolverType.GeneralizedAlpha, updateInitialValues=False, storeSolver=True, showHints=False, showCausingItems=True, autoAssemble=True) -> bool: 
        """solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; this function is also available in exudyn (using exudyn.SolveDynamic(...))."""
    ...

    @overload
    def ComputeLinearizedSystem(self, simulationSettings=None, projectIntoConstraintNullspace=False, singularValuesTolerance=1e-12, returnConstraintJacobian=False, returnConstraintNullspace=False, autoAssemble=True) -> [ArrayLike, ArrayLike, ArrayLike]: 
        """compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints; for computation of eigenvalues and advanced computation with constrained systems, see ComputeODE2Eigenvalues; the current implementation is also able to project into the constrained space, however, this currently does not generally work with non-holonomic systems."""
    ...

    @overload
    def ComputeODE2Eigenvalues(self, simulationSettings=None, useSparseSolver=False, numberOfEigenvalues=0, constrainedCoordinates=[], convert2Frequencies=False, useAbsoluteValues=True, computeComplexEigenvalues=False, ignoreAlgebraicEquations=False, singularValuesTolerance=1e-12, autoAssemble=True) -> [ArrayLike, ArrayLike]: 
        """compute eigenvalues for unconstrained ODE2 part of mbs, which represent the square of the eigenfrequencies (in radiant) of the undamped system; the computation may include constraints in case that ignoreAlgebraicEquations=False (however, this currently does not generally work with non-holonomic systems); for algebraic constraints, however, a dense singular value decomposition of the constraint jacobian is used for the nullspace projection; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState, SetSystemState, see theDoc.pdf); note that mass and stiffness matrices are computed in dense mode so far, while eigenvalues are computed according to useSparseSolver."""
    ...

    @overload
    def ComputeSystemDegreeOfFreedom(self, simulationSettings=None, threshold=1e-12, verbose=False, useSVD=False, autoAssemble=True) -> dict: 
        """compute system DOF numerically, considering Gr{'u}bler-Kutzbach formula as well as redundant constraints; uses numpy matrix rank or singular value decomposition of scipy (useSVD=True)."""
    ...

    @overload
    def CreateDistanceSensorGeometry(self, meshPoints, meshTrigs, rigidBodyMarkerIndex, searchTreeCellSize=[8,8,8]) -> int: 
        """Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on;; Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact."""
    ...

    @overload
    def CreateDistanceSensor(self, generalContactIndex, positionOrMarker, dirSensor, minDistance=-1e7, maxDistance=1e7, cylinderRadius=0, selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList, storeInternal=False, fileName='', measureVelocity=False, addGraphicsObject=False, drawDisplaced=True, color=exudyn.graphics.color.red) -> SensorIndex: 
        """Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker."""
    ...

    @overload
    def DrawSystemGraph(self, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False, addItemTypeNames=True, multiLine=True, fontSizeFactor=1., layoutDistanceFactor=3., layoutIterations=100, showLegend=True, tightLayout=True, showGraph=True, addItemData=False, addAnnotations=False) -> [Any, Any, Any]: 
        """helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph; the graph visualization uses randomizer, which results in different graphs after every run!"""
    ...

