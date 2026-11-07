# The test models added since 2026-09-20: which show their simulation

Revision2026c step RG2.12.2, #3322; written 2026-11-07. A working record for the step, removed when it is done.

**The question.** When a test model is run directly (`testIsActive` False), it writes its solution and opens the
SolutionViewer, so that a reader sees the animation of what the model tests and checks the result by eye (#3321). That
helps where the subject of the test is a motion. Where the subject is a number - a Jacobian, an order, a file, an output
variable read once - the animation shows nothing that explains the test, and it is left out.

**The models.** 184 test models were added to `python/TestModels` from 2026-09-20 (`git log --diff-filter=A`, still
present): 14 show their simulation already, 47 do not simulate, and 123 simulate without showing it. They are sorted
below by reading the `Details:` header and the code; the change of each file checks its group again.

**The change** (groups A and B), after the solve of the shown case:

```python
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
...
if not testIsActive:
    SC.visualizationSettings.general.renderWindowString = 'case 3 of 3: the tip torque, dynamic' #group B only
    mbs.SolutionViewer()
```

plus graphics where an item draws nothing: a sphere per mass point (`CreateMassPoint(..., graphicsDataList=...)` or
`VObjectMassPoint(graphicsData=...)`), `addEdges=True` on a rolling ball, `show=True` on the visualization of a contact
item (#3326). A model that solves several cases in one `mbs` shows the case solved last; group B either solves the
shown case last or keeps its solution file apart. Checked without a window: `SC.renderer.GetGraphicsData()` must
contain the shapes.

**Recommendation:** change groups A (41 models) and B (7); leave C (75) and the 47 without a simulation as they are.

## A1 Beams and flexible bodies: they draw themselves - show (9)

`ANCFCable2DBendingTest`, `ANCFThinPlateRotationTest`, `flexiblePendulumBeamComparison`, `geometricallyExactBeamCurvedTest`, `geometricallyExactBeamElbowCantilever`, `geometricallyExactBeamRightAngleFrame`, `geometricallyExactBeamOutputTest`, `geometricallyExactBeamMarkerTest`, `createFFRFReducedOrderObjectTest`

## A2 Rigid bodies, joints, robots: drawn by their graphics or by the Create functions - show (17)

`RigidPendulumTest`, `SliderCrank2DTest`, `SlidingJoint2DTest`, `SwitchingConstraintsTest`, `constraintDriftTest`, `createLinearSpringDamperTest`, `kinematicTreeFloatingBaseTest`, `kinematicTreeUserFunctionTest`, `robotFrictionEndStopsTest`, `robotInverseDynamicsTest`, `urdfRobotHDF5Test`, `implicitRungeKuttaLieGroupTest`, `symbolicUserJointsTest`, `symbolicUserRigidBodiesTest`, `symbolicUserTorsionalTurnsTest`, `symbolicUserConnectorsTest`, `symbolicUserContactFrictionTest`

## A3 Mass points, balls and contacts: a sphere per mass point (graphics to add) - show (15)

`MathematicalPendulumTest`, `SpringDamperMesh`, `CartesianSpringDamperTest`, `CoordinateSpringDamperTest`, `initialConstraintsCheckTest`, `contactComparisonTest`, `contactCurveCirclesFrictionTest`, `contactFrictionCircleCable2DnoFrictionTest`, `contactSphereTorusMomentumTest`, `generalContactTriangleMomentumTest`, `symbolicUserContactTest`, `symbolicUserItemsTest`, `multithreadingUserFunctionTest`, `symbolicUserFunctionThreadsTest`, `springDamperVelocityOffsetTest`

## B Several cases: one case shown, named by renderWindowText - show (7)

`ANCFBeamGeometricallyExactComparisonTest`, `geometricallyExactBeam2DquadraticTest`, `userConnectorTest`, `userNodeTest`, `explicitSolversPostNewtonTest`, `implicitRungeKuttaSolversTest`, `coSimulationTest`

## C1 Jacobians, matrices, orders, step sizes, accuracy: numbers, not a motion - leave (24)

`ANCFThinPlateRevisionTest`, `geometricallyExactBeamJacobianTest`, `geometricallyExactBeamMassTest`, `kinematicTreeMarkerCacheTest`, `kinematicTreePrismaticJacobianTest`, `genericODE1JacobianTest`, `genericODE1duplicateNodeTest`, `ode1LoadsJacobianTest`, `sparseSystemMatricesTest`, `symbolicJointJacobianTest`, `symbolicMassMatrixDerivativeTest`, `symbolicSparseMatrixProductTest`, `symbolicSparseMatrixTest`, `symbolicVectorUserFunctionTest`, `symbolicNumpyCompatibilityTest`, `symbolicLookupTableTest`, `generalizedAlphaODE1OrderTest`, `implicitRungeKuttaErrorEstimateTest`, `implicitRungeKuttaSmallStepTest`, `explicitStepSizeControlTest`, `initialAccelerationsTest`, `solverOutputWithinStepTest`, `symbolicSolversAdaptiveStepTest`, `symbolicTrapezoidalGGLStepSizeTest`

## C2 Output variables, parameters, interfaces of items: read after a short solve - leave (23)

`accessFunctionsTest`, `connectorFrameOutputsTest`, `connectorOutputVariablesTest`, `coordinateSpringDamperExtVelocityOffsetTest`, `createItemNumbersTest`, `energiesFlexibleBodiesTest`, `energiesTest`, `homogeneousTransformationParameterTest`, `homogeneousTransformationTest`, `markerNodeCoordinateReferenceTest`, `pythonUserFunctionArgumentsTest`, `rotationMarkerDeprecationTest`, `sensorUserFunctionItemsTest`, `symbolicItemOutputTest`, `systemUtilitiesTest`, `userTypesTest`, `genericODE2matrixFormatsTest`, `robotDictionaryTest`, `robotModelsTest`, `modelLibraryTest`, `symbolicUserFunctionSignaturesTest`, `symbolicUserFunctionCopyTest`, `symbolicUserItemsCreateExamplesTest`

## C3 ObjectGenericODE1/DAE and SymbolicSystem: coordinates without geometry - leave (16)

`genericDAEDataTest`, `genericDAESymbolicTest`, `genericDAETest`, `genericDAEUserFunctionsTest`, `symbolicEulerParametersTest`, `symbolicExplicitConstraintTest`, `symbolicExplicitSolverTest`, `symbolicGeneralizedAlphaTest`, `symbolicSystemDataTest`, `symbolicSystemGenericDAETest`, `symbolicSystemItemJacobiansTest`, `symbolicSystemJointsTest`, `symbolicSystemLoadsTest`, `symbolicSystemODE1AETest`, `symbolicSystemODE1Test`, `symbolicTrapezoidalGGLTest`

## C4 Files, solver infrastructure, the renderer itself - leave (12)

`appendSystemTest`, `binarySolutionFileTest`, `emptySystemTest`, `fileHeaderThreadsTest`, `restartFileTest`, `solutionFileWithoutCoordinatesTest`, `solverFailureMessageTest`, `systemContainerComputeOnlyTest`, `systemContainerLifetimeTest`, `userFunctionTimerTest`, `rendererItemChangesTest`, `scopeTest`

## Already shown (14)

`contactDistanceOutputTest`, `contactUserFunctionTest`, `sphereCylinderContactTest`, `sphereTrianglePlaneTest`, `userANCFBeam3DTest`, `userANCFCable2DTest`, `userConnectorContactTest`, `userJointTest`, `userMarkerTest`, `userMassIntegralTest`, `userOutputVariablesTest`, `userParametersTest`, `userRigidBodyTest`, `userTetrahedronTest`

## Without a simulation (47) - leave

`GraphicsDataTest`, `analyticInverseKinematicsTest`, `cUtilitiesTest`, `convexDistanceTest`, `graphicsDataErrorsTest`, `homogeneousTransformationInterfaceTest`, `inactiveConnectorForceTest`, `inspectTest`, `itemParameterRenamesTest`, `kinematicChainTest`, `libraryDeprecationTest`, `modelLibrary`, `numericalInverseKinematicsTest`, `orientTrianglesTest`, `raytracerBackFaceTest`, `rendererDictionaryTest`, `robotCartesianPathsTest`, `robotTrajectoryProfilesTest`, `shellMeshSlopesScalingTest`, `simulationSettingsDeprecationTest`, `simulationSettingsRenamesTest`, `stlFileImportTest`, `symbolicDeepRecordingTest`, `symbolicExportTest`, `symbolicFunctionTest`, `symbolicHTTest`, `symbolicJacobianExpressionsTest`, `symbolicOnePassJacobianTest`, `symbolicOptimizeTest`, `symbolicParameterJacobianTest`, `symbolicRigidBodyInertiaTest`, `symbolicSolverOutputTest`, `symbolicStatisticsTest`, `symbolicSystemEquationsTest`, `symbolicSystemParametersTest`, `symbolicSystemTest`, `symbolicSystemVariablesTest`, `symbolicUserItemsDrawingTest`, `symbolicUtilitiesTest`, `symbolicVariableScopeTest`, `symbolicVectorDiffTest`, `textureGraphicsTest`, `textureTest`, `trajectoryBlendingCartesianTest`, `trajectoryBlendingTest`, `userGraphicsDataTest`, `zoomToItemTest`
