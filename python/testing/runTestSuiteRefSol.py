#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This file constains reference solutions for test suite
#
# Author:   Johannes Gerstmayr
# Date:     2021-02-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys

#%%+++++++++++++++++++++++++++++++++++++++
#return reference solutions for test examples in dictionary
def TestExamplesReferenceSolution():

    #ALL values below were re-measured on 2026-09-16 (#2466) with the
    #BASELINE-ISA Windows module. Until then the default Windows module was compiled with
    #/arch:AVX2 while Linux had none, so these values were AVX2 values and Linux could not meet
    #them; that is what UnresolvedOnLinux() below is a list of. 85 of the 113 values moved, 33 of
    #them by more than the 5e-14 tolerance - the AVX branches of Use_avx.h sum in a different
    #order. The previous (AVX2) values are in git history, one commit back.
    refSol = {
        #ten small tests that each compute an ERROR against a reference value written into them in
        #2019, so each result IS that error and the reference here is 0; each model states
        #its own tolerance, exu.sys['testTolerance'] = 4e-13, which is what
        #RunAllModelUnitTests compared against
        'ANCFCable2DBendingTest.py': 0.,
        'CartesianSpringDamperTest.py': 0.,
        'CoordinateSpringDamperTest.py': 0.,
        'GraphicsDataTest.py': 0.,
        'MathematicalPendulumTest.py': 0.,
        'RigidPendulumTest.py': 0.,
        'SliderCrank2DTest.py': 0.,
        'SlidingJoint2DTest.py': 0.,
        'SpringDamperMesh.py': 0.,
        'SwitchingConstraintsTest.py': 0.,
        'abaqusImportTest.py': 0.0005885208722206048,               #new 2023-04-20; 5 modes as 8 modes have sensitive "half mode included"
        'allExudynModulesTest.py': 1.0,                               #new 2026-02-03; test all modules (if some major error is contained...)
        'ANCFBeamTest.py': 1.0104863123004104,                       #new 2023-04-04, after resolving local kappa bug
        'ANCFcable2DuserFunction.py': 0.6015588367721973,           #new 2023-12-13
        'ANCFCableBeamDampingTest.py': 0.18992335572077274,         #new 2026-03-25, checking damping between ANCFCable2D and ANCFBeam
        'ANCFbeltDrive.py': -0.0011715885324992126,               #new 2026-09-17: the model was retuned (16 elements
                                                                #per section, tEnd=0.1) and the result now comes from the
                                                                #sensor rather than an ODE2 coordinate (#2368)
        'ANCFcontactCircleTest.py':-0.4842698420787613,
        'ANCFcontactFrictionTest.py':-0.014187561328096003,         #with old ObjectContactFrictionCircleCable2D until : 2022-03-09: -0.014188649931059739,
        'ANCFgeneralContactCircle.py':-0.581654253165756,          #new 2022-07-11 (CState Parallel); #before some update to contact module(iterations decreased!):-0.5816521429557808, #2022-02-01
        'ANCFmovingRigidBodyTest.py':-0.12893096934990356,          #new 2022-12-25; old solution differs for 1e-10 since several updates -0.12893096921737698,
        'ANCFslidingAndALEjointTest.py':-4.426408394755277,         #before 2023-05-01 (loads jacobian): -4.426408390697862,         #before 2022-12-25(resolved BUG 1274): -4.426403044189653; with old ObjectContactFrictionCircleCable2D until: 2022-03-09: -4.42640304418963,
        'ballBearingTest.py':0.037852414033278825,                 #2026-10-01: the cage's CartesianSpringDamper on the connector interface (#2745), round-off of the projection; before 0.03785241402944885
        'bricardMechanism.py': 4.172189649306508,
        'carRollingDiscTest.py':-0.2394004871711386,
        'compareAbaqusAnsysRotorEigenfrequencies.py':0.0004185480476228394,
        'compareFullModifiedNewton.py':0.00020079676000188396,
        'complexEigenvaluesTest.py':0.42816392078752485,            #new 2024-05-04 testing ComputeODE2Eigenvalues2 for complex case
        'computeODE2AEeigenvaluesTest.py': 0.3881173295041342,
        'computeODE2EigenvaluesTest.py':-2.747979063144612e-11,
        'connectorGravityTest.py': 1014867.2330320379,
        'connectorRigidBodySpringDamperTest.py':0.18276224743555652, #new 2022-07-11 (CState Parallel); 
        'contactCoordinateTest.py':0.0553131995062827,
        'contactCurveExample.py':0.3096143279681373,                #new 2025-05-11
        'contactSphereSphereTest.py':0.4416615668422053, #2026-09-30: step size recommended also where a contact ends (#2109); before 0.5348463502652304;           #new 2025-02-03
        'contactSphereSphereTestEAPM.py': 0.20000219249662216,      #new 2025-02-03
        'ConvexContactTest.py':0.011770267410492958,                #new 2022-07-11 (CState Parallel); #before 2022-01-25?: 0.05737886603111926, 
        'coordinateSpringDamperExt.py':17.084935539349033,          #new 2023-01-23
        'coordinateVectorConstraint.py':-1.0825265797698307,
        'coordinateVectorConstraintGenericODE2.py':-1.0825265797698307,
        'createKinematicTreeTest.py':3.3408301427307276,            #new 2025-06-14
        'createFunctionsTest.py':0.04228833966560114,              #new 2025-05-11
        'createRollingDiscPenaltyTest.py':2.1129927199922123,       #new 2025-02-27
        'createRollingDiscTest.py':4.009716209090303,               #new 2025-03-05
        'createSphereQuadContact.py':1.124394416977545, #2026-09-30: step size recommended also where a contact ends (#2109); before 1.1243776621604573;             #new 2025-06-29
        'createSphereQuadContact2.py':0.15616582432943388,          #new 2025-07-05
        'createSphereTriangleContact.py':4.840244058316264, #2026-09-30: step size recommended also where a contact ends (#2109); before 4.840960219289836;        #new 2026-09-11; tEnd shortened 0.65->0.25 on adding
        'deleteItemsTest.py':-0.9860528006518324,                   #new 2025-05-10
        'distanceSensor.py':1.86776431077868,
        'driveTrainTest.py':-9.26985560534277e-08,                 #new 2023-05-20 (mainSystemExtensions); before:-9.269311940229841e-08,
        'explicitLieGroupIntegratorPythonTest.py':149.84739395407578,
        'explicitLieGroupIntegratorTest.py':0.16164013319819118,
        'explicitLieGroupMBSTest.py':3.028987107923892,             #new 2026-09-11; endTime shortened 1->0.1 on adding, step size unchanged
        'fourBarMechanismTest.py':-2.376335780518213,
        'fourBarMechanismIftomm.py':0.17216652717785863,
        'generalContactCylinderTest.py':12.24658398056691,         #new 2024-03-17 (spurious trig-sphere contact forces)
        'generalContactCylinderTrigsTest.py':5.48690843091258,     #new 2024-03-17 (internal sphere-sphere contact)
        'generalContactFrictionTests.py':12.022654145378834,        #changed 2025-05-06 (seems to now be closer to linux; differences with object8); new 2024-03-17: 12.027740342293988 (doubled damping; fixed sphere-sphere and trig-sphere contact); old: 12.464092000879125,        #new 2022-07-11 (CState Parallel); #before 2022-01-25 (changed some velocity computation in GeneralContact): 10.133183086232139, #changed GeneralContact and implicit solver; before 2022-01-18: 10.132106712933348 , 
        'generalContactImplicit1.py':0.7758155402165082,             #new 2026-09-11
        'generalContactImplicit2.py':0.5000000537869635,             #new 2026-09-11
        'generalContactSpheresTest.py':-1.113854772025744,         #new 2022-07-22 (parallel Lie group updates); new 2022-07-11 (CState Parallel); #before 2022-01-25(minor diff, due to round off errors in multithreading; now changed to 1 thread):-1.113854772026123, #changed GeneralContact and implicit solver; before 2022-01-18: -1.0947542400425323, #before 2021-12-02: -1.0947542400427703,
        'genericJointUserFunctionTest.py':1.1922383967562884,
        'genericODE2test.py':0.03604546349894506,                  #new 2022-07-11 (CState Parallel); #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.036045463498793825,
        'geneticOptimizationTest.py':0.10117518366826603,           #before 2022-02-20 (accuracy of internal sensors is higher); 0.10117518367051619, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.10117518366934351,
        'geometricallyExactBeam2Dtest.py':-2.211502835379855,       #2026-01-09 update due to autodifferentiation
        'geometricallyExactBeamTest.py':1.012821233280551,         #before 2026-09-29: 1.012820942859896 (full Jacobian, #1550); before 2023-01-29: 1.012822053539261; before 2023-05-05: 1.0128218992948643 (changed Texp function); new 2023-04-06 may still include small errors in implementation
        'gridGeomExactBeam2D.py':-1.5827965743262553,                #new 2024-01-28
        'heavyTop.py':33.423125751743804,                            #new 2022-07-11 (CState Parallel); 
        'hydraulicActuatorSimpleTest.py':7.130440021870289,
        'jointArgsTest.py':0.00426904955009082,                    #2025-05-10
        'kinematicTreeAndMBStest.py':263.88120463802585,           #the raw value since 2026-09-24: the model used
                                                                 #to multiply it by 1e-7, which hid its tolerance inside the
                                                                 #number; it states exu.sys['testTolerance'] = 5e-7 instead,
                                                                 #which is the same comparison. Until then: 2.6388120463802584e-05
        'kinematicTreeConstraintTest.py':1.8135975384620298 ,
        'symbolicUserFunctionCopyTest.py':0.38643688092501255, #a copy through Get/SetDictionary stays symbolic (#1888)
        'flexiblePendulumBeamComparison.py':-1.5235597679330848, #2026-09-30: consistent mass matrix (#1273); before -1.508742103106691 #three beam elements, one pendulum (#2730)
        'geometricallyExactBeamMarkerTest.py':0.22853106396053813, #2026-09-30: consistent mass matrix (#1273); #body markers on the 3D beam = loads on its nodes (#2730)
        'geometricallyExactBeamJacobianTest.py':4.282489188464191, #analytic against numerical Jacobian of the 3D beam (#1550)
        'geometricallyExactBeamCurvedTest.py':4.561491685469841, #the 45-degree bend, stress-free in its curved reference configuration (#1494)
        'geometricallyExactBeamRightAngleFrame.py':3.306181714013268, #lateral buckling of the right-angle frame at 1.088 N (#1499)
        'rightAngleFrame.py':4.371823197993865, #the same frame driven by displacement, past the buckling point (#2762)
        'energiesTest.py':8.724884363749222, #kinetic and potential energy of simple bodies and spring-dampers (#2202)
        'geometricallyExactBeamOutputTest.py':-6.079487513916353, #section forces, moments and strains of the 3D beam (#2753)
        'geometricallyExactBeamElbowCantilever.py':-5.57992601861755, #2026-09-30: consistent mass matrix (#1273); #right-angle cantilever of Simo and Vu-Quoc 1988, free oscillations (#2730)
        'geometricallyExactBeamMassTest.py':2.9942070791403967, #consistent mass matrix of the 3D beam: rigid motion exact (#1273)
        'geometricallyExactBeam2DquadraticTest.py':0.7426300926712416, #the 3-node planar geometrically exact beam (#2208)
        'genericODE1duplicateNodeTest.py':3.1091750014354522, #numerical ODE1 Jacobian with a coordinate addressed twice (#1424)
        'contactSphereTorusMomentumTest.py':4.227231105376637, #2026-09-30: step size recommended also where a contact ends (#2109); before 4.227231105610667; #momentum conservation of the sphere-torus contact (#2127)
        'explicitSolversPostNewtonTest.py':4.563482002223096, #2026-09-30: DOPRI5 with large steps added, and the step size recommended where a contact ends (#2109); before 4.097033066855782; #PostNewton states with every explicit integrator (#2754)
        'contactComparisonTest.py':1.200116276111891, #2026-09-30: step size recommended also where a contact ends (#2109); before 1.200040705928356; #deepest points and end heights, three contact objects, two laws (#2749, #2750)
        'kinematicTreePrismaticJacobianTest.py':1.249999999999995, #5 cases of 0.25 (#2740)
        'kinematicTreeTest.py':-1.3093839602164064,
        'laserScannerTest.py':2.695064443768281 ,                   #new 2024-04-29
        'linearFEMgenericODE2.py': 0.38767197129755937,              #new 2024-10-06 for jacobianUserFunction in GenericODE2
        'loadUserFunctionTest.py': 1.8051173706570727,              #new 2024-10-10 for visualization of time-dependent loads
        'LShapeGeomExactBeam2D.py':-0.9181474510515215,             #2026-01-09 update due to autodifferentiation
        'mainSystemExtensionsTests.py': 57.646394469414666,          #updated 2023-11-16; updated 2023-06-09; old: new 2023-05-19
        'mainSystemUserFunctionsTest.py': 4.069301305919595,        #new 2024-10-17
        'manualExplicitIntegrator.py':2.0596986296922988,
        'matrixContainerTest.py':56.5,                              #new 2024-10-09
        'mecanumWheelRollingDiscTest.py':0.2714267238324344,
        'movingGroundRobotTest.py':0.003840899497986155,           #updated 2026-09-09, with factor 0.5 for tolerance too close
        'NGsolveCMStest.py': 0.06953224923173146,                   #re-measured 2026-09-16 (#2466) against the COMMITTED testData/netgenTestMesh.pkl; the model rewrites that tracked file whenever its load fails, and the result then moves by 2.4e-8 (#2469); changed 2025-05-05 (new .pkl file with newer ngsolve); until 2024-10-11: 0.06953227339277462
        'objectFFRFreducedOrderAccelerations.py':0.10000570245889191,#before 2022-07-22 (because often small fails); 0.5000285122944431,#before 2022-02-20 (accuracy of internal sensors is higher): 0.5000285122930983,
        'objectFFRFreducedOrderTest.py':0.005355233268058772,      #until 2022-03-18 (div result by 5): 0.026776166340247865,
        'objectFFRFTest.py':0.0064600108120842666,                  #before 2022-02-20 (accuracy of internal sensors is higher): 0.006460010812070858,
        'objectFFRFTest2.py':0.035521880690182486,                   #before 2022-02-20 (accuracy of internal sensors is higher): 0.03552188069032863,
        'objectGenericODE2Test.py':-2.316378897585508e-05,
        'PARTS_ATEs_moving.py':0.44656762760262064,
        'pendulumFriction.py':0.39999998776982154,
        'parameterConversionTest.py':0.0,                             #new 2026-09-14: number of differences to parameterConversionTestReference.txt
        'typeInformationTest.py':0.0,                                 #new 2026-09-15: number of disagreements of exudyn.types with the C++ module
        'exceptionTypesTest.py':0.0,                                  #new 2026-09-18: number of provoked errors that raised NOTHING
        'pickleCopyMbs.py':0.2583013564103506,                      #new 2025-05-10
        'plotSensorTest.py':1.0,
        'postNewtonStepContactTest.py':0.057286638346409235,
        'raytracerNOGLFWtest.py':0.28151013387134,                  #new 2026-01-03
        'reevingSystemSpringsTest.py':2.215557571743302,           #new 2023-07-17 (old solution contained compression forces: 2.213190117855691),
        'relativeRotationTranslationMechanism.py': 1.509631854432179,#new 2026-09-11
        'resultsMonitorTest.py': 1.0,                               #new 2026-09-19; exudyn.misc.resultsMonitor: the four file types, incremental reading, the CLI return codes
        'revoluteJointPrismaticJointTest.py':1.2538806799241744,    #new 2022-07-11 (CState Parallel); #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver (modified Newton restart, etc.); before 2022-01-18: 1.2538806799243265,
        'rigidBody2Dtest.py': -0.5055295700922418,                  #new 2025-02-05: added arbitrary COM to 2D rigid body
        'rigidBodyAsUserFunctionTest.py':8.950865271552148,
        'rigidBodyCOMtest.py':3.409431467726291,
        'rigidBodySpringDamperIntrinsic.py':0.5472368462870515,     #2026-10-01: on the connector interface (#2745), round-off of the projection; before 0.5472368462985283; new 2023-11-30 (intrinsic formulation for rigid body spring damper)
        'rollingCoinTest.py':1.0634381189361193,                     #until 2024-04-29 (without force): 0.0020040999273379673
        'rollingDiscTangentialForces.py':1.0342017404650015,        #new 2024-05-04: RollingDiscPenalty: switch to local computation of tangential forces
        'rollingCoinPenaltyTest.py':0.03489603106786701,
        'rotatingTableTest.py':7.838680375029852,                   #until 2024-05-04 (before slight change in RollingDiscPenalty): 7.838680371309492
        'scissorPrismaticRevolute2D.py':27.20255648904438,          #new 2022-07-11 (CState Parallel); #added JacobianODE2, but example computed with numDiff forODE2connectors, 2022-01-18: 27.202556489044145,
        'sensorUserFunctionTest.py':45.0,            
        'serialRobotTest.py':0.7681856909844541,                    #until 2022-04-21: 0.7680031232063571 wrong static torque compensation
        #value changed 2026-09-18 (#2502): the joint helpers now
        #convert the joint position with a fixed summation order, so this result no longer
        #depends on which numpy release built the marker positions. It is the value numpy
        #2.2.4 produced and the one an explicit sum produces; the old 7.256859912845965 was
        #what numpy 2.4.6's matmul happened to give
        'sliderCrank3Dbenchmark.py':7.256859914829453,              #new 2026-09-11; tEnd shortened 5->0.5, the value the file itself calls converged
        'sliderCrank3Dtest.py':3.364276178092191,
        'sliderCrankFloatingTest.py':0.591649163378833,
        'solverExplicitODE1ODE2test.py':3.3767933275918964,         #new 2022-07-11 (CState Parallel); 
        'sparseMatrixSpringDamperTest.py':-0.06779862812271391,     #changed to analytic Spring-Damper jacobian (missing d(vel)/dpos term): -0.06779862983767654,
        'sphereTriangleTest2.py':4.35616383223589, #2026-09-30: step size recommended also where a contact ends (#2109); before 4.35608275479331;                 #changed: 2026-01-23 (sparse acc(vel) initialization); new 2025-06-22
        'sphericalJointTest.py':4.409080446575154,                  #new 2022-07-11 (CState Parallel); 
        'springDamperUserFunctionTest.py':0.5062872273010924,
        'stiffFlyballGovernor.py':0.8962488779114738,
        'superElementRigidJointTest.py':0.015217208913989099,       #before 2022-02-20 (accuracy of internal sensors is higher): 0.015217208913983024,
        'symbolicUserFunctionTest.py':0.10039884426884882,          #2023-12-13
        'symbolicModuleTest.py':0.9484129575069745,                 #2023-12-14
        'taskmanagerTest.py':-0.23406814272950313,                  #2024-10-08
        'velocityVerletTest.py':4.365184132226787,                  #2024-10-07
        }

    #++++++++++++++++++++
    #special solutions for 32bit:
    import platform
    if platform.architecture()[0] != '64bit':
        #refSol['ACNFslidingAndALEjointTest.py']=-4.426403043824947 #works now with original value: 22-09-2021
        refSol['genericODE2test.py']=0.0360454634988472, #before 2021-12-02: 0.036045463499109365
        refSol['heavyTop.py']=33.42312575172905, #before 2021-12-02: 33.42312575176021
        #refSol['objectFFRFreducedOrderTest.py']=0.026776166340291847 #changes due to eigenvalue solver
        #refSol['scissorPrismaticRevolute2D.py']=27.202556489044472 #not needed with updated 64bit solution
        #refSol['serialRobotTest.py']=0.7712176106962295 #works now with original value: 22-09-2021
        refSol['connectorRigidBodySpringDamperTest.py']=0.18276224743611413, #before 2021-12-02: 0.1827622474328367


    #add new reference values here (only uses new solver):
    refSol['sensorUserFunctionTest.py'] = 45

    if sys.platform == 'darwin':
            # offscreen RedrawAndGetImage crashes on macOS since Exudyn 1.11.0
            refSol.pop('raytracerNOGLFWtest.py', None)


    return refSol


#%%+++++++++++++++++++++++++++++++++++++++
#test models that take noticeably longer than the rest, measured 2026-09-16 on Windows cp313;
#the whole suite is 22 seconds, so 'slow' here means 'above 0.6 s', not 'minutes'. Data, not
#decorators: runTestSuite.py --fast and the pytest marker both read this
#list, so there is one definition. Update it when a model changes substantially.
def SlowTests():

    return {
        'parameterConversionTest.py': 4.4,   #walks every parameter of every item
        'createSphereQuadContact.py': 1.4,
        'sphereTriangleTest2.py': 1.3,
        'createSphereTriangleContact.py': 1.0,
        'abaqusImportTest.py': 0.9,          #reads and converts several meshes
        'taskmanagerTest.py': 0.8,
        'objectFFRFTest2.py': 0.7,
        'explicitLieGroupMBSTest.py': 0.6,
        'objectFFRFTest.py': 0.6,
        }


#%%+++++++++++++++++++++++++++++++++++++++
#test models that need an OPTIONAL package, i.e. one that is not in [project.optional-dependencies]
#'tests' or that a small CI runner should not have to install. A pull-request run can skip these
#(runTestSuite.py --fast, pytest -m 'not optionalPackage'), a nightly run must not.
#Derived 2026-09-16 from the imports of the models themselves.
def OptionalPackageTests():

    return {
        'ACFtest.py': 'ngsolve',
        'NGsolveCMStest.py': 'ngsolve',
        'allExudynModulesTest.py': 'stable_baselines3',  #the RL part; skipped without it
        }


#%%+++++++++++++++++++++++++++++++++++++++
#return the test models which exist but are deliberately NOT executed by the test suite,
#name -> reason. A reason is required: a bare exclusion list is how the set rotted in the
#first place, and a sentence per entry makes an unjustified
#exclusion visible when the file is read.
#
#Anything listed here is skipped by the coverage check. Anything NOT listed and not in a
#reference list makes the check fail - see testRunnerTools.CheckTestCoverage.
#All 19 entries below were triaged on 2026-09-10 by running each file the way runTestSuite.py
#does (exudynTestGlobals.useGraphics = False) and recording result, runtime and every file it
#writes. The reasons are what that run showed, not a guess from the file name. Six of them are
#candidates to be ADDED once trimmed - they are listed here only until that decision is taken.
def DeliberatelyNotRun():

    return {
        #--- not headless: a hard 'useGraphics = True' AFTER the exudynTestGlobals block
        #    overrides the runner's False, so these open a render window and never return
        'ANCFThinPlateTests.py':
            'useGraphics=True at line 32 overrides the runner; opens AnimateModes (timeout >300s)',
        'ANCFoutputTest.py':
            'useGraphics=True at line 33 overrides the runner (timeout >300s)',
        'doublePendulum2DControl.py':
            'unconditional SC.renderer.Start(); an interactive demo, has no exudynTestGlobals',
        'objectFFRFreducedOrderShowModes.py':
            'AnimateModes viewer demo, not a test; produces no value',

        #--- broken or drifted: they run, but what they produce cannot be used as a reference
        'ANCFBeamEigTest.py':
            'runs clean in 0.23s but its testError/testResult lines are commented out (line 232)',
        'LieGroupIntegrationUnitTests.py':
            'ALL 10 of its tests pass since 2026-09-18 (#2494 settled it: '
            'the composition rule deliberately does not map into the principal range, and TEST 2 '
            'now checks that the composed vector is the IDENTITY rotation rather than comparing '
            'against the Matlab principal-range value). What still keeps it out of the suite is '
            'that it PRINTS its results instead of setting testResult - give it one and it can be '
            'a test model',
        'createContactSphereSphere.py':
            'calls SolutionViewer although writeSolutionToFile=useGraphics, so it raises when '
            'run headless; the value -0.21704884156413973 is computed before the crash',
        'objectFFRFreducedOrderStressModesTest.py':
            "reads 'TestModels/testData/rotorAnsys...' but the runner's cwd IS TestModels; "
            'path bug, and the stress-mode import needs the optional pyansys',
        'ACFtest.py':
            'needs the optional netgen/ngsolve, reads back a sensor file it does not write in '
            'this configuration, and forces useGraphics=True at line 370',
        'simulatorCouplingTwoMbs.py':
            'did not finish within 300s headless; needs investigation before it can be a test',

        #--- unstable, not merely inaccurate
        'sphereTriangleTest.py':
            'the explicit integrator goes unstable in a certain configuration: 3.8226 on an '
            'AVX2 Windows build, 69880 on a baseline build and 59370 on Linux. A smaller step '
            'size removes the divergence but leaves a solution that looks wrong on inspection, '
            'so this is a model defect, not a tolerance question; phase R10 (#2466)',

        #--- not test models in the reference-value sense
        'interfaceTest.py':
            'API smoke script: no exudynTestGlobals, produces no value; 13.9s',

        }

#%%+++++++++++++++++++++++++++++++++++++++
#return per-test factors which are multiplied with the global test tolerance,
#for tests which are known to be less reproducible than the rest
def TestExamplesToleranceFactors():

    tolFact = {
        'serialRobotTest.py': 100,                                  #sparse eigenvalue solver
        }

    return tolFact

#%%+++++++++++++++++++++++++++++++++++++++
#return the set of tests which are not reproducible across machines by their nature:
#  - contact and friction models, which are chaotic, so a different machine gives a
#    materially different error and the size of that error says nothing about correctness
#  - sparse eigenvalue problems solved with ARPACK, which starts from a random vector
#    that cannot be seeded
#These tests are still executed and their failures are reported prominently, but they do
#NOT set the process exit code (see runTestSuite.py --exit-code), because an automated run
#which goes red at random is an alarm nobody reads.
#
#POPULATE FROM EVIDENCE, NOT FROM FILE NAMES: run the suite on Windows, Linux and macOS and
#compare the per-test ERROR values. Anything varying by orders of magnitude belongs here. A
#name match on contact/friction/eigen hits about a third of the suite and is far too coarse.
def SensitiveTests():

    sensitive = set([
        ])

    return sensitive

#%%+++++++++++++++++++++++++++++++++++++++
#return the set of tests with KNOWN, UNRESOLVED differences between Windows and Linux.
#
#These are contact and friction models whose results differ between the two platforms - some
#by little, some by a lot. The reference values are the Windows ones, so on Linux these tests
#report a failure which is real but already known, and is therefore excluded from the exit
#code ON LINUX ONLY. On Windows they must still pass: nothing here weakens the platform the
#reference values come from.
#
#This is deliberately separate from SensitiveTests(): those are non-deterministic everywhere
#and can never be pinned down, whereas these are reproducible differences with a cause that
#has not been found yet. They are scheduled for investigation of the revision plan;
#the list should SHRINK as they are resolved, and each entry removed is a real fix.
#
#Measured 2026-09-10 on manylinux_2_28 / cp313 / numpy 2.4.6, relative to the Windows
#reference values (Linux tolerance is 3e-11):
def UnresolvedOnLinux():

    unresolved = set([
        'coordinateSpringDamperExt.py',         #rel. 3.4e-11
        'rigidBodySpringDamperIntrinsic.py',    #rel. 1.9e-10
        'rollingDiscTangentialForces.py',       #rel. 1.5e-09
        'contactSphereSphereTest.py',           #rel. 6.2e-09
        'sphereTriangleTest2.py',               #rel. 1.7e-05
        'generalContactCylinderTest.py',        #rel. 2.2e-05
        'generalContactFrictionTests.py',       #rel. 4.9e-04
        #added 2026-09-12 from the nightly GitLab run of 1.11.31.dev1 (manylinux_2_28 / cp313).
        #These two were the reason the Linux job exited non-zero while every other difference was
        #already excluded; they are the same kind of reproducible platform difference (#2379).
        'sliderCrank3Dbenchmark.py',            #rel. 2.0e-10
        'generalContactImplicit1.py',           #rel. 6.8e-08
        #added 2026-09-18 from the GitLab run of 1.11.162.dev1 (manylinux_2_28 / cp314 /
        #numpy 2.5.3): absolute error -5.7e-11 against a 3e-11 tolerance, i.e. rel. 1.2e-11 - the
        #smallest entry in this list and only 1.9x over. It is a contact model, the same family as
        #the six above, BUT it may equally be the numpy-version effect of #2502 (fact 28) rather
        #than a platform difference: this leg runs numpy 2.5.3 and the model builds its contact
        #through the Create* helpers whose setup arithmetic that issue is about. Resolve it with
        ##2502 before spending time on it as a Linux question.
        'createSphereTriangleContact.py',       #rel. 1.2e-11
        ])

    return unresolved

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def UnresolvedOnMacOS():
    """the models whose result differs on macOS by more than their tolerance

    What UnresolvedOnLinux() is to Linux. The reference values are the Windows ones, so a difference
    here is the same kind of reproducible platform difference - a different compiler, a different
    libm - and not a fault of the model. Measured on 2026-09-26 by the maintainer, on macOS ARM with
    Python 3.13 and V1.12.68.dev1 (tmp/testSuiteLog_V1.12.68.dev1_darwin-ARM-64bit-P3.13.txt):
    thirteen test models and one mini example, of which nine are also unresolved on Linux.

    A model that is in this set and passes is not a problem: the set says "a difference here proves
    nothing", not "there must be one"."""

    unresolved = UnresolvedOnLinux() | set([
        #macOS ARM only, 2026-09-26; the relative error against the 3e-11 tolerance
        'ANCFbeltDrive.py',                     #rel. 8.9e-06
        'ANCFcontactCircleTest.py',             #rel. 1.5e-07
        'ANCFgeneralContactCircle.py',          #rel. 7.8e-11
        'ANCFslidingAndALEjointTest.py',        #rel. 1.4e-09
        'connectorGravityTest.py',              #rel. 3.9e-13 of a value of 1.0e+06
        #the only mini example that differs; the test model of the same connector
        #(rigidBodySpringDamperIntrinsic.py) is in the Linux list above
        'ObjectConnectorRigidBodySpringDamper.py',  #rel. 2.5e-09
        ])

    return unresolved

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#return reference solutions for mini examples in dictionary
def MiniExamplesReferenceSolution():
    refSol = {
        #results after change to new Jacobian, diff about 1e-12
        'LoadMassProportional.py':-4.904999999999998,
        'MarkerSuperElementPosition.py':1.0039999999354785,
        'ObjectANCFCable2D.py':-0.5013058140308901,
        'ObjectANCFCable.py':-0.5013058140308919, #added 2023-10-15
        'ObjectConnectorSpringDamper.py':0.9733828995763039, #until 2022-01-25 (before analytical Jac for SpringDamper):0.9733828995759499,
        'ObjectConnectorCartesianSpringDamper.py':-0.0009999999999750209,
        'ObjectConnectorRigidBodySpringDamper.py':-0.534929955894111,
        'ObjectConnectorLinearSpringDamper.py':0.0004999866342440002, #previously had error, did not run
        'ObjectConnectorTorsionalSpringDamper.py':0.0004999866342439527,
        'ObjectConnectorCoordinateSpringDamper.py':0.0019995154213252597,
        'ObjectConnectorGravity.py':1.0000000000000484,
        'ObjectConnectorDistance.py':-0.9861806726069355,
        'ObjectConnectorCoordinate.py':0.04999999999999982,
        'ObjectGenericODE2.py':1.0039999999354785,
        'ObjectGenericODE1.py':-0.8206847097689384,
        'ObjectJointRevoluteZ.py':0.49999999999999795,
        'ObjectKinematicTree.py':-3.134018551808591,
        'ObjectMass1D.py':2.0,
        'ObjectMassPoint.py':2.0,
        'ObjectMassPoint2D.py':2.0,
        'ObjectRigidBody2D.py':4.356194490192344,
        'ObjectRotationalMass1D.py':2.0,
        #the objects (#2732)
        'ObjectGround.py':-0.09809999999999997,
        'ObjectRigidBody.py':1.0949978647619167,
        'ObjectANCFBeam.py':-0.3381249577076517,
        'ObjectBeamGeometricallyExact2D.py':-0.3381249616596102,
        'ObjectBeamGeometricallyExact.py':-0.33812496093079464,
        'ObjectANCFThinPlate.py':-0.06987553667440825,
        'ObjectALEANCFCable2D.py':0.2500092950450501,
        'ObjectJointALEMoving2D.py':0.7249978133196032,
        'ObjectConnectorCoordinateSpringDamperExt.py':0.050495049504950505,
        'ObjectConnectorCoordinateVector.py':0.49999999999999867,
        'ObjectContactCoordinate.py':-0.000999999999999999,
        'ObjectContactCircleCable2D.py':-0.0834019844819408,
        'ObjectContactFrictionCircleCable2D.py':-0.09630714157535598,
        'ObjectContactSphereSphere.py':1.0999019,
        'ObjectContactSphereTriangle.py':0.09990189999999996,
        'ObjectContactSphereTorus.py':0.10101,
        'ObjectContactCurveCircles.py':0.09949999999999998,
        'ObjectConnectorRollingDiscPenalty.py':-1.9999976983485357,
        'ObjectJointRollingDisc.py':-1.999997664339446,
        'ObjectJointGeneric.py':-3.1340201965908596,
        'ObjectJointPrismaticX.py':0.999999999999996,
        'ObjectJointSpherical.py':-0.003786192314531511,
        'ObjectJointRevolute2D.py':-3.1334196587393026,
        'ObjectJointPrismatic2D.py':0.24999999999999895,
        'ObjectJointSliding2D.py':1.0999999999999979,
        'ObjectJointSliding.py':1.0999999999999979,
        'ObjectConnectorReevingSystemSprings.py':-1.0000000000001228,
        'ObjectConnectorHydraulicActuatorSimple.py':1.0,
        #the markers, loads and sensors (#2732)
        'MarkerBodyMass.py':-4.904999999999998,
        'MarkerBodyPosition.py':-0.1,
        'MarkerBodyRigid.py':0.010000000002617884,
        'MarkerNodePosition.py':-0.49698039550402484,
        'MarkerNodeRigid.py':0.9999999999999968,
        'MarkerNodeCoordinate.py':0.1,
        'MarkerNodeCoordinates.py':0.49999999999999867,
        'MarkerNodeODE1Coordinate.py':0.6321205587976437,
        'MarkerNodeRotationCoordinate.py':2.0,
        'MarkerBodiesRelativeTranslationCoordinate.py':0.3,
        'MarkerBodiesRelativeRotationCoordinate.py':2.0,
        'MarkerSuperElementRigid.py':0.49999999999999795,
        'MarkerKinematicTreeRigid.py':0.24999999999999897,
        'MarkerObjectODE2Coordinates.py':0.49999999999999795,
        'MarkerBodyCable2DCoordinates.py':1.0999999999999979,
        'MarkerBodyBeamShape.py':1.0999999999999979,
        'MarkerBodyCable2DShape.py':-0.0834019844819408,
        'LoadForceVector.py':0.24999999999999897,
        'LoadTorqueVector.py':0.9996440089647525,
        'LoadCoordinate.py':0.16667520761043855,
        'SensorNode.py':2.0,
        'SensorObject.py':10.000000000000009,
        'SensorBody.py':0.7, #2026-10-01: the body at [0.5,0.2] (#2764); before 0.5
        'SensorSuperElement.py':0.499999999999998,
        'SensorKinematicTree.py':0.749999999999999,
        'SensorMarker.py':0.9999999999999999,
        'SensorLoad.py':-2.0,
        'SensorUserFunction.py':2.23606797749979,
        #the nodes (#2732)
        'Node1D.py':1.99999999999999,
        'NodeGenericData.py':0.051,
        'NodeGenericODE1.py':0.36787944120235555,
        'NodeGenericODE2.py':1.0999997699320834,
        'NodePoint.py':3.5,
        'NodePoint2D.py':-2.9049999999999967,
        'NodePoint2DSlope1.py':-0.3333332497979799,
        'NodePointGround.py':0.10000000000000009,
        'NodePointSlope1.py':-0.3333332497979799,
        'NodePointSlope12.py':-0.06987553667440825,
        'NodePointSlope23.py':-0.3381249577076517,
        'NodeRigidBody2D.py':3.0,
        'NodeRigidBodyEP.py':1.5707880511179813,
        'NodeRigidBodyRotVecLG.py':1.5707963267949456,
        'NodeRigidBodyRxyz.py':1.5707963267948934,
        }
    import exudyn as exu

    if 'experimentalNewSolver' in exu.sys: #needs some corrected results
        refSol['ObjectConnectorRigidBodySpringDamper.py'] = -0.5349299542344889 #diff to other solvers: 3.6e-9

    if 'AVX2' not in exu.config.Version(True): #for nonAVX2 versions in Windows as well as other platforms
        #a build without AVX leads to a different solution: since 2022-07-11 (StateVector with ResizableVectorParallel)
        refSol['ObjectConnectorRigidBodySpringDamper.py'] = -0.534929955894111

    
    return refSol




def PerformanceTestsReferenceSolution():

    refSol = {
        #the file-level value of a model with several runs is the result of its LAST run; the
        #single runs below are what is actually judged (#2460)
        #split out of TestModels/generalContactSpheresTest.py (#2513);
        #the value is unchanged, because the copy keeps the branch the performance run took
        'generalContactSpheresPerf.py': -1.779402864432933, #2026-09-16: performance run shortened to tEnd*0.2; before: -5.98425321234168
        'perf3DRigidBodies.py':4.541173417942123, #2026-09-16: tEnd 1 -> 0.7; before: 5.307943301446709
        'perfObjectFFRFreducedOrder.py':21.00863102425483, 
        'perfRigidPendulum.py':2.4735499200766586, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 2.4745344452543323,
        'perfSpringDamperExplicit.py':0.52,
        'perfSpringDamperUserFunction.py':0.5065575310983877,
        'perfConnectorInterface.py':1.8871766817354405, #new 2026-10-01 (#2745)
        'perfLargeMassSpringChain.py':0.01426191722384829, #2026-09-16: rigid body chain, last run n=20000; before (mass points): 0.03136079550415616

        #the single runs of the models that solve several sizes or thread counts. Measured
        #2026-09-16 on Windows cp313; deterministic (repeated runs bit-identical).
        #The three contact runs solve the SAME system with 1, 4 and 8 threads, so they share one
        #value - a deviation between thread counts is a finding, not a tolerance question.
        'generalContactSpheresPerf:nt1': -1.779402864432934,
        'generalContactSpheresPerf:nt4': -1.779402864432934,
        'generalContactSpheresPerf:nt8': -1.779402864432934,
        'perfLargeMassSpringChain:rigid-n1000-explicit' : 0.00969140311923411,
        'perfLargeMassSpringChain:rigid-n1000-implicit' : 0.02018347680683519,
        'perfLargeMassSpringChain:rigid-n5000-explicit' : 0.03706120534025104,
        'perfLargeMassSpringChain:rigid-n5000-implicit' : 0.01663000270673365,
        'perfLargeMassSpringChain:rigid-n20000-explicit': 0.01426191722384829,
        #the connector interface (#2745): each pair of runs, legacy path and new one, shares its value - measured
        #2026-10-01 on Windows cp313; the pairs agree to round-off (the implicit ones to 1e-15 relative)
        'perfConnectorInterface:spring-n200-implicit-legacy':      8.222900073211406,
        'perfConnectorInterface:spring-n200-implicit':             8.222900073211406,
        'perfConnectorInterface:spring-n200-explicit-legacy':      6.071378919533291,
        'perfConnectorInterface:spring-n200-explicit':             6.071378919533291,
        'perfConnectorInterface:gravity-n200-implicit-legacy':     17.452516360952007,
        'perfConnectorInterface:gravity-n200-implicit':            17.452516360952007,
        'perfConnectorInterface:coordinate-n1000-explicit-legacy': 0.9526968492542502,
        'perfConnectorInterface:coordinate-n1000-explicit':        0.9526968492542502,
        'perfConnectorInterface:rigid-n100-explicit-legacy':       1.8871766817354405,
        'perfConnectorInterface:rigid-n100-explicit':              1.8871766817354405,
        }

    return refSol












#%%+++++++++++++++++++++++++++++++++++++++

#%%+++++++++++++++++++++++++++++++++++++++
#the SECOND reference set: what changes when the module carries the AVX2 vector extensions
#(#2470).
#
#The regular module exudynCPP is compiled for the baseline instruction set (#2466) and
#the values above are ITS values; exudynCPPfast additionally carries AVX2, and the AVX branches of
#Use_avx.h sum in a different order, which moves results. This dict is an UPDATE to the values
#above, not a copy of them: only the models that move by more than their tolerance appear here, so
#the two sets cannot drift apart for the other 80+ models.
#
#THIS LIST IS MEANT TO SHRINK. Most entries are last-digit rounding; the large ones are models
#which amplify it (stick-slip, rolling contact). Each entry is either explained and removed by a
#fix, or removed by choosing model parameters that do not amplify roundoff - see phase R10.
#Ordered by drift, largest first, so the worst offenders are the work list.
#
#Recorded 2026-09-17 on Windows cp313 with exudynCPPfast (AVX2). The module reports its vector
#extensions in exudyn.config.Version(True); testRunnerTools.ModuleUsesAVX2() reads that.
def AVX2ReferenceSolutionUpdate():

    refSolAVX2 = {
        'generalContactFrictionTests.py':         12.030182715125177,        #drift 7.5e-03
        'generalContactCylinderTest.py':          12.246626442545603,        #drift 4.2e-05
        'sphereTriangleTest2.py':                 4.356119232231812,         #drift 3.6e-05
        'generalContactImplicit1.py':             0.775815593379039,         #drift 5.3e-08
        'ANCFbeltDrive.py':                       -0.0011715990134242293,    #drift 1.0e-08; added 2026-09-17 with #2495
        'contactSphereSphereTest.py':             0.5348463536059522,        #drift 3.3e-09
        'rollingDiscTangentialForces.py':         1.0342017388721547,        #drift 1.6e-09
        'ObjectConnectorRigidBodySpringDamper.py':-0.5349299545315868,       #drift 1.4e-09
        'coordinateSpringDamperExt.py':           17.084935539925155,        #drift 5.8e-10
        #re-measured 2026-09-18 with the deterministic joint setup of #2502;
        #the Python-side change moves the fast module exactly as it moves the regular one
        'sliderCrank3Dbenchmark.py':              7.256859912756364,         #drift 2.9e-10
        'rigidBodySpringDamperIntrinsic.py':      0.5472368463500469,        #drift 5.2e-11
        'createSphereTriangleContact.py':         4.8409602192504355,        #drift 3.9e-11
        'fourBarMechanismIftomm.py':              0.1721665271840173,        #drift 6.2e-12
        'ballBearingTest.py':                     0.037852414023965573,      #drift 5.5e-12
        'solverExplicitODE1ODE2test.py':          3.3767933275970896,        #drift 5.2e-12
        'connectorRigidBodySpringDamperTest.py':  0.1827622474318292,        #drift 3.7e-12
        'ANCFgeneralContactCircle.py':            -0.5816542531620952,       #drift 3.7e-12
        'createSphereQuadContact.py':             1.124377662163088,         #drift 2.6e-12
        'rollingCoinPenaltyTest.py':              0.03489603106689881,       #drift 9.7e-13
        'bricardMechanism.py':                    4.172189649307425,         #drift 9.2e-13
        'mainSystemExtensionsTests.py':           57.64639446941554,         #drift 8.7e-13
        'rollingCoinTest.py':                     1.063438118935288,         #drift 8.3e-13
        'revoluteJointPrismaticJointTest.py':     1.2538806799249347,        #drift 7.6e-13
        'generalContactSpheresTest.py':           -1.1138547720263723,       #drift 6.3e-13
        'heavyTop.py':                            33.42312575174431,         #drift 5.0e-13
        'createKinematicTreeTest.py':             3.340830142730491,         #drift 2.4e-13
        'ConvexContactTest.py':                   0.011770267410694153,      #drift 2.0e-13
        'scissorPrismaticRevolute2D.py':          27.20255648904422,         #drift 1.6e-13
        'createSphereQuadContact2.py':            0.15616582432927872,       #drift 1.6e-13
        'genericODE2test.py':                     0.036045463499024655,      #drift 8.0e-14
        'ANCFcable2DuserFunction.py':             0.6015588367721263,        #drift 7.1e-14
        'sphericalJointTest.py':                  4.409080446575089,         #drift 6.5e-14
        'generalContactCylinderTrigsTest.py':     5.486908430912642,         #drift 6.2e-14
        }

    return refSolAVX2

#%%+++++++++++++++++++++++++++++++++++++++
#models which only the REGULAR module exudynCPP can judge, name -> reason. This is not about
#accuracy: under exudynCPPfast the value such a model produces means something different, so it is
#not run there at all (#2470). Both entries were measured on 2026-09-17.
def NotJudgedOutsideRegularModule():

    return {
        'parameterConversionTest.py':
            'its result is the number of parameter outcomes differing from the recorded behaviour, '
            'and a module without range checks legitimately reports different outcomes for invalid '
            'input - 89 of them; it records behaviour, so only the regular module can judge it',
        'exceptionTypesTest.py':
            'it counts errors that were NOT raised, and exudynCPPfast compiles the range checks '
            'away - two of its ten cases legitimately raise nothing there; the point of the model '
            'is what a user gets from the regular module',
        'NGsolveCMStest.py':
            'it loads FEM data from the tracked testData/netgenTestMesh.pkl, which carries exudyn '
            'C++ types; under a second module the load raises \'type "Real" is already registered\', '
            'the model then REGENERATES the mesh - overwriting the tracked file - and its result '
            'moves by 2.4e-8 (#2469)',
        }

