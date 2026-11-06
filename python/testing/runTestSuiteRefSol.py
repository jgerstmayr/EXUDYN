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
    #2026-10-10 (#2954): the math functions are the portable ones (EXUDYN_PORTABLE_MATH); 49 values here and 4 of the
    #mini examples were recorded again on Windows, 17 of them had moved beyond their tolerance; the values of the
    #C++ library are in git history and listed in the log of revision2026c step RG4.1.3.4
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
        'analyticInverseKinematicsTest.py': 27.199999999999996, #2026-11-05: the checks of ArmConfiguration (#3265), before 19.199999999999996; new 2026-11-05: the analytic inverse kinematics of KinematicChain and Robot (#3260)
        'numericalInverseKinematicsTest.py': 22.2, #new 2026-11-05: the Levenberg-Marquardt inverse kinematics of KinematicChain and Robot (#3269)
        'robotCartesianPathsTest.py': 24.036760791972597, #new 2026-11-05: circular, screw and spline paths, keepConfiguration, the jumps in Trajectory (#3272)
        'trajectoryBlendingTest.py': 10.527218840445627, #new 2026-11-05: the blending of the profiles of Trajectory (#3274)
        'trajectoryBlendingCartesianTest.py': 7.984211565491496, #new 2026-11-06: auto blend times, the velocity check, the blending in Cartesian space (#3283)
        'explicitStepSizeControlTest.py': 25, #new 2026-11-06: the PI controller and the absolute values in the error scale of ODE23 and DOPRI5 (#3245)
        'solverOutputWithinStepTest.py': 5.000036595791131, #new 2026-11-06: the output of the C++ solvers at the multiples of the write periods, interpolated within the step (#3286)
        'urdfRobotHDF5Test.py': 11.896535862956235, #new 2026-11-06: a robot of a URDF file in the repository, its import stored as HDF5: masses from the meshes, corrected inertia, flipped joint, texture (#3285)
        'robotInverseDynamicsTest.py': 14.002447714997835, #new 2026-11-05: KinematicChain.InverseDynamics, Robot.InverseDynamics, the dynamic compensation (#3276)
        'robotModelsTest.py': 14.429203529941592, #new 2026-11-05: the models of exudyn.robotics.models, their configurations and friction (#3277)
        'robotDictionaryTest.py': 9.516012946428752, #new 2026-11-05: Robot.GetDictionary and SetDictionary, saved with SaveDictToHDF5 (#3267)
        'robotFrictionEndStopsTest.py': 11.680923235698522, #new 2026-11-05: friction, end stops, q0, control and sensors of both models of Robot (#3270)
        'raytracerBackFaceTest.py': 0.7, #new 2026-11-05: raytracer.advanced.backFaceColor (#3275)
        'stlFileImportTest.py': 10.398437499999995, #2026-11-05: the file outward as Exudyn, a check of orientTriangles (#3275), before 9.398437499999998; new 2026-11-05: graphics.FromSTLfile without numpy-stl, the mass properties of the mesh (#3264)
        'appendSystemTest.py': 6.274157091191782,                     #new 2026-10-04: SC.AppendSystem of a copy and of the system of another container, deleted once (#2842)
        'ANCFBeamTest.py': 1.0105101312118379, #2026-10-14: the strains in the symmetric basis of the slopes (#2992), +2.4e-5; before 1.010486312300453; 2026-10-13: the frame of the slopes is their symmetric basis (#2228), +2.6e-10; before 1.0104863120355159; 2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 1.010486312035481;new 2023-04-04, after resolving local kappa bug; 2026-10-02: the rotation of the slope nodes consistent (#2763), -2.6e-10
        'ANCFcable2DuserFunction.py': 0.6015588367721794, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.6015588367721973;new 2023-12-13
        'ANCFCableBeamDampingTest.py': 0.1899233558939727, #2026-10-14: the strains of ObjectANCFBeam in the symmetric basis (#2992), +1.5e-11; before 0.18992335574301988; 2026-10-13: the symmetric basis of the slopes (#2228), +2.3e-11; before 0.1899233557201726; new 2026-03-25, checking damping between ANCFCable2D and ANCFBeam; 2026-10-02: the rotation of the slope nodes consistent (#2763)
        'ANCFbeltDrive.py': -0.0011715990134787968, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before -0.0011715885324992126;new 2026-09-17: the model was retuned (16 elements
                                                                #per section, tEnd=0.1) and the result now comes from the
                                                                #sensor rather than an ODE2 coordinate (#2368)
        'ANCFcontactCircleTest.py':-0.4842698420787613,
        'ANCFcontactFrictionTest.py':-0.014187561328096003,         #with old ObjectContactFrictionCircleCable2D until : 2022-03-09: -0.014188649931059739,
        'ANCFgeneralContactCircle.py':-0.5816542531639006, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before -0.581654253165756;new 2022-07-11 (CState Parallel); #before some update to contact module(iterations decreased!):-0.5816521429557808, #2022-02-01
        'ANCFmovingRigidBodyTest.py':-0.1289309693499045,          #new 2022-12-25; old solution differs for 1e-10 since several updates -0.12893096921737698,
        'ANCFslidingAndALEjointTest.py':-4.426408394755268,         #before 2023-05-01 (loads jacobian): -4.426408390697862,         #before 2022-12-25(resolved BUG 1274): -4.426403044189653; with old ObjectContactFrictionCircleCable2D until: 2022-03-09: -4.42640304418963,
        'ballBearingTest.py':0.037852414007063746, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 0.03785241401039906; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.037852414003386826; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 0.03785241403274696; 2026-10-01: the cage's CartesianSpringDamper on the connector interface (#2745), round-off of the projection; before 0.03785241402944885
        'bricardMechanism.py': 4.172189592784395, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 4.172189592784658; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 4.172189651042737; 2026-10-02: JointGeneric on the connector interface, reaction forces summed in another order (#2745), before 4.172189649306256; #2026-10-02: rigid joints on the connector interface, reaction forces summed in another order (#2745), before 4.172189649306508;
        'carRollingDiscTest.py':-0.2394004875461003, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before -0.23940048754619528; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -0.23940048692143728; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before -0.2394004871711386
        'compareAbaqusAnsysRotorEigenfrequencies.py':0.0004185480476228394,
        'compareFullModifiedNewton.py':0.039080737277342445, #2026-11-06: the solution file at each write period 1e-4 within the steps of 0.02 (#3286); before 0.00020079676000188396
        'complexEigenvaluesTest.py':0.4281639207875235, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.42816392078752485;new 2024-05-04 testing ComputeODE2Eigenvalues2 for complex case
        'computeODE2AEeigenvaluesTest.py': 0.3881173295041329, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.3881173295041342;
        'computeODE2EigenvaluesTest.py':-2.747979063144612e-11,
        'connectorGravityTest.py': 1014867.2330320379,
        'connectorOutputVariablesTest.py':-1.557855069028514, #new 2026-10-01: output variables of connectors and joints (#2735), from python/testing/test_connectorOutputVariables.py
        'connectorRigidBodySpringDamperTest.py':0.18276224743194636, #new 2022-07-11 (CState Parallel); 
        'contactCoordinateTest.py':0.0553131995062827,
        'contactCurveExample.py':0.30961432796813537,                #new 2025-05-11
        'contactSphereSphereTest.py':0.4416615668422053, #2026-09-30: step size recommended also where a contact ends (#2109); before 0.5348463502652304;           #new 2025-02-03
        'contactSphereSphereTestEAPM.py': 0.20000219249662216,      #new 2025-02-03
        'ConvexContactTest.py':0.011770267407335158, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 0.011770267407519278; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.011770267735619335; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 0.011770267410491468; new 2022-07-11 (CState Parallel); #before 2022-01-25?: 0.05737886603111926,
        'coordinateSpringDamperExt.py':17.432698710805155, #2026-11-02: #3204: the Jacobian with factor0/factor1, the contacts on the connector interface with dv/dq (#3201); before 17.084935539349033; new 2023-01-23
        'coordinateVectorConstraint.py':-1.0825265797698307,
        'coordinateVectorConstraintGenericODE2.py':-1.0825265797698307,
        'createKinematicTreeTest.py':3.340830142730577,            #2026-10-04: the tree on the placements of its links (#2829), -1.2e-13; before 3.3408301427307276; new 2025-06-14
        'createFFRFReducedOrderObjectTest.py':1.2839677095544506,  #new 2026-10-30: flexible pendulum, rigidBodyNodeType and localHTs of the markers (#3164)
        'createFunctionsTest.py':0.04228833970121666, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.04228833966560139; new 2025-05-11
        'createRollingDiscPenaltyTest.py':2.1129927199922123,       #new 2025-02-27
        'createRollingDiscTest.py':4.009716209090303,               #new 2025-03-05
        'createSphereQuadContact.py':1.1243943015734743, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 1.12439429771927; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.124394415274499; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 1.124394416977545; 2026-09-30: step size recommended also where a contact ends (#2109); before 1.1243776621604573;             #new 2025-06-29
        'createSphereQuadContact2.py':0.15616582432923728, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 0.15616582432943793; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.15616582432918033; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 0.1561658243294339; 2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.15616582432943388;new 2025-07-05
        'createSphereTriangleContact.py':4.840244058976285, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 4.840244059597701; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 4.840244058569561; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 4.840244058316264; 2026-09-30: step size recommended also where a contact ends (#2109); before 4.840960219289836;        #new 2026-09-11; tEnd shortened 0.65->0.25 on adding
        'deleteItemsTest.py':-0.9860528006518324,                   #new 2025-05-10
        'distanceSensor.py':1.7491018054920742, #2026-11-04: RK44 with the clamped coordinates of the cable fixed in its stages (#3240), converged (4 times the steps: 6e-10), before 1.86776431077868
        'driveTrainTest.py':-9.269827527802477e-08, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -9.269855494320467e-08; new 2023-05-20 (mainSystemExtensions); before:-9.269311940229841e-08,
        'explicitLieGroupIntegratorPythonTest.py':149.8473939540758, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 149.84739395407578;
        'explicitLieGroupIntegratorTest.py':0.1619186350154579, #2026-11-06: the PI step size controller and the absolute values in the error scale of DOPRI5 (#3245); before 0.1616401331981908
        'explicitLieGroupMBSTest.py':3.0289871079241024, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 3.028987107923892;new 2026-09-11; endTime shortened 1->0.1 on adding, step size unchanged
        'fourBarMechanismTest.py':-2.376335780518213,
        'fourBarMechanismIftomm.py':0.17216652718868367, #2026-10-01: constraints on the connector interface, reaction forces summed in another order (#2745), before 0.17216652717785863
        'generalContactCylinderTest.py':12.246910936384044, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 12.246428561485022;new 2024-03-17 (spurious trig-sphere contact forces)
        'generalContactCylinderTrigsTest.py':5.549717885864959, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 5.549717885864965;2026-10-02: the contact mesh of graphics.Cylinder from 6-node triangles - the same facets on more triangles (#2709), before 5.48690843091258; new 2024-03-17 (internal sphere-sphere contact)
        'generalContactFrictionTests.py':12.025130873085953,        #changed 2025-05-06 (seems to now be closer to linux; differences with object8); new 2024-03-17: 12.027740342293988 (doubled damping; fixed sphere-sphere and trig-sphere contact); old: 12.464092000879125,        #new 2022-07-11 (CState Parallel); #before 2022-01-25 (changed some velocity computation in GeneralContact): 10.133183086232139, #changed GeneralContact and implicit solver; before 2022-01-18: 10.132106712933348 , 
        'generalContactImplicit1.py':0.7758155406175806,             #new 2026-09-11
        'generalContactImplicit2.py':0.5000000383827361,             #2026-10-04: forced friction mode, #1947 (exact 0.5; was 0.5000000537869635)
        'generalContactSpheresTest.py':-1.113854772025744,         #new 2022-07-22 (parallel Lie group updates); new 2022-07-11 (CState Parallel); #before 2022-01-25(minor diff, due to round off errors in multithreading; now changed to 1 thread):-1.113854772026123, #changed GeneralContact and implicit solver; before 2022-01-18: -1.0947542400425323, #before 2021-12-02: -1.0947542400427703,
        'genericJointUserFunctionTest.py':1.192238398080757, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 1.1922383980810023; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.1922383967562484; 2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 1.192238396756316;
        'genericODE2test.py':0.03604546349908962, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.03604546349894506;new 2022-07-11 (CState Parallel); #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.036045463498793825,
        'geneticOptimizationTest.py':0.2253744528007068,           #2026-11-06: the sensors at each write period 2e-3 within the steps of 0.01 (#3286); before 0.10117518366826603; before 2022-02-20 (accuracy of internal sensors is higher); 0.10117518367051619, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.10117518366934351,
        'geometricallyExactBeam2Dtest.py':-2.2115028353798545, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before -2.211502835379855;2026-01-09 update due to autodifferentiation
        'geometricallyExactBeamTest.py':1.0128212332805506, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 1.0128212332805504;before 2026-09-29: 1.012820942859896 (full Jacobian, #1550); before 2023-01-29: 1.012822053539261; before 2023-05-05: 1.0128218992948643 (changed Texp function); new 2023-04-06 may still include small errors in implementation
        'gridGeomExactBeam2D.py':-1.5827965743262549,                #new 2024-01-28
        'heavyTop.py':33.42312575172178, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 33.42312575172271; 2026-10-02: loads through the marker functions (#2745), before 33.423125751743804;                            #new 2022-07-11 (CState Parallel);
        'hydraulicActuatorSimpleTest.py':7.130440021896105, #2026-11-02: the hydraulic actuator and the intrinsic rigid-body spring-damper on the connector interface (#3201), before 7.130440021870288
        'jointArgsTest.py':0.004269049505507017, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.00426904955009082; 2025-05-10
        'kinematicTreeAndMBStest.py':263.8821921198248, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 263.88120375754687; the raw value since 2026-09-24: the model used
                                                                 #to multiply it by 1e-7, which hid its tolerance inside the
                                                                 #number; it states exu.sys['testTolerance'] = 5e-7 instead,
                                                                 #which is the same comparison. Until then: 2.6388120463802584e-05
        'kinematicTreeConstraintTest.py':1.813597538299325, #2026-11-02: the Jacobian of ObjectKinematicTree its own, with d(M q_tt)/dq (#3211), before 1.813597538291927; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.8135975384620422
        'symbolicUserFunctionCopyTest.py':0.38643688092501255, #a copy through Get/SetDictionary stays symbolic (#1888)
        'flexiblePendulumBeamComparison.py':-1.5235597679330848, #2026-09-30: consistent mass matrix (#1273); before -1.508742103106691 #three beam elements, one pendulum (#2730)
        'geometricallyExactBeamMarkerTest.py':0.22853106396053818, #2026-09-30: consistent mass matrix (#1273); #body markers on the 3D beam = loads on its nodes (#2730)
        'geometricallyExactBeamJacobianTest.py':4.282489188464191, #analytic against numerical Jacobian of the 3D beam (#1550)
        'geometricallyExactBeamCurvedTest.py':4.561491685469839, #the 45-degree bend, stress-free in its curved reference configuration (#1494)
        'geometricallyExactBeamRightAngleFrame.py':3.3061810769580746, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 3.306181715978533; 2026-10-02: JointGeneric on the connector interface, reaction forces summed in another order (#2745), before 3.306181714013268; #lateral buckling of the right-angle frame at 1.088 N (#1499)
        'rightAngleFrame.py':4.371823197994703, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 4.3718231979938516; 2026-10-02: JointGeneric on the connector interface, reaction forces summed in another order (#2745), before 4.371823197993865; #the same frame driven by displacement, past the buckling point (#2762)
        'energiesFlexibleBodiesTest.py':5.850170605188874, #2026-11-02: the Jacobian of ObjectKinematicTree its own, with d(M q_tt)/dq (#3211), before 5.850170605200625; 2026-10-14: the strains of ObjectANCFBeam in the symmetric basis (#2992), +2.8e-10; before 5.850170604918973; 2026-10-05: the ANCF thin plate of the working copy 2026 (#2857), round-off; before 5.850170604919896; #elastic energy of beams, plate and kinematic tree in free oscillations (#2202)
        'accessFunctionsTest.py':1.255546526424125, #new 2026-10-02: the access functions of a body at a local position, and what Assemble() refuses (#2744); 2026-10-02: a force at the rim of ObjectRotationalMass1D (#2775)
        'computeItemTest.py':71.26280786962762, #new 2026-10-02: mbs.ComputeItem and NumericalJacobian on a double pendulum (#2779)
        'emptySystemTest.py':124.09765181769477, #new 2026-10-02: a system without coordinates through every solver, dense and sparse (#2790)
        'homogeneousTransformationParameterTest.py':8.981932851167217, #2026-11-02: the Jacobian of ObjectKinematicTree its own, with d(M q_tt)/dq (#3211), before 8.981932851056355; 2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 8.981932846823627; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 8.981932745662196; new 2026-10-03: the HT parameters of the items, ObjectGround, Create functions, rigid markers, kinematic tree, robotics (#2793, #2794, #2795, #2798, #2799)
        'simulationSettingsDeprecationTest.py':126, #new 2026-10-03: a renamed simulation setting forwards under its old name with a DeprecationWarning (#2588)
        'simulationSettingsRenamesTest.py':49, #2026-11-01: forODE2connectors has no effect (#3205); new 2026-10-04: every renamed simulation setting forwards both ways to its new place (#2813)
        'itemParameterRenamesTest.py':66, #new 2026-10-04: every renamed item parameter forwards in the dictionary, Get and SetObjectParameter (#2814)
        'binarySolutionFileTest.py':0.1888874815147666, #new 2026-10-04: the binary solution file as float or double by solution.precision (#2816)
        'solutionFileWithoutCoordinatesTest.py':53, #new 2026-10-10: a model without coordinates writes and reads a solution file with the time only (#2950)
        'sparseSystemMatricesTest.py':0.2334531495931824, #new 2026-10-10: GetSystemJacobian/GetSystemMassMatrix(sparse=True), ComputeLinearizedSystem(useSparseMatrices=True) and sparse eigenvalues against the dense ones (#1529)
        'cUtilitiesTest.py':70.25112145133696, #new 2026-10-11: the kinematics functions of exudyn.cUtilities by what they mean, and what they refuse (#2960)
        'markerNodeCoordinateReferenceTest.py':16.65, #new 2026-10-11: MarkerNodeCoordinate(includeReferenceCoordinate=True), CreateCoordinateConstraint(includeReferenceCoordinates), MarkerNodeCoordinates(includeReferenceCoordinates=False) (#990)
        'coSimulationTest.py':0.07053481177618996, #new 2026-10-11: an implicit and an explicit system coupled in the PreStepUserFunction, against one system (#1912)
        'springDamperVelocityOffsetTest.py':2.287839441565404, #new 2026-10-12: velocityOffset of the coordinate, Cartesian and rigid-body spring-dampers against the exact solution (#829, #1631)
        'userRigidBodyTest.py':13.475755021376248, #2026-10-17: the step of the derivative of J^T f by the rule of numericalDifferentiation (#2995), before 13.475755021376353; new 2026-10-12: ObjectUserBody as a rigid body on NodeRigidBodyRxyz against ObjectRigidBody, and what it refuses (#2986)
        'userANCFCable2DTest.py':7.797666638067246, #new 2026-10-12: the 2D ANCF cable as ObjectUserBody against ObjectANCFCable2D (#2987)
        'userTetrahedronTest.py':264.5322453238412, #new 2026-10-12: the linear tetrahedron as ObjectUserBody, nonlinear patch test against the exact solution (#2988)
        'graphicsDataErrorsTest.py':11, #new 2026-10-13: errors in graphics data name the item and the entry, not-a-number refused (#2304, #2305)
        'solverFailureMessageTest.py':0, #new 2026-10-13: the exception of a failed solve carries the whole message (#1977)
        'sensorUserFunctionItemsTest.py':7.537499976593686, #new 2026-10-13: SensorUserFunction with itemNumbers, the kinetic energy of a group of bodies (#2977)
        'userFunctionTimerTest.py':3, #new 2026-10-13: the solver timer userFunctions, flags only (#2007)
        'symbolicVectorUserFunctionTest.py':0.5094476793074225, #2026-10-16: with a component of a vector argument (#3007), before 0.5094818486008039; with a force of every kind of vector node (#3006), before 0.45054179156603763; new 2026-10-15: a symbolic forceUserFunction with 10 coordinates equals the Python one (#2998)
        'constraintDriftTest.py':9.796449624399628e-06, #new 2026-10-15: the drift of TrapezoidalIndex2 checked, the drift in the current state four ways (#565)
        'systemUtilitiesTest.py':1.6708368709431474, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.670836871616957; new 2026-10-16: SystemEnergy, SystemConstraintDrift and ComputeSystemGraph of exudyn.systemUtilities (#2997)
        'pythonUserFunctionArgumentsTest.py':0.5418115020412166, #new 2026-10-16: numpy arrays as vector arguments of Python user functions, userFunctionListArguments (#3008)
        'symbolicUserFunctionSignaturesTest.py':58.302161797439574, #2026-11-01: with the postNewtonStepUserFunction of ObjectGenericDAE (#3195), before 57.302161797439574; 2026-11-01: with the recordable postNewtonStepUserFunction of ObjectConnectorUser (#3194), before 56.302161797439574; 2026-11-01: with the 3 signatures of the new user functions of ObjectGenericDAE (#3191), before 53.302161797439574; 2026-10-30: with the 2 signatures of ObjectGenericDAE (#3170), before 51.302161797439574; 2026-10-26: with the forceUserFunction of the marker states of ObjectConnectorUser (#3119), before 50.302161797439574; 2026-10-24: with the 2 signatures of ObjectJointUser and the 4 of MarkerUser (#2976, #3065), before 44.302161797439574; 2026-10-23: with the massIntegralUserFunction of ObjectUserBody (#3063), before 43.302161797439574; 2026-10-17: with the 9 signatures of matrix results and ObjectGenericODE2 with mass matrix and Jacobian (#3020), before 30.255735007657506; new 2026-10-16: the 30 symbolic user function signatures and seven items against their Python versions (#2985); 2026-10-18: 43 with the 4 of NodeUser (#2974)
        'symbolicNumpyCompatibilityTest.py':306.26230077004766, #2026-10-17: with A.T, rows, slices, scalars and np.sin of vectors (#3018), before 242.87935007040414; new 2026-10-17: numpy and symbolic agree where symbolic has the operation and refuse what numpy defines differently (#3015)
        'symbolicSparseMatrixTest.py':0.19711034463698704, #new 2026-10-17: sparse matrix results of symbolic user functions with the pattern fixed when recorded, Python and symbolic, sparse and dense solver (#3022)
        'orientTrianglesTest.py':475.0, #new 2026-10-18: graphics.OrientTriangles and CheckTriangleOrientation, the STL import with orientTriangles (#2281)
        'textureTest.py':103.0, #new 2026-10-18: SC.renderer.AddTexture, textured TriangleList and 6-node triangles, GetGraphicsData texturedTriangles (#2154, #2155, #3029, #3030)
        'textureGraphicsTest.py':3684.0, #new 2026-10-18: textures in the raytracer, graphics.Quad/Cylinder/ProjectTexture/CheckerBoard with a texture, graphics.ImageFromFile (#3031, #3034, #3035, #3036)
        'symbolicExplicitConstraintTest.py':4.4070517618, #new 2026-11-04: SolveExplicit with fixed coordinates eliminated, against SolveDynamic, a coupled mass matrix, the refusal (#3238)
        'symbolicSolverOutputTest.py':186.19805581, #new 2026-11-04: the output of the symbolic solvers at the exact write times, interpolated within the step, through mbs as well (#3241)
        'symbolicTrapezoidalGGLTest.py':818.43141589, #2026-11-04: the velocity error in the estimate and the PI controller (#3243), before 420.43141589; new 2026-11-04: SolveTrapezoidalGGL of exudyn.symbolics.advancedSolvers, order 2 with constant steps on a pendulum and a slider crank, ODE1 coordinates, the automatic step size (#3242)
        'symbolicTrapezoidalGGLStepSizeTest.py':650.639057, #2026-11-04: the chain starts horizontally, before 812.639057; new 2026-11-04: the automatic step size of SolveTrapezoidalGGL on a bouncing ball and a whipping chain (#3243)
        'symbolicImplicitRungeKuttaTest.py':17.40285849, #2026-11-05: renamed from symbolicLobatto4Test.py, ODE1 and data coordinates, errorEstimate (#3253), before 16.5816714; 2026-11-04: the error estimate of the order of the method, the slider crank in 12 steps instead of 68 (#3250), before 72.5816714; 2026-11-04: SolveImplicitRungeKutta with LobattoIIIA4, LobattoIIIC4-GGL and Radau3 (#3247), before 73.73288071; 2026-11-04: the variants with the constraints at the end, the automatic step size (#3246), before 4.58157969; new 2026-11-04: SolveLobatto4, IIIC-GGL, IIIA-GGL and IIIC, orders on a pendulum and a slider crank, a stiff spring (#3244)
        'symbolicEulerParametersTest.py':6.24213131, #new 2026-11-04: the user rigid body on Euler parameters, its node equation in SymbolicSystem, SolveExplicit with the projection (#3238)
        'rendererDictionaryTest.py':11, #new 2026-11-05: SC.renderer.GetDictionary and SetDictionary (#2075)
        'rendererItemChangesTest.py':1.1932480692, #new 2026-11-04: mbs.Reset, item changes and Assemble under the lock of the system, which the render thread takes too (#3239)
        'scopeTest.py':0, #new 2026-11-04: the scopes of the render window in the raytracer image, the automatic mode, the sensors as rows of 2 or 5 (#3237)
        'zoomToItemTest.py':0.02, #new 2026-10-18: SC.renderer.ZoomToItem, centered and zoomed to an item (#3027)
        'userNodeTest.py':30.45974515191901, #new 2026-10-18: NodeUser against NodePoint and NodeRigidBodyRxyz, Jacobians given and numerical (#2974)
        'userANCFBeam3DTest.py':-0.0057294228573775445, #new 2026-10-19: fully parametrized ANCF beam on NodeUser and ObjectUserBody, tolerance 1e-9 of its own (#2974)
        'initialConstraintsCheckTest.py':2.937101858808996, #new 2026-10-14: the check of the initial conditions against the constraints, the projection of the initial velocities (#2234)
        'shellMeshSlopesScalingTest.py':0.0005808783959917463, #new 2026-10-14: the slope scaling of ShellMesh on a quarter cylinder and a flat trapezoid (#2226)
        'systemContainerComputeOnlyTest.py':4.0, #new 2026-10-14: SystemContainer(attachToRenderer=False), the renderer stays with the container of the model, also for InverseKinematicsNumerical (#2259)
        'ANCFBeamGeometricallyExactComparisonTest.py':5.999501744103005, #2026-10-14: the strains of ObjectANCFBeam in the symmetric basis (#2992); before 5.9995017472560175; new 2026-10-13: ObjectANCFBeam against ObjectBeamGeometricallyExact, tip torque on node and element (#2228)
        'ANCFThinPlateRotationTest.py':0.009394817004517305, #new 2026-10-13: the frame of ObjectANCFThinPlate and NodePointSlope12, torque on element and node (#2228)
        'multithreadingUserFunctionTest.py':0.04949999999999993, #new 2026-10-25: connectors with Python and symbolic user functions with 4 threads against 1 thread (#3093)
        'symbolicItemOutputTest.py':3.420794832400101, #new 2026-10-25: outputs of nodes, bodies, markers, sensors and connectors read by symbolic user functions at every call, against Python and 4 threads, the refusals (#2984)
        'symbolicOptimizeTest.py':212.46417414604238, #new 2026-10-25: exudyn.symbolic.Optimize, the operations per level, the values, the nodes freed, CreateSymbolicUserFunction(optimize) (#3085)
        'symbolicDeepRecordingTest.py':41.235299378685376, #new 2026-10-25: a recording of 200000 operations recorded, measured and deleted, Evaluate and Diff refused above 10000, a deep symbolic user function, SetValue refused while recording (#3090)
        'symbolicUserFunctionThreadsTest.py':860.0074721246586, #new 2026-10-25: symbolic user functions and user bodies in the threads, user functions set during the simulation, a Python postNewtonStepUserFunction, 4 threads against 1 thread (#2978, #3095)
        'symbolicExportTest.py':11450.057870734236, #new 2026-10-25: GetFlatProgramData, GetFlatProgram and ToString in the formats Python, symbolic, numpy and sympy against the recordings (#3087, #3088)
        'symbolicFunctionTest.py':0, #new 2026-10-29: VariableSet as scope of named Reals, Vectors and Matrices, esym.Function of declared inputs: Evaluate, EvaluateJacobian, numpy code, esym.variables, the refusals (#3149)
        'symbolicStatisticsTest.py':0, #new 2026-10-28: esym.GetStatistics and GetStatistics() of expressions and recordings, the counts by hand, a tree of 2^61-1 nodes, Optimize, flatOperations (#3142)
        'symbolicSystemTest.py':3, #new 2026-10-26: SymbolicSystem, 3 kinds of user bodies and a generic ODE2, mass matrix, forces and residual equal to the solver (#3110)
        'symbolicExplicitSolverTest.py':1, #new 2026-10-26: SolveExplicit, explicit Euler, RK44 and velocity Verlet on a SymbolicSystem equal to SolveDynamic, NumpySystem (#3112)
        'symbolicGeneralizedAlphaTest.py':1, #new 2026-10-27: SolveGeneralizedAlpha on a stiff chain of recorded user items against SolveDynamic, generalized-alpha, Newmark, modified Newton (#3126)
        'symbolicMassMatrixDerivativeTest.py':1, #new 2026-11-02: Newton with and without d(M q_tt)/dq on rigid bodies on Tait-Bryan angles, 417 / 614 iterations (#3228)
        'symbolicRigidBodyInertiaTest.py':0, #new 2026-11-02: RigidBodyInertia of expressions, the symbolic inertia of a user rigid body in SymbolicSystem, its derivatives by the parameters (#3159)
        'symbolicParameterJacobianTest.py':0, #new 2026-11-03: the parameters as an input of SymbolicSystem, SetParameterValues without a rebuild, the derivatives by the parameters (#3159)
        'symbolicJointJacobianTest.py':0, #new 2026-11-03: JointJacobianUserFunction, the screw joint by its constraints, the difference of twists (#3235)
        'symbolicUserItemsCreateExamplesTest.py':0, #new 2026-11-03: the examples of the Create functions with the user items of UserItemCreator against the built-in items
        'symbolicVariableScopeTest.py':0, #new 2026-11-02: the scopes of the named symbolic variables, mbs.symbolicVariables and esym.variables, the refusals, Reset, the dictionary, the deprecation flag (#3208)
        'symbolicSystemEquationsTest.py':1, #new 2026-10-28: SymbolicSystem.Equations and GetFlatProgram, the equations of a generic ODE2 system and of a user rigid body with a mass point, connectors and loads as exudyn.symbolic and sympy expressions equal to the system (#3130)
        'symbolicOnePassJacobianTest.py':1, #new 2026-10-28: EvaluateJacobian in one pass over the flat program, every operation with a rule, against the tree and central differences (#3116)
        'symbolicSystemJointsTest.py':1, #new 2026-10-28: SymbolicSystem with the ObjectJointUser of UserItemCreator: constraint equations, reaction forces against central differences, SolveGeneralizedAlpha equal to SolveDynamic (#3141)
        'symbolicSystemODE1Test.py':0, #new 2026-10-28: ObjectGenericODE1 with recorded rhsUserFunction and LoadCoordinate on ODE1 coordinates in SymbolicSystem, ODE1RightHandSide equal to the solver, ODE1Jacobian, SolveGeneralizedAlpha and SolveExplicit equal to SolveDynamic (#3144)
        'symbolicSystemGenericDAETest.py':3.952459921895505, #new 2026-11-01: ObjectGenericDAE in SymbolicSystem, SolveGeneralizedAlpha against SolveDynamic, the exact SystemJacobian, the export (#3190)
        'genericDAEUserFunctionsTest.py':8.59069847555063, #new 2026-11-01: massMatrixUserFunction, constraintUserFunction and jacobianUserFunction of ObjectGenericDAE, Python and recorded, against built-in items and SymbolicSystem (#3191)
        'symbolicLookupTableTest.py':11.007450360798327, #2026-11-01: IfThenElse of the utilities (#3203), before 8.007450360798327; new 2026-11-01: esym.LookupTable against numpy.interp, its derivatives, Optimize, the export, a spring by its characteristic curve, SampleTime and Noise (#3192)
        'symbolicUserConnectorsTest.py':6.340335689918898, #2026-11-01: also one rigid-body spring-damper of each formulation in one system (#3203), before 4.340335689918898; new 2026-11-01: CreateCartesianSpringDamper, CreateRigidBodySpringDamper, CreateCoordinateConstraint, CreateTorque of UserItemCreator against the built-in items and SymbolicSystem (#3193)
        'ode1LoadsJacobianTest.py':2.927867492140418, #new 2026-11-02: the Jacobian of loads on ODE1 coordinates and of the ODE1 dependencies of loads, AddODE1LoadDependencies, fewer Newton iterations (#1548)
        'genericDAEDataTest.py':3.0016605544097006, #new 2026-11-01: data nodes and postNewtonStepUserFunction of ObjectGenericDAE, a held controller against a PreStepUserFunction (#3195)
        'symbolicSystemDataTest.py':5.237348361433805, #new 2026-11-01: SymbolicSystem with data coordinates and post-Newton steps, the discontinuous iteration of the Python solvers against SolveDynamic (#3196)
        'symbolicSparseMatrixProductTest.py':5.994733361306955, #new 2026-11-01: SparseMatrix against the dense product, its operations, the export, a recorded chain of springs (#3199)
        'symbolicSystemODE1AETest.py':1, #new 2026-10-28: ODE1 and algebraic coordinates in SymbolicSystemBase: a NumpySystem of three masses, a lag and a constraint against ObjectGenericODE2, ObjectGenericODE1 and ObjectConnectorCoordinate, SolveGeneralizedAlpha and SolveExplicit equal to SolveDynamic (#3134)
        'symbolicSystemParametersTest.py':0, #new 2026-10-29: named exudyn.symbolic.Real as parameters of user items: UserItemCreator keeps them, Equations() keeps m1 and k, the derivatives by them against central differences by SetParameterValues, the numpy code with them as arguments, a wrong value refused (#3152)
        'symbolicSystemVariablesTest.py':0, #new 2026-10-29: SystemVariables, the system variables as one VariableSet and the marker states as expressions of them, equal to the marker outputs of mbs, a Jacobian against central differences, Equations() in them (#3151)
        'symbolicSystemLoadsTest.py':1, #new 2026-10-27: the built-in loads in SymbolicSystem on point nodes, coordinates and a user rigid body, SolveExplicit equal to SolveDynamic, the Jacobian, the refusals (#3118)
        'symbolicUserJointsTest.py':1, #new 2026-10-28: UserItemCreator, revolute, prismatic and spherical joints and a distance constraint as ObjectJointUser, equal to the built-in joints, Python and recorded (#3140)
        'symbolicUserRigidBodiesTest.py':1, #new 2026-10-28: UserItemCreator, a chain of user rigid bodies with spring-dampers and torsional spring-dampers by the Create functions of mbs, equal to the built-in items, Python and recorded, RK44 and generalized-alpha (#3128); as SymbolicSystem, SolveExplicit and SolveGeneralizedAlpha equal to SolveDynamic, the Jacobian, the times (#3129)
        'symbolicJacobianExpressionsTest.py':1, #new 2026-10-29: esym.Jacobian, the derivative as new expressions, against EvaluateJacobian, the Hessian (#3163)
        'symbolicSystemItemJacobiansTest.py':1, #2026-10-29: against central differences and SolveDynamic, the chain rule removed (#3169); new 2026-10-29: the one-pass Jacobians of the connectors and joints of SymbolicSystem (#3166)
        'symbolicUserItemsDrawingTest.py':0, #new 2026-10-29: the connectors and joints of UserItemCreator drawn by the graphicsDataUserFunction of ObjectConnectorUser and ObjectJointUser, in GetGraphicsData, hidden with connectors.show = False, following the bodies (#3147, #3161)
        'symbolicUserTorsionalTurnsTest.py':18.07629768112802, #new 2026-11-02: CreateTorsionalSpringDamper of UserItemCreator with unlimitedRotations, the turns in a data node, against the built-in item and SymbolicSystem (#3218)
        'symbolicUserContactFrictionTest.py':8.713599747497733, #new 2026-11-02: CreateSphereSphereContact of UserItemCreator with friction, a ball sliding until it rolls, against the built-in contact and SymbolicSystem (#3219)
        'symbolicSolversAdaptiveStepTest.py':6.282526878006822, #new 2026-11-02: SolveGeneralizedAlpha and SolveExplicit with the recommended step size of a contact, against SolveDynamic (#3220)
        'symbolicUserContactTest.py':8.784501778567048, #2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 8.784501778454816; 2026-11-01: switched by a data node, both impact models, against built-in and SymbolicSystem (#3197), before 0.8675607974381709; new 2026-11-01: CreateSphereSphereContact of UserItemCreator against mbs.CreateSphereSphereContact, recorded, SymbolicSystem (#3187)
        'symbolicUserItemsTest.py':1, #new 2026-10-27: UserItemCreator, a chain of user mass points and spring-dampers by the Create functions of mbs, equal to the built-in items, recorded, SolveExplicit (#3125)
        'symbolicVectorDiffTest.py':1, #new 2026-10-26: Diff of symbolic vectors and matrices, EvaluateJacobian, cMath with symbolic arguments (#3115, #3114)
        'symbolicHTTest.py':0, #new 2026-11-01: kinematics.HT and Twist against exudyn.HT and exudyn.cUtilities, RelativeTwist and esym.Jacobian against central differences (#3174)
        'symbolicUtilitiesTest.py':7.024076141931356, #new 2026-10-23: exudyn.symbolics.kinematics with exudyn.symbolic, numpy, math and cMath against cUtilities (#3069)
        'userJointTest.py':28.411748466374412, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 28.411748466347245; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 28.411748466345937; 2026-10-25: the screw over 2.5 turns, periodic, with its jacobianUserFunction, a missing Jacobian refused (#3100, #3101), before -1.8272301913592162; new 2026-10-24: ObjectJointUser, a screw joint against the analytic solution at index 3 and 2, a spherical joint against ObjectJointSpherical, the refusals (#2976)
        'userMarkerTest.py':-0.3569348653534026, #new 2026-10-24: MarkerUser, the midpoint of two mass points and a rigid marker in a revolute joint against built-in markers, the refusals (#3065)
        'userConnectorContactTest.py':26.270810078736748, #2026-11-06: the sensors at the multiples of the write period within the reduced steps (#3286); before 26.270810078732666; new 2026-10-25: ObjectContactSphereSphere as ObjectConnectorUser with data node and postNewtonStepUserFunction, against the built-in contact, the refusals (#3102)
        'userConnectorTest.py':0.3942810318188026, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 0.3942810318186798; new 2026-10-23: ObjectConnectorUser with ConnectorMarkers: a spring-damper, a rigid body, a ball on an ANCF cable, the refusals (#2975)
        'userGraphicsDataTest.py':5.65312, #new 2026-10-23: graphicsData of ObjectUserBody in local coordinates, mapped by the positionUserFunction, against ObjectRigidBody and on a bent user cable, Python and symbolic (#3064)
        'userMassIntegralTest.py':-5.686955690889934, #new 2026-10-23: MarkerBodyMass and LoadMassProportional on ObjectUserBody by its massIntegralUserFunction, a user ANCF cable and a mass point (#3063)
        'userOutputVariablesTest.py':2.0832023621123557, #new 2026-10-22: strain, potential energy and acceleration of ObjectUserBody by its outputVariableUserFunction, Python and symbolic (#3057)
        'userParametersTest.py':-0.1640216949061788, #new 2026-10-22: the parameters of ObjectUserBody, one symbolic recording for two parameter sets, ParameterLayout (#3056)
        'userTypesTest.py':2.0, #new 2026-10-12: the user types of OutputVariableType, NodeType, MarkerType, their names and aliases, ObjectUserBody.outputVariableUserFunction (#2989)
        'rotationMarkerDeprecationTest.py':-0.1340286192730491, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -0.1340286194620787; new 2026-10-03: rotationMarker0/1 against localHT of the markers, the deprecation warning once per session (#2745, #2801)
        'contactFrictionCircleCable2DnoFrictionTest.py':0.8052196344904814, #new 2026-10-05: no tangential force without a friction model, whatever the slip state (#1290); 1000 steps 2026-10-09 (#2912)
        'systemContainerLifetimeTest.py':-3.809999999999995, #new 2026-10-05: the MainSystem of AddSystem and GetSystem keeps its SystemContainer alive (#2851)
        'contactCurveCirclesFrictionTest.py':8.399999999953684, #new 2026-10-07: friction and output variables of ObjectContactCurveCircles (#2867)
        'coordinateSpringDamperExtVelocityOffsetTest.py':0.6999999999999977, #new 2026-10-07: velocityOffset in the damper force (#2867)
        'fileHeaderThreadsTest.py':33,                                #new 2026-10-07: the threads in the version line of the file headers (#1450)
        'connectorFrameOutputsTest.py':26, #2026-10-06: SlidingCoordinate of JointSliding2D counted once (#2872), before 25; new 2026-10-06: the frame outputs of joints and connectors, HomogeneousTransformation(Local), Displacement, the 2D joints (#2870)
        'createLinearSpringDamperTest.py':2.9930635558130834, #new 2026-10-05: mbs.CreateLinearSpringDamper, static deflection in three ways and a dynamic run (#1953, #1954)
        'initialAccelerationsTest.py':-2.587, #new 2026-10-05: the initial accelerations of the generalized-alpha method with a constraint and initial velocity (#2235)
        'genericODE2matrixFormatsTest.py':2.0892372406725492, #new 2026-10-05: ObjectGenericODE2 with dense and sparse matrices and solvers (#777)
        'restartFileTest.py':-2.93185, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -2.931759; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before -2.93176; 2026-10-06: the final state rounded to 6 digits, it differs on Linux in the 10th (#2874), before -2.9317602918648324; new 2026-10-05: a simulation continued from its restart file computes what the uninterrupted one computes, generalized-alpha and RK44 (#2850)
        'createItemNumbersTest.py':10.515707884914624, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 10.51570788484358; new 2026-10-05: the Create functions take bodies, nodes or markers in itemNumbers, bodyNumbers deprecated (#2863)
        'implicitRungeKuttaSolversTest.py': 206.0, #new 2026-11-05: the C++ solvers TrapezoidalGGL, LobattoIIIA4, LobattoIIIC4GGL, Radau3: their orders and the automatic step size (#3261)
        'kinematicChainTest.py':37.53287798230574, #new 2026-11-05: exudyn.cUtilities.KinematicChain against the Python functions of Robot (#3259)
        'libraryDeprecationTest.py':4, #new 2026-10-03: a deprecated function and argument of the Python library warn once per session at the user's line and are counted (#2807)
        'modelLibraryTest.py':18.4910098095576, #new 2026-11-05: the models of modelLibrary.py, each with its step and half of it (#3257)
        'homogeneousTransformationInterfaceTest.py':9.245684006275722, #new 2026-10-03: exu.HT with the node coordinates, Relative, the interpolations, angle and axis (#2810)
        'homogeneousTransformationTest.py':178.9377398330175, #new 2026-10-02: exudyn.HT against the 4x4 matrices of rigidBodyUtilities, and the output variable HomogeneousTransformation (#2780)
        'inspectTest.py':49, #2026-10-04: the node types a node marker requests (#2817), before 47; 2026-10-02: the output variable HomogeneousTransformation of the rigid body and its node (#2780), before 45; 2026-10-02: a rigid body reports PotentialEnergy = 0 (#2202), before 44; new 2026-10-01: mbs.Inspect (#2203); the number of listed members, refusals included
        'energiesTest.py':17.614884358875663, #kinetic and potential energy of simple bodies and spring-dampers (#2202); 2026-10-01 with what the energies refuse and the gravity connector; before 8.724884363749222
        'geometricallyExactBeamOutputTest.py':-6.07948751391636, #section forces, moments and strains of the 3D beam (#2753)
        'geometricallyExactBeamElbowCantilever.py':-5.579926018587773, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -5.579926018617533; 2026-09-30: consistent mass matrix (#1273); #right-angle cantilever of Simo and Vu-Quoc 1988, free oscillations (#2730)
        'geometricallyExactBeamMassTest.py':2.9942070791403967, #consistent mass matrix of the 3D beam: rigid motion exact (#1273)
        'geometricallyExactBeam2DquadraticTest.py':0.7426300926718995, #the 3-node planar geometrically exact beam (#2208)
        'genericODE1duplicateNodeTest.py':3.1091750014354522, #2026-10-29: by a rhsUserFunction, so still numerical (#3143); #numerical ODE1 Jacobian with a coordinate addressed twice (#1424)
        'genericDAESymbolicTest.py':3.372634031542532, #new 2026-11-01: ObjectGenericDAE with recorded force and rhs user functions, their Jacobians in one pass against the numerical ones (#3189)
        'genericDAETest.py':4.961903224846149, #new 2026-10-30: ObjectGenericDAE against ObjectGenericODE2, ObjectGenericODE1, a CoordinateConstraint and SolveGeneralizedAlpha (#3170)
        'genericODE1JacobianTest.py':5.4903220576244625, #new 2026-10-29: the analytic ODE1 Jacobian of ObjectGenericODE1, the system matrix, against the numerical one of a rhsUserFunction (#3143)
        'contactSphereTorusMomentumTest.py':4.227231105381315, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 4.227231105386786; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 4.2272311053853535; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 4.227231105376637; 2026-09-30: step size recommended also where a contact ends (#2109); before 4.227231105610667; #momentum conservation of the sphere-torus contact (#2127)
        'explicitSolversPostNewtonTest.py':4.557718115990415, #2026-11-06: the PI step size controller of DOPRI5 and ODE23 (#3245); before 4.563482811411022; 2026-11-06: the sensors at the multiples of the write period within the recommended steps (#3286); before 4.563482002223684; 2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 4.563482002223224;2026-10-01: Jacobian of the torsional spring-damper by AD (#2745); before 4.563482002223096;  #2026-09-30: DOPRI5 with large steps added, and the step size recommended where a contact ends (#2109); before 4.097033066855782; #PostNewton states with every explicit integrator (#2754)
        'contactComparisonTest.py':4.773734739232028, #2026-10-05: GeneralContact with keepContactWhilePenetrating as a fifth case (#2848); before 4.561317500364976; #2026-10-04: GeneralContact as a fourth case and the sliding ball (#1848, #1947); before 1.200116276111891; #2026-09-30: step size recommended also where a contact ends (#2109); before 1.200040705928356; #deepest points and end heights, three contact objects, two laws (#2749, #2750)
        'generalContactTriangleMomentumTest.py':4.035299999516741, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 4.035299999516676; new 2026-10-05: momentum conservation of the GeneralContact sphere-triangle contact (#2849)
        'inactiveConnectorForceTest.py':16204.677320386909, #new 2026-10-05: an inactive connector reports no force (#984)
        'ANCFThinPlateRevisionTest.py':20, #2026-10-05: ShellMesh instead of the builder, the thickness function, the visualization settings (#2859, #2864); before 18; #new 2026-10-05: the tests of the ANCF thin plate of the working copy 2026, the number of tests that passed (#2857, #2858)
        'kinematicTreeFloatingBaseTest.py':-0.35177115398691783, #new 2026-11-02: the floating base on three kinds of nodes against a redundant model, RK44 on the Lie group node (#1549)
        'kinematicTreeMarkerCacheTest.py':2.597552382049265, #2026-11-02: the Jacobian of ObjectKinematicTree its own, with d(M q_tt)/dq (#3211), before 2.5975523820602433; new 2026-11-02: the markers on the cache of the frames, one and four threads (#3229)
        'kinematicTreePrismaticJacobianTest.py':1.249999999999995, #5 cases of 0.25 (#2740)
        'kinematicTreeTest.py':-1.3093839602164108, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before -1.309383960216409;
        'kinematicTreeUserFunctionTest.py': 0.1452255602102048,      #2026-11-02: the Jacobian of ObjectKinematicTree its own, with d(M q_tt)/dq (#3211), before 0.14522556021139238; new 2026-10-04: the forceUserFunction of ObjectKinematicTree gets the velocities (#2845)
        'laserScannerTest.py':2.695064443768281 ,                   #new 2024-04-29
        'linearFEMgenericODE2.py': 0.38767197129755937,              #new 2024-10-06 for jacobianUserFunction in GenericODE2
        'loadUserFunctionTest.py': 1.8051173706570725, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 1.8051173706570727;new 2024-10-10 for visualization of time-dependent loads
        'LShapeGeomExactBeam2D.py':-0.9181474510515215,             #2026-01-09 update due to autodifferentiation
        'mainSystemExtensionsTests.py': 57.64639447537324, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 57.646394475373; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 57.64639446941533; 2026-10-02: loads through the marker functions (#2745), before 57.64639446941518; JointGeneric on the connector interface, reaction forces summed in another order (#2745), before 57.64639446941507; #2026-10-01: constraints on the connector interface, reaction forces summed in another order (#2745), before 57.646394469414666;          #updated 2023-11-16; updated 2023-06-09; old: new 2023-05-19
        'mainSystemUserFunctionsTest.py': 4.069301305919595,        #new 2024-10-17
        'manualExplicitIntegrator.py':2.0596986296922988,
        'matrixContainerTest.py':56.5,                              #new 2024-10-09
        'mecanumWheelRollingDiscTest.py':3.8895688162157533, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 3.8895688163077313; 2026-10-14: a Trajectory and TorsionalSpringDamper motors, 4 s (#948); before 0.2714267238324342 with torques controlled in a load user function, 0.5 s
        'movingGroundRobotTest.py':0.003840899498205428, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.0038408994979840755; updated 2026-09-09, with factor 0.5 for tolerance too close
        'NGsolveCMStest.py': 0.06953224923173146,                   #re-measured 2026-09-16 (#2466) against the COMMITTED testData/netgenTestMesh.pkl; the model rewrites that tracked file whenever its load fails, and the result then moves by 2.4e-8 (#2469); changed 2025-05-05 (new .pkl file with newer ngsolve); until 2024-10-11: 0.06953227339277462
        'objectFFRFreducedOrderAccelerations.py':0.10000570245889191,#before 2022-07-22 (because often small fails); 0.5000285122944431,#before 2022-02-20 (accuracy of internal sensors is higher): 0.5000285122930983,
        'objectFFRFreducedOrderTest.py':0.005355233268058772,      #until 2022-03-18 (div result by 5): 0.026776166340247865,
        'objectFFRFTest.py':0.0064600108120842666,                  #before 2022-02-20 (accuracy of internal sensors is higher): 0.006460010812070858,
        'objectFFRFTest2.py':0.035521880690182486,                   #before 2022-02-20 (accuracy of internal sensors is higher): 0.03552188069032863,
        'objectGenericODE2Test.py':-2.3163788975858212e-05, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before -2.316378897585508e-05;
        'PARTS_ATEs_moving.py':0.44656762760262064, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.4465676276026208;
        'pendulumFriction.py':0.3999999877698231, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.3999999877698214;
        'parameterConversionTest.py':0.0,                             #new 2026-09-14: number of differences to parameterConversionTestReference.txt
        'typeInformationTest.py':0.0,                                 #new 2026-09-15: number of disagreements of exudyn.types with the C++ module
        'exceptionTypesTest.py':0.0,                                  #new 2026-09-18: number of provoked errors that raised NOTHING
        'pickleCopyMbs.py':0.2583013564102547, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.2583013564103506; new 2025-05-10
        'plotSensorTest.py':1.0,
        'postNewtonStepContactTest.py':0.057286638346409235,
        'raytracerNOGLFWtest.py':0.28013531271596,                  #2026-10-10: the edges of the bricks in front of the focal plane are drawn (#2951); before 0.28161678591179; 2026-10-01: the node of the rigid body is raytraced as a sphere (#2709); new 2026-01-03
        'reevingSystemSpringsTest.py':2.2155575717433043, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 2.215557571743304;new 2023-07-17 (old solution contained compression forces: 2.213190117855691),
        'relativeRotationTranslationMechanism.py': 1.509631854432837, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.509631854432179; new 2026-09-11
        'resultsMonitorTest.py': 1.0,                               #new 2026-09-19; exudyn.misc.resultsMonitor: the four file types, incremental reading, the CLI return codes
        'revoluteJointPrismaticJointTest.py':1.2538806796368958, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 1.253880679636072; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.253880679925025; 2026-10-02: rigid joints on the connector interface, reaction forces summed in another order (#2745), before 1.2538806799241744;    #new 2022-07-11 (CState Parallel); #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver (modified Newton restart, etc.); before 2022-01-18: 1.2538806799243265,
        'robotTrajectoryProfilesTest.py': 24.364104965254423, #new 2026-11-05: ProfilePolynomial5, ProfileCartesianLinear on the Staubli TX2-90L (#3262)
        'rigidBody2Dtest.py': -0.5055295700922421,                  #new 2025-02-05: added arbitrary COM to 2D rigid body
        'rigidBodyAsUserFunctionTest.py':8.95086527155215,
        'rigidBodyCOMtest.py':3.4094315141663936, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 3.409431467726291
        'rigidBodySpringDamperIntrinsic.py':0.5472368463004207, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 0.5472368463072981; 2026-11-02: the hydraulic actuator and the intrinsic rigid-body spring-damper on the connector interface (#3201), before 0.5472368463287056; 2026-10-01: on the connector interface (#2745), round-off of the projection; before 0.5472368462985283; new 2023-11-30 (intrinsic formulation for rigid body spring damper)
        'rollingCoinTest.py':1.0634381189351096, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 1.0634381189361193; until 2024-04-29 (without force): 0.0020040999273379673
        'rollingDiscTangentialForces.py':1.0342017423165806, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 1.0342017348669337; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 1.034201740283854; 2026-10-02: JointGeneric on the connector interface, reaction forces summed in another order (#2745), before 1.034201740253807; #2026-10-01: constraints on the connector interface, reaction forces summed in another order (#2745), before 1.0342017404650015;        #new 2024-05-04: RollingDiscPenalty: switch to local computation of tangential forces
        'rollingCoinPenaltyTest.py':0.03489603106780028, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 0.03489603106786523; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.03489603106322919; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 0.03489603106786701
        'rotatingTableTest.py':7.838280050939175, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 7.838280045161607; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 7.838280044952864; 2026-10-15: step size 0.0025 instead of 0.005, the drift of TrapezoidalIndex2 below constraintDriftTolerance (#565); before 7.838680414273868; 2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 7.838680414273867;2026-10-01: Jacobian of the rigid-body spring-damper by AD (#2745), before 7.838680375029852; until 2024-05-04 (before slight change in RollingDiscPenalty): 7.838680371309492
        'scissorPrismaticRevolute2D.py':26.722556541316987,          #2026-11-01: the analytic connector Jacobians, 48 Newton iterations fewer, as forODE2Connectors has no effect (#3205); 2022-07-11 (CState Parallel): 27.202556489044277
        'sensorUserFunctionTest.py':45.0,            
        'serialRobotTest.py':0.7681856910283053, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.7681856909831933; until 2022-04-21: 0.7680031232063571 wrong static torque compensation
        #value changed 2026-09-18 (#2502): the joint helpers now
        #convert the joint position with a fixed summation order, so this result no longer
        #depends on which numpy release built the marker positions. It is the value numpy
        #2.2.4 produced and the one an explicit sum produces; the old 7.256859912845965 was
        #what numpy 2.4.6's matmul happened to give
        'sliderCrank3Dbenchmark.py':5.7911915103432765, #2026-11-05: the IFToMM model of 2026 (#2326) - Lie group nodes, joints in the body frames, generalized-alpha with the default spectral radius - replaces the former model (Euler parameters, 7.2568600591641665); tEnd 0.5 of the 5 s of the benchmark
        'sliderCrank3Dtest.py':3.3642761721569, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 3.364276178092191
        'sliderCrankFloatingTest.py':0.5916491633788337,
        'solverExplicitODE1ODE2test.py':2.0696926769200483, #2026-11-06: the PI step size controller and the absolute values in the error scale of DOPRI5, smaller errors (#3245); before 3.3767933275918964; new 2022-07-11 (CState Parallel); 
        'sparseMatrixSpringDamperTest.py':-0.06779862812271391,     #changed to analytic Spring-Damper jacobian (missing d(vel)/dpos term): -0.06779862983767654,
        'sphereTriangleTest2.py':4.356293936759868, #2026-11-02: the rigid body's own Jacobian by automatic differentiation (#3209), before 4.356293936758741; 2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 4.356293936760721; 2026-11-02: the contacts on the connector interface with dv/dq (#3201); before 4.356293935400209; 2026-09-30: step size recommended also where a contact ends (#2109); before 4.35608275479331;                 #changed: 2026-01-23 (sparse acc(vel) initialization); new 2025-06-22
        'sphericalJointTest.py':4.4090804459244035, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 4.409080446574851; 2026-10-01: constraints on the connector interface, reaction forces summed in another order (#2745), before 4.409080446575154;                  #new 2022-07-11 (CState Parallel);
        'springDamperUserFunctionTest.py':0.5062872273010876, #2026-10-11: initial values from exudyn.cMath and cUtilities (#2962, #2963); before 0.5062872273010924;
        'stiffFlyballGovernor.py':0.8962488779114737,
        'superElementRigidJointTest.py':0.015217208913989099,       #before 2022-02-20 (accuracy of internal sensors is higher): 0.015217208913983024,
        'symbolicUserFunctionTest.py':0.10039884426884882,          #2023-12-13
        'symbolicModuleTest.py':0.9484129575069745,                 #2023-12-14
        'taskmanagerTest.py':-0.23406814273662005,                  #2026-10-29: the analytic Jacobian of ObjectGenericODE1 (#3167); before -0.23406814272950313; #2024-10-08
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
        'modelLibrary.py':
            'the library of models for tests (GetModel), not a test model; modelLibraryTest.py runs it (#3257)',
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
        #the energies after a dynamic run: -5.9e-11 (rel. 3.3e-12) with numpy 2.2 (venvP310, Windows, 2026-10-07) and on
        #Linux (WSL, 1.12.372) against the reference of numpy 2.4; a round-off difference, not a platform one (#2900)
        'energiesTest.py': 1e4,
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

    #2026-10-10 (#2954): with the portable math functions, nine of the twelve entries agree with Windows to the last
    #bit (manylinux_2_28 / cp313 / numpy 2.5.3 in docker, and WSL / glibc 2.39 / numpy 2.2.4): coordinateSpringDamperExt,
    #rigidBodySpringDamperIntrinsic, rollingDiscTangentialForces, contactSphereSphereTest, sphereTriangleTest2,
    #generalContactFrictionTests, sliderCrank3Dbenchmark, generalContactImplicit1, computeItemTest - their differences
    #came from the math library.
    #2026-10-11 (#2963): the last three - generalContactCylinderTest, createSphereTriangleContact,
    #createSphereQuadContact - computed initial values and points with math.sin and math.cos of the platform; with
    #exudyn.cMath they agree with Windows to the last bit (WSL / glibc 2.39 / numpy 2.2.4)
    unresolved = set([])

    return unresolved

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def UnresolvedOnMacOS():
    """the models whose result differs on macOS by more than their tolerance

    What UnresolvedOnLinux() is to Linux. The reference values are the Windows ones, so a difference
    here is the same kind of reproducible platform difference and not a fault of the model. The set
    also applies to Linux on ARM (aarch64).

    Since the portable math functions (#2954) and Eigen without fused multiply-add on arm64 (#2990),
    macOS ARM gives the Windows results: V1.13.90.dev1, Python 3.13
    (tmp/MacOS/testSuiteLog_V1.13.90.dev1_darwin-ARM-64bit-P3.13.txt), all 193 test models and 94 mini
    examples within their tolerance, the seven models of this list with an error of 0 or 1e-16.

    A model that is in this set and passes is not a problem: the set says "a difference here proves
    nothing", not "there must be one"."""

    unresolved = UnresolvedOnLinux() | set([
        #GitHub CI of 1.13.3 on cp314 only (numpy 2.5.3): the modes of the reduced order FFRF model come from numpy's
        #eigenvalue solver, not from Exudyn; passed with Python 3.13 on 2026-10-16 (#2919)
        'objectFFRFreducedOrderTest.py',        #rel. 1.3e-08
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
        'ObjectConnectorRigidBodySpringDamper.py':-0.5349299510886986, #2026-10-01: Jacobian by AD (#2745); the same in exudynCPPfast since 2026-10-09 (#2912)
        'ObjectConnectorLinearSpringDamper.py':0.0004999866342440002, #previously had error, did not run
        'ObjectConnectorTorsionalSpringDamper.py':0.0004999866342439671,
        'ObjectConnectorCoordinateSpringDamper.py':0.0019995154213252597,
        'ObjectConnectorGravity.py':1.0000000000000484,
        'ObjectConnectorDistance.py':-0.9861806727696236,
        'ObjectConnectorCoordinate.py':0.04999999999999982,
        'ObjectGenericODE2.py':1.0039999999354785,
        'ObjectGenericODE1.py':-0.8206847097689384,
        'ObjectGenericDAE.py':0.04815547453526271,
        'ObjectUserBody.py':-4.904999999999998, #new 2026-10-12 (#2986)
        'ObjectJointRevoluteZ.py':0.49999999999999795,
        'ObjectKinematicTree.py':-3.134018551808591,
        'ObjectMass1D.py':2.0,
        'ObjectMassPoint.py':2.0,
        'ObjectMassPoint2D.py':2.0,
        'ObjectRigidBody2D.py':4.356194490192344,
        'ObjectRotationalMass1D.py':2.0,
        #the objects (#2732)
        'ObjectGround.py':-0.09809999999999997,
        'ObjectRigidBody.py':1.0949978521569106,
        'ObjectANCFBeam.py':-0.3381249577076517,
        'ObjectBeamGeometricallyExact2D.py':-0.3381249616596102,
        'ObjectBeamGeometricallyExact.py':-0.33812496093079464,
        'ObjectANCFThinPlate.py':-0.06986346700419381, #2026-10-13: the clamping joints act on the symmetric basis of the slopes (#2228), before -0.06987478663348738; 2026-10-05: material curvature measure, Gauss 5x5 (#2857); before -0.06987553667440825
        'ObjectALEANCFCable2D.py':0.2500092945807542,
        'ObjectJointALEMoving2D.py':0.7249978132591198,
        'ObjectConnectorCoordinateSpringDamperExt.py':0.050495049504950505,
        'ObjectConnectorCoordinateVector.py':0.49999999999999867,
        'ObjectContactCoordinate.py':-0.000999999999999999,
        'ObjectContactCircleCable2D.py':-0.08340198465657866,
        'ObjectContactFrictionCircleCable2D.py':-0.0963071435860746,
        'ObjectContactSphereSphere.py':1.0999019,
        'ObjectContactSphereTriangle.py':0.09990189999999996,
        'ObjectContactSphereTorus.py':0.10101,
        'ObjectContactCurveCircles.py':0.09949999999999998,
        'ObjectConnectorRollingDiscPenalty.py':-1.9999976933948558, #2026-11-02: the contacts on the connector interface with dv/dq (#3201); before -1.999997689449951
        'ObjectJointRollingDisc.py':-1.9999976540123938,
        'ObjectJointGeneric.py':-3.134020197069236, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -3.134020196852591
        'ObjectJointPrismaticX.py':0.999999999999996,
        'ObjectJointSpherical.py':-0.003786192075315309, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before -0.003786192183655731
        'ObjectJointRevolute2D.py':-3.133419661240955,
        'ObjectJointPrismatic2D.py':0.24999999999999895,
        'ObjectJointSliding2D.py':1.0999999999999979,
        'ObjectJointSliding.py':1.0999999999999979,
        'ObjectConnectorReevingSystemSprings.py':-0.9999999999998965,
        'ObjectConnectorUser.py':0.07074575611653322, #new 2026-10-23 (#2975)
        'ObjectJointUser.py':1.0, #new 2026-10-24 (#2976)
        'MarkerBodyUser.py':0.07074575611265864, #new 2026-10-24 (#3065); renamed from MarkerUser.py 2026-10-26 (#3103); 2026-11-04 with its positionJacobianUserFunction instead of central differences, 7e-12 (#3107)
        'ObjectConnectorHydraulicActuatorSimple.py':1.0,
        #the markers, loads and sensors (#2732)
        'MarkerBodyMass.py':-4.904999999999998,
        'MarkerBodyPosition.py':-0.1,
        'MarkerBodyRigid.py':0.01000000000259357, #2026-11-02: the local Jacobians along the Euler parameters, the reaction forces of the system-wide one by lambda (#3201), before 0.009999999998437646; 2026-10-01: Jacobian of the rigid-body spring-damper by AD (#2745), before 0.010000000002617884
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
        'MarkerBodyCable2DShape.py':-0.08340198465657866,
        'LoadForceVector.py':0.24999999999999897,
        'LoadTorqueVector.py':0.9996439632248505,
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
        'NodeUser.py':0.8414709848078965, #new 2026-10-18 (#2974)
        'NodePointSlope1.py':-0.3333332497979799,
        'NodePointSlope12.py':-0.06986346700419381, #2026-10-13: the clamping joints act on the symmetric basis of the slopes (#2228), before -0.06987478663348738; 2026-10-05: material curvature measure, Gauss 5x5 (#2857); before -0.06987553667440825
        'NodePointSlope23.py':-0.3381249577076517,
        'NodeRigidBody2D.py':3.0,
        'NodeRigidBodyEP.py':1.5707880511569134,
        'NodeRigidBodyRotVecLG.py':1.5707963267949465,
        'NodeRigidBodyRxyz.py':1.5707963267948934,
        }
    import exudyn as exu

    if 'experimentalNewSolver' in exu.sys: #needs some corrected results
        refSol['ObjectConnectorRigidBodySpringDamper.py'] = -0.5349299542344889 #diff to other solvers: 3.6e-9

    
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
        'perfRigidPendulum.py':2.473544662086979, #2026-10-10: the portable math functions (#2954), a long implicit run that amplifies one ULP of sin/cos; before 2.473549926393305; 2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 2.4735499200766586; changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 2.4745344452543323,
        'perfSpringDamperExplicit.py':0.52,
        'perfSpringDamperUserFunction.py':0.5065575310983877,
        'perfConnectorInterface.py':1.8871766817354405, #new 2026-10-01 (#2745)
        'perfAccessFunctionsAD.py':439.41627874357835, #new 2026-10-02 (#2744)
        'perfLargeMassSpringChain.py':0.01426191722384829, #2026-09-16: rigid body chain, last run n=20000; before (mass points): 0.03136079550415616
        'perfUserFunctions.py':0.6945286696349366, #new 2026-10-16: typical user functions, Python and symbolic (#2996); 2026-10-18: rigid6, ode1x6, mass6, preStep (#2982)

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
        #the connector interface (#2745) - measured 2026-10-01 on Windows cp313
        'perfConnectorInterface:spring-n200-implicit':             8.222900066918616, #2026-11-02: the local Jacobians along the Euler parameters (#3201), before 8.222900067049855; 2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 8.222900073211406
        'perfConnectorInterface:spring-n200-explicit':             6.071378919533291,
        'perfConnectorInterface:gravity-n200-implicit':            17.452516360952007,
        'perfConnectorInterface:coordinate-n1000-explicit':        0.9526968492542502,
        'perfConnectorInterface:rigid-n100-implicit':              1.552725084801644, #2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 1.552725088397863
        'perfConnectorInterface:rigid-n100-explicit':              1.8871766817354405,
        #typical user functions, each as Python and as symbolic function, which compute the same (#2996) - measured
        #2026-10-16 on Windows cp313
        'perfUserFunctions:spring-python':                0.05386283407605861,
        'perfUserFunctions:spring-symbolic':              0.05386283407605861,
        'perfUserFunctions:cartesian-python':             0.0014831340538506632,
        'perfUserFunctions:cartesian-symbolic':           0.0014831340538506632,
        'perfUserFunctions:load-python':                  0.010228708405763877,
        'perfUserFunctions:load-symbolic':                0.010228708405763877,
        'perfUserFunctions:generic6-python':              0.0021661492424455936,
        'perfUserFunctions:generic6-symbolic':            0.0021661492424455936,
        'perfUserFunctions:generic10-python':             0.0026733098226563758,
        'perfUserFunctions:generic10-symbolic':           0.0026733098226563758,
        'perfUserFunctions:generic100-python':            0.21138030449447787,
        'perfUserFunctions:generic100-symbolic':          0.21138030449447787,
        'perfUserFunctions:rigid6-python':                0.0008349292410866855,
        'perfUserFunctions:rigid6-symbolic':              0.0008349292410866855,
        'perfUserFunctions:ode1x6-python':                0.00032001095136912787,
        'perfUserFunctions:ode1x6-symbolic':              0.00032001095136912787,
        'perfUserFunctions:mass6-python':                 0.016573990749079354,
        'perfUserFunctions:mass6-symbolic':               0.016573990749079354,
        'perfUserFunctions:preStep-python':               0.047740963780680185,
        'perfUserFunctions:preStep-symbolic':             0.047740963780680185,
        #the access functions of the bodies (#2744): hand-written, and for the cable the derivative of J^T f by automatic
        #differentiation - measured 2026-10-02 on Windows cp313
        'perfAccessFunctionsAD:rigidEP-n100-implicit':           1.552725084801644, #2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 1.552725088397863
        'perfAccessFunctionsAD:rigidRxyz-n100-implicit':         38.06977882602512, #2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 38.06978409054861
        'perfAccessFunctionsAD:rigidRxyz-n100-explicit':         39.70816787868968,
        'perfAccessFunctionsAD:rigid2D-n200-implicit':           333.2416865462153, #2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 333.2416865794323
        'perfAccessFunctionsAD:cable-n200-implicit':             0.004663297110155585, #2026-10-08: modified Newton is the default of timeIntegration.newton since 2026-10-03 (#2815, #2900); before: 0.005112208211437852
        'perfAccessFunctionsAD:rigid2D-n200-explicit':           439.41627874357835,
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
#Recorded again 2026-10-10 with the portable math functions (#2954), the 16 entries below. Recorded 2026-10-09 on Windows with the exudynCPPfast (AVX2) of 1.13.0 - cp310 and cp313 give identical values. The
#first record (2026-09-17) was not renewed when the regular values above moved (the modified Newton default #2815,
#the deterministic joint setup #2502, ...), so the fast-module suite of the 1.13 release failed on stale entries (#2912).
#Re-record from the '_fast' logs whenever the regular values move. The module reports its vector extensions in
#exudyn.config.Version(True); testRunnerTools.ModuleUsesAVX2() reads that.
def AVX2ReferenceSolutionUpdate():

    refSolAVX2 = {
        'generalContactFrictionTests.py':         12.019469178274406,        #drift 7.5e-03
        'generalContactCylinderTest.py':          12.246589561590294,        #drift 4.2e-05
        'sphereTriangleTest2.py':                 4.356416611527587,         #drift 3.6e-05
        'generalContactImplicit1.py':             0.775815592989908,         #drift 5.3e-08
        'explicitSolversPostNewtonTest.py':       4.5634820142002335,         #drift 1.2e-08
        'ANCFbeltDrive.py':                       -0.0011715990134786303,    #drift 1.0e-08
        'geometricallyExactBeamRightAngleFrame.py':3.306181716261244,         #drift 6.2e-09
        'computeItemTest.py':                     71.26280786921733,         #drift 2.0e-09
        'bricardMechanism.py':                    4.1721896493074,           #drift 1.7e-09
        'coordinateSpringDamperExt.py':           17.084935539925155,        #drift 5.8e-10
        'geometricallyExactBeamMassTest.py':      2.994207078782524,         #drift 3.6e-10
        'createSphereQuadContact.py':             1.1243944166809874,        #drift 3.0e-10
        'rollingDiscTangentialForces.py':         1.0342017404283694,        #drift 1.4e-10
        'createSphereTriangleContact.py':         4.8402440583486355,        #drift 3.2e-11
        'heavyTop.py':                            33.42312575175418,        #drift 3.0e-11
        'rigidBodySpringDamperIntrinsic.py':      0.5472368462865441,        #drift 2.2e-11
        'contactSphereTorusMomentumTest.py':      4.227231105387477,         #drift 1.1e-11
        'ballBearingTest.py':                     0.03785241402460491,       #drift 6.7e-12
        'solverExplicitODE1ODE2test.py':          3.3767933275970896,        #drift 5.2e-12
        'connectorRigidBodySpringDamperTest.py':  0.1827622474313096,        #drift 3.7e-12
        'ANCFgeneralContactCircle.py':            -0.5816542531639005,       #drift 3.7e-12
        'ObjectContactFrictionCircleCable2D.py':  -0.09630714358841744,      #drift 2.3e-12
        'genericODE2matrixFormatsTest.py':        2.089237240674427,         #drift 1.9e-12
        'rollingCoinPenaltyTest.py':              0.03489603106689881,       #drift 9.7e-13
        'rollingCoinTest.py':                     1.063438118935288,         #drift 8.3e-13
        'generalContactSpheresTest.py':           -1.1138547720263723,       #drift 6.3e-13
        'revoluteJointPrismaticJointTest.py':     1.2538806799245639,        #drift 4.6e-13
        'fourBarMechanismIftomm.py':              0.1721665271764104,       #drift 4.6e-13
        'sphericalJointTest.py':                  4.4090804465751265,        #drift 2.8e-13
        'ConvexContactTest.py':                   0.011770267410696558,      #drift 2.0e-13
        'generalContactTriangleMomentumTest.py':  4.035299999516489,         #drift 1.9e-13
        'ObjectJointGeneric.py':                  -3.1340201968524144,       #drift 1.8e-13
        'scissorPrismaticRevolute2D.py':          27.202556489044532,        #drift 1.7e-13
        'createSphereQuadContact2.py':            0.15616582432927872,       #drift 1.6e-13
        'ObjectJointSpherical.py':                -0.003786192183744071,     #drift 8.8e-14
        'mainSystemExtensionsTests.py':           57.64639446941541,         #drift 8.5e-14
        'geometricallyExactBeam2DquadraticTest.py':0.7426300926716076,         #drift 8.3e-14
        'genericODE2test.py':                     0.03604546349916921,      #drift 8.0e-14
        'createKinematicTreeTest.py':             3.340830142730632,         #drift 7.4e-14
        'ANCFcable2DuserFunction.py':             0.6015588367721516,        #drift 7.1e-14
        'ANCFBeamTest.py':                        1.0105101312117235,        #2026-10-14: the strains in the symmetric basis (#2992); before 1.0104863123003804
        'energiesFlexibleBodiesTest.py':          5.850170605200533,         #drift 9.2e-14 (#2992)
        'userRigidBodyTest.py':                   13.4757550213763,          #drift 5.3e-14 (#2986)
        #a single run of runPerformanceTests.py: its result moves by 20% with the Newton tolerance (#2914)
        'perfAccessFunctionsAD:cable-n200-implicit': 0.004662459533372243,  #drift 8.4e-07
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

