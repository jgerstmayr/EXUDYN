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
    
    refSol = {
        'abaqusImportTest.py': 0.0005885208722206333,               #new 2023-04-20; 5 modes as 8 modes have sensitive "half mode included"
        'ANCFBeamTest.py': 1.010486312300459,                       #new 2023-04-04, after resolving local kappa bug
        'ANCFcable2DuserFunction.py': 0.6015588367721232,           #new 2023-12-13
        'ANCFcontactCircleTest.py':-0.4842698420787613,
        'ANCFcontactFrictionTest.py':-0.014187561328096003,         #with old ObjectContactFrictionCircleCable2D until : 2022-03-09: -0.014188649931059739,
        'ANCFgeneralContactCircle.py':-0.5816542531620952,          #new 2022-07-11 (CState Parallel); #before some update to contact module(iterations decreased!):-0.5816521429557808, #2022-02-01
        'ANCFmovingRigidBodyTest.py':-0.12893096934983617,          #new 2022-12-25; old solution differs for 1e-10 since several updates -0.12893096921737698,
        'ANCFslidingAndALEjointTest.py':-4.426408394755261,         #before 2023-05-01 (loads jacobian): -4.426408390697862,         #before 2022-12-25(resolved BUG 1274): -4.426403044189653; with old ObjectContactFrictionCircleCable2D until: 2022-03-09: -4.42640304418963,
        'bricardMechanism.py': 4.172189649307425,
        'carRollingDiscTest.py':-0.23940048717113782,
        'compareAbaqusAnsysRotorEigenfrequencies.py':0.0004185480476228555,
        'compareFullModifiedNewton.py':0.00020079676000188396,
        'computeODE2AEeigenvaluesTest.py': 0.38811732950413347,
        'computeODE2EigenvaluesTest.py':-2.749026293713541e-11,
        'connectorGravityTest.py': 1014867.2330320379,
        'connectorRigidBodySpringDamperTest.py':0.1827622474318292, #new 2022-07-11 (CState Parallel); 
        'contactCoordinateTest.py':0.0553131995062827,
        'ConvexContactTest.py':0.011770267410694153,                #new 2022-07-11 (CState Parallel); #before 2022-01-25?: 0.05737886603111926, 
        'coordinateSpringDamperExt.py':17.084935539925155,          #new 2023-01-23
        'coordinateVectorConstraint.py':-1.0825265797698322,
        'coordinateVectorConstraintGenericODE2.py':-1.0825265797698322,
        'distanceSensor.py':1.867764310778691,
        'driveTrainTest.py':-9.269855516524927e-08,                 #new 2023-05-20 (mainSystemExtensions); before:-9.269311940229841e-08,
        'explicitLieGroupIntegratorPythonTest.py':149.8473939540758,
        'explicitLieGroupIntegratorTest.py':0.16164013319819065,
        'fourBarMechanismTest.py':-2.376335780518213,
        'fourBarMechanismIftomm.py':0.1721665271840173,
        'generalContactFrictionTests.py':12.464092000879125,        #new 2022-07-11 (CState Parallel); #before 2022-01-25 (changed some velocity computation in GeneralContact): 10.133183086232139, #changed GeneralContact and implicit solver; before 2022-01-18: 10.132106712933348 , 
        'generalContactSpheresTest.py':-1.1138547720263323,         #new 2022-07-22 (parallel Lie group updates); new 2022-07-11 (CState Parallel); #before 2022-01-25(minor diff, due to round off errors in multithreading; now changed to 1 thread):-1.113854772026123, #changed GeneralContact and implicit solver; before 2022-01-18: -1.0947542400425323, #before 2021-12-02: -1.0947542400427703,
        'genericJointUserFunctionTest.py':1.1922383967562884,
        'genericODE2test.py':0.036045463499024655,                  #new 2022-07-11 (CState Parallel); #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.036045463498793825,
        'geneticOptimizationTest.py':0.10117518366826603,           #before 2022-02-20 (accuracy of internal sensors is higher); 0.10117518367051619, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.10117518366934351,
        'geometricallyExactBeam2Dtest.py':-2.2115028353806547,
        'geometricallyExactBeamTest.py':1.012822053539261,          #before 2023-05-05: 1.0128218992948643 (changed Texp function); new 2023-04-06 may still include small errors in implementation
        'heavyTop.py':33.42312575174431,                            #new 2022-07-11 (CState Parallel); 
        'hydraulicActuatorSimpleTest.py':7.130440021870293,
        'kinematicTreeAndMBStest.py':2.6388120463802767e-05,        #original but too sensitive to disturbances: 263.88120463802767,
        'kinematicTreeConstraintTest.py':1.8135975384620484 ,
        'kinematicTreeTest.py': -1.309383960216414,
        'mainSystemExtensionsTests.py': 57.64639446941554,          #updated 2023-11-16; updated 2023-06-09; old: new 2023-05-19
        'manualExplicitIntegrator.py':2.059698629692295,
        'mecanumWheelRollingDiscTest.py':0.2714267238324343,
        'objectFFRFreducedOrderAccelerations.py':0.1000057024588858,#before 2022-07-22 (because often small fails); 0.5000285122944431,#before 2022-02-20 (accuracy of internal sensors is higher): 0.5000285122930983,
        'objectFFRFreducedOrderTest.py':0.0053552332680605694,      #until 2022-03-18 (div result by 5): 0.026776166340247865,
        'objectFFRFTest.py':0.0064600108120842666,                  #before 2022-02-20 (accuracy of internal sensors is higher): 0.006460010812070858,
        'objectFFRFTest2.py':0.03552188069017914,                   #before 2022-02-20 (accuracy of internal sensors is higher): 0.03552188069032863,
        'objectGenericODE2Test.py':-2.316378897486015e-05,
        'PARTS_ATEs_moving.py':0.44656762760262214,
        'pendulumFriction.py':0.39999998776982304,
        'plotSensorTest.py':1,
        'postNewtonStepContactTest.py':0.057286638346409235,
        'reevingSystemSpringsTest.py':2.2155575717433007,           #new 2023-07-17 (old solution contained compression forces: 2.213190117855691),
        'revoluteJointprismaticJointTest.py':1.2538806799249342,    #new 2022-07-11 (CState Parallel); #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver (modified Newton restart, etc.); before 2022-01-18: 1.2538806799243265,
        'rigidBodyAsUserFunctionTest.py':8.950865271552148,
        'rigidBodyCOMtest.py':3.409431467726291,
        'rigidBodySpringDamperIntrinsic.py':0.5472368463500464,     #new 2023-11-30 (intrinsic formulation for rigid body spring damper)
        'rollingCoinTest.py':0.0020040999273379673,
        'rollingCoinPenaltyTest.py':0.03489603106689881,
        'rotatingTableTest.py':7.838680371309492 ,
        'scissorPrismaticRevolute2D.py':27.20255648904422,          #new 2022-07-11 (CState Parallel); #added JacobianODE2, but example computed with numDiff forODE2connectors, 2022-01-18: 27.202556489044145,
        'sensorUserFunctionTest.py':45.0,            
        'serialRobotTest.py':0.7681856909852399,                    #until 2022-04-21: 0.7680031232063571 wrong static torque compensation
        'sliderCrank3Dtest.py':3.3642761780921897,
        'sliderCrankFloatingTest.py':0.591649163378833,
        'solverExplicitODE1ODE2test.py':3.3767933275970896,         #new 2022-07-11 (CState Parallel); 
        'sparseMatrixSpringDamperTest.py':-0.06779862812271394,     #changed to analytic Spring-Damper jacobian (missing d(vel)/dpos term): -0.06779862983767654,
        'sphericalJointTest.py':4.409080446575089,                  #new 2022-07-11 (CState Parallel); 
        'springDamperUserFunctionTest.py':0.5062872273010911,
        'stiffFlyballGovernor.py':0.8962488779114738,
        'superElementRigidJointTest.py':0.015217208913989071,       #before 2022-02-20 (accuracy of internal sensors is higher): 0.015217208913983024,
        'symbolicUserFunctionTest.py':0.10039884426884882,          #2023-12-13  
        }

    if (sys.version_info.major == 3 and sys.version_info.minor == 6): #different solutions without AVX
        replaceRefSol = {
            #Python version without AVX leads to different solution: since 2022-07-11 (StateVector with ResizableVectorParallel)
            'ANCFgeneralContactCircle.py':-0.5816542531657561, #before some update to contact module(iterations decreased!):-0.5816521429557808, #2022-02-01
            'ConvexContactTest.py':0.011770267410492958, #before 2022-01-25?: 0.05737886603111926, 
            'generalContactFrictionTests.py':12.720590570382422, #before 2022-01-25 (changed some velocity computation in GeneralContact): 10.133183086232139, #changed GeneralContact and implicit solver; before 2022-01-18: 10.132106712933348 , 
            'generalContactSpheresTest.py':-1.1138547720260847, #before 2022-01-25(minor diff, due to round off errors in multithreading; now changed to 1 thread):-1.113854772026123, #changed GeneralContact and implicit solver; before 2022-01-18: -1.0947542400425323, #before 2021-12-02: -1.0947542400427703,
            'genericODE2test.py':0.03604546349894506, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 0.036045463498793825,
            'heavyTop.py':33.423125751743804,
            'revoluteJointprismaticJointTest.py':1.2538806799241744, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver (modified Newton restart, etc.); before 2022-01-18: 1.2538806799243265,
            'rollingCoinPenaltyTest.py':0.034896031067866894,
            'scissorPrismaticRevolute2D.py':27.202556489044145, #added JacobianODE2, but example computed with numDiff forODE2connectors, 2022-01-18: 27.202556489044145,
            'solverExplicitODE1ODE2test.py':3.3767933275918964,
            'sphericalJointTest.py':4.409080446575154,
            'connectorRigidBodySpringDamperTest.py':0.18276224743555652,
            }

        refSol = {key: replaceRefSol.get(key, refSol[key]) for key in refSol}

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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#return reference solutions for mini examples in dictionary
def MiniExamplesReferenceSolution():
    refSol = {
        #results after change to new Jacobian, diff about 1e-12
        'LoadMassProportional.py':-4.904999999999998,
        'MarkerSuperElementPosition.py':1.0039999999354785,
        'ObjectANCFCable2D.py':-0.5013058140308901,
        'ObjectANCFCable.py':-0.5013058140308919, #added 2023-10-15
        'ObjectANCFThinPlate.py':0.0,
        'ObjectConnectorSpringDamper.py':0.9733828995763039, #until 2022-01-25 (before analytical Jac for SpringDamper):0.9733828995759499,
        'ObjectConnectorCartesianSpringDamper.py':-0.0009999999999750209,
        'ObjectConnectorRigidBodySpringDamper.py':-0.5349299545315868,
        'ObjectConnectorLinearSpringDamper.py':0.0004999866342439289, #previously had error, did not run
        'ObjectConnectorTorsionalSpringDamper.py':0.0004999866342439527,
        'ObjectConnectorCoordinateSpringDamper.py':0.0019995154213252597,
        'ObjectConnectorGravity.py':1.000000000000048,
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
        }
    import exudyn as exu

    if 'experimentalNewSolver' in exu.sys: #needs some corrected results
        refSol['ObjectConnectorRigidBodySpringDamper.py'] = -0.5349299542344889 #diff to other solvers: 3.6e-9

    if 'AVX2' not in exu.GetVersionString(True): #for nonAVX2 versions in Windows as well as other platforms
        #if (sys.version_info.major == 3 and sys.version_info.minor == 6): #different solutions without AVX
        #Python version without AVX leads to different solution: since 2022-07-11 (StateVector with ResizableVectorParallel)
        refSol['ObjectConnectorRigidBodySpringDamper.py'] = -0.534929955894111

    
    return refSol




def PerformanceTestsReferenceSolution():

    refSol = {
        'generalContactSpheresTest.py': -5.98425321234168, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: -5.946497644233068,
        'perf3DRigidBodies.py':5.307943301446709,
        'perfObjectFFRFreducedOrder.py':21.00863102425483, 
        'perfRigidPendulum.py':2.4735499200766586, #changed to some analytic Connector jacobians (CartSpringDamper), implicit solver(modified Newton restart, etc.); before 2022-01-18: 2.4745344452543323,
        'perfSpringDamperExplicit.py':0.52,
        'perfSpringDamperUserFunction.py':0.5065575310983877,
        }

    return refSol












#%%+++++++++++++++++++++++++++++++++++++++
#OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD 
#differences between old and new implicit trapezoidal solver:
errDiff = {#obtained on 2021-02-06 (Python3.7, 64bits): shows differences to old test suite, ALL 39 EXAMPLE TESTS SUCCESSFUL
 'ANCFcontactCircleTest.py': 0.0,
 'ANCFcontactFrictionTest.py': 0.0,
 'ANCFmovingRigidBodyTest.py': 0.0,
 'ACNFslidingAndALEjointTest.py': -7.94475596421762e-13,
 'carRollingDiscTest.py': 0.0,
 'compareAbaqusAnsysRotorEigenfrequencies.py': 0,
 'compareFullModifiedNewton.py': 0.0,
 'computeODE2EigenvaluesTest.py': 0.0,
 'driveTrainTest.py': -1.4432899320127035e-15,
 'explicitLieGroupIntegratorPythonTest.py': -2.842170943040401e-14,
 'explicitLieGroupIntegratorTest.py': 0.0,
 'fourBarMechanismTest.py': 0.0,
 'genericJointUserFunctionTest.py': -5.3290705182007514e-14,
 'genericODE2test.py': -2.1239260350469635e-13,
 'geneticOptimizationTest.py': 0.0,
 'heavyTop.py': -4.339284487286932e-11,
 'manualExplicitIntegrator.py': -3.552713678800501e-15,
 'mecanumWheelRollingDiscTest.py': 1.6653345369377348e-16,
 'objectFFRFreducedOrderAccelerations.py': 0.0,
 'objectFFRFreducedOrderTest.py': 9.431344594190705e-15,
 'objectFFRFTest.py': 1.2654807757250808e-15,
 'objectFFRFTest2.py': -1.1102230246251565e-15,
 'objectGenericODE2Test.py': -1.1263098743594102e-15,
 'PARTS_ATEs_moving.py': -5.551115123125783e-17,
 'pendulumFriction.py': -4.440892098500626e-16,
 'rigidBodyCOMtest.py': 0.0,
 'rollingCoinTest.py': 0.0,
 'rollingCoinPenaltyTest.py': 9.71445146547012e-17,
 'scissorPrismaticRevolute2D.py': 1.1823431123048067e-13,
 'serialRobotTest.py': 9.393517075295676e-12,
 'sliderCrank3Dtest.py': 0.0,
 'sliderCrankFloatingTest.py': 2.220446049250313e-16,
 'solverExplicitODE1ODE2test.py': 0.0,
 'sparseMatrixSpringDamperTest.py': -8.615330671091216e-15,
 'sphericalJointTest.py': 0.0,
 'springDamperUserFunctionTest.py': -4.3298697960381105e-15,
 'stiffFlyballGovernor.py': 0.0,
 'superElementRigidJointTest.py': 0.0,
 'connectorRigidBodySpringDamperTest.py': 0.0}




#%%+++++++++++++++++++++++++++++++++++++
#old results before 2021-09-27 (change to new CSystem JacobianODE2RHS implementation)
            # #obtained on 2021-02-06(Python3.7, 64bits): with new implicit trapezoidal solver (Arnold/Bruls)
            # 'ANCFcontactCircleTest.py':-0.4842656133238705, #2021-05-07, switched from StaticSolveOldSolver to exu.SolveStatic
            # 'ANCFcontactFrictionTest.py':-0.014188649931863358,
            # 'ANCFmovingRigidBodyTest.py':-0.1289309692152536, #until ~2021-06-27: -0.12893096921481131,
            # 'ACNFslidingAndALEjointTest.py':-4.42640288393854, #until ~2021-06-27: -4.426403043826658, #2021-02-17 (added mass proportional load in sALE direction): -4.426403043826658 #2021-02-06: -4.426403044452073,
            # 'carRollingDiscTest.py':-0.23940048717113455,
            # 'compareAbaqusAnsysRotorEigenfrequencies.py':0.0004185480476228511,
            # 'compareFullModifiedNewton.py':0.00020079676000188396,
            # 'computeODE2EigenvaluesTest.py':-2.7613614363986015e-11,
            # 'contactCoordinateTest.py':0.055313199503736685, #new 2021-08-13
            # 'driveTrainTest.py':-9.26931189582092e-08, 
            # 'explicitLieGroupIntegratorPythonTest.py':149.84739395407578,
            # 'explicitLieGroupIntegratorTest.py':0.16164013319819076,
            # 'fourBarMechanismTest.py':-2.376335780518213,
            # 'genericJointUserFunctionTest.py':1.1922383967562729,
            # 'genericODE2test.py':0.03604546349894412,
            # 'geneticOptimizationTest.py':0.10117518367000587,
            # 'geometricallyExactBeam2Dtest.py':-2.2115028353806547, #new 2021-03-25
            # 'heavyTop.py':33.42312575172122,
            # 'manualExplicitIntegrator.py':2.0596986296922988,
            # 'mecanumWheelRollingDiscTest.py':0.2714267238324343,
            # 'objectFFRFreducedOrderAccelerations.py':0.5000285122931072,
            # 'objectFFRFreducedOrderTest.py':0.026776166340298804,
            # 'objectFFRFTest.py':0.00646001081207057,
            # 'objectFFRFTest2.py':0.03552188069030117,
            # 'objectGenericODE2Test.py':-2.316378897598925e-05,
            # 'PARTS_ATEs_moving.py':0.44656762760262225,
            # 'pendulumFriction.py':0.3999999877698232,
            # 'postNewtonStepContactTest.py':0.057286638346409235, #new 2021-03-20
            # 'revoluteJointprismaticJointTest.py':1.2538806799246283, #new 2021-07-01
            # 'rigidBodyAsUserFunctionTest.py':8.950865271552146, #new 2021-06-28
            # 'rigidBodyCOMtest.py':3.409431467726292,
            # 'rollingCoinTest.py':0.002004099927337848,
            # 'rollingCoinPenaltyTest.py':0.03489603106696451,
            # 'scissorPrismaticRevolute2D.py':27.202556489044575, #until 2021-03-20: 27.202556489044397,
            # 'serialRobotTest.py': 0.7680031232088501, #until 2021-09-10: 0.768003123206452, #until 2021-08-19(changed robotics.py): 0.7680031232091771 , #old controller (loadUserFunction): 0.7712176106978085,#change in EP constraints to nodes causes tiny error, seems to be error propagation; up to 2021-06-28: 0.7712176106955341; -4.309882450925784e-10 diff between old corrected and new gen alpha solver
            # 'sliderCrank3Dtest.py':3.3642761780921897,
            # 'sliderCrankFloatingTest.py':0.5916491633788336,
            # 'solverExplicitODE1ODE2test.py':3.3767933275918964,
            # 'sparseMatrixSpringDamperTest.py':-0.06779862983767654,
            # 'sphericalJointTest.py':4.409080446574593, #change in EP constraints to nodes causes tiny error ==> solution is identical (all digits) up to 100 steps; up to 2021-06-28: 4.409080446580333
            # 'springDamperUserFunctionTest.py':0.506287227301091,
            # 'stiffFlyballGovernor.py':0.8962488779114738,
            # 'superElementRigidJointTest.py':0.015217208913982934,  #until 2021-04-27 (improved MarkerSuperElementRigid): 0.015214887106830069,
            # 'connectorRigidBodySpringDamperTest.py':0.18276224743116654,            




