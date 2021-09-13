#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This file constains reference solutions for test suite
#
# Author:   Johannes Gerstmayr
# Date:     2021-02-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from modelUnitTests import exudynTestGlobals
import exudyn as exu


#%%+++++++++++++++++++++++++++++++++++++++
#return reference solutions for test examples in dictionary
def TestExamplesReferenceSolution():

    if exudynTestGlobals.useCorrectedAccGenAlpha or exudynTestGlobals.useNewGenAlphaSolver: #corrected version + new implicit solver!
        refSol = {
            #obtained on 2021-02-06(Python3.7, 64bits): with new implicit trapezoidal solver (Arnold/Bruls)
            'ANCFcontactCircleTest.py':-0.4842656133238705, #2021-05-07, switched from StaticSolveOldSolver to exu.SolveStatic
            'ANCFcontactFrictionTest.py':-0.014188649931863358,
            'ANCFmovingRigidBodyTest.py':-0.1289309692152536, #until ~2021-06-27: -0.12893096921481131,
            'ACNFslidingAndALEjointTest.py':-4.42640288393854, #until ~2021-06-27: -4.426403043826658, #2021-02-17 (added mass proportional load in sALE direction): -4.426403043826658 #2021-02-06: -4.426403044452073,
            'carRollingDiscTest.py':-0.23940048717113455,
            'compareAbaqusAnsysRotorEigenfrequencies.py':0.0004185480476228511,
            'compareFullModifiedNewton.py':0.00020079676000188396,
            'computeODE2EigenvaluesTest.py':-2.7613614363986015e-11,
            'contactCoordinateTest.py':0.055313199503736685, #new 2021-08-13
            'driveTrainTest.py':-9.26931189582092e-08, 
            'explicitLieGroupIntegratorPythonTest.py':149.84739395407578,
            'explicitLieGroupIntegratorTest.py':0.16164013319819076,
            'fourBarMechanismTest.py':-2.376335780518213,
            'genericJointUserFunctionTest.py':1.1922383967562729,
            'genericODE2test.py':0.03604546349894412,
            'geneticOptimizationTest.py':0.10117518367000587,
            'geometricallyExactBeam2Dtest.py':-2.2115028353806547, #new 2021-03-25
            'heavyTop.py':33.42312575172122,
            'manualExplicitIntegrator.py':2.0596986296922988,
            'mecanumWheelRollingDiscTest.py':0.2714267238324343,
            'objectFFRFreducedOrderAccelerations.py':0.5000285122931072,
            'objectFFRFreducedOrderTest.py':0.026776166340298804,
            'objectFFRFTest.py':0.00646001081207057,
            'objectFFRFTest2.py':0.03552188069030117,
            'objectGenericODE2Test.py':-2.316378897598925e-05,
            'PARTS_ATEs_moving.py':0.44656762760262225,
            'pendulumFriction.py':0.3999999877698232,
            'postNewtonStepContactTest.py':0.057286638346409235, #new 2021-03-20
            'revoluteJointprismaticJointTest.py':1.2538806799246283, #new 2021-07-01
            'rigidBodyAsUserFunctionTest.py':8.950865271552146, #new 2021-06-28
            'rigidBodyCOMtest.py':3.409431467726292,
            'rollingCoinTest.py':0.002004099927337848,
            'rollingCoinPenaltyTest.py':0.03489603106696451,
            'scissorPrismaticRevolute2D.py':27.202556489044575, #until 2021-03-20: 27.202556489044397,
            'serialRobotTest.py': 0.7680031232088501, #until 2021-09-10: 0.768003123206452, #until 2021-08-19(changed robotics.py): 0.7680031232091771 , #old controller (loadUserFunction): 0.7712176106978085,#change in EP constraints to nodes causes tiny error, seems to be error propagation; up to 2021-06-28: 0.7712176106955341; -4.309882450925784e-10 diff between old corrected and new gen alpha solver
            'sliderCrank3Dbenchmark.py':3.3642761780921897,
            'sliderCrankFloatingTest.py':0.5916491633788336,
            'solverExplicitODE1ODE2test.py':3.3767933275918964,
            'sparseMatrixSpringDamperTest.py':-0.06779862983767654,
            'sphericalJointTest.py':4.409080446574593, #change in EP constraints to nodes causes tiny error ==> solution is identical (all digits) up to 100 steps; up to 2021-06-28: 4.409080446580333
            'springDamperUserFunctionTest.py':0.506287227301091,
            'stiffFlyballGovernor.py':0.8962488779114738,
            'superElementRigidJointTest.py':0.015217208913982934,  #until 2021-04-27 (improved MarkerSuperElementRigid): 0.015214887106830069,
            'connectorRigidBodySpringDamperTest.py':0.18276224743116654,            
            }
    else: #old solver version, with inconsistent algorithmic accelerations; checked with previous stored results (2021-02-04), agrees upt to 2e-16 
        refSol = {
            #obtained on 2021-02-06(Python3.7, 64bits): #checked with old test suite, ALL 39 EXAMPLE TESTS SUCCESSFUL
            'ANCFcontactCircleTest.py':-0.4842656547442095,
            'ANCFcontactFrictionTest.py':-0.014188649931870346,
            'ANCFmovingRigidBodyTest.py':-0.1289310238888096,
            'ACNFslidingAndALEjointTest.py':-4.426403044450976,
            'carRollingDiscTest.py':-0.23940048717113419,
            'compareAbaqusAnsysRotorEigenfrequencies.py':0.0004185480476228478,
            'compareFullModifiedNewton.py':0.0001583478719999567,
            'computeODE2EigenvaluesTest.py':-2.7613614363986015e-11,
            'driveTrainTest.py':-9.269312173376676e-08,
            'explicitLieGroupIntegratorPythonTest.py':149.84739395407578,
            'explicitLieGroupIntegratorTest.py':0.16164013319819076,
            'fourBarMechanismTest.py':-2.354666317492353,
            'genericJointUserFunctionTest.py':1.1878327690760053,
            'genericODE2test.py':0.03604546349877444,
            'geneticOptimizationTest.py':0.10117518367229393,
            'heavyTop.py':33.42312575172991,
            'manualExplicitIntegrator.py':2.0596986296922988,
            'mecanumWheelRollingDiscTest.py':0.27142672383243466,
            'objectFFRFreducedOrderAccelerations.py':0.5,
            'objectFFRFreducedOrderTest.py':0.026772650552909505,
            'objectFFRFTest.py':0.0064453695609377765,
            'objectFFRFTest2.py':0.035537463693879306,
            'objectGenericODE2Test.py':-2.2737401293308742e-05,
            'PARTS_ATEs_moving.py':0.4465676276026222,
            'pendulumFriction.py':0.39999998776982004,
            'rigidBodyCOMtest.py':3.409431467726293,
            'rollingCoinTest.py':0.002004099927340136,
            'rollingCoinPenaltyTest.py':0.03489603106769774,
            'scissorPrismaticRevolute2D.py':27.202556489044397,
            'serialRobotTest.py':0.7713193176846507,
            'sliderCrank3Dbenchmark.py':3.36427617809219,
            'sliderCrankFloatingTest.py':0.5916491633788336,
            'solverExplicitODE1ODE2test.py':3.3767933275918964,
            'sparseMatrixSpringDamperTest.py':-0.06779862983767654,
            'sphericalJointTest.py':4.409004179180698,
            'springDamperUserFunctionTest.py':0.5062872273010854,
            'stiffFlyballGovernor.py':0.8962488779114738,
            'superElementRigidJointTest.py':0.015213599619996633,
            'connectorRigidBodySpringDamperTest.py':0.18276224743714353,
            }
    if exudynTestGlobals.useCorrectedAccGenAlpha and not exudynTestGlobals.useNewGenAlphaSolver:
        refSol['serialRobotTest.py']=0.7712176102645458#-4.309882450925784e-10 diff between old corrected and new gen alpha solver

    #++++++++++++++++++++
    #special solutions for 32bit:
    import platform
    if platform.architecture()[0] != '64bit':
        refSol['ACNFslidingAndALEjointTest.py']=-4.426403043824947
        refSol['genericODE2test.py']=0.036045463499109365
        refSol['heavyTop.py']=33.42312575176021 
        #refSol['objectFFRFreducedOrderTest.py']=0.026776166340291847 #changes due to eigenvalue solver
        #refSol['scissorPrismaticRevolute2D.py']=27.202556489044472 #not needed with updated 64bit solution
        refSol['serialRobotTest.py']=0.7712176106962295
        refSol['connectorRigidBodySpringDamperTest.py']=0.1827622474328367


    #add new reference values here (only uses new solver):
    refSol['sensorUserFunctionTest.py'] = 45

    return refSol

#%%+++++++++++++++++++++++++++++++++++++++
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
 'sliderCrank3Dbenchmark.py': 0.0,
 'sliderCrankFloatingTest.py': 2.220446049250313e-16,
 'solverExplicitODE1ODE2test.py': 0.0,
 'sparseMatrixSpringDamperTest.py': -8.615330671091216e-15,
 'sphericalJointTest.py': 0.0,
 'springDamperUserFunctionTest.py': -4.3298697960381105e-15,
 'stiffFlyballGovernor.py': 0.0,
 'superElementRigidJointTest.py': 0.0,
 'connectorRigidBodySpringDamperTest.py': 0.0}

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#return reference solutions for mini examples in dictionary
def MiniExamplesReferenceSolution():
    if exudynTestGlobals.useCorrectedAccGenAlpha or exudynTestGlobals.useNewGenAlphaSolver: #new version
        refSol = {
            'ObjectMassPoint.py':2.0,
            'ObjectMassPoint2D.py':2.0,
            'ObjectMass1D.py':2.0,
            'ObjectRotationalMass1D.py':2.0,
            'ObjectRigidBody2D.py':4.356194490192344,
            'ObjectGenericODE2.py':1.0039999999354785,
            'ObjectGenericODE1.py':-0.8206847097689384,
            'ObjectConnectorSpringDamper.py':0.9733828995736554, #until 2021-03-20: 0.9733828995835538,
            'ObjectConnectorCartesianSpringDamper.py':-0.0009999999999750209,
            'ObjectConnectorRigidBodySpringDamper.py':-0.5349299542344889, #before final switching to new genAlpha: -0.5349299506130816,
            'ObjectConnectorCoordinateSpringDamper.py':0.0019995154213252597,
            'ObjectConnectorDistance.py':-0.9861806726069355,
            'ObjectConnectorCoordinate.py':0.04999999999999982,
            'MarkerSuperElementPosition.py':1.0039999999354785,
            'LoadMassProportional.py':-4.9049999999999985,
            }
    else: #old version, with inconsistent algorithmic accelerations; checked with previous stored results (2021-02-04), agrees upt to 2e-16 
        refSol = {
            'ObjectMassPoint.py':2.0,
            'ObjectMassPoint2D.py':2.0,
            'ObjectMass1D.py':2.0,
            'ObjectRotationalMass1D.py':2.0,
            'ObjectRigidBody2D.py':4.356194490192344, #2+0.75*pi
            'ObjectGenericODE2.py':1.0039999999354785,
            'ObjectGenericODE1.py':-0.8206847097689384,
            'ObjectConnectorSpringDamper.py':0.9736596225944887,
            'ObjectConnectorCartesianSpringDamper.py':-0.00099999999997058,
            'ObjectConnectorRigidBodySpringDamper.py':-0.4612983535925061, #large differences to old solver because of large steps
            'ObjectConnectorCoordinateSpringDamper.py':0.0019995158325691875,
            'ObjectConnectorDistance.py':-0.9845225086606828,
            'ObjectConnectorCoordinate.py':0.049999999999272404,
            'MarkerSuperElementPosition.py':1.0039999999354785,
            'LoadMassProportional.py':-4.9049999999999985, #(-9.81/2)
            }
    if 'experimentalNewSolver' in exu.sys: #needs some corrected results
        refSol['ObjectConnectorRigidBodySpringDamper.py'] = -0.5349299542344889 #diff to other solvers: 3.6e-9


    #special solutions for 32bit:
    # import platform
    # if platform.architecture()[0] != '64bit':
    #     refSol[''] = xy
    
    return refSol

