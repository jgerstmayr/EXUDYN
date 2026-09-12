#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is a submodule of the EXUDYN python robotics library
#
# Details:  The utilities contains functionality for mobile robots 
#           based on the EXUDYN example MecanumWheel RollingDiscPenality
#           specific friction angle of rolling disc is used to model rolls of mecanum wheels
#           
#
# Author:   Martin Sereinig, Peter Manzl and Johannes Gerstmayr
# Date:     2021-10-01
# Updated:  2023-09-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute 
# it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# Notes: formulation is still under development
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# exudyn imports
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.graphicsDataUtilities import *
import exudyn.graphics as graphics
from exudyn.robotics import *

import numpy as np

#**function:    add items to existing mbs to build up a mobile robot platform, 
#               there are options that can be passed as args / kwargs, which can contains options as described below. 
#               The robot platform is built out of rigid bodies where the wheels can be modeled as rolling discs 
#               (mecanum wheel x/o configuration) or with a detailed mecanum wheel simulation approach 
#**input: 
#   mbs: the multibody system which will be extended
#   markerGround: a rigid body marker, at which the robot will be placed (usually ground)
#   mobileRobot: a dictionary including all information about the mobile robot platform 
#**output: the function returns a dictionary containing nodes, body, object and marker numbers of individual mobile robot parts 
#           nPlatformList, bPlatformList, oPlatformList, mPlatformList; nodes, bodies, objects and marker of the platform [nPlattform] [bPlattform] [oPlattform]  []  
#           oAxisList, mAxlesList; objects and marker of the axles  [a1, a2, a3, a4]
#           nWheelsList, bWheelsList, oRollingDiscsList, mWheelsList; nodes, bodys, objects and markers of the four wheels [w1, w2, w3, w4]
#**notes: for coordinate system, see Python function definition
def MobileRobot2MBS(mbs, mobileRobot, markerGround, flagGraphicsRollers=True, *args, **kwargs):
    # platform setup:
    # ^Y
    # |    W3 +---------+ W1
    # |       |         |
    # |       |    +    | car center point
    # |       |         |
    # |    W4 +---------+ W2 
    # |
    # |
    # +-------->X
    # define mobile robot platform, example values taken from the mobile robot Leobot, build from the university of Innsbruck 2019-2021 
    # mobileRobot = { 'gravity':                    [0,0,-9.81],             # gravity in m/s^2
    #                 'platformDimensions':        [0.5, 0.7 , 0.2612-0.0452],       # [width, length, hight]
    #                 'platformMass':               58.6-16.06,                    # platform mass- manipulator mass 
    #                 'platformInitialPose':        HTrotateZ(0)@HTtranslate([0.0,0,(0.0452+(0.2612-0.0452)/2.0)]),  # platform initial pose as HT middle of platform (box representation) 
    #                 'platformInitialOmega':       [0,0,0],  # platform initial rotational velocity around x,y,z axis
    #                 'platformInitialVelocity':    [0,0,0],  # platform initial translational velocity in x,y,z direction
    #                 'platformCOM':                [0.0, 0.0, 0.0],       # center of mass shift to base coordinate system
    #                 'platformBaseCoordinate':     [0.0 ,0.0 ,0.0], # geometric center  in middle of platform                   
    #                 'platformInertia':            InertiaCuboid,     # platform inertia w.r.t. COM!                    
    #                 'platformRepresentation':     'stl',          # 'box' or 'stl' graphical representation of the mobile platform 
    #                 'platformStlFile':            'stl/huellgeometrie.STL',  # path to the used stl file 
    #                 'wheelType':                  0,                      # 0=wheeltype wheel o-config, 1=mecanum wheel x-config, 2=standard wheel  (always in bottom view)
    #                 'friction':                   [0.4, 0.0075, 0.05],    # [dryFriction1, dryFriction2,rollFriction]= [0.4,0.0075,0.05] for LeoBot (Master Thesis Manzl)
    #                 'frictionAngel':              pi/4,                   # friction angle theta=pi/4 for mecanum wheel, theta=0 for standard wheel  
    #                 'wheelBase':                  0.400,                    # distance between center of wheels (wheel axes) between front and back  
    #                 'wheelTrack':                 0.390,                    # distance between center of wheels between left and right 
    #                 'wheelRoh':                   1800,                    # density of wheel in kg/m^3
    #                 'wheelRadius':                0.0762,                   # radius of wheel in m 
    #                 'wheelWidth':                 0.076,                  # width of wheel in m, just for graphics     
    #                 'wheelMass':                  2.406,                    # Mass of one mecanum wheel, leobot measured
    #                 'wheelInertia':               InertiaCylinder,   # inertia for infinitely small ring:
    #                 'wheelNumbers':               4,                       # number of wheels on platform
    #                 'serialRobotMountpoint':      HTtranslate([0.211,0,-0.105]),  # serial robot mount point as HT from platform center coordinate system in middle of platform center
    #                 'debugOffset':                debugOffsetNumber       # number to set the initial position in z, to a new value avoid oscillations
    #                 'flagFlexible':                False                   # if True change RigidBody plate to flexible plate
    #                 }  

    # node, body,marker and object number for platform 
    nPlatformList = []           
    bPlatformList = []           
    mPlatformList = []    
    oPlatformList =[]   

    # node, body, marker number for wheels
    nWheelsList = [] 
    bWheelsList = []
    mWheelsList = []
    oRollingDiscsList = []   
    mAxlesList = []
    oAxlesList =  []
    if not('linearRegularization' in mobileRobot): 
        mobileRobot['linearRegularization'] = True

    # wheel parameter
    rWheel = mobileRobot['wheelRadius']        
    wWheel = mobileRobot['wheelWidth']        
    p0Wheel = [0,0,rWheel+mobileRobot['debugOffset']    ]      # origin of disc center point at reference, such that initial contact point is at [0,0,0]
    omega0Wheel = [0,0,0]       # initial angular velocity around z-axis
    inertiaWheel = mobileRobot['wheelInertia']
    mobileRobot['wheelRoh'] = mobileRobot['wheelMass'] /  (wWheel*rWheel**2*np.pi) # mass / Volume
    inertiaWheel = InertiaCylinder(density=mobileRobot['wheelRoh']  , length=wWheel, outerRadius=rWheel, axis=0)

    #platform parameters:
    p0Car = HT2translation(mobileRobot['platformInitialPose']) 
    p0Car[2] += mobileRobot['debugOffset'] 
    wCar = mobileRobot['platformDimensions'][0]
    lCar = mobileRobot['platformDimensions'][1]
    hCar = mobileRobot['platformDimensions'][2] 
    mCar = mobileRobot['platformMass']-4*inertiaWheel.mass #
    inertiaPlatform = mobileRobot['platformInertia']
    inertiaPlatform = InertiaCuboid(density=mCar/(lCar*wCar*hCar), sideLengths=[wCar, lCar, hCar])
    inertiaPlatform = inertiaPlatform.Translated(mobileRobot['comShiftPlatform'])
    inertiaPlatform.com  = mobileRobot['platformCOM'] #translate COM

    # to make mobileRobot dictionary global 
    mbs.variables['mobileRobot'] = mobileRobot
    ################
    graphicsPlatformList=[]
    # drawing platform coordinate system 
    graphicsPlatformList += [graphics.Cylinder([0,0,0], [0.5,0,0], 0.001, graphics.color.red)]
    graphicsPlatformList += [graphics.Cylinder([0,0,0], [0,0.5,0], 0.001, graphics.color.green)]
    graphicsPlatformList += [graphics.Cylinder([0,0,0], [0,0,0.2], 0.001, graphics.color.blue)]

    if mobileRobot['platformRepresentation']=='box':
        graphicsPlatformList += [graphics.Brick(centerPoint=[0.0,0.0,0.0],size=[lCar, wCar-1.1*wWheel, hCar], color=graphics.color.steelblue[0:3]+[0.2])]
    if mobileRobot['platformRepresentation'] == 'stl':
        try:
            stlGrafics = graphics.FromSTLfileASCII(mobileRobot['platformStlFile'],color=[1,1,1,1])
            graphicsPlatformList += [stlGrafics]
        except:
            exu.Print('stl not found, maybe wrong directory, use box instead')
            graphicsPlatformList += [graphics.Brick(centerPoint=[0,0,2.0],size=[lCar, wCar-1.1*wWheel, hCar], color=graphics.color.steelblue[0:3]+[0.2])]

    rb = mbs.CreateRigidBody(inertia=inertiaPlatform,
                             referencePosition=p0Car,
                             referenceRotationMatrix=HT2rotationMatrix(mobileRobot['platformInitialPose']),
                             initialAngularVelocity=mobileRobot['platformInitialOmega'],
                             initialVelocity=mobileRobot['platformInitialVelocity'],
                             gravity=mobileRobot['gravity'],
                             graphicsDataList=graphicsPlatformList,
                             returnDict=True)
    [nPlatform, bPlatform] = [rb['nodeNumber'], rb['bodyNumber']]
    
    mbs.SetObjectParameter(bPlatform, 'name', 'Base')
    mbs.SetNodeParameter(nPlatform, 'name', 'BaseCenter')
    nPlatformList += [nPlatform]
    bPlatformList += [bPlatform]
    # marker in center of platform
    mCenterPlatform = mbs.AddMarker(MarkerBodyRigid(name='mCenterPlatform',bodyNumber=bPlatform, localPosition=[0,0,0]))   #p0Car     
    mPlatformList += [mCenterPlatform]
    # marker in center of baseplate
    mCenterBase = mbs.AddMarker(MarkerBodyRigid(name='mCenterBase',bodyNumber=bPlatform, localPosition=[0,0,-mobileRobot['platformDimensions'][2]/2]))     #VAdd (p0Car)
    mPlatformList += [mCenterBase]
    # generic joint in center of base plate            
    oBasePlateJoint = mbs.AddObject(GenericJoint(markerNumbers=[markerGround, mCenterBase], activeConnector = True, 
                            constrainedAxes=[0,0,0,0,0,0],
                            visualization=VObjectJointGeneric(axesRadius=0.05, axesLength=2.4,color = [0.5,1.,0,0.1])))  
    oPlatformList += [oBasePlateJoint]
    
    # marker on platform for serial robot placement
    # note that the local coordinate system of the base must be in accordance with the DH-parameters, i.e., the z-axis must be the first rotation axis. For correction of the base coordinate system, use rotationMarkerBase
    mRobotArmMounting = mbs.AddMarker(MarkerBodyRigid(name='mRobotArmMounting',bodyNumber=bPlatform, localPosition= HT2translation(mobileRobot['serialRobotMountpoint'])))
    mPlatformList += [mRobotArmMounting]
    mRevoluteBody0 = []
    mRevoluteBody1 = []
    # wheel parameter
    param = {'r':  mobileRobot['wheelRadius'], 
            'lRoll': wWheel, 
            'nRolls': 8, 
            'delta': np.pi/4,
            'rRoll': mobileRobot['wheelRadius']*0.25}
    param['dRoll'] = param['r'] - param['rRoll']
    
    if flagGraphicsRollers: 
        Generatrix2Polynomial(param, GeneratrixRoll, tol=1e-8, nTest = 1001)
    rCyl = param['rRoll']
    nCyl = param['nRolls']
    
    ################## build all 4 wheels and add them to main platform body 
    for iWheel in range( mobileRobot['wheelNumbers'] ):
        strWheelNum = str(iWheel)

        # setup friction angle regarding wheelType 0->0configuration, 1->Xconfiguration, 2->standard wheel
        frictionAngle = -mobileRobot['frictionAngle']*pow(-1,(1-mobileRobot['wheelType'])) #45°
        if iWheel == 1 or iWheel == 2: #difference in diagonal
            frictionAngle *= -1
        if mobileRobot['wheelType']==2:
            frictionAngle = 0
        # additional graphics for visualization of rollers on the wheel (JUST FOR DRAWING!):
        # graphicsWheel = [graphics.Brick(centerPoint=[0,0,0],size=[1.1*wWheel,0.7*rWheel,0.7*rWheel], color=graphics.color.lightred)]
        graphicsWheel = [graphics.Cylinder(pAxis=[-0.55*wWheel,0,0], vAxis=[1.1*wWheel,0,0], radius=0.34*rWheel, color=graphics.color.lightred)]
        if 0: # debugging roller orientations
            graphicsWheel += [graphics.Cylinder([-0.1,0,0], [0.001, 0, 0], radius = param['r'], nTiles = 64, color=graphics.color.lawngreen)]
        
        if flagGraphicsRollers: 
            contour =  [[-param['lRoll']/2, 0]]  
            x = np.linspace(start = - param['lRoll']/2, stop = param['lRoll']/2, num=21) 
            for i in range(np.size(x)):
                contour+= [[x[i], np.polyval(param['aPoly'], x[i])]]
            contour += [[param['lRoll']/2, 0]] # for a closed contour
            graphRoll = []
                
        # draw cylinders on each wheel
        for i in range(nCyl): 
            iPhi = i/nCyl*2*np.pi
            # drawing as cylinders
            if flagGraphicsRollers: # draw as cylinders
                pAxle = np.array([0,-(param['r']-param['rRoll'])*np.sin(iPhi), - (param['r']-param['rRoll'])*np.cos(iPhi)])    
                vAxle = RotationMatrixX(-iPhi ) @  RotationMatrixZ(frictionAngle) @ [0.5*param['lRoll'],0,0]
                graphicsWheel += [graphics.SolidOfRevolution(pAxis=pAxle, vAxis=vAxle, contour=contour, color=graphics.color.blue[0:3]+[1],# color=graphics.color.blue[0:3] +[alpha], 
                                                nTiles = 16)]        
            else:
                pAxle = np.array([0,rWheel*np.sin(iPhi),-rWheel*np.cos(iPhi)])
                vAxle = [0.5*wWheel*np.cos(frictionAngle+np.pi/2),0.5*wWheel*np.sin(frictionAngle+np.pi/2),0]
                vAxle2 = RotationMatrixX(iPhi)@vAxle
                rColor = graphics.color.grey
                if i >= nCyl/2: rColor = graphics.color.darkgrey
                graphicsWheel += [graphics.Cylinder(pAxis=pAxle-vAxle2, vAxis=2*vAxle2, radius=rCyl, 
                                                color=rColor)]
        # mounting wheels according platform setup
        dx = 0.5 * mobileRobot['wheelBase']
        dy = 0.5 * mobileRobot['wheelTrack']
        if iWheel == 2 or iWheel == 3: dx *= -1
        if iWheel == 1 or iWheel == 3: dy *= -1
        kRolling = mobileRobot['wheelContactStiffness']
        dRolling = mobileRobot['wheelContactDamping']
        #v0Wheel = Skew(omega0Wheel) @ initialRotationWheel @ [0,0,rWheel]   #initial angular velocity of center point
        v0Wheel = mobileRobot['platformInitialVelocity'] #approx.
        initialRotation = RotationMatrixZ(-pi/2)
        pOff = [dx,dy,0-p0Car[2]]    #[dx,dy,0]
        poseWheel = VAdd(p0Car, HT2rotationMatrix(mobileRobot['platformInitialPose'])  @ VAdd(p0Wheel,pOff))
        #add a wheel body to the main platform body
        dictWheeln = mbs.CreateRigidBody(referencePosition=poseWheel,  
                                         referenceRotationMatrix=HT2rotationMatrix(mobileRobot['platformInitialPose']) @ initialRotation,  
                                         initialVelocity=v0Wheel,  
                                         initialAngularVelocity=omega0Wheel,  
                                         inertia=inertiaWheel,  
                                         gravity=mobileRobot['gravity'],  
                                         graphicsDataList=graphicsWheel,  
                                         returnDict=True)  
        nWheelsList += [dictWheeln['nodeNumber']]
        bWheelsList += [dictWheeln['bodyNumber']]
        #markers for rigid body:
        mWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictWheeln['bodyNumber'], localPosition=[0,0,0]))
        mWheelsList += [mWheel]
        mAxle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPlatform, localPosition=VAdd(pOff,[0,0,p0Wheel[2]])))
        mAxlesList += [mAxle]
        jointLink = mbs.CreateRevoluteJoint(bodyNumbers=[bPlatform, dictWheeln['bodyNumber']], 
                                            position=pOff[0:2] + [pOff[2] + p0Wheel[2]], 
                                            axis=[0,1,0],
                                            useGlobalFrame=False, 
                                            show=True,
                                            axisRadius=wWheel*0.05, 
                                            axisLength=wWheel*1.2)
        mRevoluteBody0 += [mbs.GetObject(jointLink)['markerNumbers'][0]]
        mRevoluteBody1 += [mbs.GetObject(jointLink)['markerNumbers'][1]]
        oAxlesList +=[jointLink]  
        nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
        dataGap = (poseWheel[-1] - mobileRobot['wheelRadius'])
        if dataGap < 0: # initialize contact with the current gap between the wheel and ground
            mbs.SetNodeParameter(nGeneric, 'initialCoordinates', [0,0,dataGap])
        oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(name='wheel'+strWheelNum,markerNumbers=[markerGround, mWheel], nodeNumber = nGeneric,
                                                    discRadius=rWheel, dryFriction=[mobileRobot['friction'][0], mobileRobot['friction'][1]], dryFrictionAngle=frictionAngle,
                                                    viscousFriction = mobileRobot['viscousFrictionWheel'], 
                                                    dryFrictionProportionalZone= mobileRobot['proportionalZone'], 
                                                    rollingFrictionViscous=mobileRobot['friction'][2], 
                                                    useLinearProportionalZone=mobileRobot['linearRegularization'], 
                                                    contactStiffness=kRolling, contactDamping=dRolling,
                                                    # visualization=VObjectConnectorRollingDiscPenalty(discWidth=wWheel, color=graphics.color.blue)))
                                                    visualization=VObjectConnectorRollingDiscPenalty(show=False)))
        oRollingDiscsList += [oRolling]
    d = {'nPlatformList': nPlatformList,
        'bPlatformList': bPlatformList,
        'oRollingDiscsList': oRollingDiscsList,
        'oAxlesList': oAxlesList, 
        'oPlatformList':oPlatformList,
        'mPlatformList': mPlatformList,
        'nWheelsList':nWheelsList,
        'bWheelsList':bWheelsList,
        'mWheelsList': mWheelsList,
        'mAxlesList': mAxlesList,
        'mAxlesBodyRot0': mRevoluteBody0, 
        'mAxlesBodyRot1':  mRevoluteBody1
        }
    return d

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: calculate 4 wheel velocities for a mecanum wheel driven platform with given platform velocities  
#**author: Peter Manzl, Johannes Gerstmayr
#**notes: still under development; wheel axis is mounted at y-axis; positive angVel rotates CCW in x/y plane viewed from top; for coordinate system, see Python class definition
class MobileKinematics:
# platform setup:
# ^Y
# |    W3 +---------+ W1
# |       |         |
# |       |    +    | car center point
# |       |         |
# |    W4 +---------+ W2 
# |
# |
# +-------->X

    #**classFunction: initialize mobileKinematics class
    #**input: 
    #  R: wheel radius
    #  lx: wheel track width 
    #  ly: wheel base 
    #  wheeltype: 1=x-config (bad), 0=o-config (good) 
    #**author: Peter Manzl
    def __init__(self, R, lx, ly, flagAdjusted = False, lcx=0, lcy=0, wheeltype=0): 
        if wheeltype ==0: cc = 1
        elif wheeltype == 1: cc = -1
        else: exu.Print('wheeltype {} not implemented (yet)!'.format(wheeltype))
        
        if not(flagAdjusted): 
            self.Jacobian = 1/R * np.array([[1, -cc, -lx - cc*ly], 
                                            [1,   cc,   lx + cc*ly], 
                                            [1,   cc,  -lx - cc*ly], 
                                            [1,   -cc,  lx + cc*ly]])
        else:  # todo: add wheeltype to adjusted, this is used to test adjusted kinematics 
            # self.Jacobian = 1/R * np.array([[1, -1, -lx - ly + lcx + lcy], 
            #                                [1,   1,   lx + ly-lcx + lcy], 
            #                                [1,   1,  -lx - ly - lcx + lcy], 
            #                                [1,   -1,  lx + ly + lcx + lcy]])
            c1 = 1/(4*lx*ly) # m*g/(4*Lx*Ly)
            c2 = ly*lcx
            F1 = ((ly+lcy)*lx + c2) * c1
            F2 = ((ly-lcy)*lx + c2) * c1
            F3 = ((ly+lcy)*lx - c2) * c1
            F4 = ((ly-lcy)*lx - c2) * c1
            self.Jacobian = 1/R * np.array([[1,  -1*F1, -lx - ly + lcx + lcy], 
                                            [1,    1*F2,   lx + ly-lcx + lcy], 
                                            [1,    1*F3,  -lx - ly - lcx + lcy], 
                                            [1,   -1*F4,  lx + ly + lcx + lcy]])
        self.JacobianPInv = np.linalg.pinv(self.Jacobian)
    
    #**classFunction: calculate wheel velocities from Cartesian velocities
    #**input:
    #  vDes: desired velocity [vx, vy, omega] in the robot's local frame
    #  vx: platform  translational velocity in local x direction
    #  vy: platform translational velocity in local y direction
    #  omega: platform rotational velocity around local z axis
    #**output: 
    #   w: wheel velocities w=[w0,w1,w2,w3]
    #**author: Peter Manzl
    def GetWheelVelocities(self, vDes):
        if len(vDes) == 3: 
            return self.Jacobian @ vDes
        else: 
            if len(vDes[0,:]) > len(vDes[:,0]): vDes = vDes.transpose()
            if len(vDes.shape) != 2: 
                raise Exception('kinematics can not be calculated for given vDes')
            if len(vDes[0,:]) != 3: 
                raise Exception('wrong dimensions for Kinematics equation! ')
            nSteps = len(vDes[:,0])
            w = np.zeros([nSteps, 4]) 
            for i in range(nSteps): 
                w[i,:] = self.Jacobian @ vDes[i,:]
            return w

    #**classFunction: calculate Cartesian velocities from wheel velocities
    #**input:
    #   w: wheel velocities w=[w0,w1,w2,w3]
    #**output: 
    #  v: Cartesian velocity [vx, vy, omega] in the robot's local frame
    #  vx: platform  translational velocity in local x direction
    #  vy: platform translational velocity in local y direction
    #  omega: platform rotational velocity around local z axis
    #**author: Peter Manzl
    def GetCartesianVelocities(self, w): 
        if len(w) == 4: 
            return self.JacobianPInv @ w
        else: 
            if len(w[0,:]) > len(w[:,0]): w = w.transpose()
            if len(w.shape) != 2: 
                raise Exception('kinematics can not be calculated for given vDes')
            if len(w[0,:]) != 4: 
                raise Exception('wrong dimensions for Kinematics equation! ')
            nSteps = len(w[:,0])
            v = np.zeros([nSteps, 3]) 
            for i in range(nSteps): 
                v[i,:] = self.JacobianPInv @ w[i,:]
            return v   

#**function: create a polynomial describing a generatrix function 
#**input: 
#   param: list containing data (lRoll, aPoly, ...)
#   
#**author: Peter Manzl
#**note: create and fit a polynomial of an order high enough to approximate the given GeneratrixFunction
#   with a given tolerance. The error is measured as the Chebyshev distance.  
def Generatrix2Polynomial(param, GeneratrixFunction, tol=1e-14, nFit=101, nTest = 1001): 
    u = np.linspace(-np.pi/4, np.pi/4 , nFit) # calculate error with more! 
    x, y = GeneratrixFunction(u, param)

    # to check error 
    uTest = np.linspace(-np.pi/4, np.pi/4 , nTest)
    xTest, yTest =  GeneratrixFunction(uTest, param)

    iRoll = np.array(x > -param['lRoll']/2) & np.array(x < param['lRoll']/2)
    iRollTest = np.array(xTest > -param['lRoll']/2) & np.array(xTest < param['lRoll']/2)
    try: # add one additional index at start and end of the Roll if it exists. 
        iRoll[np.where(iRoll==True)[0][[0,-1]] + [-1,1]] = True 
        iRollTest[np.where(iRollTest==True)[0][[0,-1]] + [-1,1]] = True 
    except: 
        pass
    x = x[iRoll]
    y = y[iRoll]
    xTest = xTest[iRollTest]
    yTest = yTest[iRollTest]
    coefficients = []
    y_fit = []
    err = []
    order = [] 
    nFits = 10
    #++++++++++++++++++++++++++++++++++++++++++
    #approximate function values y via polynomial given by ** coeffs
    #iterate over polyfit until error of approximation is smaller than tol
    #==>nFits ==> int(maxOrder/2)
    for i in range(nFits): 
        order += [2*i]
        coefficients += [np.polyfit(x, y, order[i], rcond=None, full=False, w=None, cov=False)]
        # for j in range(i):  #  only use the even coefficients
        #     if (j % 2) == 1:
        #         coefficients[i][j] = 0
        err     += [yTest - np.polyval(coefficients[i], xTest)]
        # exu.Print('Polynomial of order ' + str(order[i]) + ' is used for Roll Geometry, max err = ' + str(max(abs(err[i]))))
        
        #break for loop if tolerance reached
        if max(abs(err[i])) < tol:
            ibest = i
            # exu.Print('tolerance reached for order '+ str(order[i]) + '.')
            break
    maxErr = np.max(np.abs(err),1)
    if np.min(maxErr) > tol:
        exu.Print('Warning: err fitted polynomial = ' + str(np.min(maxErr))+  ' of Polynomial > tol = ' + str(tol))
        ibest = np.argmin(maxErr)
    param['aPoly'] = coefficients[ibest]
    param['aPoly0'] = coefficients[1] # use for starting point of Newton
    # a strictly convex function is required, therefore the derivative is 
    # changing monotonously and the second derivative is positive! 
    ddy = np.zeros(xTest.size)
    for i in range(xTest.size): 
        ddy[i] = FunDDiffPoly(xTest[i], param['aPoly']) #
    if (min(ddy) >= 0): 
        exu.Print('Warning: the function seems not to be strictly convex: dd(f)/ddt <= 0 occurs! ')
        
    param['dyBoundary'] = [-FunDiffPoly(-param['lRoll']/2, param['aPoly']), -FunDiffPoly(param['lRoll']/2, param['aPoly'])] 
    param['xBoundary'] = [-param['lRoll']/2, param['lRoll']/2]
    return 

#**function: generatrix function for a roll of a Mecanum wheel
#**input: 
#   u: parameter, max. +- pi/2
#   param['r']: radius of the associated Mecanum wheel
#   param['delta']: angle of the rolls rotation axis to the wheels rotation axis
#   param['dRoll']: smallest distance of roll axis to the wheel axis
#**output: 
#   x and y values for the function in the local frame. The rotation around the 
#   local x-yxis creates the surface of the roll. 
#**author: Peter Manzl
#**notes: parametric equation, x,y are the generatrix of the roll in 
#   its local frame with the axis of rotation x, see \cite{Gfrerrer2008}.
def GeneratrixRoll(u, param): 
    x = param['dRoll']*np.cos(param['delta'])**2/np.sin(param['delta']) * np.tan(u) + param['r'] *np.sin(param['delta'])*np.sin(u);
    y = np.sqrt(np.cos(param['delta'])**2 * np.tan(u)**2 + 1) * (param['r']*np.cos(u)- param['dRoll']);
    return x, y

#**function: calculates the derivative of the polynomial $a0*x^n + ... $
#**input:
#   x: value at which the polynomial is evaluated
#   a: coefficients
#**output:
#   f: 
#**author: Peter Manzl
#**note: helper function polynomial describing a generatrix function 
def FunDiffPoly(x,a): 
    f = 0
    k = np.size(a)-1
    for i in range(k): 
        f += (k-i)*x**(k-i-1)*a[i] 
    return f

#**function:  calculates the second derivative of a polynomial
#**input:
#   x: value at which the polynomial is evaluated
#   a: coefficients
#**output:
#   f: 
#**author: Peter Manzl
#**note: helper function polynomial describing a generatrix function 
def FunDDiffPoly(x, a): 
    k = a.size -2  # order of the new polynomial 
    ddf = 0
    for i in range(k):
        ddf += (k-i+1)*(k-i)*(x** (k-i-1)) *a[i]
    return ddf





# old functions deprecated
################################################################################
def MecanumXYphi2WheelVelocities(xVel, yVel, angVel, R, Lx, Ly, wheeltype):
    if wheeltype == 0: # O-configuration
        LxLy2 = (Lx+Ly)/2
        mat = (1/R)*np.array([[1,-1,-LxLy2],
                            [1,+1,+LxLy2],
                            [1,+1,-LxLy2],
                            [1,-1,+LxLy2]])
    elif wheeltype == 1:# X-configuration 
        LxLy2 = (Lx-Ly)/2
        mat = (1/R)*np.array([[-1,-1,-LxLy2],
                            [+1,-1,+LxLy2],
                            [+1,-1,-LxLy2],
                            [-1,-1,+LxLy2]])    
        return mat @ [xVel, yVel, angVel]
    elif wheeltype == 2: # standard wheel
        exu.Print('Warning! Standardwheel not implemented yet, this Function is only a Placeholder!')
        LxLy2 = (Lx+Ly)/2
        mat = (1/R)*np.array([[-1,+1,+LxLy2],
                            [-1,-1,-LxLy2],
                            [-1,-1,+LxLy2],
                            [-1,+1,-LxLy2]])
    return mat @ [xVel, yVel, angVel]

def MecanumWheelVelocity2XYphi(w, R, Lx, Ly, wheeltype): 
    c = 0
    LxLy2 = 0
    
    if wheeltype == 0: 
        LxLy2 = (Lx+Ly)/2
        c = 1
    elif wheeltype == 1: 
        LxLy2 = (Lx-Ly)/2
        c = -1
    elif wheeltype == 2: 
        exu.Print('ToDo: Implement Standard wheels')
    else: 
        exu.Print('Warning: Only Wheeltypes 0 (O-Configuration) and 1 (X-Configuration) are considered!') 
        return []
    mat = R/(4) * np.array([[1, 1, 1, 1], 
                            [-c, c, c, -c], 
                            [-1/LxLy2, 1/LxLy2, -1/LxLy2, 1/LxLy2]])
    return mat @ w
    
