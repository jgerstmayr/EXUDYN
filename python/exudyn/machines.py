#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The machines library includes helper functions and classes for
#           mechanical engineering and machine elements, in particular bearings, gears, mechanisms
#
# Authors:  Johannes Gerstmayr
# Date:     2023-01-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import numpy as np
from math import radians, cos, sin, sqrt, tan, atan, atan2, pi

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute involute (x being in radians): $y=\\tan(x)-x$; 
def Involute(x):
    if abs(x) < 0.02:
        return ApproxInvolute(x)
    return tan(x)-x

#**function: Approximate involute for $|x| < 0.02$, being more accurate than tan: $y=\\tan(x)-x \approx (1/3) x^3 + (2/15) x^5$; 
def ApproxInvolute(x):
    return (1./3.) * x**3 + (2./15.) * x**5 + (17./315.) * x**7

#**function: compute inverse of involute, see Involute(x); computes $x$ for given $y$ in $y=\\tan(x)-x$ using Newton-Raphson method
#**input: y provides given value; if warn==True, a warning is displayed if no convergence is achieved
#**notes: uses Newton-Raphson method (iteratively); usually converges within 4-5 steps
def InvInvolute(y, warn=True):
    #compute starting value:
    if abs(y) > 2:
        x0 = atan(y)
    else:
        x0 = np.cbrt(3*y) - 2./5.*y  #numpy cubic root also includes negative case!
    
    converged = False
    maxIt = 10
    it = 0
    while not converged and it < maxIt:
        tanX0 = tan(x0)
        if abs(x0) < 0.02:
            d = (y - ApproxInvolute(x0))/tanX0**2
        else:
            d = (y-(tanX0-x0))/tanX0**2
        if abs(d) < 1e-13: #last iteration should do the rest ...
            converged = True
        x0 += d
        it += 1

    if it == maxIt and warn:
        raise ValueError('ERROR: InvInvolute(...) did not converge within 10 steps!')

    return x0




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GEARS

#**class: class InvoluteGear is based on github project py_gear_gen, but improved for compatibility with Exudyn; Note the involute profile is generated using the coordinates with parametrization phi (not the polar angle): $x = r_b (\cos(\varphi) + \varphi * \sin(\varphi))$, $y = r_b (\sin(\varphi) - \varphi * \cos(\varphi))$, which avoids numerical computation of the involute function
class InvoluteGear:
    #**classFunction: Initialize class with involute gear parameters, to be then generated with GenerateGear()
    #**input:
    #  module: module of the gear (pitch diameter / number of teeth)
    #  nTeeth: number of teeth on the gear
    #  pressureAngleDeg: pressure angle of the gear in degrees
    #  fillet: radius of the fillet connecting a tooth to the root circle
    #  backlash: circumferential play between meshed teeth
    #  maxSteps: maximum steps for generating the involute profile, not all of them used
    #  arcStepSize: step size for generating arcs for root
    #  reductionToleranceDeg: angle tolerance for polyline reduction in degrees
    #  dedendumFactor: factor for dedendum (depth below the pitch circle)
    #  addendumFactor: factor for addendum (height above the pitch circle)
    #  isInternalGear: set flag True for internal gear
    #  tolerance: points closer than this distance are removed; this avoids problems with meshing later on
    def __init__(self, module=1, nTeeth=12, pressureAngleDeg=20, fillet=0, backlash=0,
                 maxSteps=100, arcStepSize=0.1, reductionToleranceDeg=0, 
                 dedendumFactor=1.157, addendumFactor=1.0, isInternalGear=False,
                 tolerance = 1e-6):
        self.module = module
        self.nTeeth = nTeeth
        self.pressureAngle = radians(pressureAngleDeg)
        self.reductionTolerance = radians(reductionToleranceDeg)
        self.isInternalGear = isInternalGear
        self.tolerance = tolerance

        # Addendum and dedendum
        self.addendum = addendumFactor * module
        self.dedendum = dedendumFactor * module

        # Adjust for internal gears
        if isInternalGear:
            self.addendum, self.dedendum = self.dedendum, self.addendum

        # Radii
        self.pitchRadius = (module * nTeeth) / 2
        self.baseRadius = cos(self.pressureAngle) * self.pitchRadius
        self.outerRadius = self.pitchRadius + self.addendum
        self.rootRadius = self.pitchRadius - self.dedendum
        self.filletRadius = fillet if not isInternalGear else 0

        # Angular properties
        self.angleToothAndGap = 2 * pi / nTeeth
        angularBacklash = backlash / (2 * self.pitchRadius)
        self.angleTooth = self.angleToothAndGap / 2 - angularBacklash if not isInternalGear else angularBacklash
        self.anglePitchIntersect = None
        self.angleFullTooth = None

        self.maxSteps = maxSteps
        self.arcStepSize = arcStepSize

    #**classFunction: returns a 2x2 numpy array for vertical mirroring
    def MirrorMatrixV(self):
        return np.array([[1, 0], [0, -1]])

    #**classFunction: compute 2D rotation matrix; not imported from exudyn.rigidBodyUtilities to avoid cyclic imports
    #**input: angle around out-of-plane axis in radiant
    #**output: 2x2 rotation matrix as np.array
    def RotationMatrix2D(self, angleRad):
        return np.array([ [np.cos(angleRad),-np.sin(angleRad)],
                          [np.sin(angleRad), np.cos(angleRad)] ]);
    
    #**classFunction: convert polarCoordinates as [radius,angle] into Cartesian coordinates [x,y] 
    def PolarToCartesian(self, polarCoordinates):
        r, ang = polarCoordinates
        return np.array([r * cos(ang), r * sin(ang)])
    
    
    #**classFunction: convert cartesianCoordinates [x,y] into polar coordinates as [radius,angle]
    def CartesianToPolar(self, cartesianCoordinates):
        x, y = cartesianCoordinates
        return np.array([sqrt(x * x + y * y), atan2(y, x)])

    #**classFunction: generate half of an involute tooth profile; later on mirrored for full tooth
    #**output: numpy array of shape (2, n) representing the half-tooth profile
    def GenerateHalfTooth(self):
        phiList = np.linspace(0, pi, self.maxSteps)
        points = []
        reachedLimit = False
        self.anglePitchIntersect = None

        for phi in phiList:
            x = self.baseRadius * (cos(phi) + phi * sin(phi))
            y = self.baseRadius * (sin(phi) - phi * cos(phi))
            point = (x, y)
            dist, theta = self.CartesianToPolar(point).tolist()

            if self.anglePitchIntersect is None and dist >= self.pitchRadius:
                self.anglePitchIntersect = theta
                self.angleFullTooth = 2 * self.anglePitchIntersect + self.angleTooth
            elif self.anglePitchIntersect is not None and theta >= self.angleFullTooth / 2:
                reachedLimit = True
                break

            if dist >= self.outerRadius:
                points.append(self.PolarToCartesian((self.outerRadius, theta)).tolist())
            elif dist <= self.rootRadius:
                points.append(self.PolarToCartesian((self.rootRadius, theta)).tolist())
            else:
                points.append((x, y))

        if not reachedLimit:
            raise ValueError("Tooth profile incomplete; check profile parameters!")

        return np.array(points).T

    #**classFunction: generate half of the root profile between teeth; later on mirrored for full root profile
    #**output: numpy array of shape (2, n) representing the half-root profile
    def GenerateHalfRoot(self):
        rootArcLength = (self.angleToothAndGap - self.angleFullTooth) * self.rootRadius
        pointsRoot = []

        for theta in np.arange(self.angleFullTooth, self.angleToothAndGap / 2 + self.angleFullTooth / 2,
                               self.arcStepSize / self.rootRadius):
            arcPosition = (theta - self.angleFullTooth) * self.rootRadius
            inFillet = min(rootArcLength - arcPosition, arcPosition) < self.filletRadius

            r = self.rootRadius
            if inFillet:
                circlePos = min(arcPosition, rootArcLength - arcPosition)
                r += self.filletRadius - sqrt(self.filletRadius**2 - (self.filletRadius - circlePos)**2)

            pointsRoot.append(self.PolarToCartesian((r, theta)).tolist())

        return np.array(pointsRoot).T

    #**classFunction: generate roots on either side of the first tooth
    #**output: list of two numpy arrays, each of shape (2, n)
    def GenerateRoots(self):
        halfRoot = self.GenerateHalfRoot()
        halfRoot = self.RotationMatrix2D(-self.angleFullTooth / 2) @ halfRoot
        pointsSecondHalf = self.MirrorMatrixV() @ halfRoot
        pointsSecondHalf = np.flip(pointsSecondHalf, axis=1)
        return [pointsSecondHalf, halfRoot]

    #**classFunction: generate a single involute tooth profile
    #**output: numpy array of shape (2, n) representing the tooth profile
    def GenerateTooth(self):
        halfTooth = self.GenerateHalfTooth()
        halfTooth = self.RotationMatrix2D(-self.angleFullTooth / 2 ) @ halfTooth
        pointsSecondHalf = self.MirrorMatrixV() @ halfTooth
        pointsSecondHalf = np.flip(pointsSecondHalf, axis=1)
        tooth = np.concatenate((halfTooth, pointsSecondHalf), axis=1)
        return tooth

    #**classFunction: a single tooth and the adjacent root profile
    #**output: numpy array of shape (2, n) representing the tooth and gap profile
    def GenerateToothAndGap(self):
        tooth = self.GenerateTooth()
        roots = self.GenerateRoots()
        toothAndGap = np.concatenate((roots[0], tooth, roots[1]), axis=1)
        return toothAndGap

    #**classFunction: generate gear according to dimensions and return Cartesian coordinates of profile
    #**output: numpy array of shape (n, 2) representing the gear outline; [x,y] coordinates are in rows of 2D numpy array
    def GenerateGear(self):
        toothAndGap = self.GenerateToothAndGap()
        teeth = [self.RotationMatrix2D(self.angleToothAndGap * n) @ toothAndGap for n in range(self.nTeeth)]
        gear = np.concatenate(teeth, axis=1)
        gear = gear.T
        
        gearFixed = []
        pLast = gear[0,:]
        for p in gear[1:]:
            if np.linalg.norm(p-pLast) >= self.tolerance:
                gearFixed.append(p)
                pLast = p
        
        return np.array(gearFixed)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ROLLING-ELEMENT BEARINGS


#**function: based on some nominal data, this function computes the data for a ball bearning, using a couple of common assumptions
#**input:
#  outsideDiameter: outer radius of outer ring
#  boreDiameter: inner radius of inner ring
#  width: width of bearing
#  radiusBalls: radius of balls; if not provided it is assumed as 58% of (outsideDiameter-boreDiameter)
#  ballsAngleOffset: angle offset for location of balls (in radiant)
#  radiusCage: radius of centers of balls; if not provided it is assumed as (outsideDiameter+boreDiameter)/2
#  innerGrooveRadius: depends on osculation (usually radiusBalls*1.02 ... radiusBalls*1.04); if not provided assumed as radiusBalls*1.04
#  outerGrooveRadius: same as rGrooveInner if not provided
#  innerRingShoulderRadius: outer radius of inner ring
#  outerRingShoulderRadius: inner radius of outer ring
#  heightCage: radial dimension of cage (just for drawing)
#  widthCage: width of cage (just for drawing)
#  outerEdgeChamfer: chamfer or radius
#  innerEdgeChamfer: chamfer or radius
#**output: returns dictionary with all data as used by further bearing functions; note: 'ballPositions' are in the local bearing coordinate system!
def GetBallBearingData(axis, outsideDiameter, boreDiameter, width, nBalls, 
                       radiusBalls=None, ballsAngleOffset=0,
                       radiusCage=None, innerGrooveRadius=None, outerGrooveRadius=None,
                       innerRingShoulderRadius=None, outerRingShoulderRadius=None, 
                       widthCage=None, heightCage=None,
                       innerEdgeChamfer=None, outerEdgeChamfer=None):

    from exudyn.rigidBodyUtilities import ComputeOrthonormalBasisVectors
    outsideRadius = 0.5*outsideDiameter
    boreRadius = 0.5*boreDiameter
    
    #some default parameters (these are rough approximations!)
    if radiusBalls==None:
        radiusBalls = 0.58*(outsideRadius-boreRadius)
    if radiusCage==None:
        radiusCage = 0.5*(outsideRadius+boreRadius)
    if innerGrooveRadius==None:
        innerGrooveRadius = 1.04*radiusBalls
    if outerGrooveRadius==None:
        outerGrooveRadius = 1.04*radiusBalls
    if innerRingShoulderRadius==None:
        innerRingShoulderRadius=radiusCage-0.3*radiusBalls
    if outerRingShoulderRadius==None:
        outerRingShoulderRadius=radiusCage+0.3*radiusBalls
    if widthCage==None:
        widthCage=2.5*radiusBalls #just for drawing
    if heightCage==None:
        heightCage=(outerRingShoulderRadius-innerRingShoulderRadius)*0.7 #just for drawing
    if outerEdgeChamfer==None:
        outerEdgeChamfer=radiusBalls/8
    if innerEdgeChamfer==None:
        innerEdgeChamfer=radiusBalls/8
    
    lenAxis = np.linalg.norm(axis)
    vAxis0 = np.array(axis)/lenAxis
    innerGrooveTorusRadius = radiusCage+innerGrooveRadius-radiusBalls
    outerGrooveTorusRadius = radiusCage+radiusBalls-outerGrooveRadius

    
    [v,ny,nz] = ComputeOrthonormalBasisVectors(vAxis0)
    ballPositions = []
    for i in range(nBalls):
        phi = i/nBalls*2*pi
        pos = radiusCage*(cos(phi)*ny + sin(phi)*nz)
        ballPositions.append(pos)

    data = {#from input data
            'axis':vAxis0,
            'outsideDiameter':outsideDiameter,
            'boreDiameter':boreDiameter,
            'width':width,
            'nBalls':nBalls,
            'radiusBalls':radiusBalls,
            'radiusCage':radiusCage,
            'innerGrooveRadius':innerGrooveRadius,
            'outerGrooveRadius':outerGrooveRadius,
            'innerRingShoulderRadius':innerRingShoulderRadius,
            'outerRingShoulderRadius':outerRingShoulderRadius,
            'widthCage':widthCage,
            'heightCage':heightCage,
            'innerEdgeChamfer':innerEdgeChamfer,
            'outerEdgeChamfer':outerEdgeChamfer,
            #additional data:
            'innerGrooveTorusRadius':innerGrooveTorusRadius,
            'outerGrooveTorusRadius':outerGrooveTorusRadius,
            'ballPositions':ballPositions
            }
    
    return data

#**function: create ball bearing in mbs using bearingData from function GetBallBearingData; requires rigid body markers for inner ring and outer ring and creates rigid bodies for cage and balls; also adds contact between balls and inner ring and balls and outer ring
#**input:
#  mbs: a MainSystem where bearing objects are added
#  bearingData: dictionary as returned from GetBallBearingData; note that axis represents the local axis relative to markers markerInnerRing and markerOuterRing
#  markerInnerRing: a rigid body marker where the inner ring is attached to; this represents the bearing center position and orientation; bearing axis is relative to this marker
#  markerOuterRing: a rigid body marker where the outer ring is attached to
#  densityBalls: the material density for sphere rigid bodies
#  densityCage: the material density for cage rigid body, assuming a hollow cylinder with dimensions given in bearingData; density should be adjusted to weight of cage
#  cageInitialAngularVelocity: the global initial angular velocity vector of the cage
#  ballsInitialAngularVelocity: the global initial angular velocity vector of the balls
#  gravity: either [0,0,0] or otherwise the gravity added to balls and cage
#  springStiffnessCage: if non-zero, for simplicity spring-dampers are added between balls and cage instead of contact with cage
#  springDampingCage: if non-zero, for simplicity springs-dampers are added between balls and cage instead of contact with cage
#  contactParametersRingBalls: a dictionary with contact parameters according to ObjectContactSphereTorus, including usually contactStiffness, contactDamping, dynamicFriction, and contactStiffnessExponent
#  colorCage: cage RGBA color
#  nTilesRings: circumferential tiling of rings
#  nTilesGrooves: tiling of grooves
#  colorInnerRing: inner ring RGBA color
#  colorOuterRing: outer ring RGBA color
#  nTilesBalls: tiling of spheres for balls
#  colorBalls: balls RGBA color
#  addBallsBasis: if True, basis vectors are added to drawing of balls, to see rotation
#  ballsDrawRadiusFactor: a factor for drawing balls smaller to avoid drawing artifacts between rings and balls; ideally should be 1
#**output: returns dictionary with newly created items: objectsBalls, objectCage, innerRingBallContacts, outerRingBallContacts, objectsCageBallContact
def CreateBallBearing(mbs, bearingData, markerInnerRing, markerOuterRing, densityBalls, densityCage,
                      cageInitialAngularVelocity=[0,0,0], ballsInitialAngularVelocity=[0,0,0],
                      gravity=[0,0,0],
                      springStiffnessCage=0, springDampingCage=0,
                      contactParametersRingBalls={'contactStiffness':2e6,'contactDamping':2e2,'dynamicFriction':0.2,'contactStiffnessExponent':1},
                      nTilesRings=32, nTilesGrooves=12, colorCage=[0.6,0.57,0.4,0.4], 
                      colorInnerRing=[0.5,0.5,0.5,1], colorOuterRing=[0.5,0.5,0.5,1],
                      nTilesBalls=32, colorBalls=[0.6,0.6,0.65,1], addBallsBasis=False, ballsDrawRadiusFactor=1
                      ):
    
    import exudyn.graphics as graphics
    import exudyn.itemInterface as eii
    from exudyn.rigidBodyUtilities import InertiaSphere, InertiaCylinder, ComputeOrthonormalBasis

    bearingGraphics = graphics.BallBearingRings(**bearingData,colorCage=colorCage,nTilesRings=64)
    
    bearingCenterPos = mbs.GetMarkerOutput(markerInnerRing, exu.OutputVariableType.Position,
                                           configuration=exu.ConfigurationType.Reference)
    bearingRot = mbs.GetMarkerOutput(markerInnerRing, exu.OutputVariableType.RotationMatrix,
                                     configuration=exu.ConfigurationType.Reference).reshape((3,3))
    localBearingAxis = np.array(bearingData['axis'])
    #globalBearingAxis = bearingRot @ localBearingAxis
    radiusBalls = bearingData['radiusBalls']
    widthCage = bearingData['widthCage']
    heightCage = bearingData['heightCage']
    radiusCage = bearingData['radiusCage']
    ballPositions = bearingData['ballPositions']


    #++++++++++++++++++++++++++++++++++++++++++++++++++
    #ball objects
    objectsBalls = []
    for i, pos in enumerate(ballPositions):
        oBall = mbs.CreateRigidBody(referencePosition=bearingCenterPos+bearingRot@pos,
                                    initialVelocity=np.cross(cageInitialAngularVelocity,bearingRot@pos),
                                    initialAngularVelocity=ballsInitialAngularVelocity, # omegaBalls * globalBearingAxis, 
                                    inertia=InertiaSphere(density=densityBalls, radius=radiusBalls),
                                    gravity = gravity,
                                    graphicsDataList=[graphics.Sphere(radius=radiusBalls*ballsDrawRadiusFactor,color=colorBalls, nTiles=nTilesBalls)]+
                                                      addBallsBasis*[graphics.Basis(length=radiusBalls*1.3)],
                                    )
        objectsBalls.append(oBall)

    #++++++++++++++++++++++++++++++++++++++++++++++++++
    #cage object
    bearingLocalRot = ComputeOrthonormalBasis(localBearingAxis)
    inertiaCage = InertiaCylinder(densityCage, widthCage,
                                  outerRadius=radiusCage+0.5*heightCage,
                                  innerRadius=radiusCage-0.5*heightCage,
                                  axis=0) #always axis 0, as axis 0 is used in ComputeOrthonormalBasis

    inertiaCage = inertiaCage.Rotated(bearingLocalRot) #not this is accordint to the local bearing axis!
    
    cageGraphicsList = [bearingGraphics['cageGraphics']] if 'cageGraphics' in bearingGraphics else []
    
    objectCage = mbs.CreateRigidBody(referencePosition=bearingCenterPos,
                                     referenceRotationMatrix=bearingRot,
                                     inertia=inertiaCage,
                                     initialAngularVelocity=cageInitialAngularVelocity,
                                     gravity = gravity,
                                     graphicsDataList=cageGraphicsList)

    #++++++++++++++++++++++++++++++++++++++++++++++++++
    #contact / spring-dampers for cage-balls interaction
    objectsCageBallContact = []
    if springStiffnessCage != 0 or springDampingCage != 0:
        for k, oBall in enumerate(objectsBalls):
            oCSD = mbs.CreateCartesianSpringDamper(bodyNumbers=[oBall,objectCage],
                                                   localPosition0=[0,0,0],
                                                   localPosition1=ballPositions[k],
                                                   stiffness=[springStiffnessCage]*3, 
                                                   damping=[springDampingCage]*3, 
                                                   drawSize=radiusBalls*0.2)
            objectsCageBallContact.append(oCSD)


    #++++++++++++++++++++++++++++++++++++++++++++++++++
    #contact balls - rings
    innerRingBallContacts = []
    outerRingBallContacts = []
    for i, oMass in enumerate(objectsBalls):
        mBall = mbs.AddMarker(eii.MarkerBodyRigid(bodyNumber=oMass))
    
        for k, mRing in enumerate([markerInnerRing,markerOuterRing][0:2]):
            torusMinorRadius = bearingData['innerGrooveRadius'] if k==0 else bearingData['outerGrooveRadius']
            torusMajorRadius = bearingData['innerGrooveTorusRadius'] if k==0 else bearingData['outerGrooveTorusRadius']
    
            nData1 = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[0,0,0,0], #contact assumed at beginning
                                                numberOfDataCoordinates=4))
            oSSC = mbs.AddObject(eii.ObjectContactSphereTorus(markerNumbers=[mBall,mRing],
                                                          nodeNumber=nData1,
                                                          radiusSphere=radiusBalls,
                                                          torusMajorRadius = torusMajorRadius, 
                                                          torusMinorRadius = torusMinorRadius, 
                                                          torusAxis = localBearingAxis,
                                                          **contactParametersRingBalls,
                                                          visualization=eii.VObjectContactSphereTorus(show=False),
                                                          ))
            if k==0:
                innerRingBallContacts.append(oSSC)
            else:
                outerRingBallContacts.append(oSSC)


    data = {'objectsBalls':objectsBalls,
            'objectCage':objectCage,
            'objectsCageBallContact':objectsCageBallContact,
            'innerRingBallContacts':innerRingBallContacts,
            'outerRingBallContacts':outerRingBallContacts,
            }
    return data


#%%++++++++++++++++++++++++
#testing of involute and approximated involute
if __name__ == '__main__':
    import exudyn

    if False:
        for i in range(40):
            x = 2**(-i/2)
            exudyn.Print('x=', x,'Inv=', Involute(x), ', Approx Inv=', ApproxInvolute(x), ', diff=', ApproxInvolute(x)-Involute(x), 'err term=', (62/2835)*x**9)  
            #==> error term < diff for x<0.02

    #test InvInvolute:
    #iteration number largest for x \approx 1
    for i in range(80): #80 goes up to x=2e-46, invinv=9e-16
        x = 100*2**(-2*i) 
        exudyn.Print('x=', x,'InvInv=', InvInvolute(x), ', Involute(InvInv)-x=', Involute(InvInvolute(x))-x)  
        
    for i in range(80): #80 goes up to x=2e-46, invinv=9e-16
        x = -100*2**(-2*i)
        exudyn.Print('x=', x,'InvInv=', InvInvolute(x), ', Involute(InvInv)-x=', Involute(InvInvolute(x))-x)  

