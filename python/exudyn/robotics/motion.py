#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  functionality for motion including generation of trajectories with acceleration profiles,
#           path planning and motion
#
# Author:   Johannes Gerstmayr
# Date:     2022-02-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#constants and fixed structures:
import numpy as np

from copy import copy
import exudyn
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform (#2962)
import exudyn.rigidBodyUtilities as erb

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'BasicProfile', 'ProfileConstantAcceleration', 'ProfileLinearAccelerationsList', 'ProfilePTP',
    'ProfilePolynomial5', 'ProfileCartesianLinear', 'ProfileCartesianCircular', 'ProfileCartesianScrew',
    'ProfileCartesianSpline', 'ProfileJointPath', 'Trajectory', 'BasicProfileLinearAcceleration',
    ]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#internal class, which stores information on PTP profiles
#DO NOT USE this class, as it is an internal interface, which will be adapted significantly in future!
class BasicProfile:
    #
    def __init__(self, coordinateSets, initialTime, finalTime,
                 accelerationTimes, maxVelocities, maxAccelerations):
        self.initialTime = initialTime
        self.finalTime = finalTime
        self.accelerationTimes = accelerationTimes
        self.maxVelocities = maxVelocities
        self.maxAccelerations = maxAccelerations
        self.coordinateSets = []
        self.coordinateSets += [np.array(coordinateSets[0],dtype=float)]
        self.coordinateSets += [np.array(coordinateSets[1],dtype=float)]
        self.distances = self.coordinateSets[1]-self.coordinateSets[0]
    
    #classFunction: iterator allows conversion into dict and easy inspection of data
    def __iter__(self):
        yield('initialTime',self.initialTime)
        yield('finalTime',self.finalTime)
        yield('accelerationTimes',self.accelerationTimes)
        yield('maxVelocities',self.maxVelocities)
        yield('maxAccelerations',self.maxAccelerations)
        yield('coordinateSets',self.coordinateSets)
        yield('distances',self.distances)

    #classFunction: allow operator [] access as dictionary, allowing easy access to data via subclasses
    def __getitem__(self, key):
        return dict(self)[key]

    #classFunction: representation of profile is given as dict string, allowing easy inspection of data
    def __repr__(self):
        return str(dict(self))
    
    #classFunction: return set of coordinates at start of profile
    def GetInitialCoordinates(self):
        return self.coordinateSets[0]

    #classFunction: return set of coordinates at end of profile
    def GetFinalCoordinates(self):
        return self.coordinateSets[-1]

    #classFunction: return interpolation of coordinates, velocities and accelerations at given time
    #output: [s, v, a] as numpy arrays representing coordinates, velocities and accelerations
    def Evaluate(self, time):
        deltaT = self.finalTime - self.initialTime

        s0 = copy(self.coordinateSets[0])
        s = 0.*s0
        v = 0.*s0
        a = 0.*s0
        sign = np.sign(self.distances)
        
        tAcc = self.accelerationTimes
        vMax = self.maxVelocities
        aMax = self.maxAccelerations
        
        t = time - self.initialTime
        #needs for loop because of possible differene in tAcc for joints
        for i in range(len(s)):
            if t < 0:
                s[i] = s0[i]
            elif t < tAcc[i]:
                s[i] = s0[i] + sign[i]*0.5*aMax[i]*(t*t)
                v[i] = t*sign[i]*aMax[i]
                a[i] = sign[i]*aMax[i]
            elif t < deltaT-tAcc[i]:
                s[i] = s0[i] + sign[i]*(vMax[i] * (t-tAcc[i]) + 0.5*aMax[i]*(tAcc[i]**2))
                v[i] = sign[i]*vMax[i]
            elif t <= deltaT:
                tv = deltaT-tAcc[i]
                s[i] = s0[i] + sign[i]*(vMax[i]*tv - aMax[i]*0.5*(deltaT-t)**2)
                v[i] = sign[i]*aMax[i]*(deltaT-t)
                a[i] = -sign[i]*aMax[i]
            else:
                s[i] = self.coordinateSets[1][i]

        return [s,v,a]
    
    #interpolate coordinates at time
    def EvaluateCoordinate(self, time, coordinate):
        deltaT = self.finalTime - self.initialTime

        s = 0.
        v = 0.
        a = 0.

        s0 = self.coordinateSets[0][coordinate]
        sign = np.sign(self.distances[coordinate])
        
        tAcc = self.accelerationTimes[coordinate]
        vMax = self.maxVelocities[coordinate]
        aMax = self.maxAccelerations[coordinate]
        
        t = time - self.initialTime

        if t < 0:
            s = s0
        elif t < tAcc:
            s = s0 + sign*0.5*aMax*(t*t)
            v = t*sign*aMax
            a = sign*aMax
        elif t < deltaT-tAcc:
            s = s0 + sign*(vMax * (t-tAcc) + 0.5*aMax*(tAcc*tAcc))
            v = sign*vMax
        elif t <= deltaT:
            tv = deltaT-tAcc
            s = s0 + sign*(vMax*tv - aMax*0.5*(deltaT-t)**2)
            v = sign*aMax*(deltaT-t)
            a = -sign*aMax
        else:
            s = self.coordinateSets[1][coordinate]

        return [s,v,a]
    
class ProfileConstantAcceleration:
    """class to create a constant acceleration (optimal) PTP trajectory; trajectory ignores global max. velocities and accelerations

    Args:
        finalCoordinates: list or numpy array with final coordinates for profile
        duration: duration (time) for profile

    Returns:
        returns profile object, which is then used to compute interpolated trajectory
    """
    def __init__(self, finalCoordinates, duration):
        """initialize ProfileConstantAcceleration with vector of final coordinates and duration (time span)
        """
        self.duration = duration
        self.finalCoordinates = np.array(finalCoordinates,dtype=float)
    
    def GetBasicProfile(self, initialTime, initialCoordinates,
                      globalMaxVelocities, globalMaxAccelerations):
        """return a class representing profile which is used in Trajectory
        """
        if len(globalMaxVelocities)+len(globalMaxAccelerations) != 0:
            exudyn.Print('WARNING: ProfileConstantAcceleration: max acceleration and velocities in Trajectory not applicable')
        distances = self.finalCoordinates-initialCoordinates
        aMax=abs(4.*distances/self.duration**2)
        return BasicProfile(coordinateSets=[np.array(initialCoordinates,dtype=float),self.finalCoordinates],
                          initialTime=initialTime, 
                          finalTime=initialTime+self.duration,
                          accelerationTimes=[0.5*self.duration]*len(aMax), 
                          maxVelocities=cMath.sqrt(abs(aMax * distances)),
                          maxAccelerations=aMax)

class ProfileLinearAccelerationsList:
    """class to create a linear acceleration PTP profile, using a list of accelerations to define the profile; the (joint) coordinates and velocities are computed relative to values of previous profiles; ignores global max. accelerations and velocities of Trajectory

    Args:
        accelerationList: list of tuples (relativeTime, accelerationVector) in which relativeTime is the time relative to the start of the profile (first time must be zero!) and accelerationVector is the list of accelerations of this time point, which is then linearly interpolated

    Returns:
        returns profile object, which is then used to compute interpolated trajectory in class Trajectory

    Example:
        profile = ProfileLinearAccelerationsList([(0,[0.,1.,2]), (0,[1.,1.,-2])])
    """
    def __init__(self, accelerationList):
        """initialize ProfileLinearAccelerationsList with a list of tuples containing time and acceleration vector
        """
        self.accelerationList = accelerationList
        
        if len(accelerationList) < 2:
            raise ValueError('ProfileLinearAccelerationsList (robotics.motion): profile needs at least two times and accelerations')
        if accelerationList[0][0] != 0.:
            raise ValueError('ProfileLinearAccelerationsList (robotics.motion): first time in acceleration profile needs to be zero!')
            
    
    def GetBasicProfile(self, initialTime, initialCoordinates,
                      globalMaxVelocities, globalMaxAccelerations):
        """return a class representing profile which is used in Trajectory
        """
        if len(self.accelerationList[0][1]) != len(initialCoordinates):
            raise ValueError('ProfileLinearAccelerationsList (robotics.motion): length of acceleration vectors are different from size of initialCoordinates!')
        if len(globalMaxVelocities)+len(globalMaxAccelerations) != 0:
            exudyn.Print('WARNING: ProfileLinearAccelerationsList: max acceleration and velocities in Trajectory not applicable')

        return BasicProfileLinearAcceleration(initialCoordinates, initialTime, self.accelerationList)
        
class ProfilePTP:
    """class to create a synchronous motion PTP trajectory, using max. accelerations and max velocities; duration automatically computed

    Args:
        finalCoordinates: list or numpy array with final coordinates for profile
        syncAccTimes: True: all coordinates accelerate and decelerate at the same times, the motion is a straight line in the coordinates; False: each coordinate uses its largest acceleration, so that the motion deviates from this line (#3390)
        maxVelocities: list or numpy array with maximum velocities; may be empty list []; used if smaller than globalMaxVelocities
        maxAccelerations: list or numpy array with maximum accelerations; may be empty list []; used if smaller than globalMaxAccelerations

    Returns:
        returns profile object, which is then used to compute interpolated trajectory
    """
    def __init__(self, finalCoordinates, syncAccTimes=True, maxVelocities=[], maxAccelerations=[]):
        """initialize ProfilePTP with final coordinates of motion, optionally max. velocities and accelerations just for this profile (overrides global settings)
        """
        self.finalCoordinates = np.array(finalCoordinates,dtype=float)
        self.maxVelocities = np.array(maxVelocities,dtype=float)
        self.maxAccelerations = np.array(maxAccelerations,dtype=float)
        self.syncAccTimes = syncAccTimes
        if min(list(maxVelocities)+[1]) <= 0:
            raise ValueError('ProfilePTP: maxVelocities must by > 0')
        if min(list(maxAccelerations)+[1]) <= 0:
            raise ValueError('ProfilePTP: maxAccelerations must by > 0')
        
    def GetBasicProfile(self, initialTime, initialCoordinates,
                      globalMaxVelocities, globalMaxAccelerations):
        """return a class representing profile which is used in Trajectory
        """
        
        if len(globalMaxAccelerations)+len(self.maxAccelerations)==0:
            raise ValueError('ProfilePTP: maxAccelerations must either in Trajectory or in ProfilePTP be non-empty')

        sMax = self.finalCoordinates - initialCoordinates
        
        if len(globalMaxAccelerations) != 0:
            if len(self.maxAccelerations) != 0:
                aMax = np.minimum(globalMaxAccelerations,self.maxAccelerations)
            else:
                aMax = globalMaxAccelerations
        else:
            aMax = self.maxAccelerations

        if min(aMax) == 0.:
            raise ValueError('ProfilePTP: maxAccelerations may not be zero for any coordinate; check your trajectory')


        vMax = np.sqrt(aMax*abs(sMax))
        if len(globalMaxVelocities) != 0:
            vMax = np.minimum(vMax, globalMaxVelocities)
        if len(self.maxVelocities) != 0:
            vMax = np.minimum(vMax, self.maxVelocities)
        
        durations = vMax/aMax
        for i in range(len(durations)):
            if vMax[i] != 0:
                durations[i] = durations[i] + abs(sMax[i])/vMax[i]
            elif sMax[i] != 0:
                raise ValueError('ProfilePTP: maxVelocity may only be zero in case that distance is zero; check your trajectory')
        
        
        maxDuration = max(durations)

        if self.syncAccTimes: #one path parameter from 0 to 1 for all coordinates, with the velocity and acceleration all of them allow
            moving = np.abs(sMax) > 0
            aMax = np.array(aMax, dtype=float)
            limitVelocity = np.full(len(aMax), np.inf)
            if len(globalMaxVelocities) != 0:
                limitVelocity = np.minimum(limitVelocity, globalMaxVelocities)
            if len(self.maxVelocities) != 0:
                limitVelocity = np.minimum(limitVelocity, self.maxVelocities)
            pathAcceleration = np.min(aMax[moving]/np.abs(sMax[moving]))
            pathVelocity = min(np.min(limitVelocity[moving]/np.abs(sMax[moving])), np.sqrt(pathAcceleration))
            tAcc = pathVelocity/pathAcceleration
            return BasicProfile(coordinateSets=[np.array(initialCoordinates,dtype=float),self.finalCoordinates],
                                initialTime=initialTime,
                                finalTime=initialTime + tAcc + 1./pathVelocity,
                                accelerationTimes=tAcc*np.ones(len(aMax)),
                                maxVelocities=pathVelocity*np.abs(sMax),
                                maxAccelerations=pathAcceleration*np.abs(sMax))

        #solve quadratic equation for vMax of joint axis (may be vectorized?):
        vMaxSync = copy(vMax)
        tAccSync = 0.*vMaxSync

        for joint in range(len(aMax)):
            
            b=-aMax[joint]*maxDuration
            c=abs(sMax[joint])*aMax[joint]
            det = b*b-4*c
            if det < 0:
                if det > -1e-10: #small round off, ignore because this is at max velocity
                    det = 0
                else:
                    raise ValueError('SynchronousMotion: invalid parameters in trajectory: cannot resolve synchronous profile')
            vMaxSync[joint] = min((-b-np.sqrt(det))*0.5, vMax[joint])
            tAccSync[joint] = vMaxSync[joint]/aMax[joint]

        return BasicProfile(coordinateSets=[np.array(initialCoordinates,dtype=float),self.finalCoordinates],
                          initialTime=initialTime, 
                          finalTime=initialTime+maxDuration,
                          accelerationTimes=tAccSync, 
                          maxVelocities=vMaxSync,
                          maxAccelerations=aMax)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _TimeLaw(timeLaw, tau, duration):
    """the path parameter sigma in [0, 1] and its first and second time derivative at tau = (t - t0)/duration in [0, 1]:
    'polynomial5', the polynomial of degree 5 with zero velocity and acceleration at both ends; 'constantAcceleration',
    accelerate in the first half, decelerate in the second"""
    if timeLaw == 'polynomial5':
        return (tau**3*(10. - 15.*tau + 6.*tau**2), 30.*tau**2*(1. - tau)**2/duration,
                60.*tau*(1. - 3.*tau + 2.*tau**2)/duration**2)
    if timeLaw == 'constantAcceleration':
        if tau < 0.5:
            return (2.*tau**2, 4.*tau/duration, 4./duration**2)
        return (1. - 2.*(1. - tau)**2, 4.*(1. - tau)/duration, -4./duration**2)
    raise ValueError("timeLaw must be 'polynomial5' or 'constantAcceleration', not " + str(timeLaw))


class _BasicProfileTimeLaw:
    """internal: a profile of the coordinates as a function of the time, for Trajectory; Function(t) returns [s, v, a]"""
    def __init__(self, initialCoordinates, finalCoordinates, initialTime, finalTime, Function):
        self.coordinateSets = [np.array(initialCoordinates, dtype=float), np.array(finalCoordinates, dtype=float)]
        self.initialTime = initialTime
        self.finalTime = finalTime
        self.Function = Function

    def __repr__(self):
        return str({'initialTime': self.initialTime, 'finalTime': self.finalTime, 'coordinateSets': self.coordinateSets})

    def GetInitialCoordinates(self):
        return self.coordinateSets[0]

    def GetFinalCoordinates(self):
        return self.coordinateSets[-1]

    def Evaluate(self, time):
        """[s, v, a], the coordinates, velocities and accelerations at time"""
        return self.Function(min(max(time, self.initialTime), self.finalTime))

    def EvaluateCoordinate(self, time, coordinate):
        """[s, v, a] of one coordinate at time"""
        return [x[coordinate] for x in self.Evaluate(time)]


class ProfilePolynomial5:
    """a point-to-point profile in which each coordinate follows the polynomial of degree 5 from rest to rest, with zero
    velocity and acceleration at both ends (#3262): s(t) = s0 + (s1 - s0) (10 tau^3 - 15 tau^4 + 6 tau^5), tau = (t -
    t0)/duration; the maximum velocity is 1.875 and the maximum acceleration 5.77 times (s1 - s0)/duration and (s1 -
    s0)/duration^2; the global maximum velocities and accelerations of Trajectory are not used

    Args:
        finalCoordinates: list or numpy array with the coordinates at the end of the profile
        duration: the time of the profile
    """
    def __init__(self, finalCoordinates, duration):
        if duration <= 0:
            raise ValueError('ProfilePolynomial5: duration must be > 0')
        self.duration = duration
        self.finalCoordinates = np.array(finalCoordinates, dtype=float)

    def GetBasicProfile(self, initialTime, initialCoordinates, globalMaxVelocities, globalMaxAccelerations):
        """the profile in Trajectory"""
        q0 = np.array(initialCoordinates, dtype=float)
        distances = self.finalCoordinates - q0
        def Function(t):
            (sigma, sigma_t, sigma_tt) = _TimeLaw('polynomial5', (t - initialTime)/self.duration, self.duration)
            return [q0 + sigma*distances, sigma_t*distances, sigma_tt*distances]
        return _BasicProfileTimeLaw(q0, self.finalCoordinates, initialTime, initialTime + self.duration, Function)


class _ProfileCartesian:
    """internal: the common part of the Cartesian profiles (#3262, #3272): the tool frame on a path HT(sigma), sigma in
    [0, 1] by the time law; the joint coordinates by the analytic inverse kinematics at each evaluation, the solution
    nearest to the last one evaluated, in the arm configuration of the start if keepConfiguration; the joint velocities
    from the twist of the path by the Jacobian, the accelerations by their difference in time. A subclass gives
    _Path(HT0), a function sigma -> (HT, dp, omega): the tool frame and the derivatives of its position and of its
    rotation (global angular velocity) by sigma"""
    def __init__(self, robot, duration, timeLaw, keepConfiguration):
        name = type(self).__name__
        if duration <= 0:
            raise ValueError(name + ': duration must be > 0')
        _TimeLaw(timeLaw, 0., 1.) #checks timeLaw
        self.chain = robot.GetKinematicChain()
        self.toolPosition = np.array(robot.tool.HT.translation)
        self.duration = duration
        self.timeLaw = timeLaw
        self.keepConfiguration = keepConfiguration

    def GetBasicProfile(self, initialTime, initialCoordinates, globalMaxVelocities, globalMaxAccelerations):
        """the profile in Trajectory; the joint coordinates of the end by following the path from the start"""
        name = type(self).__name__
        chain = self.chain
        q0 = np.array(initialCoordinates, dtype=float)
        Path = self._Path(chain.ToolHT(q0))
        configuration = chain.ArmConfiguration(q0) if self.keepConfiguration else None
        state = {'q': q0}
        def Pose(sigma, qReference):
            solutions = chain.InverseKinematicsAnalytic(Path(sigma)[0], qReference, configuration)
            if len(solutions) == 0:
                if self.keepConfiguration and len(chain.InverseKinematicsAnalytic(Path(sigma)[0], qReference)) != 0:
                    raise ValueError(name + ': the pose at the path parameter ' + str(sigma) + ' is not reachable in the arm configuration ' + str(configuration)
                                     + ' of the start; the path crosses a singular configuration (keepConfiguration=False follows the nearest solution)')
                raise ValueError(name + ': the pose at the path parameter ' + str(sigma) + ' is out of reach')
            return solutions[0]
        def Velocities(q, sigma, sigma_t):
            (H, dp, omega) = Path(sigma)
            return np.linalg.solve(chain.Jacobian(q, -1, self.toolPosition), np.concatenate([dp, omega])*sigma_t)
        qEnd = q0
        sigmaPrevious = 0.
        for sigma in np.linspace(0., 1., 201)[1:]: #the end along the path, its reach and its continuity
            q = Pose(sigma, qEnd)
            step = np.max(np.abs(q - qEnd))
            if step > 0.5: #a jump: tens of radians per path, a singular configuration on the path or close to it
                raise ValueError(name + ': the joint coordinates jump by ' + str(round(step, 3)) + ' rad between the path parameters '
                                 + str(sigmaPrevious) + ' and ' + str(sigma) + '; the path crosses or passes close to a singular configuration'
                                 + (', in which the arm configuration of the start cannot be kept (keepConfiguration=False follows the nearest solution)' if self.keepConfiguration else ''))
            (qEnd, sigmaPrevious) = (q, sigma)
        def Function(t):
            tau = (t - initialTime)/self.duration
            (sigma, sigma_t, sigma_tt) = _TimeLaw(self.timeLaw, tau, self.duration)
            q = Pose(sigma, state['q'])
            state['q'] = q
            v = Velocities(q, sigma, sigma_t)
            taus = [max(0., tau - 1e-6), min(1., tau + 1e-6)]
            sigmas = [_TimeLaw(self.timeLaw, x, self.duration) for x in taus]
            vs = [Velocities(Pose(x[0], q), x[0], x[1]) for x in sigmas]
            a = (vs[1] - vs[0])/((taus[1] - taus[0])*self.duration) if taus[1] > taus[0] else 0.*v
            return [q, v, a]
        profile = _BasicProfileTimeLaw(q0, qEnd, initialTime, initialTime + self.duration, Function)
        def ToolMotion(t): #the tool frame and its velocity and global angular velocity at the time t (#3283)
            (sigma, sigma_t, sigma_tt) = _TimeLaw(self.timeLaw, (min(max(t, profile.initialTime), profile.finalTime) - initialTime)/self.duration, self.duration)
            (H, dp, omega) = Path(sigma)
            return (np.array(H.translation), np.array(H.rotation), np.array(dp)*sigma_t, np.array(omega)*sigma_t)
        profile.cartesian = {'ToolMotion': ToolMotion, 'chain': chain, 'toolPosition': self.toolPosition, 'configuration': configuration}
        return profile


def _CheckHT(name, H, argument):
    if not isinstance(H, exudyn.HT):
        raise TypeError(name + ': ' + argument + ' must be an exudyn.HT')


class ProfileCartesianLinear(_ProfileCartesian):
    """a Cartesian profile of a robot: the tool frame moves on a straight line from its frame at the start of the
    profile to finalHT, with its rotation about one fixed axis (exu.HT.InterpolateSO3), by the time law timeLaw
    (#3262); the joint coordinates by the analytic inverse kinematics of the robot (KinematicChain.InverseKinematicsAnalytic)
    at each evaluation, the solution nearest to the last one evaluated, the joint velocities from the twist of the tool by
    the Jacobian, the accelerations by their difference in time; a ValueError if a pose of the path is out of reach. It
    needs a robot of the analytic inverse kinematics (6 revolute joints with a spherical wrist or of the UR geometry); a
    path close to a singular configuration gives large joint velocities, a jump of the joint coordinates on the path
    (more than 0.5 rad in 1/200 of it) raises a ValueError

    Args:
        robot: the exudyn.robotics.Robot
        finalHT: the tool frame at the end, exu.HT in global coordinates
        duration: the time of the profile
        timeLaw: 'polynomial5' (default) or 'constantAcceleration', the path parameter in time
        keepConfiguration: True: the solutions in the arm configuration of the start (KinematicChain.ArmConfiguration), as a robot controller keeps it on a Cartesian path; a path on which it cannot be kept without a jump of the joint coordinates, through or close to a singular configuration, raises a ValueError; False: the solution nearest to the last one, which may change the configuration (#3272)
    """
    def __init__(self, robot, finalHT, duration, timeLaw='polynomial5', keepConfiguration=True):
        _ProfileCartesian.__init__(self, robot, duration, timeLaw, keepConfiguration)
        _CheckHT('ProfileCartesianLinear', finalHT, 'finalHT')
        self.finalHT = finalHT

    def _Path(self, HT0):
        HT1 = self.finalHT
        dp = np.array(HT1.translation) - np.array(HT0.translation)
        omega = np.array(HT0.rotation) @ (HT0.Relative(HT1).GetRotationVector()) #the rotation of the path, global
        return lambda sigma: (HT0.InterpolateSO3(HT1, sigma), dp, omega)


class ProfileCartesianCircular(_ProfileCartesian):
    """a Cartesian profile of a robot (#3272): the tool position moves on the arc of the circle from its position at the
    start of the profile through viaPosition to the position of finalHT, the rotation of the tool from the start to that
    of finalHT about one fixed axis (exu.HT.InterpolateSO3), both by the time law timeLaw; the joint coordinates as in
    ProfileCartesianLinear; for a tool that turns with the arc, see ProfileCartesianScrew

    Args:
        robot: the exudyn.robotics.Robot
        viaPosition: a point of the arc between the start and the end, in global coordinates
        finalHT: the tool frame at the end, exu.HT in global coordinates
        duration: the time of the profile
        timeLaw: 'polynomial5' (default) or 'constantAcceleration', the path parameter in time
        keepConfiguration: True: the arm configuration of the start is kept, see ProfileCartesianLinear
    """
    def __init__(self, robot, viaPosition, finalHT, duration, timeLaw='polynomial5', keepConfiguration=True):
        _ProfileCartesian.__init__(self, robot, duration, timeLaw, keepConfiguration)
        _CheckHT('ProfileCartesianCircular', finalHT, 'finalHT')
        self.viaPosition = np.array(viaPosition, dtype=float)
        self.finalHT = finalHT

    def _Path(self, HT0):
        HT1 = self.finalHT
        p0 = np.array(HT0.translation)
        a = self.viaPosition - p0
        b = np.array(HT1.translation) - p0
        n = np.cross(a, b)
        if cMath.norm(n) <= 1e-12*max(1., cMath.norm(a)*cMath.norm(b)):
            raise ValueError('ProfileCartesianCircular: the start, viaPosition and the end are on one line; no circle passes through them')
        center = p0 + np.cross(a @ a * b - b @ b * a, n)/(2.*(n @ n)) #the center of the circle through the three points
        n = n/cMath.norm(n) #the axis about which start, via point and end follow each other
        r0 = p0 - center
        r1 = np.array(HT1.translation) - center
        angle = cMath.atan2(n @ np.cross(r0, r1), r0 @ r1)
        if angle <= 0:
            angle += 2*np.pi #the arc through the via point, counterclockwise about n
        omega = np.array(HT0.rotation) @ (HT0.Relative(HT1).GetRotationVector())
        def Path(sigma):
            r = erb.RotationVector2RotationMatrix(sigma*angle*n) @ r0
            H = HT0.InterpolateSO3(HT1, sigma)
            return (exudyn.HT(rotation=H.rotation, translation=center + r), angle*np.cross(n, r), omega)
        return Path


class ProfileCartesianScrew(_ProfileCartesian):
    """a Cartesian profile of a robot (#3272): the tool frame moves as a rigid body by a screw motion from its frame at
    the start of the profile, the rotation by angle about the axis through axisPoint along axis together with the
    translation translation along it, both by the time law timeLaw; translation = 0 is the circle about the axis with
    the tool turning with it, angle = 0 the straight line along the axis; the joint coordinates as in
    ProfileCartesianLinear

    Args:
        robot: the exudyn.robotics.Robot
        axisPoint: a point of the screw axis, in global coordinates
        axis: the direction of the screw axis, in global coordinates (normalized)
        angle: the angle of the rotation about the axis, in rad, positive by the right-hand rule
        translation: the translation along the axis
        duration: the time of the profile
        timeLaw: 'polynomial5' (default) or 'constantAcceleration', the path parameter in time
        keepConfiguration: True: the arm configuration of the start is kept, see ProfileCartesianLinear
    """
    def __init__(self, robot, axisPoint, axis, angle, translation, duration, timeLaw='polynomial5', keepConfiguration=True):
        _ProfileCartesian.__init__(self, robot, duration, timeLaw, keepConfiguration)
        if cMath.norm(axis) == 0:
            raise ValueError('ProfileCartesianScrew: axis must not be zero')
        self.axisPoint = np.array(axisPoint, dtype=float)
        self.axis = np.array(axis, dtype=float)/cMath.norm(axis)
        self.angle = angle
        self.translation = translation

    def _Path(self, HT0):
        e = self.axis
        r0 = np.array(HT0.translation) - self.axisPoint
        R0 = np.array(HT0.rotation)
        def Path(sigma):
            A = erb.RotationVector2RotationMatrix(sigma*self.angle*e)
            r = A @ r0
            return (exudyn.HT(rotation=A @ R0, translation=self.axisPoint + r + sigma*self.translation*e),
                    self.angle*np.cross(e, r) + self.translation*e, self.angle*e)
        return Path


class ProfileCartesianSpline(_ProfileCartesian):
    """a Cartesian profile of a robot (#3272): the tool position moves on the natural cubic spline through its position
    at the start of the profile and the positions of viaHTs, parametrized by the length of the chords between them; the
    rotation of the tool by the natural cubic spline of the rotation vectors of the frames relative to the start, on the
    same parameter (the rotations less than pi from the start), so that the position and the angular velocity are
    continuous at the via frames; the path parameter by the time law timeLaw; the last frame of viaHTs is the end; the
    joint coordinates as in ProfileCartesianLinear

    Args:
        robot: the exudyn.robotics.Robot
        viaHTs: a list of the tool frames the path passes, exu.HT in global coordinates; the last is the end
        duration: the time of the profile
        timeLaw: 'polynomial5' (default) or 'constantAcceleration', the path parameter in time
        keepConfiguration: True: the arm configuration of the start is kept, see ProfileCartesianLinear
    """
    def __init__(self, robot, viaHTs, duration, timeLaw='polynomial5', keepConfiguration=True):
        _ProfileCartesian.__init__(self, robot, duration, timeLaw, keepConfiguration)
        if len(viaHTs) == 0:
            raise ValueError('ProfileCartesianSpline: viaHTs must contain at least the end')
        for H in viaHTs:
            _CheckHT('ProfileCartesianSpline', H, 'every entry of viaHTs')
        self.viaHTs = list(viaHTs)

    def _Path(self, HT0):
        frames = [HT0] + self.viaHTs
        points = np.array([H.translation for H in frames])
        chords = cMath.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        if min(chords) <= 0:
            raise ValueError('ProfileCartesianSpline: two successive positions of the path are equal')
        u = np.concatenate([[0.], np.cumsum(chords)])/np.sum(chords) #the parameter of each point
        R0 = np.array(HT0.rotation)
        rotationVectors = np.array([erb.RotationMatrix2RotationVector(R0.T @ np.array(H.rotation)) for H in frames])
        (Mp, Mr) = (_NaturalSpline(u, points), _NaturalSpline(u, rotationVectors))
        def Path(sigma):
            (p, dp) = _EvaluateSpline(u, points, Mp, sigma)
            (r, dr) = _EvaluateSpline(u, rotationVectors, Mr, sigma)
            return (exudyn.HT(rotation=R0 @ erb.RotationVector2RotationMatrix(r), translation=p), dp, R0 @ (_TangentOperator(r) @ dr))
        return Path


def _NaturalSpline(u, values):
    """the second derivatives at the parameters u of the natural cubic spline through the rows of values (zero at both
    ends)"""
    m = len(u) - 1
    M = np.zeros(values.shape)
    if m > 1:
        h = np.diff(u)
        A = np.zeros((m - 1, m - 1))
        rhs = np.zeros((m - 1, values.shape[1]))
        for k in range(1, m):
            A[k - 1, k - 1] = 2.*(h[k - 1] + h[k])
            if k > 1:
                A[k - 1, k - 2] = h[k - 1]
            if k < m - 1:
                A[k - 1, k] = h[k]
            rhs[k - 1] = 6.*((values[k + 1] - values[k])/h[k] - (values[k] - values[k - 1])/h[k - 1])
        M[1:m] = np.linalg.solve(A, rhs)
    return M


def _EvaluateSpline(u, values, M, sigma):
    """the value and the derivative of the cubic spline of _NaturalSpline at sigma"""
    k = max(min(int(np.searchsorted(u, sigma, side='right')) - 1, len(u) - 2), 0)
    h = u[k + 1] - u[k]
    (x0, x1) = (u[k + 1] - sigma, sigma - u[k])
    (c0, c1) = (values[k]/h - M[k]*h/6., values[k + 1]/h - M[k + 1]*h/6.)
    return ((M[k]*x0**3 + M[k + 1]*x1**3)/(6.*h) + c0*x0 + c1*x1, (-M[k]*x0**2 + M[k + 1]*x1**2)/(2.*h) - c0 + c1)


def _TangentOperator(r):
    """the matrix T with omega = T(r) r_t, the angular velocity of exp(r) in the frame it is turned from"""
    angle = cMath.norm(r)
    rTilde = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    if angle < 1e-6: #the series, to its second order
        return np.eye(3) + 0.5*rTilde + rTilde @ rTilde/6.
    return np.eye(3) + (1. - cMath.cos(angle))/angle**2*rTilde + (angle - cMath.sin(angle))/angle**3*(rTilde @ rTilde)


class ProfileJointPath:
    """a motion along a path of waypoints in joint space which does not stop at the waypoints (#3397): straight lines
    between the waypoints, whose corners are rounded by a quadratic Bezier curve in the plane of the two edges; the
    velocity along the path is the largest one which the limits of the velocities and accelerations of the joints
    allow, from rest to rest (a time-optimal parametrization of the path, computed at numberOfPoints points per edge
    and per curve, between which the acceleration along the path is constant); it slows down where the path is
    curved, and stops at a corner which is not rounded. The joint velocities are continuous; the accelerations jump,
    as those of ProfilePTP, and exceed the limits by about 1 to 2 percent with the default numberOfPoints

    Args:
        waypoints: the coordinates of the waypoints, one per row; the first one is where the trajectory is
        blendRadius: the size of the rounding of each corner, relative to the shorter adjacent edge: the curve starts
            and ends at blendRadius/2 times its length before and after the waypoint; 1: the curves of the two corners
            of the shorter edge meet in its middle; 0: no rounding, the motion stops at each corner
        maxVelocities: the largest velocity per joint; empty: those of Trajectory
        maxAccelerations: the largest acceleration per joint; empty: those of Trajectory
        numberOfPoints: the points per edge and per curve at which the velocity along the path is computed; more
            points keep the limits closer and take longer to compute

    Example:
        trajectory = Trajectory(path[0], maxVelocities=[1.5]*6, maxAccelerations=[4]*6)
        trajectory.Add(ProfileJointPath(path, blendRadius=1))
    """
    def __init__(self, waypoints, blendRadius=1., maxVelocities=[], maxAccelerations=[], numberOfPoints=100):
        if not 0. <= blendRadius <= 1.:
            raise ValueError('ProfileJointPath: blendRadius must be in [0, 1], not ' + str(blendRadius))
        points = []
        for q in waypoints:
            q = np.array(q, dtype=float)
            if len(points) == 0 or np.any(q != points[-1]):
                points += [q]
        if len(points) < 2:
            raise ValueError('ProfileJointPath: at least two different waypoints are needed')
        self.waypoints = points
        self.blendRadius = float(blendRadius)
        self.maxVelocities = maxVelocities
        self.maxAccelerations = maxAccelerations
        self.numberOfPoints = max(int(numberOfPoints), 4)

    def _Segments(self):
        """the pieces of the path: ('line', q0, q1) or ('curve', A, P, C), the Bezier curve from A to C with control point P"""
        P = self.waypoints
        corners = [] #per interior waypoint: the start and the end of its curve
        for k in range(1, len(P) - 1):
            (eIn, eOut) = (P[k] - P[k - 1], P[k + 1] - P[k])
            (lIn, lOut) = (float(cMath.norm(eIn)), float(cMath.norm(eOut)))
            r = 0.5*self.blendRadius*min(lIn, lOut)
            corners += [(P[k] - r/lIn*eIn, P[k] + r/lOut*eOut) if r > 0. else None]
        segments = []
        start = P[0]
        for k in range(1, len(P)):
            corner = corners[k - 1] if k < len(P) - 1 else None
            end = corner[0] if corner is not None else P[k]
            if np.any(end != start):
                segments += [('line', start, end)]
            if corner is not None:
                segments += [('curve', corner[0], P[k], corner[1])]
                start = corner[1]
            else:
                start = P[k]
        return segments

    @staticmethod
    def _Derivatives(segment, u):
        """q, dq/du and d2q/du2 of a segment at its parameter u in [0, 1]"""
        if segment[0] == 'line':
            (q0, q1) = segment[1:]
            return (q0 + u*(q1 - q0), q1 - q0, 0.*q0)
        (A, P, C) = segment[1:]
        return ((1. - u)**2*A + 2.*u*(1. - u)*P + u**2*C, 2.*(1. - u)*(P - A) + 2.*u*(C - P), 2.*(A - 2.*P + C))

    def GetBasicProfile(self, initialTime, initialCoordinates, globalMaxVelocities, globalMaxAccelerations):
        """the profile in Trajectory"""
        vMax = np.array(self.maxVelocities if len(self.maxVelocities) else globalMaxVelocities, dtype=float)
        aMax = np.array(self.maxAccelerations if len(self.maxAccelerations) else globalMaxAccelerations, dtype=float)
        if len(vMax) == 0 or len(aMax) == 0:
            raise ValueError('ProfileJointPath: maxVelocities and maxAccelerations are needed, of the profile or of Trajectory')
        segments = self._Segments()
        n = self.numberOfPoints
        #the path parameter p: segment i for p in [starts[i], starts[i] + lengths[i]], with |dq/dp| = 1 at the ends of
        #each segment, so that the joint velocities are continuous where two segments meet
        lengths = [float(cMath.norm(seg[2] - seg[1]))*(1. if seg[0] == 'line' else 2.) for seg in segments]
        starts = np.concatenate([[0.], np.cumsum(lengths)])
        def Derivatives(i, pLocal):
            """q, dq/dp and d2q/dp2 in segment i at p = starts[i] + pLocal"""
            (q, q_u, q_uu) = self._Derivatives(segments[i], pLocal/lengths[i])
            return (q, q_u/lengths[i], q_uu/lengths[i]**2)
        p, Qu, Quu, corners = [], [], [], []
        for (i, segment) in enumerate(segments):
            for u in np.linspace(0., 1., n + 1)[(0 if i == 0 else 1):]:
                (q, q_u, q_uu) = Derivatives(i, u*lengths[i])
                if u == 1. and i + 1 < len(segments): #a point between two segments must suit the derivatives of both
                    (q, q_u1, q_uu1) = Derivatives(i + 1, 0.)
                    (q_u, q_uu) = (np.concatenate([q_u, q_u1]), np.concatenate([q_uu, q_uu1]))
                    if segment[0] == 'line' and segments[i + 1][0] == 'line': #a corner without a curve
                        corners += [len(p)]
                else:
                    (q_u, q_uu) = (np.concatenate([q_u, q_u]), np.concatenate([q_uu, q_uu]))
                p += [starts[i] + u*lengths[i]]
                Qu += [q_u]
                Quu += [q_uu]
        (p, Qu, Quu) = (np.array(p), np.array(Qu), np.array(Quu))
        (vMax2, aMax2) = (np.concatenate([vMax, vMax]), np.concatenate([aMax, aMax]))
        moving = np.abs(Qu) > 1e-12

        def Bounds(k, x):
            """the smallest and the largest d2p/dt2 at point k for x = (dp/dt)^2 which keep every joint within its
            acceleration: |q_uu x + q_u pDDot| <= aMax; None if there is none"""
            if np.any(np.abs(Quu[k][~moving[k]])*x > aMax2[~moving[k]]*(1. + 1e-12)):
                return None
            (qu, quu, a) = (Qu[k][moving[k]], Quu[k][moving[k]], aMax2[moving[k]])
            if len(qu) == 0:
                return (-np.inf, np.inf)
            (b0, b1) = ((-a - quu*x)/qu, (a - quu*x)/qu)
            (lower, upper) = (np.max(np.minimum(b0, b1)), np.min(np.maximum(b0, b1)))
            return (lower, upper) if lower <= upper*(1. + 1e-12) + 1e-12 else None

        #the largest x at each point: the velocities of the joints, and the accelerations, by bisection
        xMax = np.min((vMax2/np.maximum(np.abs(Qu), 1e-14))**2, axis=1)
        for k in range(len(p)):
            if Bounds(k, xMax[k]) is None:
                (x0, x1) = (0., xMax[k])
                for iteration in range(40):
                    x = 0.5*(x0 + x1)
                    (x0, x1) = (x, x1) if Bounds(k, x) is not None else (x0, x)
                xMax[k] = x0
        xMax[corners] = 0. #the direction changes at a corner without a curve: the motion stops
        x = xMax.copy()
        (x[0], x[-1]) = (0., 0.)
        def Largest(i, j, xKnown, yLimit, unknownAtI):
            """the largest x of the unknown end of the interval [p_i, p_j], at most yLimit, for which the constant
            acceleration d2p/dt2 = (x_j - x_i)/(2 (p_j - p_i)) keeps all joints within their accelerations at both
            ends: |q_uu x + q_u d2p/dt2| <= aMax at p_i and at p_j, each linear in the unknown x"""
            h = 2.*(p[j] - p[i])
            (lower, upper) = (0., yLimit)
            for (k, atK) in [(i, unknownAtI), (j, not unknownAtI)]:
                #pDDot = (x_j - x_i)/h = sign*y/h + offset, with y the unknown
                (sign, offset) = (-1./h, xKnown/h) if unknownAtI else (1./h, -xKnown/h)
                alpha = Qu[k]*sign + (Quu[k] if atK else 0.)
                beta = Qu[k]*offset + (0. if atK else Quu[k]*xKnown)
                for (al, be, a) in zip(alpha, beta, aMax2):
                    if abs(al) < 1e-14:
                        continue
                    (b0, b1) = ((-a - be)/al, (a - be)/al)
                    (lower, upper) = (max(lower, min(b0, b1)), min(upper, max(b0, b1)))
            return max(min(upper, yLimit), 0.) if lower <= upper else max(min(lower, yLimit), 0.)
        for k in range(len(p) - 1): #forward with the largest, backward with the smallest acceleration
            x[k + 1] = Largest(k, k + 1, x[k], x[k + 1], False)
        for k in range(len(p) - 1, 0, -1):
            x[k - 1] = Largest(k - 1, k, x[k], x[k - 1], True)
        pDot = np.sqrt(x)
        dt = 2.*np.diff(p)/np.maximum(pDot[:-1] + pDot[1:], 1e-14)
        times = initialTime + np.concatenate([[0.], np.cumsum(dt)])
        pDDot = (pDot[1:]**2 - pDot[:-1]**2)/(2.*np.diff(p)) #constant in each interval

        def Function(t):
            k = int(min(max(np.searchsorted(times, t, side='right') - 1, 0), len(dt) - 1))
            tau = t - times[k]
            pp = min(p[k] + pDot[k]*tau + 0.5*pDDot[k]*tau**2, p[k + 1])
            pd = max(pDot[k] + pDDot[k]*tau, 0.)
            i = int(min(max(np.searchsorted(starts, pp, side='right') - 1, 0), len(segments) - 1))
            (q, q_u, q_uu) = Derivatives(i, min(pp - starts[i], lengths[i]))
            return [q, q_u*pd, q_uu*pd**2 + q_u*pDDot[k]]
        return _BasicProfileTimeLaw(self.waypoints[0], self.waypoints[-1], initialTime, times[-1], Function)


class Trajectory:
    """class to define (PTP) trajectories for robots and multibody systems; trajectories are defined for a set of coordinates (e.g. joint angles or other coordinates which need to be interpolated over time)

    Example:
        #create simple trajectory for two joint coordinates:
        traj = Trajectory(initialCoordinates=[1,1], initialTime=1)
        #add optimal trajectory with max. accelerations:
        traj.Add(ProfileConstantAcceleration([2.,3.],2.))
        traj.Add(ProfileConstantAcceleration([3.,-1.],2.))
        #add profile with limited velocities and accelerations:
        traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))
        #now evaluate trajectory at certain time point (this could be now applied in a user function)
        [s,v,a] = traj.Evaluate(t=0.5)
    """
    def __init__(self, initialCoordinates, initialTime = 0, maxVelocities=[], maxAccelerations=[], jumpTolerance=1e-6):
        """initialize robot link with parameters, being self-explaining

        Args:
            initialTime: initial time for initial coordinates
            initialCoordinates: initial coordinates for profile
            maxVelocities: list or numpy array to describe global maximum velocities per coordinate
            maxAccelerations: list or numpy array to describe global maximum accelerations per coordinate
            jumpTolerance: Add raises a ValueError if a profile starts at coordinates that differ by more than this from the end of the profile before, e.g. a Cartesian profile whose inverse kinematics starts in another solution (#3272)
        """
        self.initialTime = initialTime
        self.initialCoordinates = initialCoordinates
        self.jumpTolerance = jumpTolerance
        self.globalMaxVelocities = maxVelocities
        self.globalMaxAccelerations = maxAccelerations

        self.profiles = []
        self.blendTimes = [] #per profile, the overlap with the next one (#3274)
        self.blendSpaces = [] #per profile, 'joint' or 'cartesian' (#3283)
        self.initialized = False #this marks that the trajectory has been changed and is not initialized
        self.timesList = []
        if min(list(maxVelocities)+[1]) <= 0:
            raise ValueError('Trajectory: maxVelocities must by > 0')
        if min(list(maxAccelerations)+[1]) <= 0:
            raise ValueError('Trajectory: maxAccelerations must by > 0')

    @classmethod
    def FromWaypoints(cls, waypoints, maxVelocities, maxAccelerations, blend='auto', blendFactor=1., initialTime=0.):
        """A trajectory of point-to-point motions (ProfilePTP) through waypoints, blended so that the robot does not stop at each of them (#3383).

        Args:
            waypoints: the coordinates of the waypoints, one per row, at least two different ones; repeated waypoints are left out
            maxVelocities: the largest velocity per coordinate
            maxAccelerations: the largest acceleration per coordinate
            blend: blendTimeWithNext for all profiles, 'auto' or a time; see Add
            blendFactor: the blend times are multiplied by this factor; 0 stops at each waypoint
            initialTime: the time at the start

        Returns:
            the Trajectory
        """
        points = []
        for q in waypoints:
            q = np.array(q, dtype=float)
            if len(points) == 0 or np.any(q != points[-1]):
                points += [q]
        if len(points) < 2:
            raise ValueError('Trajectory.FromWaypoints: at least two different waypoints are needed')
        trajectory = cls(points[0], initialTime=initialTime, maxVelocities=maxVelocities, maxAccelerations=maxAccelerations)
        for q in points[1:]:
            trajectory.Add(ProfilePTP(q, syncAccTimes=True), blendTimeWithNext=blend) #a straight line in the coordinates, as the edges of a path
        if blendFactor != 1.:
            trajectory.blendTimes = [b if b == 'auto' else b*blendFactor for b in trajectory.blendTimes]
            trajectory.initialized = False
        return trajectory

    def GetFinalCoordinates(self):
        """returns the coordinates at the end of the (currently) Final profile
        """
        if len(self.profiles) == 0:
            return self.initialCoordinates
        else:
            return self.profiles[-1].GetFinalCoordinates()


    def Add(self, profile, blendTimeWithNext=0., blendSpace='joint'):
        """add successively profiles, using MotionProfile class; raises a ValueError if the profile starts at other
        coordinates than the trajectory ends, by more than jumpTolerance

        Args:
            profile: the profile, e.g. ProfilePTP, ProfilePolynomial5 or ProfileCartesianLinear
            blendTimeWithNext: the time by which the next profile starts before this one ends (#3274): in this time, the
                trajectory is the sum of the motions of both (the superposition of the two rest-to-rest profiles), so
                that it passes near the end of this profile without stopping and reaches the end of the next one
                exactly; at most the duration of this profile and of the next; e.g. the deceleration time of this
                profile, if the next one accelerates in the same time, keeps the velocity of an axis that moves on in
                the same direction; without a next profile it has no effect; see GetBlendings and Evaluate(t,
                enableBlending). 'auto': when the next profile is added, the largest overlap - at most the deceleration
                phase of this profile and the acceleration phase of the next (half their durations for the profiles
                without phases) - in which the blended velocities stay within the limits: the peak velocities of the two
                profiles, so that the blending is not faster than they are, or maxVelocities of Trajectory where they
                are larger (#3283). With maxVelocities of Trajectory, a given blend time whose velocities exceed these
                limits raises a ValueError
            blendSpace: 'joint': the motions of the joints are summed; 'cartesian': this profile and the next are
                Cartesian profiles (ProfileCartesianLinear, ...) of the same robot, and in the overlap the motions of the
                tool - its position and rotation from the start of the next profile - are summed, the joint coordinates
                follow by the inverse kinematics in the arm configuration of this profile; the tool then rounds the
                corner in the plane of two lines, while the blending of the joints lets it leave the paths (#3283)
        """
        self.initialized = False
        previousCoordinates = np.array(self.GetFinalCoordinates(), dtype=float)
        ptp = profile.GetBasicProfile(self._UnblendedTimes()[-1], previousCoordinates,
                                    self.globalMaxVelocities,self.globalMaxAccelerations)
        jump = np.max(np.abs(np.array(ptp.Evaluate(ptp.initialTime)[0]) - previousCoordinates)) #the check of jumps (#3272)
        if jump > self.jumpTolerance:
            raise ValueError('Trajectory.Add: the profile ' + type(profile).__name__ + ' (number ' + str(len(self.profiles))
                             + ') starts with a jump of ' + str(jump) + ' in the coordinates, more than jumpTolerance=' + str(self.jumpTolerance))
        duration = ptp.finalTime - ptp.initialTime
        if blendTimeWithNext != 'auto' and (blendTimeWithNext < 0 or blendTimeWithNext > duration):
            raise ValueError('Trajectory.Add: blendTimeWithNext=' + str(blendTimeWithNext) + " must be 'auto' or in [0, " + str(duration) + '], the duration of the profile')
        if blendSpace not in ['joint', 'cartesian']:
            raise ValueError("Trajectory.Add: blendSpace must be 'joint' or 'cartesian', not " + str(blendSpace))
        if len(self.profiles) != 0 and self.blendTimes[-1] != 'auto':
            if self.blendTimes[-1] > duration:
                raise ValueError('Trajectory.Add: the blendTimeWithNext=' + str(self.blendTimes[-1]) + ' of the profile before is longer than this profile, ' + str(duration))
        if len(self.profiles) != 0 and self.blendSpaces[-1] == 'cartesian':
            if not (hasattr(self.profiles[-1], 'cartesian') and hasattr(ptp, 'cartesian')):
                raise ValueError("Trajectory.Add: blendSpace='cartesian' needs two Cartesian profiles, this one (" + type(profile).__name__ + ') and the one before')
        self.profiles+=[ptp]
        self.blendTimes+=[blendTimeWithNext]
        self.blendSpaces+=[blendSpace]
        if len(self.profiles) > 1: #the blending with the profile before (#3283)
            k = len(self.profiles) - 2
            if self.blendTimes[k] == 'auto':
                self.blendTimes[k] = self._AutoBlendTime(k)
            elif self.blendTimes[k] > 0 and len(self.globalMaxVelocities) != 0:
                ratio = self._BlendingVelocityRatio(k)
                if ratio > 1. + 1e-6:
                    raise ValueError('Trajectory.Add: the blending of the profiles ' + str(k) + ' and ' + str(k+1) + ' reaches ' + str(round(ratio, 4))
                                     + ' times its limit, the peak velocity of the profiles or maxVelocities; a shorter blendTimeWithNext, or ' + "'auto'")
        self.initialized = False

    def _BlendingVelocities(self, k, numberOfSamples=51):
        """the largest absolute velocities per coordinate in the blending of the profiles k and k+1, and the peak
        velocities of the two profiles without blending (#3283)"""
        self.Initialize()
        b = self.blendTimes[k]
        tEnd = self.GetTimes()[k+1]
        blended = np.max([np.abs(self._EvaluateBlended(t)[1]) for t in np.linspace(tEnd - b, tEnd, numberOfSamples)], axis=0)
        profiles = np.max([np.abs(profile.Evaluate(t)[1]) for profile in self.profiles[k:k+2]
                           for t in np.linspace(profile.initialTime, profile.finalTime, numberOfSamples)], axis=0)
        return (blended, profiles)

    def _BlendingVelocityRatio(self, k):
        """the largest ratio of the blended velocity to its limit: the peak velocity of the two profiles, or
        maxVelocities if larger (#3283); a profile that exceeds maxVelocities itself is not the blending's fault"""
        (blended, profiles) = self._BlendingVelocities(k)
        limits = np.maximum(np.array(self.globalMaxVelocities, dtype=float), profiles) if len(self.globalMaxVelocities) != 0 else profiles
        tolerance = 1e-12*max(1., np.max(limits))
        return max([blended[i]/limits[i] if limits[i] > tolerance else (1. if blended[i] <= tolerance else np.inf)
                    for i in range(len(blended))])

    def _AutoBlendTime(self, k):
        """the largest blend time of the profiles k and k+1 within the phases of deceleration and acceleration whose
        velocities stay within the limits (#3283)"""
        def Phase(profile): #the acceleration (deceleration) time, or half the duration for a profile without phases
            times = getattr(profile, 'accelerationTimes', None)
            duration = profile.finalTime - profile.initialTime
            return min(duration, max(times)) if times is not None and len(times) != 0 else 0.5*duration
        bMax = min(Phase(self.profiles[k]), Phase(self.profiles[k+1]))
        for b in np.linspace(bMax, 0., 21)[:-1]:
            self.blendTimes[k] = b
            if self._BlendingVelocityRatio(k) <= 1. + 1e-6:
                return b
        self.blendTimes[k] = 0.
        return 0.

    def _UnblendedTimes(self):
        """the times of start and end of the profiles, one after the other"""
        timesList = [self.initialTime]
        for profile in self.profiles:
            timesList+=[profile.finalTime]
        return timesList

    def _Shifts(self):
        """per profile, the time by which it starts earlier with blending: the blend times of the profiles before it"""
        return np.concatenate([[0.], np.cumsum(self.blendTimes[:-1])]) if len(self.profiles) != 0 else np.zeros(0)

    def GetTimes(self, enableBlending=True):
        """return vector of times of start/end of profiles; with blending (#3274), the start time and the times at which
        the profiles end, each earlier by the blend times of the profiles before it
        """
        if not enableBlending:
            return self._UnblendedTimes()
        shifts = self._Shifts()
        return [self.initialTime] + [profile.finalTime - shifts[k] for (k, profile) in enumerate(self.profiles)]

    def HasBlending(self):
        """True if a profile, which has a next one, has a blendTimeWithNext"""
        return any(b != 'auto' and b > 0 for b in self.blendTimes[:-1])

    def Initialize(self):
        """initialize some parameters for faster evaluation
        """
        self.timesList = self._UnblendedTimes()
        self.shifts = self._Shifts()
        self.hasBlending = self.HasBlending()
        times = self.GetTimes()
        #the overlaps blended in Cartesian space: [start, end, k] in the time of the trajectory (#3283)
        self.cartesianBlendings = [[times[k+1] - self.blendTimes[k], times[k+1], k] for k in range(len(self.profiles) - 1)
                                   if self.blendSpaces[k] == 'cartesian' and self.blendTimes[k] > 0]
        self.initialized = True

    def _EvaluateBlended(self, t, coordinate=None):
        """the superposition of the profiles (#3274): the initial coordinates plus the displacement of each profile from
        its start, at the time t shifted into the unblended time of the profile; the profiles that have not started add
        nothing, those that have ended their whole displacement"""
        def Values(profile, time):
            return profile.Evaluate(time) if coordinate is None else profile.EvaluateCoordinate(time, coordinate)
        for (tStart, tEnd, k) in self.cartesianBlendings:
            if tStart < t < tEnd:
                [s, v, a] = self._CartesianBlended(t, k)
                return [s, v, a] if coordinate is None else [s[coordinate], v[coordinate], a[coordinate]]
        initial = np.array(self.profiles[0].GetInitialCoordinates(), dtype=float)
        s = initial.copy() if coordinate is None else initial[coordinate]
        v = 0.*s
        a = 0.*s
        for (k, profile) in enumerate(self.profiles):
            time = t + self.shifts[k]
            if time <= profile.initialTime:
                break #the later profiles start later
            start = np.array(profile.GetInitialCoordinates(), dtype=float)
            if time >= profile.finalTime:
                end = np.array(profile.GetFinalCoordinates(), dtype=float)
                s = s + ((end - start) if coordinate is None else (end - start)[coordinate])
            else:
                [sk, vk, ak] = Values(profile, time)
                s = s + (sk - start if coordinate is None else sk - start[coordinate])
                v = v + vk
                a = a + ak
        return [s, v, a]

    def _CartesianBlended(self, t, k, joint=True):
        """the blending of the Cartesian profiles k and k+1 at t in their overlap (#3283): the tool frame of profile k
        moved by the motion of the tool of profile k+1 from its start - position p_k + p_k1 - p_k1(start), rotation
        A R_k with A = R_k1 R_k1(start)^T -, its velocity v_k + v_k1 and angular velocity omega_k1 + A omega_k; the
        joint coordinates by the inverse kinematics nearest to the blending of the joints, the joint velocities by the
        Jacobian, the accelerations by their difference in time"""
        (profile0, profile1) = (self.profiles[k], self.profiles[k+1])
        data = profile0.cartesian
        chain = data['chain']
        def Joint(time):
            (p0, R0, v0, w0) = data['ToolMotion'](time + self.shifts[k])
            (p1, R1, v1, w1) = profile1.cartesian['ToolMotion'](time + self.shifts[k+1])
            (pS, RS, vS, wS) = profile1.cartesian['ToolMotion'](profile1.initialTime)
            A = R1 @ RS.T
            H = exudyn.HT(rotation=A @ R0, translation=p0 + p1 - pS)
            qReference = self._EvaluateJointBlended(time)
            solutions = chain.InverseKinematicsAnalytic(H, qReference, data['configuration'])
            if len(solutions) == 0:
                raise ValueError('Trajectory: the Cartesian blending of the profiles ' + str(k) + ' and ' + str(k+1) + ' leaves the reach of the robot at t=' + str(time))
            q = solutions[0]
            q_t = np.linalg.solve(chain.Jacobian(q, -1, data['toolPosition']), np.concatenate([v0 + v1, w1 + A @ w0]))
            return (q, q_t)
        (q, q_t) = Joint(t)
        dt = 1e-6
        q_tt = (Joint(t + dt)[1] - Joint(t - dt)[1])/(2.*dt)
        return [q, q_t, q_tt]

    def _EvaluateJointBlended(self, t):
        """the joint coordinates of the superposition of the profiles in joint space, the reference of the inverse
        kinematics of the Cartesian blending"""
        initial = np.array(self.profiles[0].GetInitialCoordinates(), dtype=float)
        s = initial.copy()
        for (k, profile) in enumerate(self.profiles):
            time = t + self.shifts[k]
            if time <= profile.initialTime:
                break
            start = np.array(profile.GetInitialCoordinates(), dtype=float)
            end = np.array(profile.GetFinalCoordinates(), dtype=float) if time >= profile.finalTime else np.array(profile.Evaluate(time)[0], dtype=float)
            s = s + end - start
        return s

    def Evaluate(self, t, enableBlending=True):
        """return interpolation of trajectory for coordinates, velocities and accelerations at given time

        Args:
            t: the time
            enableBlending: True: the profiles overlap by their blendTimeWithNext (#3274); False: one after the other, at rest at the end of each, to compare with the blended trajectory

        Returns:
            [s, v, a] as numpy arrays representing coordinates, velocities and accelerations
        """
        if not self.initialized: self.Initialize()
        if enableBlending and self.hasBlending:
            return self._EvaluateBlended(t)

        cnt = 0
        while cnt < len(self.timesList) and t > self.timesList[cnt]:
            cnt += 1

        if cnt > 0 and cnt < len(self.timesList):
            return self.profiles[cnt-1].Evaluate(t)
        elif cnt == 0:
            u0 = self.profiles[0].GetInitialCoordinates()
            n = len(u0)
            return [u0, np.zeros(n), np.zeros(n)]
        else:
            uL = self.profiles[-1].GetFinalCoordinates()
            n = len(uL)
            return [uL, np.zeros(n), np.zeros(n)]

    def EvaluateCoordinate(self, t, coordinate, enableBlending=True):
        """return interpolation of trajectory for coordinate, including velocity and acceleration coordinate at given time

        Args:
            t: the time
            coordinate: the index of the coordinate
            enableBlending: True: the profiles overlap by their blendTimeWithNext (#3274); False: one after the other

        Returns:
            [s, v, a] being scalar position, velocity and acceleration

        Note:
            faster for single coordinate than Evaluate(...)
        """
        if not self.initialized:
            self.Initialize()
        if enableBlending and self.hasBlending:
            return self._EvaluateBlended(t, coordinate)

        cnt = 0
        while cnt < len(self.timesList) and t > self.timesList[cnt]:
            cnt += 1

        if cnt > 0 and cnt < len(self.timesList):
            return self.profiles[cnt-1].EvaluateCoordinate(t, coordinate)
        elif cnt == 0:
            return [self.profiles[0].GetInitialCoordinates()[coordinate], 0., 0.]
        else:
            return [self.profiles[-1].GetFinalCoordinates()[coordinate], 0., 0.]

    def GetBlendings(self, kinematicChain=None, numberOfSamples=101):
        """the blendings of the trajectory (#3274), one dict per profile with a blendTimeWithNext and a next profile:
        'profile', its index; 'startTime' and 'endTime', the time in which the two profiles overlap; 'viaCoordinates',
        the end of the profile, which the blended trajectory passes near; 'startDistance', the distance (2-norm of the
        coordinates) from the via coordinates at which the blending starts; 'minDistance', the closest the blended
        trajectory comes to them; with a kinematicChain (Robot.GetKinematicChain()), also 'viaToolPosition',
        'toolStartDistance' and 'toolMinDistance', the same for the position of the tool

        Args:
            kinematicChain: an exudyn.cUtilities.KinematicChain of the robot, for the distances of the tool; None: the coordinates only
            numberOfSamples: the number of times in each blending at which the distances are evaluated

        Returns:
            a list of dicts
        """
        if not self.initialized: self.Initialize()
        blendings = []
        times = self.GetTimes()
        for k in range(len(self.profiles) - 1):
            b = self.blendTimes[k]
            if b <= 0:
                continue
            via = np.array(self.profiles[k].GetFinalCoordinates(), dtype=float)
            qs = [self._EvaluateBlended(t)[0] for t in np.linspace(times[k+1] - b, times[k+1], numberOfSamples)]
            distances = [cMath.norm(q - via) for q in qs]
            d = {'profile': k, 'startTime': times[k+1] - b, 'endTime': times[k+1], 'viaCoordinates': via,
                 'startDistance': distances[0], 'minDistance': min(distances)}
            if kinematicChain is not None:
                pVia = np.array(kinematicChain.ToolHT(via).translation)
                toolDistances = [cMath.norm(np.array(kinematicChain.ToolHT(q).translation) - pVia) for q in qs]
                d.update({'viaToolPosition': pVia, 'toolStartDistance': toolDistances[0], 'toolMinDistance': min(toolDistances)})
            blendings += [d]
        return blendings

    def __iter__(self):
        """iterator allows to use for x in trajectory: ... constructs
        """
        return iter(self.profiles)

    def __getitem__(self, key):
        """access to profiles via operator [], allowing trajectory[0], etc.
        """
        return self.profiles[key]

    def __len__(self):
        """allow using len(trajectory)
        """
        return len(self.profiles)


    def __repr__(self):
        """representation of Trajectory is given a list of profiles, allowing easy inspection of data
        """
        return str(self.profiles)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#internal class, which stores information on PTP profiles with linear accelerations
#DO NOT USE this class, as it is an internal interface, which will be adapted significantly in future!
#provides initialCoordinates of profile, initial time and list of tuples with (tOffset, acceleration), in which tOffset is relative to initial time and acceleration is the current value of acceleration, linearly interpolated
class BasicProfileLinearAcceleration:
    #
    def __init__(self, initialCoordinates, initialTime, 
                 accelerationList):
        self.initialTime = initialTime
        self.accelerationList= accelerationList

        self.coordinateSets = [np.array(initialCoordinates,dtype=float)]
        self.coordinateSets_t = [0.*np.array(initialCoordinates,dtype=float)] #zero velocities at beginning

        self.Initialize() #computes according missing values

    #classFunction: initialize profile, finalTime, final coordinates and distances
    def Initialize(self):
        nAcc = len(self.accelerationList)
        if nAcc < 2:
            raise ValueError('BasicProfileLinearAcceleration (robotics.motion): profile needs at least two times and accelerations')
        
        pos0 = copy(self.coordinateSets[0])
        vel0 = copy(self.coordinateSets_t[0])
        tSpan = 0.
        
        lenAcc = len(self.accelerationList[0][1])
        #update position and velocity of profile points, using quadratic velocity and cubic position trajectory
        for i in range(nAcc-1):
            accTime0 = self.accelerationList[i]
            accTime1 = self.accelerationList[i+1]
            t0 = accTime0[0]
            if i == 0 and t0 != 0.:
                raise ValueError('BasicProfileLinearAcceleration (robotics.motion): first time in acceleration profile needs to be zero!')
            acc0 = np.array(accTime0[1],dtype=float)
            t1 = accTime1[0]
            acc1 = np.array(accTime1[1],dtype=float)
            if len(acc1) != lenAcc:
                raise ValueError('BasicProfileLinearAcceleration (robotics.motion): acceleration vectors seem to be inconsistent; check, if all acceleration vectors have same length!')
            dt = t1-t0
            if dt == 0.:
                raise ValueError('BasicProfileLinearAcceleration (robotics.motion): duration of single acceleration profiles must not be zero!')
            tSpan += dt
            
            deltaPos = vel0*dt + (0.5*dt*dt)*acc0 + (dt**2/6.)*(acc1-acc0)
            deltaVel = dt*acc0 + (dt*0.5)*(-acc0 + acc1)
            
            pos0 += deltaPos 
            vel0 += deltaVel
        
            self.coordinateSets += [copy(pos0)]
            self.coordinateSets_t += [copy(vel0)] #zero velocities at beginning

        #to be computed:
        self.finalTime = self.initialTime + tSpan
        self.distances = self.coordinateSets[-1]-self.coordinateSets[0]
    
    #classFunction: iterator allows conversion into dict and easy inspection of data
    def __iter__(self):
        yield('initialTime',self.initialTime)
        yield('accelerationList',self.accelerationList)
        yield('finalTime',self.finalTime)
        yield('coordinateSets',self.coordinateSets)
        yield('coordinateSets_t',self.coordinateSets_t)
        yield('distances',self.distances)

    #classFunction: allow operator [] access as dictionary, allowing easy access to data via subclasses
    def __getitem__(self, key):
        return dict(self)[key]

    #classFunction: representation of profile is given as dict string, allowing easy inspection of data
    def __repr__(self):
        return str(dict(self))
    
    #classFunction: return set of coordinates at start of profile
    def GetInitialCoordinates(self):
        return self.coordinateSets[0]

    #classFunction: return set of coordinates at end of profile
    def GetFinalCoordinates(self):
        return self.coordinateSets[-1]

    #classFunction: return interpolation of positions, velocities and accelerations at given time
    #output: [s, v, a] as numpy arrays representing coordinates, velocities and accelerations
    def Evaluate(self, time):

        nAcc = len(self.accelerationList)
        
        t = time-self.initialTime
        
        if time < self.initialTime:
            return [self.coordinateSets[0], self.coordinateSets_t[0], self.accelerationList[0][1]]
        if time > self.finalTime:
            return [self.coordinateSets[-1], self.coordinateSets_t[-1], self.accelerationList[-1][1]]

        for i in range(nAcc-1):
            accTime1 = self.accelerationList[i+1]
            t1 = accTime1[0]
            if t > t1: #search for according interval
                continue

            accTime0 = self.accelerationList[i]
            t0 = accTime0[0]
            pos0 = self.coordinateSets[i]
            vel0 = self.coordinateSets_t[i]
            acc0 = np.array(accTime0[1],dtype=float)
            acc1 = np.array(accTime1[1],dtype=float)

            dt = t1-t0
            tRel = time - (self.initialTime + t0) #relative time within sub-interval with linear acceleration (acc0,acc1)
            
            posAct = pos0 + 0.5*(tRel*tRel)*acc0 + tRel*vel0 + (tRel**3/(6.*dt))*(acc1-acc0)
            velAct = vel0 + tRel*acc0 + (tRel*tRel)*(acc1-acc0)/(2.*dt)
            accAct = acc0 + (tRel/dt)*(acc1-acc0)

            return [posAct, velAct, accAct]
    
    #classFunction: return interpolation of chosen (joint) position, velocity and acceleration at given time, joint coordinate/axis=coordinate
    #output: [s, v, a] as scalar values representing position, velocity and acceleration
    def EvaluateCoordinate(self, time, coordinate):
        [posAct, velAct, accAct] = self.Evaluate(time)
        return [posAct[coordinate], velAct[coordinate], accAct[coordinate]]
    







