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

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'BasicProfile', 'ProfileConstantAcceleration', 'ProfileLinearAccelerationsList', 'ProfilePTP',
    'ProfilePolynomial5', 'ProfileCartesianLinear', 'Trajectory', 'BasicProfileLinearAcceleration',
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
        if syncAccTimes:
            raise ValueError('ProfilePTP: syncAccTime must be False; other case yet not implemented')
        
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


class ProfileCartesianLinear:
    """a Cartesian profile of a robot: the tool frame moves on a straight line from its frame at the start of the
    profile to finalHT, with its rotation about one fixed axis (exu.HT.InterpolateSO3), by the time law timeLaw
    (#3262); the joint coordinates by the analytic inverse kinematics of the robot (KinematicChain.InverseKinematicsAnalytic)
    at each evaluation, the solution nearest to the last one evaluated, the joint velocities from the twist of the tool by
    the Jacobian, the accelerations by their difference in time; a ValueError if a pose of the path is out of reach. It
    needs a robot of the analytic inverse kinematics (6 revolute joints with a spherical wrist or of the UR geometry); a
    path through a singular configuration gives large joint velocities

    Args:
        robot: the exudyn.robotics.Robot
        finalHT: the tool frame at the end, exu.HT in global coordinates
        duration: the time of the profile
        timeLaw: 'polynomial5' (default) or 'constantAcceleration', the path parameter in time
    """
    def __init__(self, robot, finalHT, duration, timeLaw='polynomial5'):
        if duration <= 0:
            raise ValueError('ProfileCartesianLinear: duration must be > 0')
        _TimeLaw(timeLaw, 0., 1.) #checks timeLaw
        self.chain = robot.GetKinematicChain()
        self.toolPosition = np.array(robot.tool.HT.translation)
        if not isinstance(finalHT, exudyn.HT):
            raise TypeError('ProfileCartesianLinear: finalHT must be an exudyn.HT')
        self.finalHT = finalHT
        self.duration = duration
        self.timeLaw = timeLaw

    def GetBasicProfile(self, initialTime, initialCoordinates, globalMaxVelocities, globalMaxAccelerations):
        """the profile in Trajectory; the joint coordinates of the end by following the path from the start"""
        chain = self.chain
        q0 = np.array(initialCoordinates, dtype=float)
        HT0 = chain.ToolHT(q0)
        HT1 = self.finalHT
        velocity = np.array(HT1.translation) - np.array(HT0.translation)
        omega = np.array(HT0.rotation) @ (HT0.Relative(HT1).GetRotationVector()) #the rotation of the path, global
        state = {'q': q0}
        def Pose(sigma, qReference):
            solutions = chain.InverseKinematicsAnalytic(HT0.InterpolateSO3(HT1, sigma), qReference)
            if len(solutions) == 0:
                raise ValueError('ProfileCartesianLinear: the pose at the path parameter ' + str(sigma) + ' is out of reach')
            return solutions[0]
        def Velocities(q, sigma_t):
            return np.linalg.solve(chain.Jacobian(q, -1, self.toolPosition), np.concatenate([velocity, omega])*sigma_t)
        qEnd = q0
        for sigma in np.linspace(0., 1., 51)[1:]: #the end along the path, and its reach
            qEnd = Pose(sigma, qEnd)
        def Function(t):
            tau = (t - initialTime)/self.duration
            (sigma, sigma_t, sigma_tt) = _TimeLaw(self.timeLaw, tau, self.duration)
            q = Pose(sigma, state['q'])
            state['q'] = q
            v = Velocities(q, sigma_t)
            taus = [max(0., tau - 1e-6), min(1., tau + 1e-6)]
            sigmas = [_TimeLaw(self.timeLaw, x, self.duration) for x in taus]
            vs = [Velocities(Pose(x[0], q), x[1]) for x in sigmas]
            a = (vs[1] - vs[0])/((taus[1] - taus[0])*self.duration) if taus[1] > taus[0] else 0.*v
            return [q, v, a]
        return _BasicProfileTimeLaw(q0, qEnd, initialTime, initialTime + self.duration, Function)


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
    def __init__(self, initialCoordinates, initialTime = 0, maxVelocities=[], maxAccelerations=[]):
        """initialize robot link with parameters, being self-explaining

        Args:
            initialTime: initial time for initial coordinates
            initialCoordinates: initial coordinates for profile
            maxVelocities: list or numpy array to describe global maximum velocities per coordinate
            maxAccelerations: list or numpy array to describe global maximum accelerations per coordinate
        """
        self.initialTime = initialTime
        self.initialCoordinates = initialCoordinates
        self.globalMaxVelocities = maxVelocities
        self.globalMaxAccelerations = maxAccelerations

        self.profiles = []
        self.initialized = False #this marks that the trajectory has been changed and is not initialized
        self.timesList = []
        if min(list(maxVelocities)+[1]) <= 0:
            raise ValueError('Trajectory: maxVelocities must by > 0')
        if min(list(maxAccelerations)+[1]) <= 0:
            raise ValueError('Trajectory: maxAccelerations must by > 0')

    def GetFinalCoordinates(self):
        """returns the coordinates at the end of the (currently) Final profile
        """
        if len(self.profiles) == 0:
            return self.initialCoordinates
        else:
            return self.profiles[-1].GetFinalCoordinates()


    def Add(self, profile):
        """add successively profiles, using MotionProfile class
        """
        self.initialized = False
        ptp = profile.GetBasicProfile(self.GetTimes()[-1], self.GetFinalCoordinates(),
                                    self.globalMaxVelocities,self.globalMaxAccelerations)
        self.profiles+=[ptp]

    def GetTimes(self):
        """return vector of times of start/end of profiles
        """
        timesList = [self.initialTime]
        for profile in self.profiles:
            timesList+=[profile.finalTime]
        return timesList

    def Initialize(self):
        """initialize some parameters for faster evaluation
        """
        self.timesList = self.GetTimes();
        self.initialized = True

    def Evaluate(self, t):
        """return interpolation of trajectory for coordinates, velocities and accelerations at given time

        Returns:
            [s, v, a] as numpy arrays representing coordinates, velocities and accelerations
        """
        if not self.initialized: self.Initialize()

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

    def EvaluateCoordinate(self, t, coordinate):
        """return interpolation of trajectory for coordinate, including velocity and acceleration coordinate at given time

        Returns:
            [s, v, a] being scalar position, velocity and acceleration

        Note:
            faster for single coordinate than Evaluate(...)
        """
        if not self.initialized: 
            self.Initialize()

        cnt = 0
        while cnt < len(self.timesList) and t > self.timesList[cnt]:
            cnt += 1
        
        if cnt > 0 and cnt < len(self.timesList):
            return self.profiles[cnt-1].EvaluateCoordinate(t, coordinate)
        elif cnt == 0:
            return [self.profiles[0].GetInitialCoordinates()[coordinate], 0., 0.]
        else:
            return [self.profiles[-1].GetFinalCoordinates()[coordinate], 0., 0.]

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
    







