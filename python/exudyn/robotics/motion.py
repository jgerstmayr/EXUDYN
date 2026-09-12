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

from copy import copy, deepcopy
import exudyn

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
                #s = q0[joint] + vMax * t - 0.5*vMax**2/aMax
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
    
#**class: class to create a constant acceleration (optimal) PTP trajectory; trajectory ignores global max. velocities and accelerations
#**input: 
#  finalCoordinates: list or numpy array with final coordinates for profile
#  duration: duration (time) for profile
#**output: returns profile object, which is then used to compute interpolated trajectory
class ProfileConstantAcceleration:
    #**classFunction: initialize ProfileConstantAcceleration with vector of final coordinates and duration (time span)
    def __init__(self, finalCoordinates, duration):
        self.duration = duration
        self.finalCoordinates = np.array(finalCoordinates,dtype=float)
    
    #**classFunction: return a class representing profile which is used in Trajectory
    def GetBasicProfile(self, initialTime, initialCoordinates,
                      globalMaxVelocities, globalMaxAccelerations):
        if len(globalMaxVelocities)+len(globalMaxAccelerations) != 0:
            exudyn.Print('WARNING: ProfileConstantAcceleration: max acceleration and velocities in Trajectory not applicable')
        distances = self.finalCoordinates-initialCoordinates
        aMax=abs(4.*distances/self.duration**2)
        return BasicProfile(coordinateSets=[np.array(initialCoordinates,dtype=float),self.finalCoordinates],
                          initialTime=initialTime, 
                          finalTime=initialTime+self.duration,
                          accelerationTimes=[0.5*self.duration]*len(aMax), 
                          maxVelocities=abs(aMax * distances)**0.5,
                          maxAccelerations=aMax)

#**class: class to create a linear acceleration PTP profile, using a list of accelerations to define the profile; the (joint) coordinates and velocities are computed relative to values of previous profiles; ignores global max. accelerations and velocities of Trajectory
#**input: 
#  accelerationList: list of tuples (relativeTime, accelerationVector) in which relativeTime is the time relative to the start of the profile (first time must be zero!) and accelerationVector is the list of accelerations of this time point, which is then linearly interpolated
#**output: returns profile object, which is then used to compute interpolated trajectory in class Trajectory
#**example:
#   profile = ProfileLinearAccelerationsList([(0,[0.,1.,2]), (0,[1.,1.,-2])])
class ProfileLinearAccelerationsList:
    #**classFunction: initialize ProfileLinearAccelerationsList with a list of tuples containing time and acceleration vector
    def __init__(self, accelerationList):
        self.accelerationList = accelerationList
        
        if len(accelerationList) < 2:
            raise ValueError('ProfileLinearAccelerationsList (robotics.motion): profile needs at least two times and accelerations')
        if accelerationList[0][0] != 0.:
            raise ValueError('ProfileLinearAccelerationsList (robotics.motion): first time in acceleration profile needs to be zero!')
            
    
    #**classFunction: return a class representing profile which is used in Trajectory
    def GetBasicProfile(self, initialTime, initialCoordinates,
                      globalMaxVelocities, globalMaxAccelerations):
        if len(self.accelerationList[0][1]) != len(initialCoordinates):
            raise ValueError('ProfileLinearAccelerationsList (robotics.motion): length of acceleration vectors are different from size of initialCoordinates!')
        if len(globalMaxVelocities)+len(globalMaxAccelerations) != 0:
            exudyn.Print('WARNING: ProfileLinearAccelerationsList: max acceleration and velocities in Trajectory not applicable')

        return BasicProfileLinearAcceleration(initialCoordinates, initialTime, self.accelerationList)
        
#**class: class to create a synchronous motion PTP trajectory, using max. accelerations and max velocities; duration automatically computed
#**input: 
#  finalCoordinates: list or numpy array with final coordinates for profile
#  maxVelocities: list or numpy array with maximum velocities; may be empty list []; used if smaller than globalMaxVelocities
#  maxAccelerations: list or numpy array with maximum accelerations; may be empty list []; used if smaller than globalMaxAccelerations
#**output: returns profile object, which is then used to compute interpolated trajectory
class ProfilePTP:
    #**classFunction: initialize ProfilePTP with final coordinates of motion, optionally max. velocities and accelerations just for this profile (overrides global settings)
    def __init__(self, finalCoordinates, syncAccTimes=True, maxVelocities=[], maxAccelerations=[]):
        self.finalCoordinates = np.array(finalCoordinates,dtype=float)
        self.maxVelocities = np.array(maxVelocities,dtype=float)
        self.maxAccelerations = np.array(maxAccelerations,dtype=float)
        self.syncAccTimes = syncAccTimes
        if min(list(maxVelocities)+[1]) <= 0:
            raise ValueError('ProfilePTP: maxVelocities must by > 0')
        if min(list(maxAccelerations)+[1]) <= 0:
            raise ValueError('ProfilePTP: maxAccelerations must by > 0')
        if syncAccTimes==True:
            raise ValueError('ProfilePTP: syncAccTime must be False; other case yet not implemented')
        
    #**classFunction: return a class representing profile which is used in Trajectory
    def GetBasicProfile(self, initialTime, initialCoordinates,
                      globalMaxVelocities, globalMaxAccelerations):
        
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
        
        #does not work in case that sMax[i]=0 ==> vMax[i]=0
        #durations = abs(sMax)/vMax + vMax/aMax
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
            # aMaxSync[joint] = aMax[joint]
            tAccSync[joint] = vMaxSync[joint]/aMax[joint]

        return BasicProfile(coordinateSets=[np.array(initialCoordinates,dtype=float),self.finalCoordinates],
                          initialTime=initialTime, 
                          finalTime=initialTime+maxDuration,
                          accelerationTimes=tAccSync, 
                          maxVelocities=vMaxSync,
                          maxAccelerations=aMax)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**class: class to define (PTP) trajectories for robots and multibody systems; trajectories are defined for a set of coordinates (e.g. joint angles or other coordinates which need to be interpolated over time)
#**example:
##create simple trajectory for two joint coordinates:
#traj = Trajectory(initialCoordinates=[1,1], initialTime=1)
##add optimal trajectory with max. accelerations:
#traj.Add(ProfileConstantAcceleration([2.,3.],2.))
#traj.Add(ProfileConstantAcceleration([3.,-1.],2.))
##add profile with limited velocities and accelerations:
#traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))
##now evaluate trajectory at certain time point (this could be now applied in a user function)
#[s,v,a] = traj.Evaluate(t=0.5)
class Trajectory:
    #**classFunction: initialize robot link with parameters, being self-explaining
    #**input:
    #  initialTime: initial time for initial coordinates
    #  initialCoordinates: initial coordinates for profile
    #  maxVelocities: list or numpy array to describe global maximum velocities per coordinate
    #  maxAccelerations: list or numpy array to describe global maximum accelerations per coordinate
    def __init__(self, initialCoordinates, initialTime = 0, maxVelocities=[], maxAccelerations=[]):
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

    #**classFunction: returns the coordinates at the end of the (currently) Final profile
    def GetFinalCoordinates(self):
        if len(self.profiles) == 0:
            return self.initialCoordinates
        else:
            return self.profiles[-1].GetFinalCoordinates()


    #**classFunction: add successively profiles, using MotionProfile class
    def Add(self, profile):
        self.initialized = False
        ptp = profile.GetBasicProfile(self.GetTimes()[-1], self.GetFinalCoordinates(),
                                    self.globalMaxVelocities,self.globalMaxAccelerations)
        self.profiles+=[ptp]

    #**classFunction: return vector of times of start/end of profiles
    def GetTimes(self):
        timesList = [self.initialTime]
        for profile in self.profiles:
            timesList+=[profile.finalTime]
        return timesList

    #**classFunction: initialize some parameters for faster evaluation
    def Initialize(self):
        self.timesList = self.GetTimes();
        self.initialized = True

    #**classFunction: return interpolation of trajectory for coordinates, velocities and accelerations at given time
    #**output: [s, v, a] as numpy arrays representing coordinates, velocities and accelerations
    def Evaluate(self, t):
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

    #**classFunction: return interpolation of trajectory for coordinate, including velocity and acceleration coordinate at given time
    #**output: [s, v, a] being scalar position, velocity and acceleration
    #**notes: faster for single coordinate than Evaluate(...)
    def EvaluateCoordinate(self, t, coordinate):
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

    #**classFunction: iterator allows to use for x in trajectory: ... constructs
    def __iter__(self):
        return iter(self.profiles)

    #**classFunction: access to profiles via operator [], allowing trajectory[0], etc.
    def __getitem__(self, key):
        return self.profiles[key]

    #**classFunction: allow using len(trajectory)
    def __len__(self):
        return len(self.profiles)


    #**classFunction: representation of Trajectory is given a list of profiles, allowing easy inspection of data
    def __repr__(self):
        return str(self.profiles)

    # slower version:
    # def EvaluateCoordinate(self, t, coordinate):
    # [s,v,a] = self.Evaluate(t)
    # return [s[coordinate], v[coordinate], a[coordinate]]
            

# #Example:
# traj = Trajectory(initialCoordinates=[1,1], initialTime=1)
# traj.Add(ProfileConstantAcceleration([2.,3.],2.))
# traj.Add(ProfileConstantAcceleration([3.,-1.],2.))
# traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))

# n=200*2
# vals=np.zeros((n,3+2+2))
# for i in range(n):
#     t=i/n*12
#     vals[i,0] = t
#     [s,v,a] = traj.Evaluate(t)
#     vals[i,1:3] = s
#     vals[i,3:5] = v
#     vals[i,5:7] = a


# import matplotlib.pyplot as plt
# plt.close('all')
# plt.figure('pos')
# plt.plot(vals[:,0],vals[:,1],'r-')
# plt.plot(vals[:,0],vals[:,2],'b-')
# plt.figure('vel')
# plt.plot(vals[:,0],vals[:,3],'r-')
# plt.plot(vals[:,0],vals[:,4],'b-')
# plt.figure('acc')
# plt.plot(vals[:,0],vals[:,5],'r-')
# plt.plot(vals[:,0],vals[:,6],'b-')
# plt.show()

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

        # #to be computed:
        # self.finalTime
        # self.coordinateSets
        # self.coordinateSets_t
        # self.distances = self.coordinateSets[1]-self.coordinateSets[0]

        #not available:
        # self.maxVelocities = maxVelocities
        # self.maxAccelerations = maxAccelerations

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
    







