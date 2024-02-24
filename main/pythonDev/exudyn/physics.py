#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The physics library includes helper functions and data related to physics 
#           models and parameters; for rigid body inertia, see rigidBodyUtilities
#
# Authors:  Johannes Gerstmayr
# Date:     2021-01-20
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#import numpy as np
import numpy as np
from math import exp

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: describes regularized Stribeck function with optial viscous part for given velocity,
#  $f(v) = \begin{cases} (\mu_d + \mu_{s_{off}}) v, \quad \mathrm{if} \quad |v| <= v_{reg}\\ \mathrm{Sign}(v)\left( \mu_d + \mu_{s_{off}} \mathrm{e}^{-(|v|-v_{reg})/v_{exp}} + \mu_v (|v|-v_{reg}) \right), \quad \mathrm{else}\end{cases}$
#**input:
#  vel: input velocity $v$
#  muDynamic: dynamic friction coefficient $\mu_d$
#  muStaticOffset: $\mu_{s_{off}}$, offset to dynamic friction, which gives muStaticFriction = muDynamic + muStaticOffset
#  muViscous: $\mu_v$, viscous part, acting proportional to velocity except for regVel
#  regVel: $v_{reg}$,  small regularization velocity in which the friction is linear around zero velocity (e.g., to get Newton converged)
#  expVel: $v_{exp}$,  velocity (relative to regVel, at which the muStaticOffset decreases exponentially, at vel=expVel, the factor to muStaticOffset is exp(-1) = 36.8\%)
#**output: returns velocity dependent friction coefficient (if muDynamic and muStaticOffset are friction coefficients) or friction force (if muDynamic and muStaticOffset are on force level)
#**notes: see Isermann (2008) and Armstrong-Helouvry (1991)
def StribeckFunction(vel, muDynamic, muStaticOffset, muViscous=0, expVel=1e-3, regVel=1e-3):
    if abs(vel) <= regVel and regVel != 0:
        return (muDynamic + muStaticOffset)*vel/regVel
    else:
        s = np.sign(vel)
        v = abs(vel)-regVel
        return s*(muDynamic + muStaticOffset*exp(-v/expVel) + muViscous*v)

#**function: helper function for RegularizedFriction(...)
def RegularizedFrictionStep(x,x0,h0,x1,h1):
    if x <= x0:
        return h0
    elif x < x1:
        delta = (x-x0)/(x1-x0)
        return h0 + (h1-h0)*delta**2*(3-2*delta)
    else:
        return h1

#regularized friction model:
#**function: describes regularized friction function, with increased static friction, dynamic friction and optional viscous part
#**input:
#  vel: input velocity
#  muDynamic: dynamic friction coefficient
#  muStaticOffset: offset to dynamic friction, which gives muStaticFriction = muDynamic + muStaticOffset
#  muViscous: viscous part, acting proportional to velocity for velocities larger than velDynamic; extension to mentioned references
#  velStatic: small regularization velocity at which exactly the staticFriction is reached; for smaller velocities, the friction is smooth and zero-crossing (unphysical!) (e.g., to get Newton converged)
#  velDynamic: velocity at which muDynamic is reached for first time
#**output: returns velocity dependent friction coefficient (if muDynamic and muStaticOffset are friction coefficients) or friction force (if muDynamic and muStaticOffset are on force level)
#**notes:
#  see references: Flores et al. \cite{Flores2008}, Qian et al. \cite{Qian2018}
def RegularizedFriction(vel, muDynamic, muStaticOffset, velStatic, velDynamic, muViscous=0):
    vs = velStatic
    vd = velDynamic
    mud = muDynamic
    mus = muDynamic + muStaticOffset
    if abs(vel) > vd:
        return np.sign(vel)*(muDynamic + (abs(vel)-velDynamic)*muViscous)
    elif vs <= abs(vel) and abs(vel) <= vd:
        return np.sign(vel)*RegularizedFrictionStep(abs(vel), vs, mus, vd, mud) #error in paper of Qian, Zhang and Jin: vs, mus sitched with vd, mud
    else:
        return RegularizedFrictionStep(vel, -vs, -mus, vs, mus)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute equivalent von-Mises stress given 6 stress components or list of stress6D (or stress6D in rows of np.array)
#**input:
#  stress6D: 6 stress components as list or np.array, using ordering $[\sigma_{xx}$, $\sigma_{yy}$, $\sigma_{zz}$, $\sigma_{yz}$, $\sigma_{xz}$, $\sigma_{xy}]$
#**output: returns scalar equivalent von-Mises stress or np.array of von-Mises stresses for all stress6D
def VonMisesStress(stress6D):
    s = np.array(stress6D)
    if s.ndim == 1:
        return np.sqrt(0.5*((s[0]-s[1])**2 + 
                         (s[1]-s[2])**2 + 
                         (s[2]-s[0])**2 + 
                         6*(s[3]**2+s[4]**2+s[5]**2 )))
    else: #numpy sqrt does the job:
        return np.sqrt(0.5*((s[:,0]-s[:,1])**2 + 
                         (s[:,1]-s[:,2])**2 + 
                         (s[:,2]-s[:,0])**2 + 
                         6*(s[:,3]**2+s[:,4]**2+s[:,5]**2 )))
        

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Sensor user function to compute equivalent von-Mises stress from sensor with Stress or StressLocal OutputVariableType; if more than 1 sensor is given in sensorNumbers, then the maximum stress is computed
#**input: arguments according to \texttt{SensorUserFunction}; factors are ignored
#**output: returns scalar (maximum) equivalent von-Mises stress
#**example:
##assuming s0, s1, s2 being sensor numbers with StressLocal components
#sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=[s0,s1,s2], 
#                                         fileName='solution/sensorMisesStress.txt',
#                                         sensorUserFunction=UFvonMisesStress))
def UFvonMisesStress(mbs, t, sensorNumbers, factors, configuration):
    maxStress = 0
    for sensor in sensorNumbers:
        maxStress = max(maxStress,
                        mbs.GetSensorValues(sensor, configuration))
    return [maxStress]



