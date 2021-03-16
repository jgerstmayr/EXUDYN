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
from numpy import sign
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
def StribeckFunction(vel, muDynamic, muStaticOffset, muViscous=0, expVel=1e-3, regVel=1e-3):
    if abs(vel) <= regVel:
        return (muDynamic + muStaticOffset)*vel/regVel
    else:
        s = sign(vel)
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
        return sign(vel)*(muDynamic + (abs(vel)-velDynamic)*muViscous)
    elif vs <= abs(vel) and abs(vel) <= vd:
        return sign(vel)*RegularizedFrictionStep(abs(vel), vs, mus, vd, mud) #error in paper of Qian, Zhang and Jin: vs, mus sitched with vd, mud
    else:
        return RegularizedFrictionStep(vel, -vs, -mus, vs, mus)


