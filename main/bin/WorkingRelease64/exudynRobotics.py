#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  support functions for robotics
#
# Author:   Johannes Gerstmayr
# Date:     2020-04-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys
#constants and fixed structures:
import numpy as np

from itemInterface import *
#import exudyn as exu #do not import! causes troubles with exudynFast, etc.!!
from exudynBasicUtilities import NormL2
from exudynRigidBodyUtilities import *

#compute transformation matrix from standard denavit hartenberg parameters:
def DH2HT(DHparameters):
#    [theta, d, a, alpha] = DHparameters
#    return HTrotateZ(theta) @ HTtranslate([0,0,d]) @ HTtranslate([a,0,0]) @ HTrotateX(alpha)
    #optimized version:
    [theta, d, a, alpha] = DHparameters
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([[ct,-st*ca, st*sa, a*ct],
                     [st, ct*ca,-ct*sa, a*st],
                     [ 0, sa   , ca   , d   ],
                     [ 0, 0    , 0    , 1   ]])

#\mfour{\cos \theta_j &-\sin \theta_j \cos \alpha_j & \sin \theta_j \sin \alpha_j & a_j \cos \theta_j}
#														{\sin \theta_j & \cos \theta_j \cos \alpha_j &-\cos \theta_j \sin \alpha_j & a_j \sin \theta_j}
#														{0             & \sin \alpha_j               & \cos \alpha_j               & d_j }
#														{0 & 0 & 0 & 1}
#Test (compared with Robotcs, Vision and Control book of P. Corke:
#print("std. DH =\n", DH2HT([0.5, 0.1, 0.2, np.pi/2]).round(4))

