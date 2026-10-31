#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  exudyn.symbolics.kinematics.HT and Twist, the homogeneous transformation and the twist of a frame for
#           symbolic parts (#3174): the products, the inverse and the relative frame of named symbolic matrices and
#           vectors against exudyn.HT, also with an exudyn.HT as constant on either side; PointVelocity, LocalTwist
#           and RelativeTwist against exudyn.cUtilities, which they call for exudyn.HT and exudyn.Twist; RelativeTwist
#           against central differences of HT0.Relative(HT1) moved by the twists, and esym.Jacobian of the relative
#           translation by the rotation vector of frame 0 against central differences.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.symbolics import kinematics as kin
import numpy as np

esym = exu.symbolic
testIsActive = exu.sys.get('testIsActive', False)

def SymbolicHT(H, name):
    """H as kinematics.HT of the named leaves name_rotation and name_translation, as a recording gets it"""
    return kin.HT(esym.Matrix(name + '_rotation', H.rotation.tolist()), esym.Vector(name + '_translation', H.translation.tolist()))

def SymbolicTwist(twist, name):
    return kin.Twist(esym.Vector(name + '_velocity', twist.velocity.tolist()),
                     esym.Vector(name + '_angularVelocity', twist.angularVelocity.tolist()))

def Value(x):
    return np.array(x.Evaluate())

def DifferenceHT(symbolicHT, H):
    return np.abs(Value(symbolicHT.rotation) - H.rotation).max() + np.abs(Value(symbolicHT.translation) - H.translation).max()

def DifferenceTwist(symbolicTwist, twist):
    return (np.abs(Value(symbolicTwist.velocity) - twist.velocity).max()
            + np.abs(Value(symbolicTwist.angularVelocity) - twist.angularVelocity).max())

rng = np.random.default_rng(1)
largest = 0.
for i in range(10):
    H0 = exu.HT(rotationVector=rng.standard_normal(3), translation=rng.standard_normal(3))
    H1 = exu.HT(rotationVector=rng.standard_normal(3), translation=rng.standard_normal(3))
    twist0 = exu.Twist(rng.standard_normal(3), rng.standard_normal(3))
    twist1 = exu.Twist(rng.standard_normal(3), rng.standard_normal(3))
    point = rng.standard_normal(3)
    (S0, S1) = (SymbolicHT(H0, 'HT0'), SymbolicHT(H1, 'HT1'))
    (T0, T1) = (SymbolicTwist(twist0, 'twist0'), SymbolicTwist(twist1, 'twist1'))
    differences = [DifferenceHT(S0*S1, H0*H1), DifferenceHT(S0@S1, H0*H1),
                   DifferenceHT(H0*S1, H0*H1), DifferenceHT(S0*H1, H0*H1), DifferenceHT(H0@S1, H0*H1), #exu.HT as constant
                   DifferenceHT(S0.Inverse(), H0.Inverse()), DifferenceHT(S0.Relative(S1), H0.Relative(H1)),
                   DifferenceHT(S0.Relative(H1), H0.Relative(H1)),
                   np.abs(Value(S0*point) - H0*point).max(),
                   np.abs(Value(kin.PointVelocity(S0, T0, point)) - kin.PointVelocity(H0, twist0, point)).max(),
                   DifferenceTwist(kin.LocalTwist(S0, T0), kin.LocalTwist(H0, twist0)),
                   DifferenceTwist(kin.RelativeTwist(S0, T0, S1, T1), kin.RelativeTwist(H0, twist0, H1, twist1))]
    largest = max(largest, max(differences))
exu.Print('largest difference of the symbolic HT and Twist to exudyn.HT and exudyn.cUtilities:', largest)

#RelativeTwist is the time derivative of HT0.Relative(HT1) when both frames move with their twists
def Moved(H, twist, dt):
    return exu.HT(rotation=exu.HT(rotationVector=twist.angularVelocity*dt).rotation @ H.rotation,
                  translation=H.translation + twist.velocity*dt)
h = 1e-6
relative = kin.RelativeTwist(H0, twist0, H1, twist1)
(Hplus, Hminus) = (Moved(H0, twist0, h).Relative(Moved(H1, twist1, h)), Moved(H0, twist0, -h).Relative(Moved(H1, twist1, -h)))
velocityError = np.abs((Hplus.translation - Hminus.translation)/(2*h) - relative.velocity).max()
W = (Hplus.rotation - Hminus.rotation)/(2*h) @ H0.Relative(H1).rotation.T #the skew matrix of the relative angular velocity
angularVelocityError = np.abs(np.array([W[2, 1], W[0, 2], W[1, 0]]) - relative.angularVelocity).max()
exu.Print('RelativeTwist against central differences:', velocityError, angularVelocityError)

#esym.Jacobian through the mapped methods: the relative translation by the rotation vector of frame 0
rotationVector = esym.Vector('rotationVector', [0.3, -0.2, 0.5])
frame0 = kin.HT(kin.RotationVector2RotationMatrix(rotationVector), esym.Vector([0.1, 0.2, 0.3]))
relativeTranslation = frame0.Relative(S1).translation
jacobian = Value(esym.Jacobian(relativeTranslation, rotationVector))
def RelativeTranslation(v):
    return exu.HT(rotationVector=v, translation=[0.1, 0.2, 0.3]).Relative(H1).translation
v0 = np.array([0.3, -0.2, 0.5])
differences = np.array([(RelativeTranslation(v0 + h*e) - RelativeTranslation(v0 - h*e))/(2*h) for e in np.eye(3)]).T
jacobianError = np.abs(jacobian - differences).max()
exu.Print('esym.Jacobian of the relative translation against central differences:', jacobianError)

#the classes copy each other's parts; an exudyn.HT refuses an operand it cannot compute with
copies = (np.abs(kin.HT(H0).rotation - H0.rotation).max() == 0 and np.abs(kin.Twist(twist0).velocity - twist0.velocity).max() == 0
          and exu.Twist(twist0) == twist0 and kin.HT().translation.tolist() == [0, 0, 0])
try:
    H0*'x'
    refused = False
except TypeError:
    refused = True

u = int(largest > 1e-14) + int(velocityError > 1e-8) + int(angularVelocityError > 1e-8) + int(jacobianError > 1e-8)
u += int(not copies) + int(not refused)
exu.Print('solution of symbolicHTTest=', u)
exu.sys['testResult'] = u
