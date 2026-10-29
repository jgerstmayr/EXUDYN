#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectGenericDAE
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#an oscillator x_tt + 4 x on a NodeGenericODE2 and a first order filter y_t = -y + x on a NodeGenericODE1, coupled by
#the algebraic equation x + y = 1, whose multiplier is the coordinate of a NodeGenericAE
nODE2 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0.5], initialCoordinates_t=[0],
                                    numberOfODE2Coordinates=1))
nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0], initialCoordinates=[0.5], numberOfODE1Coordinates=1))
nAE = mbs.AddNode(NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[nODE2], nodeNumbersODE1=[nODE1], nodeNumbersAE=[nAE],
                               massMatrix=[[1]], stiffnessMatrix=[[4]],
                               systemMatrix=[[-1]], couplingMatrixODE2=[[1]],
                               constraintMatrixODE2=[[1]], constraintMatrixODE1=[[1]], constraintVector=[1]))
mbs.Assemble()
mbs.SolveDynamic()

#check result at default integration time
exu.sys['testResult'] = mbs.systemData.GetODE2Coordinates()[0]

exu.Print("example for ObjectGenericDAE completed, test result =", exu.sys['testResult'])

