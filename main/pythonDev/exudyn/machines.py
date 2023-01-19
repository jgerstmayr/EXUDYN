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

import numpy as np
from math import tan, atan
#from math import sin, cos, asin, acos, pi, exp, log, tan, atan



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
        #print(d)
    print('it=',it)
    if it == maxIt and warn:
        raise ValueError('ERROR: InvInvolute(...) did not converge within 10 steps!')

    return x0

#%%++++++++++++++++++++++++
#testing of involute and approximated involute
if __name__ == '__main__':
    if False:
        for i in range(40):
            x = 2**(-i/2)
            print('x=', x,'Inv=', Involute(x), ', Approx Inv=', ApproxInvolute(x), ', diff=', ApproxInvolute(x)-Involute(x), 'err term=', (62/2835)*x**9)  
            #==> error term < diff for x<0.02

    #test InvInvolute:
    #iteration number largest for x \approx 1
    for i in range(80): #80 goes up to x=2e-46, invinv=9e-16
        x = 100*2**(-2*i) 
        print('x=', x,'InvInv=', InvInvolute(x), ', Involute(InvInv)-x=', Involute(InvInvolute(x))-x)  
        
    for i in range(80): #80 goes up to x=2e-46, invinv=9e-16
        x = -100*2**(-2*i)
        print('x=', x,'InvInv=', InvInvolute(x), ', Involute(InvInv)-x=', Involute(InvInvolute(x))-x)  

