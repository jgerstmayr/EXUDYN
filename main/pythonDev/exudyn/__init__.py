#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This the EXUDYN module initialization file
#
# Author:   Johannes Gerstmayr
# Date:     2020-08-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#This is a workaround to let users define the 'fast' track, 
#  avoiding range checks in exudyn (speedup may be 30% and more)
#  to activate the __FAST_EXUDYN_LINALG compiled version, use the 
#  following lines (must be done befor first import of exudyn);
#  Note that this is a hack and may be changed in future; it is only available for certain exudyn versions:
#import sys
#sys.exudynFast = True
#import exudyn

import sys
useExudynFast = hasattr(sys, 'exudynFast')
if useExudynFast:
    useExudynFast = sys.exudynFast #could also be False!
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

try:
    #for regular loading in installed python package
    if useExudynFast:
        print('Importing exudyn fast version without range checks...')
        try:
            from .exudynCPPfast import *
        except:
            useExudynFast = False
            print('Import of exudyn fast version failed; falling back to regular version')
            
    if not useExudynFast:
        from .exudynCPP import *
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders); no exudynFast! :
    try:
        from exudynCPP import *
    except:
        raise ImportError('Import of exudyn C++ module failed; check 32/64 bits versions, restart your iPython console or try to uninstall and install exudyn')

#import very useful solver functionality into exudyn module (==> available as exu.SolveStatic, etc.)
try:
    from .solver import SolveStatic, SolveDynamic, ComputeODE2Eigenvalues
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from solver import SolveStatic, SolveDynamic, ComputeODE2Eigenvalues

# remove SolutionViewer as it makes problems if no tkinter or matplotlib installed
# try:
    # from .interactive import SolutionViewer
# except:
    # try:
        # #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
        # from interactive import SolutionViewer
    # except:
        # print("SolutionViewer not loaded (missing tkinter or matplotlib?)")
        # pass

__version__ = GetVersionString() #add __version__ to exudyn module ...


#add a functionality to check the current version
def RequireVersion(requiredVersionString):
    """
    Parameters
    ----------
    requiredVersionString : string
        Checks if the installed version is according to the required version.
        Major, micro and minor version must agree the required level.
    Returns
    -------
    None. But will raise RuntimeError, if required version is not met.

    Example
    ----------
    RequireVersion("1.0.26")

    """
    vExudyn=GetVersionString().split('.')
    vRequired = requiredVersionString.split('.')
    isOk = True
    if int(vExudyn[0]) < int(vRequired[0]):
        isOk = False
    elif int(vExudyn[0]) == int(vRequired[0]): #only for equal major versions
        if int(vExudyn[1]) < int(vRequired[1]): #check minor version
            isOk = False
        elif int(vExudyn[1]) == int(vRequired[1]): #only for equal minor versions
            if int(vExudyn[2]) < int(vRequired[2]): #check micro version
                isOk = False
    if not isOk:
        print("EXUDYN version "+requiredVersionString+" required, but only " + exu.GetVersionString() + " available")
        #raise RuntimeError("EXUDYN version "+requiredVersionString+" required, but only " + exu.GetVersionString() + "available")
    

#do not import itemInterface here, as it would go into exu. scope
#from .itemInterface import *


