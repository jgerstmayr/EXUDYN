#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Main Python library file for import of C++ module
#
# Author:   Johannes Gerstmayr
# Date:     2020-08-14
# Update:   2022-12-26
#
# Notes:    see https://github.com/jgerstmayr/EXUDYN for first steps
#           see theDoc.pdf for instructions, tutorials, etc.: https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf
# Example (without visualization):
#    import exudyn as exu
#    from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
#    SC = exu.SystemContainer()
#    mbs = SC.AddSystem()
#    #add a new system to work with
#    nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
#    mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
#    mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
#    mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))
#    mbs.Assemble() #assemble system and solve
#    exu.SolveDynamic(mbs, exu.SimulationSettings())
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#  Use the following workaround to define the 'fast' track, avoiding range checks in exudyn (speedup may be 30% and more);
#  to activate the __FAST_EXUDYN_LINALG compiled version, use the following lines (must be done befor first import of exudyn):
#import sys
#sys.exudynFast = True
#import exudyn #now exudyn loads with fast mode

import sys
__useExudynFast = hasattr(sys, 'exudynFast')
if __useExudynFast:
    __useExudynFast = sys.exudynFast #could also be False!

__cpuHasAVX2 = hasattr(sys, 'exudynCPUhasAVX2')
if __cpuHasAVX2:
    __cpuHasAVX2 = sys.exudynCPUhasAVX2 #could also be False!

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#use numpy.core to find if AVX+AVX2 is available ...
try:
    if sys.platform != 'darwin' and sys.platform != 'linux': #MacOS does not support AVX2; no AVX2 for linux right now; therefore there are not non-AVX modules compiled ...
        import numpy
        thisCpuHasAVX2 = True #assume this for now!
        if numpy.__version__ <= '2.0': #otherwise _multiarray_umath not available (moved to _core and raises warnings)
            from numpy.core._multiarray_umath import __cpu_features__
            if (('AVX' in __cpu_features__) and ('AVX2' in __cpu_features__) and
                (__cpu_features__['AVX'] == True) and (__cpu_features__['AVX2'] == True)):
                if not __cpuHasAVX2 and hasattr(sys, 'exudynCPUhasAVX2'):
                    print('WARNING: user deactivated AVX2 support, but support detected on current CPU')
                else:
                    __cpuHasAVX2 = True
            elif __cpuHasAVX2:
                print('WARNING: user activated AVX2 support, but no AVX2 support has been detected on current CPU; may crash')
        else: #we do not know what the user has, but assume AVX!
            if not hasattr(sys, 'exudynCPUhasAVX2'):
                __cpuHasAVX2 = True #standard case!
    else:
        __cpuHasAVX2 = True #for MacOS and Linux, this means that there is no exudynCPPnoAVX version!
except:
    print('Warning: during import of exudyn, it was detected that either numpy or the numpy.core module "_multiarray_umath" is missing')

try:
    #for regular loading in installed python package
    if __useExudynFast and __cpuHasAVX2:
        try:
            from .exudynCPPfast import *
            print('Imported exudyn fast version without range checks')
        except:
            __useExudynFast = False
            print('Import of exudyn fast version failed; falling back to regular version')
    else:
        __useExudynFast = False #in case __useExudynFast=True but no AVX

    if not __useExudynFast:
        if __cpuHasAVX2:
            try:
                from .exudynCPP import *
            except:
                raise ImportError('Warning: Import of exudyn C++ module (with AVX2) failed; check your installation or try to import without AVX by settings sys.exudynCPUhasAVX2=False')
        else:
            try:
                from .exudynCPPnoAVX import *
            except:
                raise ImportError('Import of exudyn C++ module (without AVX2) failed; non-AVX2 versions are only available in release versions (without .dev1 appendix); check your installation, Python version, conda environment and site-packages for exudyn; try re-installation')

except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders); no exudynFast! :
    try:
        from exudynCPP import *
    except:
        raise ImportError('Import of exudyn C++ module failed; check 32/64 bits versions, restart your iPython console or try to uninstall and install exudyn')

#import very useful solver functionality into exudyn module (==> available as exu.SolveStatic, etc.)
try:
    from .solver import SolveStatic, SolveDynamic, SolverSuccess, ComputeLinearizedSystem, ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from solver import SolveStatic, SolveDynamic, SolverSuccess, ComputeLinearizedSystem, ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues

try:
    from .demos import Demo1, Demo2
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from demos import Demo1, Demo2

try:
    from .mainSystemExtensions import JointPreCheckCalc #import just some function, will assign MainSystem patches
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from mainSystemExtensions import JointPreCheckCalc


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
        #print("EXUDYN version "+requiredVersionString+" required, but only " + GetVersionString() + " available")
        raise RuntimeError("EXUDYN version "+requiredVersionString+" required, but only " + GetVersionString() +
                           " available!\nYou can install the latest development version with:\npip install -U exudyn --pre\n\n")
    

#do not import itemInterface here, as it would go into exu. scope
#from .itemInterface import *


