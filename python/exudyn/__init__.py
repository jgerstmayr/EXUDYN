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
#    mbs.SolveDynamic(exu.SimulationSettings())
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

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#SINCE revision2026 step R2.10 (#2466) there are exactly TWO modules, with the same meaning on
#every platform: exudynCPP is built for the BASELINE instruction set and runs on any 64-bit CPU,
#and exudynCPPfast has no range checks and carries the vector extensions (AVX2). The third module
#exudynCPPnoAVX is gone, together with the sys.exudynCPUhasAVX2 switch that selected it: the
#default module IS the safe one now, so nothing has to be detected in order to import exudyn.
#
#The AVX2 check below therefore decides ONE thing only - whether a user's sys.exudynFast request
#can be honoured. A wrong answer costs speed, never a crash, which is why it may be this simple.
#It replaces a read of numpy.core._multiarray_umath.__cpu_features__, which no longer exists in
#numpy >= 2.0 (where the old code simply ASSUMED AVX2) and was skipped altogether on Linux.
def __CpuHasAVX2():
    """True if this CPU *and* the operating system support AVX2; False whenever that cannot be
    established, so that the safe module is used."""
    try:
        if sys.platform == 'win32':
            import ctypes
            #IsProcessorFeaturePresent also covers the OS XSAVE/YMM state, which CPUID alone
            #does not: a CPU may have AVX2 while the OS does not preserve the YMM registers
            PF_AVX2_INSTRUCTIONS_AVAILABLE = 40
            return bool(ctypes.windll.kernel32.IsProcessorFeaturePresent(
                PF_AVX2_INSTRUCTIONS_AVAILABLE))
        if sys.platform.startswith('linux'):
            with open('/proc/cpuinfo') as cpuInfoFile:
                for line in cpuInfoFile:
                    if line.startswith('flags'):
                        return 'avx2' in line.split()
            return False
        if sys.platform == 'darwin': #Apple silicon has no AVX at all; Intel Macs may
            import subprocess
            return subprocess.run(['sysctl', '-n', 'hw.optional.avx2_0'],
                                  capture_output=True, text=True).stdout.strip() == '1'
    except Exception:
        pass
    return False #unknown platform or a failed check: use the module that always works

try:
    #for regular loading in installed python package
    if __useExudynFast:
        if not __CpuHasAVX2():
            __useExudynFast = False
            print('exudyn fast version needs AVX2, which this CPU does not report; '
                  'using the regular version')
        else:
            try:
                from .exudynCPPfast import *
                print('Imported exudyn fast version without range checks')
            except:
                __useExudynFast = False
                print('Import of exudyn fast version failed; falling back to regular version')

    if not __useExudynFast:
        from .exudynCPP import *

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

#use exudyn.demos.Demo1() from 1.9.137 onwards!
try:
    from . import demos
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    pass
    #import exudyn.demos as demos

try:
    from .mainSystemExtensions import JointPreCheckCalcBodyMarkers #import just some function, will assign MainSystem patches
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from mainSystemExtensions import JointPreCheckCalcBodyMarkers


__version__ = config.Version() #add __version__ to exudyn module ...


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
    RequireVersion("1.10.0")

    """
    vExudyn = config.Version().split('.')
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
        raise RuntimeError("EXUDYN version "+requiredVersionString+" required, but only " + config.Version() +
                           " available!\nYou can install the latest development version with:\npip install -U exudyn --pre\n\n")
    



