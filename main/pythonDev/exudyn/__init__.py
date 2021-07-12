#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This the EXUDYN module initialization file
#
# Author:   Johannes Gerstmayr
# Date:     2020-08-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#import the cpp module ==> goes into exu. scope (if imported as: "import exudyn as exu") 
#from .exudynCPP import *

try:
    #for regular loading in installed python package
    from .exudynCPP import *
except:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from exudynCPP import *

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


