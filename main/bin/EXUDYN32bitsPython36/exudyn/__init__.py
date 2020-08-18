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


#do not import itemInterface here, as it would go into exu. scope
#from .itemInterface import *


