#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  This the EXUDYN robotics submodule initialization file
#
# Author:   Johannes Gerstmayr
# Date:     2021-12-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#roboticsCore is the core robotics module;
#everything inside roboticsCore goes into exudyn.robotics !
from .roboticsCore import *

##this makes the submodules available; does not work under Python 3.6:
#import exudyn.robotics.utilities as utilities
#import exudyn.robotics.future as future
#import exudyn.robotics.special as special
#import exudyn.robotics.models as models
#import exudyn.robotics.mobile as mobile


