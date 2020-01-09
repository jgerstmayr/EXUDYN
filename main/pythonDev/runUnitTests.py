#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the driver file for running the modelUnitTests (not the examples)
#
# Author:   Johannes Gerstmayr
# Date:     2019-07-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys
sys.path.append('../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
sys.path.append('TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from modelUnitTests import RunAllModelUnitTests, TestInterface

SC = exu.SystemContainer()
mbs = SC.AddSystem()

testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
RunAllModelUnitTests(mbs, testInterface)


SC.Reset()
