#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test speed of AddNode, AddObject, ...
#
# Author:   Johannes Gerstmayr
# Date:     2023-04-08
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import numpy as np
import time

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#time.sleep(3)
exu.Print('Exudyn version=',exu.config.Version(True))


n0 = mbs.AddNode(NodePoint())
#n0 = mbs.AddNode(NodeGenericData(initialCoordinates=[0,1,2], numberOfDataCoordinates=3)) #no significant difference!

total = 0
n=10000*100
ts = -time.time()
np = NodePoint(referenceCoordinates=[0,0,0])
for i in range(n):
    #np = NodePoint(name='abcdefghij'*10+str(i), referenceCoordinates=[0,0,0])
    mbs.AddNode(np)

ts += time.time()
total += ts
exu.Print('time spent for',n,'nodes: ', ts)

# time.sleep(5)
# exu.Print('reset mbs')
# mbs.Reset()
# time.sleep(5)

# import sys
# sys.exit()

ts = -time.time()
og = ObjectGround()
for i in range(n):
    # og = ObjectGround(name='12345678901234567890'+str(i))
    mbs.AddObject(og)

ts += time.time()
total += ts
exu.Print('time spent for',n,'objects: ', ts)

ts = -time.time()
marker=MarkerBodyPosition(bodyNumber=0)
for i in range(n):
    mbs.AddMarker(marker)

ts += time.time()
total += ts
exu.Print('time spent for',n,'markers: ', ts)


mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=0))
load = Force(markerNumber=mGround)
ts = -time.time()
for i in range(n):
    mbs.AddLoad(load)

ts += time.time()
total += ts
exu.Print('time spent for',n,'loads: ', ts)
exu.Print('total time : ', total)

ts = -time.time()
for i in range(n):
    mbs.GetObject(i)
ts += time.time()
total += ts
exu.Print('time spent for',n,' GetObject: ', ts)


# #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# before IMPROVEMENTS (without const py::dict& and const py::object& ):
# #results (running from console):
# #n=1.000.000; requires 1032MB on Windows! 
# #current memory footprint for 1 NodePoint is approx. 238 bytes (requires 24 bytes C-data + 24 bytes V-data and M-data: 1 std::string (~30 bytes) + 2 pointers at 8 bytes each)
# #current memory footprint for 1 ObjectGround is approx. 375 bytes (additional memory for BodyGraphicsData and std::function)
# #WINDOWS:
# (venvP39) C:\DATA\cpp\EXUDYN_git\main\pythonDev\TestModels>python interfaceTest.py
# Exudyn version= 1.6.43.dev1; Python3.9.15; Windows AVX2 FLOAT64
# time spent for 1000000 nodes:  6.910979747772217
# time spent for 1000000 objects:  8.187081098556519
# time spent for 1000000 markers:  5.106989622116089
# time spent for 1000000 loads:  5.9349682331085205

# (venvP39) C:\DATA\cpp\EXUDYN_git\main\pythonDev\TestModels>python interfaceTest.py
# Exudyn version= 1.6.43.dev1; Python3.9.15; Windows AVX2 FLOAT64
# time spent for 1000000 nodes:  11.35836911201477
# time spent for 1000000 objects:  13.258954763412476
# time spent for 1000000 markers:  10.056904077529907
# time spent for 1000000 loads:  10.958497524261475

# (venvP39) C:\DATA\cpp\EXUDYN_git\main\pythonDev\TestModels>python interfaceTest.py
# Exudyn version= 1.6.43.dev1; Python3.9.15; Windows AVX2 FLOAT64
# time spent for 1000000 nodes:  7.258582830429077
# time spent for 1000000 objects:  7.782750844955444
# time spent for 1000000 markers:  5.027644395828247
# time spent for 1000000 loads:  6.254368543624878

# (venvP39) C:\DATA\cpp\EXUDYN_git\main\pythonDev\TestModels>python interfaceTest.py
# Exudyn version= 1.6.43.dev1; Python3.9.15; Windows AVX2 FLOAT64
# time spent for 1000000 nodes:  6.892444133758545
# time spent for 1000000 objects:  7.913021087646484
# time spent for 1000000 markers:  5.233001470565796
# time spent for 1000000 loads:  6.361907720565796
# #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# #LINUX:
# Exudyn version= 1.6.43.dev1; Python3.9.12; Linux FLOAT64
# time spent for 1000000 nodes:  5.0591747760772705
# time spent for 1000000 objects:  6.125314474105835
# time spent for 1000000 markers:  4.637614727020264
# time spent for 1000000 loads:  4.479295015335083
# total time :  20.301398992538452
# (venvP39) johannes@lt73-c850:/mnt/c/DATA/cpp/EXUDYN_git/main/pythonDev/TestModels$ python interfaceTest.py
# Exudyn version= 1.6.43.dev1; Python3.9.12; Linux FLOAT64
# time spent for 1000000 nodes:  5.007477760314941
# time spent for 1000000 objects:  5.766639947891235
# time spent for 1000000 markers:  3.936932325363159
# time spent for 1000000 loads:  4.306653738021851
# total time :  19.017703771591187
# (venvP39) johannes@lt73-c850:/mnt/c/DATA/cpp/EXUDYN_git/main/pythonDev/TestModels$ python interfaceTest.py
# Exudyn version= 1.6.43.dev1; Python3.9.12; Linux FLOAT64
# time spent for 1000000 nodes:  5.565306663513184
# time spent for 1000000 objects:  5.983320951461792
# time spent for 1000000 markers:  3.9619572162628174
# time spent for 1000000 loads:  4.43130087852478
# total time :  19.941885709762573

# #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# after IMPROVEMENTS (with const py::dict& and const py::object& ):
# #results (running from console):
# #LINUX:
# Exudyn version= 1.6.45.dev1; Python3.9.12; Linux FLOAT64
# time spent for 1000000 nodes:  5.166866302490234
# time spent for 1000000 objects:  5.891016006469727
# time spent for 1000000 markers:  3.9866602420806885
# time spent for 1000000 loads:  4.5102760791778564
# total time :  19.554818630218506


