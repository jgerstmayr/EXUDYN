
.. _sec-module-robotics-future:

Module: robotics.future
-----------------------

The future module contains functionality which is currently under development
and will be moved in other robotics libraries in future

- Date:      2023-03-27 


.. _sec-future-makecorkerobot:

Function: MakeCorkeRobot
^^^^^^^^^^^^^^^^^^^^^^^^
`MakeCorkeRobot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/future.py\#L41>`__\ (\ ``robotDic``\ )

- | \ *function description*\ :
  | makeCorkeRobot, creates robot using the peter corke toolbox using standard (stdDH) or modified (modKKDH) Denavid Hartenberg parameters
- | \ *input*\ :
  | \ ``robotDic``\ : robot dictionary by exudyn robotic models
  | \ ``dhpara``\ : stDH for standard DH parameter, modKKDH for modified DH parameter
- | \ *output*\ :
  | serial robot object by corke
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | DH Parameter Information:
  | stdH = [theta, d, a, alpha] with Rz(theta) \* Tz(d) \* Tx(a) \* Rx(alpha)
  | modDH = [alpha, dx, theta, rz] with
  | used by Corke and Lynch: Rx(alpha) \* Tx(a) \* Rz(theta) \* Tz(d)
  | used by Khali:           Rx(alpha) \* Tx(d) \* Rz(theta) \* Tz(r)
  | Important note:  d(khali)=a(corke)  and r(khali)=d(corke)



----


.. _sec-future-computeik3r:

Function: ComputeIK3R
^^^^^^^^^^^^^^^^^^^^^
`ComputeIK3R <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/future.py\#L85>`__\ (\ ``robotDic``\ , \ ``HT``\ )

- | \ *function description*\ :
  | calculates the analytical inverse kinematics for 3R elbow type serial robot manipulator
- | \ *input*\ :
  | \ ``robotDic``\ : robot dictionary
  | \ ``HT``\ : desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
- | \ *output*\ :
  | solutions, list of lists with posible joint angles [q1,q2,q3] (in radiant)
  | to achive the desired position (4 posible solutions,schoulder left/right, ellbow up/down ) in following order: left/down, left/up, right/up, right/down
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | only applicable for standard Denavit-Hartenberg parameters
- | \ *status*\ :
  | testet with various configurations and joint angels



----


.. _sec-future-computeikpuma560:

Function: ComputeIKPuma560
^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeIKPuma560 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/future.py\#L157>`__\ (\ ``robotDic``\ , \ ``HT``\ )

- | \ *function description*\ :
  | calculates the analytical inverse kinematics for Puma560 serial 6R robotDic manipulator
- | \ *input*\ :
  | \ ``robotDic``\ : robotDictionary
  | \ ``HT``\ : desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
- | \ *output*\ :
  | qSolutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
  | to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
  | left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | Usage for different manipulators with sperical wrist posible, only applicable for standard Denavit-Hartenberg parameters
- | \ *status*\ :
  | tested (compared with robotDiccs, Vision and Control book of P. Corke



----


.. _sec-future-computeikur:

Function: ComputeIKUR
^^^^^^^^^^^^^^^^^^^^^
`ComputeIKUR <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/future.py\#L282>`__\ (\ ``robotDic``\ , \ ``HTdes``\ )

- | \ *function description*\ :
  | calculates the analytical inverse kinematics for UR type serial 6R robot manipulator without sperical wrist
- | \ *input*\ :
  | \ ``robotDic``\ : robot dictionary
  | \ ``HT``\ : desired position and orientation for the end effector as 4x4 homogeneous transformation matrix as list of lists or np.array
- | \ *output*\ :
  | solutions, list of lists with posible joint angles [q1,q2,q3,q4,q5,q6] (in radiant)
  | to achive the desired position and orientation (8 posible solutions,schoulder left/right, ellbow up/down, wrist flipped/notflipped (rotated by pi) )
  | [left/down/notflipped, left/down/flipped, left/up/notflipped, left/up/flipped, right/up/notflipped, right/up/flipped, right/down/notflipped, right/down/flipped]
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | Usage for different manipulators without sperical wrist posible UR3,UR5,UR10, only applicable for standard Denavit-Hartenberg parameters
- | \ *status*\ :
  | under development, works for most configurations, singularities not checked -> ZeroConfiguration not working

