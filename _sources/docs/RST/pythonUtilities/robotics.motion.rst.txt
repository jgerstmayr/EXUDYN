
.. _sec-module-robotics-motion:

Module: robotics.motion
-----------------------

functionality for motion including generation of trajectories with acceleration profiles,
path planning and motion

- Author:    Johannes Gerstmayr 
- Date:      2022-02-16 


.. _sec-module-robotics-motion-class-profileconstantacceleration:

CLASS ProfileConstantAcceleration (in module robotics.motion)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to create a constant acceleration (optimal) PTP trajectory; trajectory ignores global max. velocities and accelerations

- | \ *input*\ :
  | \ ``finalCoordinates``\ : list or numpy array with final coordinates for profile
  | \ ``duration``\ : duration (time) for profile
- | \ *output*\ :
  | returns profile object, which is then used to compute interpolated trajectory


.. _sec-motion-profileconstantacceleration---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L144>`__\ (\ ``self``\ , \ ``finalCoordinates``\ , \ ``duration``\ )

- | \ *classFunction*\ :
  | initialize ProfileConstantAcceleration with vector of final coordinates and duration (time span)

----

.. _sec-motion-profileconstantacceleration-getbasicprofile:

Class function: GetBasicProfile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetBasicProfile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L149>`__\ (\ ``self``\ , \ ``initialTime``\ , \ ``initialCoordinates``\ , \ ``globalMaxVelocities``\ , \ ``globalMaxAccelerations``\ )

- | \ *classFunction*\ :
  | return a class representing profile which is used in Trajectory

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)


.. _sec-module-robotics-motion-class-profilelinearaccelerationslist:

CLASS ProfileLinearAccelerationsList (in module robotics.motion)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to create a linear acceleration PTP profile, using a list of accelerations to define the profile; the (joint) coordinates and velocities are computed relative to values of previous profiles; ignores global max. accelerations and velocities of Trajectory

- | \ *input*\ :
  | accelerationList: list of tuples (relativeTime, accelerationVector) in which relativeTime is the time relative to the start of the profile (first time must be zero!) and accelerationVector is the list of accelerations of this time point, which is then linearly interpolated
- | \ *output*\ :
  | returns profile object, which is then used to compute interpolated trajectory in class Trajectory
- | \ *example*\ :

.. code-block:: python

  profile = ProfileLinearAccelerationsList([(0,[0.,1.,2]), (0,[1.,1.,-2])])



.. _sec-motion-profilelinearaccelerationslist---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L170>`__\ (\ ``self``\ , \ ``accelerationList``\ )

- | \ *classFunction*\ :
  | initialize ProfileLinearAccelerationsList with a list of tuples containing time and acceleration vector

----

.. _sec-motion-profilelinearaccelerationslist-getbasicprofile:

Class function: GetBasicProfile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetBasicProfile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L180>`__\ (\ ``self``\ , \ ``initialTime``\ , \ ``initialCoordinates``\ , \ ``globalMaxVelocities``\ , \ ``globalMaxAccelerations``\ )

- | \ *classFunction*\ :
  | return a class representing profile which is used in Trajectory


.. _sec-module-robotics-motion-class-profileptp:

CLASS ProfilePTP (in module robotics.motion)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to create a synchronous motion PTP trajectory, using max. accelerations and max velocities; duration automatically computed

- | \ *input*\ :
  | \ ``finalCoordinates``\ : list or numpy array with final coordinates for profile
  | \ ``maxVelocities``\ : list or numpy array with maximum velocities; may be empty list []; used if smaller than globalMaxVelocities
  | \ ``maxAccelerations``\ : list or numpy array with maximum accelerations; may be empty list []; used if smaller than globalMaxAccelerations
- | \ *output*\ :
  | returns profile object, which is then used to compute interpolated trajectory


.. _sec-motion-profileptp---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L197>`__\ (\ ``self``\ , \ ``finalCoordinates``\ , \ ``syncAccTimes = True``\ , \ ``maxVelocities = []``\ , \ ``maxAccelerations = []``\ )

- | \ *classFunction*\ :
  | initialize ProfilePTP with final coordinates of motion, optionally max. velocities and accelerations just for this profile (overrides global settings)

----

.. _sec-motion-profileptp-getbasicprofile:

Class function: GetBasicProfile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetBasicProfile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L210>`__\ (\ ``self``\ , \ ``initialTime``\ , \ ``initialCoordinates``\ , \ ``globalMaxVelocities``\ , \ ``globalMaxAccelerations``\ )

- | \ *classFunction*\ :
  | return a class representing profile which is used in Trajectory

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `serialRobotTSD.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTSD.py>`_\  (Ex)


.. _sec-module-robotics-motion-class-trajectory:

CLASS Trajectory (in module robotics.motion)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define (PTP) trajectories for robots and multibody systems; trajectories are defined for a set of coordinates (e.g. joint angles or other coordinates which need to be interpolated over time)

- | \ *example*\ :

.. code-block:: python

  #create simple trajectory for two joint coordinates:
  traj = Trajectory(initialCoordinates=[1,1], initialTime=1)
  #add optimal trajectory with max. accelerations:
  traj.Add(ProfileConstantAcceleration([2.,3.],2.))
  traj.Add(ProfileConstantAcceleration([3.,-1.],2.))
  #add profile with limited velocities and accelerations:
  traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))
  #now evaluate trajectory at certain time point (this could be now applied in a user function)
  [s,v,a] = traj.Evaluate(t=0.5)



.. _sec-motion-trajectory---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L292>`__\ (\ ``self``\ , \ ``initialCoordinates``\ , \ ``initialTime = 0``\ , \ ``maxVelocities = []``\ , \ ``maxAccelerations = []``\ )

- | \ *classFunction*\ :
  | initialize robot link with parameters, being self-explaining
- | \ *input*\ :
  | \ ``initialTime``\ : initial time for initial coordinates
  | \ ``initialCoordinates``\ : initial coordinates for profile
  | \ ``maxVelocities``\ : list or numpy array to describe global maximum velocities per coordinate
  | \ ``maxAccelerations``\ : list or numpy array to describe global maximum accelerations per coordinate

----

.. _sec-motion-trajectory-getfinalcoordinates:

Class function: GetFinalCoordinates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetFinalCoordinates <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L307>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | returns the coordinates at the end of the (currently) Final profile

----

.. _sec-motion-trajectory-add:

Class function: Add
^^^^^^^^^^^^^^^^^^^
`Add <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L315>`__\ (\ ``self``\ , \ ``profile``\ )

- | \ *classFunction*\ :
  | add successively profiles, using MotionProfile class

----

.. _sec-motion-trajectory-gettimes:

Class function: GetTimes
^^^^^^^^^^^^^^^^^^^^^^^^
`GetTimes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L322>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return vector of times of start/end of profiles

----

.. _sec-motion-trajectory-initialize:

Class function: Initialize
^^^^^^^^^^^^^^^^^^^^^^^^^^
`Initialize <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L329>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | initialize some parameters for faster evaluation

----

.. _sec-motion-trajectory-evaluate:

Class function: Evaluate
^^^^^^^^^^^^^^^^^^^^^^^^
`Evaluate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L335>`__\ (\ ``self``\ , \ ``t``\ )

- | \ *classFunction*\ :
  | return interpolation of trajectory for coordinates, velocities and accelerations at given time
- | \ *output*\ :
  | [s, v, a] as numpy arrays representing coordinates, velocities and accelerations

----

.. _sec-motion-trajectory-evaluatecoordinate:

Class function: EvaluateCoordinate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`EvaluateCoordinate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L356>`__\ (\ ``self``\ , \ ``t``\ , \ ``coordinate``\ )

- | \ *classFunction*\ :
  | return interpolation of trajectory for coordinate, including velocity and acceleration coordinate at given time
- | \ *output*\ :
  | [s, v, a] being scalar position, velocity and acceleration
- | \ *notes*\ :
  | faster for single coordinate than Evaluate(...)

----

.. _sec-motion-trajectory---iter--:

Class function: __iter__
^^^^^^^^^^^^^^^^^^^^^^^^
`__iter__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L372>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | iterator allows to use for x in trajectory: ... constructs

----

.. _sec-motion-trajectory---getitem--:

Class function: __getitem__
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`__getitem__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L376>`__\ (\ ``self``\ , \ ``key``\ )

- | \ *classFunction*\ :
  | access to profiles via operator [], allowing trajectory[0], etc.

----

.. _sec-motion-trajectory---len--:

Class function: __len__
^^^^^^^^^^^^^^^^^^^^^^^
`__len__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L380>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | allow using len(trajectory)

----

.. _sec-motion-trajectory---repr--:

Class function: __repr__
^^^^^^^^^^^^^^^^^^^^^^^^
`__repr__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/motion.py\#L385>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | representation of Trajectory is given a list of profiles, allowing easy inspection of data

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)

