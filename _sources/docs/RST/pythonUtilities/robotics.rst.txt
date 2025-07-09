
.. _sec-module-robotics:

Module: robotics
================

A library which includes support functions for robotics;
the library is built on standard Denavit-Hartenberg Parameters and
Homogeneous Transformations (HT) to describe transformations and coordinate systems;
import this library e.g. with import exudyn.robotics as robotics

- Author:    Johannes Gerstmayr 
- Date:      2020-04-14 
- Example:     New robot model uses the class Robot with class RobotLink; the old dictionary structure is defined in the example in ComputeJointHT for the definition of the 'robot' dictionary. 


.. _sec-roboticscore-stddh2ht:

Function: StdDH2HT
^^^^^^^^^^^^^^^^^^
`StdDH2HT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1025>`__\ (\ ``DHparameters``\ )

- | \ *function description*\ :
  | compute homogeneous transformation matrix HT from standard DHparameters=[theta, d, a, alpha]

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)



----


.. _sec-roboticscore-moddhkk2ht:

Function: ModDHKK2HT
^^^^^^^^^^^^^^^^^^^^
`ModDHKK2HT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1047>`__\ (\ ``DHparameters``\ )

- | \ *function description*\ :
  | compute pre- and post- homogeneous transformation matrices from modified Denavit-Hartenberg DHparameters=[alpha, d, theta, r]; returns [HTpre, HTpost]; HTpre is transformation before axis rotation, HTpost includes axis rotation and everything hereafter; modified DH-Parameters according to Khalil and Kleinfinger, 1986

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex)



----


.. _sec-roboticscore-projectangletopmpi:

Function: projectAngleToPMPi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`projectAngleToPMPi <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1060>`__\ (\ ``q0``\ )

- | \ *function description*\ :
  | This function projects an angle in the range \ :math:`[-min_{float}, +max_{float}]`\  fo the range \ :math:`[-\pi, +\pi]`\
- | \ *input*\ :
  | q0: An angle either as scalar, list or array
- | \ *output*\ :
  | qProj: The angle projected into the range \ :math:`[-\pi to \pi]`\
- | \ *author*\ :
  | Peter Manzl


.. _sec-module-robotics-class-vrobotlink:

CLASS VRobotLink (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define visualization of RobotLink


.. _sec-roboticscore-vrobotlink---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L98>`__\ (\ ``self``\ , \ ``jointRadius = 0.06``\ , \ ``jointWidth = 0.12``\ , \ ``linkWidth = 0.1``\ , \ ``showMBSjoint = True``\ , \ ``showCOM = True``\ , \ ``linkColor = [0.4,0.4,0.4,1]``\ , \ ``graphicsData = []``\ )

- | \ *classFunction*\ :
  | initialize robot link with parameters, being self-explaining
- | \ *input*\ :
  | \ ``jointRadius``\ : radius of joint to draw
  | \ ``jointWidth``\ : length or width of joint (depending on type of joint)
  | \ ``showMBSjoint``\ : if False, joint is not drawn
  | \ ``linkWidth``\ : width of link for default drawing
  | \ ``linkColor``\ : color of link for default drawing
  | \ ``showCOM``\ : if True, center of mass is marked with cube
  | \ ``graphicsData``\ : list of GraphicsData to represent link; if list is empty, link graphics will be generated from link geometry data; otherwise, drawing will be taken from graphicsData, and only showMBSjoint and showCOM flags will add additional graphics

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-robotlink:

CLASS RobotLink (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define one link of a robot


.. _sec-roboticscore-robotlink---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L138>`__\ (\ ``self``\ , \ ``mass``\ , \ ``COM``\ , \ ``inertia``\ , \ ``localHT = erb.HT0()``\ , \ ``jointType = 'Rz'``\ , \ ``parent = -2``\ , \ ``preHT = erb.HT0()``\ , \ ``PDcontrol = (None,None)``\ , \ ``visualization = VRobotLink()``\ )

- | \ *classFunction*\ :
  | initialize robot link
- | \ *input*\ :
  | \ ``mass``\ : mass of robot link
  | \ ``COM``\ : center of mass in link coordinate system
  | \ ``inertia``\ : 3x3 matrix (list of lists / numpy array) containing inertia tensor in link coordinates, with respect to center of mass
  | \ ``localHT``\ : 4x4 matrix (list of lists / numpy array) containing homogeneous transformation from local joint to link coordinates; default = identity; currently, this transformation is not available in KinematicTree, therefore the link inertia and COM must be transformed accordingly
  | \ ``preHT``\ : 4x4 matrix (list of lists / numpy array) containing homogeneous transformation from previous link to this joint; default = identity
  | \ ``jointType``\ : string containing joint type, out of: 'Rx', 'Ry', 'Rz' for revolute joints and 'Px', 'Py', 'Pz' for prismatic joints around/along the respecitive local axes
  | \ ``parent``\ : for building robots as kinematic tree; use '-2' to automatically set parents for serial robot (on fixed base), use '-1' for ground-parent and any other 0-based index for connection to parent link
  | \ ``PDcontrol``\ : tuple of P and D control values, defining position (rotation) proportional value P and velocitiy proportional value D
  | \ ``visualization``\ : VRobotLink structure containing options for drawing of link and joints; see class VRobotLink

----

.. _sec-roboticscore-robotlink-setpdcontrol:

Class function: SetPDcontrol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SetPDcontrol <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L151>`__\ (\ ``self``\ , \ ``Pvalue``\ , \ ``Dvalue``\ )

- | \ *classFunction*\ :
  | set PD control values for drive of joint related to link using position-proportional value P and differential value (velocity proportional) D

----

.. _sec-roboticscore-robotlink-haspdcontrol:

Class function: HasPDcontrol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`HasPDcontrol <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L155>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | check if contrl is available

----

.. _sec-roboticscore-robotlink-getpdcontrol:

Class function: GetPDcontrol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetPDcontrol <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L159>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get PD control values

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-vrobottool:

CLASS VRobotTool (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define visualization of RobotTool


.. _sec-roboticscore-vrobottool---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L183>`__\ (\ ``self``\ , \ ``graphicsData = []``\ )

- | \ *classFunction*\ :
  | initialize robot tool with parameters; currently only graphicsData, which is a list of GraphicsData same as in mbs Objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-robottool:

CLASS RobotTool (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    define tool of robot: containing graphics and HT (may add features in future)


.. _sec-roboticscore-robottool---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L192>`__\ (\ ``self``\ , \ ``HT = erb.HT0()``\ , \ ``visualization = VRobotTool()``\ )

- | \ *classFunction*\ :
  | initialize robot tool
- | \ *input*\ :
  | \ ``HT``\ : 4x4 matrix (list of lists / numpy array) containing homogeneous transformation to transform from last link to tool
  | \ ``graphicsData``\ : dictionary containing a list of GraphicsData, same as in exudyn Objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-vrobotbase:

CLASS VRobotBase (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define visualization of RobotBase


.. _sec-roboticscore-vrobotbase---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L206>`__\ (\ ``self``\ , \ ``graphicsData = []``\ )

- | \ *classFunction*\ :
  | initialize robot base with parameters; currently only graphicsData, which is a list of GraphicsData same as in mbs Objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-robotbase:

CLASS RobotBase (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    define base of robot: containing graphics and HT (may add features in future)


.. _sec-roboticscore-robotbase---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L216>`__\ (\ ``self``\ , \ ``HT = erb.HT0()``\ , \ ``visualization = VRobotBase()``\ )

- | \ *classFunction*\ :
  | initialize robot base
- | \ *input*\ :
  | \ ``HT``\ : 4x4 matrix (list of lists / numpy array) containing homogeneous transformation to transform from world coordinates to base coordinates (changes orientation and position of robot)
  | \ ``graphicsData``\ : dictionary containing a list of GraphicsData, same as in exudyn Objects

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-robot:

CLASS Robot (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class to define a robot


.. _sec-roboticscore-robot---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L241>`__\ (\ ``self``\ , \ ``gravity = [0,0,-9.81]``\ , \ ``base = RobotBase()``\ , \ ``tool = RobotTool()``\ , \ ``referenceConfiguration = []``\ )

- | \ *classFunction*\ :
  | initialize robot class
- | \ *input*\ :
  | \ ``base``\ : definition of base using RobotBase() class
  | \ ``tool``\ : definition of tool using RobotTool() class
  | \ ``gravity``\ : a list or 3D numpy array defining gravity
  | \ ``referenceConfiguration``\ : a list of scalar quantities defining the parameters for reference configuration

----

.. _sec-roboticscore-robot-addlink:

Class function: AddLink
^^^^^^^^^^^^^^^^^^^^^^^
`AddLink <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L267>`__\ (\ ``self``\ , \ ``robotLink``\ )

- | \ *classFunction*\ :
  | add a link to serial robot

----

.. _sec-roboticscore-robot-isserialrobot:

Class function: IsSerialRobot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`IsSerialRobot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L288>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return True, if robot is a serial robot

----

.. _sec-roboticscore-robot-getlink:

Class function: GetLink
^^^^^^^^^^^^^^^^^^^^^^^
`GetLink <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L292>`__\ (\ ``self``\ , \ ``i``\ )

- | \ *classFunction*\ :
  | return Link object of link i

----

.. _sec-roboticscore-robot-hasparent:

Class function: HasParent
^^^^^^^^^^^^^^^^^^^^^^^^^
`HasParent <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L296>`__\ (\ ``self``\ , \ ``i``\ )

- | \ *classFunction*\ :
  | True if link has parent, False if not

----

.. _sec-roboticscore-robot-getparentindex:

Class function: GetParentIndex
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetParentIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L300>`__\ (\ ``self``\ , \ ``i``\ )

- | \ *classFunction*\ :
  | Get index of parent link; for serial robot this is simple, but for general trees, there is a index list

----

.. _sec-roboticscore-robot-numberoflinks:

Class function: NumberOfLinks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`NumberOfLinks <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L305>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return number of links

----

.. _sec-roboticscore-robot-getbaseht:

Class function: GetBaseHT
^^^^^^^^^^^^^^^^^^^^^^^^^
`GetBaseHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L309>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return base as homogeneous transformation

----

.. _sec-roboticscore-robot-gettoolht:

Class function: GetToolHT
^^^^^^^^^^^^^^^^^^^^^^^^^
`GetToolHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L313>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return base as homogeneous transformation

----

.. _sec-roboticscore-robot-linkht:

Class function: LinkHT
^^^^^^^^^^^^^^^^^^^^^^
`LinkHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L317>`__\ (\ ``self``\ , \ ``q``\ )

- | \ *classFunction*\ :
  | compute list of homogeneous transformations for every link, using current joint coordinates q; leads to different results for standard and modified DH parameters because link coordinates are different!

----

.. _sec-roboticscore-robot-jointht:

Class function: JointHT
^^^^^^^^^^^^^^^^^^^^^^^
`JointHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L346>`__\ (\ ``self``\ , \ ``q``\ )

- | \ *classFunction*\ :
  | compute list of homogeneous transformations for every joint (after rotation), using current joint coordinates q

----

.. _sec-roboticscore-robot-comht:

Class function: COMHT
^^^^^^^^^^^^^^^^^^^^^
`COMHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L373>`__\ (\ ``self``\ , \ ``HT``\ )

- | \ *classFunction*\ :
  | compute list of  homogeneous transformations HT from base to every COM using HT list from Robot.JointHT(...)

----

.. _sec-roboticscore-robot-statictorques:

Class function: StaticTorques
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`StaticTorques <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L382>`__\ (\ ``self``\ , \ ``HT``\ )

- | \ *classFunction*\ :
  | compute list of joint torques for serial robot due to gravity (gravity and mass as given in robot), taking HT from Robot.JointHT()

----

.. _sec-roboticscore-robot-jacobian:

Class function: Jacobian
^^^^^^^^^^^^^^^^^^^^^^^^
`Jacobian <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L405>`__\ (\ ``self``\ , \ ``HT``\ , \ ``toolPosition = []``\ , \ ``mode = 'all'``\ , \ ``linkIndex = None``\ )

- | \ *classFunction*\ :
  | compute jacobian for translation and rotation at toolPosition using joint HT; this is using the Robot functions, but is inefficient for simulation purposes
- | \ *input*\ :
  | \ ``HT``\ : list of homogeneous transformations per joint , as computed by Robot.JointHT(...)
  | \ ``toolPosition``\ : global position at which the jacobian is evaluated (e.g., COM); if empty [], it uses the origin of the last link
  | \ ``mode``\ : 'all'...translation and rotation jacobian, 'trans'...only translation part, 'rot': only rotation part
  | \ ``linkIndex``\ : link index for which the jacobian is evaluated; if linkIndex==None, it uses the last link provided in HT
- | \ *output*\ :
  | returns jacobian with translation and rotation parts in rows (3 or 6) according to mode, and one column per HT; in the kinematic tree the columns not related to linkIndex remain zero

----

.. _sec-roboticscore-robot-createkinematictree:

Class function: CreateKinematicTree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateKinematicTree <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L479>`__\ (\ ``self``\ , \ ``mbs``\ , \ ``name = ''``\ , \ ``forceUserFunction = 0``\ )

- | \ *classFunction*\ :
  | Add a ObjectKinematicTree to existing mbs from the robot structure inside this robot class;
  | Joints defined by the kinematics as well as links (and inertia) are transferred to the kinematic tree object;
  | Current implementation only works for serial robots;
  | Control can be realized simply by adding PDcontrol to RobotLink structures, then modifying jointPositionOffsetVector and jointVelocityOffsetVector in ObjectKinematicTree; force offsets (e.g., static or dynamic torque compensation) can be added to KinematicTree jointForceVector; more general control can be added by using KinematicTree forceUserFunction;
  | The coordinates in KinematicTree (as well as jointPositionOffsetVector, etc.) are sorted in the order as the RobotLinks are added to the Robot class;
  | Note that the ObjectKinematicTree is still under development and interfaces may change.
- | \ *input*\ :
  | \ ``mbs``\ : the multibody system, which will be extended
  | \ ``name``\ : object name in KinematicTree; transferred to KinematicTree, default = ''
  | \ ``forceUserFunction``\ : defines the user function for computation of joint forces in KinematicTree; transferred to KinematicTree, default = 0
- | \ *output*\ :
  | the function returns a dictionary containing 'nodeGeneric': generic ODE2 node number ,'objectKinematicTree': the kinematic tree object, 'baseObject': the base object if created, otherwise None; further values will be added in future

----

.. _sec-roboticscore-robot-createredundantcoordinatembs:

Class function: CreateRedundantCoordinateMBS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRedundantCoordinateMBS <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L660>`__\ (\ ``self``\ , \ ``mbs``\ , \ ``baseMarker``\ , \ ``jointSpringDamperUserFunctionList = []``\ , \ ``jointLoadUserFunctionList = []``\ , \ ``createJointTorqueLoads = True``\ , \ ``rotationMarkerBase = None``\ , \ ``rigidBodyNodeType = exudyn.NodeType.RotationEulerParameters``\ )

- | \ *classFunction*\ :
  | Add items to existing mbs from the robot structure inside this robot class; robot is attached to baseMarker (can be ground object or moving/deformable body);
  | The (serial) robot is built as rigid bodies (containing rigid body nodes), where bodies represent the links which are connected by joints;
  | Add optional jointSpringDamperUserFunctionList for individual control of joints; otherwise use PDcontrol in RobotLink structure; additional joint torques/forces can be added via spring damper, using mbs.SetObjectParameter(...) function;
  | See several Python examples, e.g., \ ``serialRobotTestTSD.py``\ , in Examples or TestModels;
  | For more efficient models, use CreateKinematicTree(...) function!
- | \ *input*\ :
  | \ ``mbs``\ : the multibody system, which will be extended
  | \ ``baseMarker``\ : a rigid body marker, at which the robot will be placed (usually ground); note that the local coordinate system of the base must be in accordance with the DH-parameters, i.e., the z-axis must be the first rotation axis. For correction of the base coordinate system, use rotationMarkerBase
  | \ ``jointSpringDamperUserFunctionList``\ : NOT IMPLEMENTED yet: jointSpringDamperUserFunctionLista list of user functions for actuation of joints with more efficient spring-damper based connector (spring-damper directly emulates PD-controller); uses torsional spring damper for revolute joints and linear spring damper for prismatic joints; can be empty list (no spring dampers); if entry of list is 0, no user function is created, just pure spring damper; parameters are taken from RobotLink PDcontrol structure, which MUST be defined using SetPDcontrol(...) in RobotLink
  | \ ``jointLoadUserFunctionList``\ : DEPRECATED: a list of user functions for actuation of joints according to a LoadTorqueVector userFunction, see serialRobotTest.py as an example; can be empty list
  | \ ``createJointTorqueLoads``\ : DEPRECATED: if True, independently of jointLoadUserFunctionList, joint loads are created; the load numbers are stored in lists jointTorque0List/ jointTorque1List; the loads contain zero torques and need to be updated in every computation step, e.g., using a preStepUserFunction; unitTorque0List/ unitTorque1List contain the unit torque vector for the according body(link) which needs to be applied on both bodies attached to the joint
  | \ ``rotationMarkerBase``\ : add a numpy 3x3 matrix for rotation of the base, in order that the robot can be attached to any rotated base marker; the rotationMarkerBase is according to the definition in GenericJoint; note, that for moving base, the static compensation does not work (base rotation must be updated)
  | \ ``rigidBodyNodeType``\ : specify node type of rigid body node, e.g., exudyn.NodeType.RotationEulerParameters, etc.
- | \ *output*\ :
  | the function returns a dictionary containing per link nodes and object (body) numbers, 'nodeList', 'bodyList', the object numbers for joints, 'jointList', list of load numbers for joint torques (jointTorque0List, jointTorque1List); unit torque vectors in local coordinates of the bodies to which the torques are applied (unitTorque0List, unitTorque1List); springDamperList contains the spring dampers if defined by PDcontrol of links

----

.. _sec-roboticscore-robot-getkinematictree66:

Class function: GetKinematicTree66
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetKinematicTree66 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L899>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | export kinematicTree

----

.. _sec-roboticscore-robot-getlinkgraphicsdata:

Class function: GetLinkGraphicsData
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetLinkGraphicsData <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L929>`__\ (\ ``self``\ , \ ``i``\ , \ ``p0``\ , \ ``p1``\ , \ ``axis0``\ , \ ``axis1``\ , \ ``linkVisualization``\ )

- | \ *classFunction*\ :
  | create link GraphicsData (list) for link i; internally used in CreateRedundantCoordinateMBS(...); linkVisualization contains visualization dict of link

----

.. _sec-roboticscore-robot-buildfromdictionary:

Class function: BuildFromDictionary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`BuildFromDictionary <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L973>`__\ (\ ``self``\ , \ ``robotDict``\ )

- | \ *classFunction*\ :
  | build robot structre from dictionary; this is a DEPRECATED function, which is used in older models; DO NOT USE

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)


.. _sec-module-robotics-class-inversekinematicsnumerical():

CLASS InverseKinematicsNumerical() (in module robotics)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    This class can be used to solve the inverse kinematics problem using a multibody system
    by solving the static problem of a serial robot

- | \ *author*\ :
  | Peter Manzl, Johannes Gerstmayr
- | \ *notes*\ :
  | still under development; errors in orientations of solution may occure. proviedes mtehods to calculate inverse Kinematics


.. _sec-roboticscore-inversekinematicsnumerical()---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1084>`__\ (\ ``self``\ , \ ``robot``\ , \ ``jointStiffness = 1e0``\ , \ ``useRenderer = False``\ , \ ``flagDebug = False``\ , \ ``useAlternativeConstraints = False``\ )

- | \ *classFunction*\ :
  | initialize RigidBodyInertia with scalar mass, 3x3 inertiaTensor (w.r.t. reference point!!!) and center of mass com
- | \ *input*\ :
  | \ ``robot``\ : robot class
  | \ ``jointStiffness``\ : the stiffness used for the robot's model joints
  | \ ``useRenderer``\ : when solving the inverse kinematics the renderer is used to show the starting/end
  | configuration of the robot using the graphics objects definded in the robot object
- | \ *author*\ :
  | Peter Manzl

----

.. _sec-roboticscore-inversekinematicsnumerical()-getcurrentrobotht:

Class function: GetCurrentRobotHT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetCurrentRobotHT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1176>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | Utility function to get current Homogeneous transformation of the robot to check inverse Kinematics solution
  | \*\* output:
  | \ ``T``\ : 4x4 homogeneous Transformation matrix of the current TCP pose

----

.. _sec-roboticscore-inversekinematicsnumerical()-interpolatehts:

Class function: InterpolateHTs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InterpolateHTs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1193>`__\ (\ ``self``\ , \ ``T1``\ , \ ``T2``\ , \ ``rotStep = np.pi/16``\ , \ ``minSteps = 1``\ )

- | \ *classFunction*\ :
  | 
- | \ *input*\ :
  | \ ``T1``\ : 4x4 homogeneous transformation matrix representing the first Pose
  | \ ``T2``\ : 4x4 homogeneous transformation matrix representing the second Pose
  | \ ``rotStep``\ : the max. size of steps to take for the orientation
  | \ ``minSteps``\ : minimum number of substeps to interpolate
- | \ *output*\ :
  | T: a List of homogeneous Transformations for each step between
- | \ *author*\ :
  | Peter Manzl
- | \ *notes*\ :
  | still under development; interpolation may be changed to using logSE3

----

.. _sec-roboticscore-inversekinematicsnumerical()-solvesafe:

Class function: SolveSafe
^^^^^^^^^^^^^^^^^^^^^^^^^
`SolveSafe <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1234>`__\ (\ ``self``\ , \ ``T``\ , \ ``q0 = None``\ )

- | \ *classFunction*\ :
  | This Method can be used to solve the inverse kinematics problem by solving
  | the static problem of a serial robot using steps to interpolate between start and end position close to the function Solve.
  | This helps the function Solve() to find the correct solutions.
- | \ *input*\ :
  | \ ``T``\ : the 4x4 homogeneous transformation matrix representing the desired position and orientation of the Endeffector
  | \ ``q0``\ : The configuration (joint angles/positions) of the robot from which the numerical methods start so calculate the solution; q0=None indicates that the stored solution (from model or previous solution) shall be used for initialization
- | \ *output*\ :
  | [q, success]; q: The solution for the joint angles in which the robot's tool center point (TCP) reaches the desired homogeneous transformation matrix T; success=False indicates that all trials for inverse kinematics failed, leading to q=None
  | \ ``success``\ : flag to indicate if method was successful
- | \ *author*\ :
  | Peter Manzl, Johannes Gerstmayr
- | \ *notes*\ :
  | still under development; errors in orientations of solution may occure. works similar to ikine_LM function of the robotics toolbox from peter corke

----

.. _sec-roboticscore-inversekinematicsnumerical()-solve:

Class function: Solve
^^^^^^^^^^^^^^^^^^^^^
`Solve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/roboticsCore.py\#L1284>`__\ (\ ``self``\ , \ ``T``\ , \ ``q0 = None``\ )

- | \ *classFunction*\ :
  | This Method can be used to solve the inverse kinematics problem by solving
  | the static problem of a serial robot using steps to interpolate between start and end position close to the function Solve.
  | T his helps the fucntion Solve to find the correct solutions.
- | \ *input*\ :
  | \ ``T``\ : the 4x4 homogeneous transformation matrix representing the desired position and orientation of the Endeffector
  | \ ``q0``\ : The configuration (joint angles/positions) of the robot from which the numerical methods start so calculate the solution; q0=None indicates that the stored solution (from model or previous solution) shall be used for initialization
- | \ *output*\ :
  | [q, success]; q: The solution for the joint angles in which the robot's tool center point (TCP) reaches the desired homogeneous transformation matrix T; success=False indicates that all trials for inverse kinematics failed, leading to q=None
- | \ *author*\ :
  | Peter Manzl, Johannes Gerstmayr
- | \ *notes*\ :
  | still under development; errors in orientations of solution may occure. works similar to ikine_LM function of the robotics toolbox from peter corke

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)

