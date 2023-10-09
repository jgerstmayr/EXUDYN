
.. _sec-module-robotics-mobile:

Module: robotics.mobile
-----------------------

The utilities contains functionality for mobile robots 
based on the EXUDYN example MecanumWheel RollingDiscPenality
specific friction angle of rolling disc is used to model rolls of mecanum wheels


- Author:    Martin Sereinig, Peter Manzl and Johannes Gerstmayr 
- Date:      2021-10-01  Updated:  2023-09-15 
- Notes:  formulation is still under development 


.. _sec-mobile-mobilerobot2mbs:

Function: mobileRobot2MBS
^^^^^^^^^^^^^^^^^^^^^^^^^
`mobileRobot2MBS <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L41>`__\ (\ ``mbs``\ , \ ``mobileRobot``\ , \ ``markerGround``\ , \ ``flagGraphicsRollers = True``\ , \ ``*args``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | add items to existing mbs to build up a mobile robot platform,
  | there are options that can be passed as args / kwargs, which can contains options as described below.
  | The robot platform is built out of rigid bodies where the wheels can be modeled as rolling discs
  | (mecanum wheel x/o configuration) or with a detailed mecanum wheel simulation approach
- | \ *input*\ :
  | \ ``mbs``\ : the multibody system which will be extended
  | \ ``markerGround``\ : a rigid body marker, at which the robot will be placed (usually ground)
  | \ ``mobileRobot``\ : a dictionary including all information about the mobile robot platform
- | \ *output*\ :
  | the function returns a dictionary containing nodes, body, object and marker numbers of individual mobile robot parts
  | nPlatformList, bPlatformList, oPlatformList, mPlatformList; nodes, bodies, objects and marker of the platform [nPlattform] [bPlattform] [oPlattform]  []
  | oAxisList, mAxlesList; objects and marker of the axles  [a1, a2, a3, a4]
  | nWheelsList, bWheelsList, oRollingDiscsList, mWheelsList; nodes, bodys, objects and markers of the four wheels [w1, w2, w3, w4]
- | \ *notes*\ :
  | for coordinate system, see Python function definition

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex)



----


.. _sec-mobile-generatrix2polynomial:

Function: Generatrix2Polynomial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Generatrix2Polynomial <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L397>`__\ (\ ``param``\ , \ ``GeneratrixFunction``\ , \ ``tol = 1e-14``\ , \ ``nFit = 101``\ , \ ``nTest = 1001``\ )

- | \ *function description*\ :
  | create a polynomial describing a generatrix function
- | \ *input*\ :
  | param: list containing data (lRoll, aPoly, ...)
- | \ *author*\ :
  | Peter Manzl
  | \ ``**note``\ : create and fit a polynomial of an order high enough to approximate the given GeneratrixFunction
  | with a given tolerance. The error is measured as the Chebyshev distance.



----


.. _sec-mobile-generatrixroll:

Function: GeneratrixRoll
^^^^^^^^^^^^^^^^^^^^^^^^
`GeneratrixRoll <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L469>`__\ (\ ``u``\ , \ ``param``\ )

- | \ *function description*\ :
  | generatrix function for a roll of a Mecanum wheel
- | \ *input*\ :
  | \ ``u``\ : parameter, max. +- pi/2
  | \ ``param['r']``\ : radius of the associated Mecanum wheel
  | \ ``param['delta']``\ : angle of the rolls rotation axis to the wheels rotation axis
  | \ ``param['dRoll']``\ : smallest distance of roll axis to the wheel axis
- | \ *output*\ :
  | x and y values for the function in the local frame. The rotation around the
  | local x-yxis creates the surface of the roll.
- | \ *author*\ :
  | Peter Manzl
- | \ *notes*\ :
  | parametric equation, x,y are the generatrix of the roll in
  | its local frame with the axis of rotation x, see .



----


.. _sec-mobile-fundiffpoly:

Function: FunDiffPoly
^^^^^^^^^^^^^^^^^^^^^
`FunDiffPoly <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L482>`__\ (\ ``x``\ , \ ``a``\ )

- | \ *function description*\ :
  | calculates the derivative of the polynomial \ :math:`a0*x^n + ...`\
- | \ *input*\ :
  | \ ``x``\ : value at which the polynomial is evaluated
  | \ ``a``\ : coefficients
- | \ *output*\ :
  | f:
- | \ *author*\ :
  | Peter Manzl
  | \ ``**note``\ : helper function polynomial describing a generatrix function



----


.. _sec-mobile-funddiffpoly:

Function: FunDDiffPoly
^^^^^^^^^^^^^^^^^^^^^^
`FunDDiffPoly <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L497>`__\ (\ ``x``\ , \ ``a``\ )

- | \ *function description*\ :
  | calculates the second derivative of a polynomial
- | \ *input*\ :
  | \ ``x``\ : value at which the polynomial is evaluated
  | \ ``a``\ : coefficients
- | \ *output*\ :
  | f:
- | \ *author*\ :
  | Peter Manzl
  | \ ``**note``\ : helper function polynomial describing a generatrix function


.. _sec-module-robotics-mobile-class-mobilekinematics:

CLASS MobileKinematics (in module robotics.mobile)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    calculate 4 wheel velocities for a mecanum wheel driven platform with given platform velocities

- | \ *author*\ :
  | Peter Manzl, Johannes Gerstmayr
- | \ *notes*\ :
  | still under development; wheel axis is mounted at y-axis; positive angVel rotates CCW in x/y plane viewed from top; for coordinate system, see Python class definition


.. _sec-mobile-mobilekinematics---init--:

Class function: \_\_init\_\_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`\_\_init\_\_ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L315>`__\ (\ ``self``\ , \ ``R``\ , \ ``lx``\ , \ ``ly``\ , \ ``flagAdjusted = False``\ , \ ``lcx = 0``\ , \ ``lcy = 0``\ , \ ``wheeltype = 0``\ )

- | \ *classFunction*\ :
  | initialize mobileKinematics class
- | \ *input*\ :
  | \ ``R``\ : wheel radius
  | \ ``lx``\ : wheel track width
  | \ ``ly``\ : wheel base
  | \ ``wheeltype``\ : 1=x-config (bad), 0=o-config (good)
- | \ *author*\ :
  | Peter Manzl

----

.. _sec-mobile-mobilekinematics-getwheelvelocities:

Class function: getWheelVelocities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`getWheelVelocities <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L351>`__\ (\ ``self``\ , \ ``vDes``\ )

- | \ *classFunction*\ :
  | calculate wheel velocities from Cartesian velocities
- | \ *input*\ :
  | \ ``vDes``\ : desired velocity [vx, vy, omega] in the robot's local frame
  | \ ``vx``\ : platform  translational velocity in local x direction
  | \ ``vy``\ : platform translational velocity in local y direction
  | \ ``omega``\ : platform rotational velocity around local z axis
- | \ *output*\ :
  | w: wheel velocities w=[w0,w1,w2,w3]
- | \ *author*\ :
  | Peter Manzl

----

.. _sec-mobile-mobilekinematics-getcartesianvelocities:

Class function: getCartesianVelocities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`getCartesianVelocities <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/mobile.py\#L375>`__\ (\ ``self``\ , \ ``w``\ )

- | \ *classFunction*\ :
  | calculate Cartesian velocities from wheel velocities
- | \ *input*\ :
  | w: wheel velocities w=[w0,w1,w2,w3]
- | \ *output*\ :
  | \ ``v``\ : Cartesian velocity [vx, vy, omega] in the robot's local frame
  | \ ``vx``\ : platform  translational velocity in local x direction
  | \ ``vy``\ : platform translational velocity in local y direction
  | \ ``omega``\ : platform rotational velocity around local z axis
- | \ *author*\ :
  | Peter Manzl

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex)

