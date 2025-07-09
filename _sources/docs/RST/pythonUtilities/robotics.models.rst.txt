
.. _sec-module-robotics-models:

Module: robotics.models
-----------------------

This module contains robotics models; They can be imported by simply calling the functions,
which return the according robot dictionary;
the library is built on Denavit-Hartenberg Parameters and
Homogeneous Transformations (HT) to describe transformations and coordinate systems

- Date:      2021-01-10 


.. _sec-models-manipulator4rsimple:

Function: Manipulator4Rsimple
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Manipulator4Rsimple <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L43>`__\ ()

- | \ *function description*\ :
  | generate 4R manipulator as myRobot dictionary, settings are done in function
- | \ *output*\ :
  | myRobot dictionary
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | the 4th joint is used to simulate a paralell kinematics manipulator



----


.. _sec-models-manipulator3rsimple:

Function: Manipulator3RSimple
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Manipulator3RSimple <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L132>`__\ ()

- | \ *function description*\ :
  | generate 3R manipulator as myRobot dictionary, settings are done in function
- | \ *output*\ :
  | myRobot dictionary
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | \ ``DH-parameters``\ : [theta, d, a, alpha], according to P. Corke
  | Values according to WÃ¶rnle simple example with l1=0
  | d=[h1 0 0];
  | theta=[beta1 beta2 beta3];
  | a=[l1 l2 l3];
  | alpha=[pi/2 0 0];



----


.. _sec-models-manipulatorpanda:

Function: ManipulatorPANDA
^^^^^^^^^^^^^^^^^^^^^^^^^^
`ManipulatorPANDA <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L219>`__\ ()

- | \ *function description*\ :
  | generate Franka Emika Panda manipulator as myRobot dictionary, settings are done in function
- | \ *output*\ :
  | myRobot dictionary
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | all Parameter according to Gaz et. al 
  | \ ``DH-parameters(std)``\ : [theta, d, a, alpha], according to P. Corke
  | Standard DH Parameters, masses, inertias and com according P.Corke and Gaz et. al (they working with modified DH parameter)
  | changes to standard DH Parameter checked with P.Corke toolbox

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-models-manipulatorur5:

Function: ManipulatorUR5
^^^^^^^^^^^^^^^^^^^^^^^^
`ManipulatorUR5 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L360>`__\ ()

- | \ *function description*\ :
  | generate UR5 manipulator as myRobot dictionary, settings are done in function
- | \ *output*\ :
  | myRobot dictionary
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | define myRobot kinematics, UR5 Universal Robotics,
  | Standard DH-parameters: [theta, d, a, alpha], according to P. Corke,
  | Links modeld as cylindrical tubes, Inertia from Parham M. Kebria2016 / Kuefeta2014

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-models-manipulatorpuma560:

Function: ManipulatorPuma560
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ManipulatorPuma560 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L448>`__\ ()

- | \ *function description*\ :
  | generate puma560 manipulator as myRobot dictionary, settings are done in function
- | \ *output*\ :
  | myRobot dictionary
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | std DH-parameters: [theta, d, a, alpha], according to P. Corke page 138,
  | puma p560 limits, taken from Corke Visual Control of Robots

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `InverseKinematicsNumericalExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/InverseKinematicsNumericalExample.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex), \ `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_\  (Ex)



----


.. _sec-models-linkdict2robot:

Function: LinkDict2Robot
^^^^^^^^^^^^^^^^^^^^^^^^
`LinkDict2Robot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L546>`__\ (\ ``robotLinkDict``\ , \ ``robotClass = None``\ )

- | \ *function description*\ :
  | generate serial manipulator as robotClass object from robotLinkDict
- | \ *input*\ :
  | \ ``robotClass``\ : robot class object from roboticsCore; if robotClass is provided, gravity, tool and base are used from there
  | \ ``robotLinkDict``\ : list of robot links generated by manipulator import for individual robot dictionary
- | \ *output*\ :
  | updated robot class
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | DH Parameter Information
  | stdH = [theta, d, a, alpha] with Rz(theta) \* Tz(d) \* Tx(a) \* Rx(alpha)
  | modDH = [alpha, dx, theta, rz] with
  | used by Corke and Lynch: Rx(alpha) \* Tx(a) \* Rz(theta) \* Tz(d)
  | used by Khali:           Rx(alpha) \* Tx(d) \* Rz(theta) \* Tz(r)
  | Important note:  d(khali)=a(corke)  and r(khali)=d(corke)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ROSMobileManipulator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ROSMobileManipulator.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-models-linkdictmoddhkk2robot:

Function: LinkDictModDHKK2Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`LinkDictModDHKK2Robot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/robotics/models.py\#L600>`__\ (\ ``robotLinkDict``\ , \ ``robotClass = None``\ )

- | \ *function description*\ :
  | special test function to generate serial manipulator as robotClass object from robotLinkDict using inertia parameters defined in stdDH coordinates, but creating robot from modDHKK; will be ERASED in future
- | \ *input*\ :
  | \ ``robotLinkDict``\ : list of robot links generated by manipulator import for individual robot dictionary
  | \ ``robotClass``\ : robot class object from roboticsCore; if robotClass is provided, gravity, tool and base are used from there
- | \ *output*\ :
  | updated robot class
- | \ *author*\ :
  | Martin Sereinig
- | \ *notes*\ :
  | DEPRECATED; function uses modDHKK in robotLinkDict for creation, transforms inertia parameters; should only be used for testing!

