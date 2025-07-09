

.. _sec-item-objectground:

ObjectGround
============

A ground object behaving like a rigid body, but having no degrees of freedom; used to attach body-connectors without an action. For examples see spring dampers and joints.

\ **Additional information for ObjectGround**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Ground``\ , \ ``Body``\ 


The item \ **ObjectGround**\  with type = 'Ground' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **referencePosition** [\ :math:`\pRefG`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | reference point = reference position for ground object; local position is added on top of reference position for a ground object
* | **referenceRotation** [\ :math:`\LU{0b}{\Rot} \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | the constant ground rotation matrix, which transforms body-fixed (b) to global (0) coordinates
* | **visualization** [type = VObjectGround]:
  | parameters for visualization of item



The item VObjectGround has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectground:

DESCRIPTION of ObjectGround
---------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}} = \pRefG + \LU{0b}{\Rot} \pLocB`\ 
  | global position vector of translated local position
* | ``Displacement``\ : \ :math:`\Null`\ 
  | global displacement vector of local position
* | ``Velocity``\ : \ :math:`\Null`\ 
  | global velocity vector of local position
* | ``AngularVelocity``\ : \ :math:`\Null`\ 
  | angular velocity of body
* | ``RotationMatrix``\ : \ :math:`\LU{0b}{\Rot}`\ 
  | rotation matrix in vector form (stored in row-major order)



Equations
---------

ObjectGround has no equations, as it only provides a static object, at which joints and connectors can be attached. 
The object does not move (in general) and forces or torques do not have an effect.
However, the reference position and rotation may be changed over time. This may prescribe
motion, however, with the measured velocity still being zero at each time instant. Therefore,
such manipulation of reference position or rotation shall be treated with care.

In combination with markers, the \ ``localPosition``\  \ :math:`\pLocB`\  is transformed by the \ ``ObjectGround``\  to
a global point \ :math:`\LU{0}{{\mathbf{p}}}`\  using the reference point \ :math:`\pRefG`\ ,

.. math::

   \LU{0}{{\mathbf{p}}} = \pRefG + \LU{0b}{\Rot} \pLocB .



--------

\ **Userfunction**\ : ``graphicsDataUserFunction(mbs, itemNumber)`` 


A user function, which is called by the visualization thread in order to draw user-defined objects.
The function can be used to generate any \ ``BodyGraphicsData``\ , see Section  :ref:`sec-graphicsdata`\ .
Use \ ``exudyn.graphics``\  functions, see Section  :ref:`sec-module-graphics`\ , to create more complicated objects. 
Note that \ ``graphicsDataUserFunction``\  needs to copy lots of data and is therefore
inefficient and only designed to enable simpler tests, but not large scale problems.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides reference to mbs, which can be used in user function to access all data of the object
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access
   * - | \returnValue
     - | BodyGraphicsData
     - | list of \ ``GraphicsData``\  dictionaries, see Section  :ref:`sec-graphicsdata`\ 


--------

\ **User function example**\ :



.. code-block:: python

    import exudyn as exu
    from math import sin, cos, pi
    from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
    import exudyn.graphics as graphics

    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    #create simple system:
    mbs.AddNode(NodePoint())
    body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
    
    #user function for moving graphics:
    def UFgraphics(mbs, objectNum):
        t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization) #get time if needed
        #draw moving sphere on ground
        graphics1=graphics.Sphere(point=[sin(t*2*pi), cos(t*2*pi), 0], 
                                     radius=0.1, color=graphics.color.red, nTiles=32)
        return [graphics1] 

    #add object with graphics user function
    ground = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsDataUserFunction=UFgraphics)))
    mbs.Assemble()
    sims=exu.SimulationSettings()
    sims.timeIntegration.numberOfSteps = 10000000 #many steps to see graphics
    SC.renderer.Start() #perform zoom all (press 'a' several times) after startup to see the sphere
    mbs.SolveDynamic(sims)
    SC.renderer.Stop()




Relevant Examples and TestModels with weblink:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Examples/), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Examples/), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFcableCantilevered.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcableCantilevered.py>`_\  (Examples/), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Examples/), \ `ANCFcantileverTestDyn.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTestDyn.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TestModels/), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


