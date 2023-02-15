
.. _sec-overview-advanced:

Advanced topics
===============

This section covers some advanced topics, which may be only relevant for a smaller group of people. 
Functionality may be extended but also removed in future

.. _sec-overview-advanced-camerafollowing:

Camera following objects and interacting with model view
--------------------------------------------------------

For some models, it may be advantageous to track the translation and/or rotation of certain bodies, e.g., for cars, (wheeled) robots or bicycles. 
To do so, the current render state (\ ``SC.GetRenderState()``\ , \ ``SC.SetRenderState(...)``\ ) can be obtained and modified, in order to always follow a certain position.
As this needs to be done during redraw of every frame, it is conveniently done in a graphicsUserFunction, e.g., within the ground body. This is shown in the following example, in which \ ``mbs.variables['nTrackNode']``\  is a node number to be tracked:

.. code-block:: python

  #mbs.variables['nTrackNode'] contains node number
  def UFgraphics(mbs, objectNum):
      n = mbs.variables['nTrackNode']
      p = mbs.GetNodeOutput(n,exu.OutputVariableType.Position, 
                            configuration=exu.ConfigurationType.Visualization)
      rs=SC.GetRenderState() #get current render state
      A = np.array(rs['modelRotation'])
      p = A.T @ p #transform point into model view coordinates
      rs['centerPoint']=[p[0],p[1],p[2]]
      SC.SetRenderState(rs)  #modify render state
      return []

  #add object with graphics user function
  oGround2 = mbs.AddObject(ObjectGround(visualization=
                 VObjectGround(graphicsDataUserFunction=UFgraphics)))
  #.... further code for simulation here



.. _sec-overview-advanced-contact:

Contact problems
----------------

Since Q4 2021 a contact module is available in Exudyn. 
This separate module \ ``GeneralContact``\  [\ **still under development, consider with care!**\ ] is highly optimized and implemented with parallelization (multi-threaded) for certain types of contact elements.


.. |cpic1| image:: ../theDoc/figures/contactTests.png
   :width: 45%

.. |cpic2| image:: ../theDoc/figures/contactTests2.jpg
   :width: 45%

|cpic1| |cpic2|

[Some tests and examples using \ ``GeneralContact``\ ]




\ **Note**\ :

+  \ ``GeneralContact``\  is (in most cases) restricted to dynamic simulation (explicit or implicit [\ **still under development, consider with care!**\ ]) if friction is used; without friction, it also works in the static case
+  in addition to \ ``GeneralContact``\  there are special objects, in particular for rolling and simple 1D contacts, that are available as single objects, cf. \ ``ObjectConnectorRollingDiscPenalty``\ 
+  \ ``GeneralContact``\  is recommended to be used for large numbers of contacts, while the single objects are integrated more directly into mbs.


Currently, \ ``GeneralContact``\  includes:

+  Sphere-Sphere contact (attached to any marker); may represent circle-circle contact in 2D
+  Triangles mounted on rigid bodies, in contact with Spheres [only explicit]
+  ANCFCable2D contacting with spheres (which then represent circles in 2D) [partially implicit, needs revision]

For details on the contact formulations, see  :ref:`seccontacttheory`\ .


.. _sec-overview-advanced-openvr:

OpenVR
------

The general open source libraries from Valve, see

   https://github.com/ValveSoftware/openvr

have been linked to Exudyn . In order to get OpenVR fully integrated, you need to run \ ``setup.py``\  Exudyn with the \ ``--openvr``\  flag. For general installation instructions, see  :ref:`sec-install-installinstructions`\ .

Running OpenVR either requires an according head mounted display (HMD) or a virtualization using, e.g., Riftcat 2 to use a mobile phone with an according adapter. Visualization settings are available in \ ``interactive.openVR``\ , but need to be considered with care.
An example is provided in \ ``openVRengine.py``\ , showing some optimal flags like locking the model rotation, zoom or translation.

Everything is experimental, but contributions are welcome!


.. _sec-overview-advanced-interactwithcodes:

Interaction with other codes
----------------------------

Interaction with other codes and computers (E.g., MATLAB or other C++ codes, or other Python versions)
is possible. 
To connect to any other code, it is convenient to use a TCP/IP connection. This is enabled via 
the \ ``exudyn.utilities``\  functions

+  \ ``CreateTCPIPconnection``\ 
+  \ ``TCPIPsendReceive``\ 
+  \ ``CloseTCPIPconnection``\ 

Basically, data can be transmitted in both directions, e.g., within a preStepUserFunction. In Examples, you can find 
 TCPIPexudynMatlab.py which shows a basic example for such a connectivity.


.. _sec-overview-advanced-ros:

ROS
---

Basic interaction with ROS has been tested. However, make sure to use Python 3, as there is no (and will never be any) Python 2
support for Exudyn .






