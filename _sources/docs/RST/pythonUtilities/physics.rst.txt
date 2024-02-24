
.. _sec-module-physics:

Module: physics
===============

The physics library includes helper functions and data related to physics 
models and parameters; for rigid body inertia, see rigidBodyUtilities

- Date:      2021-01-20 


.. _sec-physics-stribeckfunction:

Function: StribeckFunction
^^^^^^^^^^^^^^^^^^^^^^^^^^
`StribeckFunction <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/physics.py\#L30>`__\ (\ ``vel``\ , \ ``muDynamic``\ , \ ``muStaticOffset``\ , \ ``muViscous = 0``\ , \ ``expVel = 1e-3``\ , \ ``regVel = 1e-3``\ )

- | \ *function description*\ :
  | describes regularized Stribeck function with optial viscous part for given velocity,
  | \ :math:`f(v) = \begin{cases} (\mu_d + \mu_{s_{off}}) v, \quad \mathrm{if} \quad |v| <= v_{reg}\\ \mathrm{Sign}(v)\left( \mu_d + \mu_{s_{off}} \mathrm{e}^{-(|v|-v_{reg})/v_{exp}} + \mu_v (|v|-v_{reg}) \right), \quad \mathrm{else}\end{cases}`\ 
- | \ *input*\ :
  | \ ``vel``\ : input velocity \ :math:`v`\ 
  | \ ``muDynamic``\ : dynamic friction coefficient \ :math:`\mu_d`\ 
  | \ ``muStaticOffset``\ : \ :math:`\mu_{s_{off}}`\ , offset to dynamic friction, which gives muStaticFriction = muDynamic + muStaticOffset
  | \ ``muViscous``\ : \ :math:`\mu_v`\ , viscous part, acting proportional to velocity except for regVel
  | \ ``regVel``\ : \ :math:`v_{reg}`\ ,  small regularization velocity in which the friction is linear around zero velocity (e.g., to get Newton converged)
  | \ ``expVel``\ : \ :math:`v_{exp}`\ ,  velocity (relative to regVel, at which the muStaticOffset decreases exponentially, at vel=expVel, the factor to muStaticOffset is exp(-1) = 36.8\%)
- | \ *output*\ :
  | returns velocity dependent friction coefficient (if muDynamic and muStaticOffset are friction coefficients) or friction force (if muDynamic and muStaticOffset are on force level)
- | \ *notes*\ :
  | see Isermann (2008) and Armstrong-Helouvry (1991)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex)



----


.. _sec-physics-regularizedfrictionstep:

Function: RegularizedFrictionStep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RegularizedFrictionStep <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/physics.py\#L39>`__\ (\ ``x``\ , \ ``x0``\ , \ ``h0``\ , \ ``x1``\ , \ ``h1``\ )

- | \ *function description*\ :
  | helper function for RegularizedFriction(...)



----


.. _sec-physics-regularizedfriction:

Function: RegularizedFriction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RegularizedFriction <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/physics.py\#L60>`__\ (\ ``vel``\ , \ ``muDynamic``\ , \ ``muStaticOffset``\ , \ ``velStatic``\ , \ ``velDynamic``\ , \ ``muViscous = 0``\ )

- | \ *function description*\ :
  | describes regularized friction function, with increased static friction, dynamic friction and optional viscous part
- | \ *input*\ :
  | \ ``vel``\ : input velocity
  | \ ``muDynamic``\ : dynamic friction coefficient
  | \ ``muStaticOffset``\ : offset to dynamic friction, which gives muStaticFriction = muDynamic + muStaticOffset
  | \ ``muViscous``\ : viscous part, acting proportional to velocity for velocities larger than velDynamic; extension to mentioned references
  | \ ``velStatic``\ : small regularization velocity at which exactly the staticFriction is reached; for smaller velocities, the friction is smooth and zero-crossing (unphysical!) (e.g., to get Newton converged)
  | \ ``velDynamic``\ : velocity at which muDynamic is reached for first time
- | \ *output*\ :
  | returns velocity dependent friction coefficient (if muDynamic and muStaticOffset are friction coefficients) or friction force (if muDynamic and muStaticOffset are on force level)
- | \ *notes*\ :
  | see references: Flores et al. , Qian et al.

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex)



----


.. _sec-physics-vonmisesstress:

Function: VonMisesStress
^^^^^^^^^^^^^^^^^^^^^^^^
`VonMisesStress <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/physics.py\#L78>`__\ (\ ``stress6D``\ )

- | \ *function description*\ :
  | compute equivalent von-Mises stress given 6 stress components or list of stress6D (or stress6D in rows of np.array)
- | \ *input*\ :
  | stress6D: 6 stress components as list or np.array, using ordering \ :math:`[\sigma_{xx}`\ , \ :math:`\sigma_{yy}`\ , \ :math:`\sigma_{zz}`\ , \ :math:`\sigma_{yz}`\ , \ :math:`\sigma_{xz}`\ , \ :math:`\sigma_{xy}]`\
- | \ *output*\ :
  | returns scalar equivalent von-Mises stress or np.array of von-Mises stresses for all stress6D



----


.. _sec-physics-ufvonmisesstress:

Function: UFvonMisesStress
^^^^^^^^^^^^^^^^^^^^^^^^^^
`UFvonMisesStress <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/physics.py\#L101>`__\ (\ ``mbs``\ , \ ``t``\ , \ ``sensorNumbers``\ , \ ``factors``\ , \ ``configuration``\ )

- | \ *function description*\ :
  | Sensor user function to compute equivalent von-Mises stress from sensor with Stress or StressLocal OutputVariableType; if more than 1 sensor is given in sensorNumbers, then the maximum stress is computed
- | \ *input*\ :
  | arguments according to \ ``SensorUserFunction``\ ; factors are ignored
- | \ *output*\ :
  | returns scalar (maximum) equivalent von-Mises stress
- | \ *example*\ :

.. code-block:: python

  #assuming s0, s1, s2 being sensor numbers with StressLocal components
  sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=[s0,s1,s2],
                                           fileName='solution/sensorMisesStress.txt',
                                           sensorUserFunction=UFvonMisesStress))


