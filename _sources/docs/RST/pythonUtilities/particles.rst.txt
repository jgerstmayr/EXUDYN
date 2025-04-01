
.. _sec-module-particles:

Module: particles
=================

This module offers methods for GeneralContact, in particular particles (DEM - discrete element method)

- Author:    Johannes Gerstmayr 
- Date:      2024-10-19 (created) 


.. _sec-particles-createparticlesinbox:

Function: CreateParticlesInBox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateParticlesInBox <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/particles.py\#L28>`__\ (\ ``minPointBox``\ , \ ``maxPointBox``\ , \ ``minRadius``\ , \ ``maxRadius = None``\ , \ ``maxNumberOfParticles = None``\ , \ ``offsetRadius = 0``\ , \ ``verbose = 0``\ )

- | \ *function description*\ :
  | create set of spherical particles densly packed inside box using hexagonal closest packing (HCP); radius is randomized between minRadius and maxRadius
- | \ *input*\ :
  | \ ``minPointBox``\ : [xMin,yMin,zMin] minimal cartesian coordinates for box
  | \ ``maxPointBox``\ : [xMax,yMax,zMax] maximal cartesian coordinates for box
  | \ ``minRadius``\ : minimal or nominal radius
  | \ ``maxRadius``\ : maximal radius for randomized variations of radius or None to use minRadius
  | \ ``maxNumberOfParticles``\ : if not None, this limits the amount of created particles; otherwise number of particles depends on geometry
  | \ ``offsetRadius``\ : additional space between spheres (by assuming a larger radius for packing)
  | \ ``verbose``\ : if > 0 some main parameters are printed
- | \ *output*\ :
  | [(point0, radius0), ...] a list of point-radius tuples containing the information of created particles

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `simulatorCouplingTwoMbs.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/simulatorCouplingTwoMbs.py>`_\  (TM)

