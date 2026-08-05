
.. _sec-module-shells:

Module: shells
==============

Shells and plates utility functions, e.g. for creation of plate / shell mesh.

- Author:    Johannes Gerstmayr 
- Date:      2026-01-11 (created) 
- Notes:     For a list of plot colors useful for matplotlib, see also utilities.PlotLineCode(...)  Extended ShellMesh class with symbolic slope computation via vertexMapping.  Copied from ANCFThinPlatePrecurved.py; intended to replace exudyn.shells.ShellMesh  once merged into the Exudyn library. 


.. _sec-module-shells-class-shellmesh:

CLASS ShellMesh (in module shells)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class for generation, representation of plate and shell meshes; creaton of Exudyn elements


.. _sec-shells-shellmesh---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/shells.py\#L51>`__\ (\ ``self``\ , \ ``vertices = [[-1,-1,0],[ 1,-1,0],[ 1, 1,0],[-1, 1,0]]``\ , \ ``numberOfElementsX = 1``\ , \ ``numberOfElementsY = 1``\ , \ ``youngsModulus = None``\ , \ ``poissonsRatio = None``\ , \ ``density = None``\ , \ ``thickness = None``\ , \ ``massProportionalDamping = 0.``\ , \ ``thicknessAtNodes = None``\ )

- | \ *classFunction*\ :
  | initialize rectangular shell mesh with geometry, discretization and physics parameters
- | \ *input*\ :
  | \ ``vertices``\ : list of four 3D vectors (numpy array or list), sorted [bottom-left, bottom-right, top-right, top-left];
  | defining the reference positions of the corner nodes; if further transformations are added, use unit coordinates!
  | \ ``numberOfElementsX``\ : number of elements in x-direction
  | \ ``numberOfElementsY``\ : number of elements in y-direction
  | \ ``youngsModulus``\ : Young's modulus; used for calculation of membrane and bending stiffness
  | \ ``poissonsRatio``\ : Poisson's ratio for inplane shear deformation
  | \ ``density``\ : average density of plate/shell
  | \ ``thickness``\ : thickness of plate/shell
  | \ ``massProportionalDamping``\ : damping parameter which introduces damping proportional to distributed mass
- | \ *notes*\ :
  | x-axis is aligned with bottom (y=min) and top (y=max); y-axis is aligned with left (x=min) and right (x=max)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFThinPlateTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFThinPlateTests.py>`_\  (TM)

