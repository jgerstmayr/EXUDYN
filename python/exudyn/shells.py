#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Shells and plates utility functions, e.g. for creation of plate / shell mesh.
#
# Author:   Johannes Gerstmayr
# Date:     2026-01-11 (created)
#
# Updated: 2026-03-24 (Michael Pieber): Added comments and helper functions for mixed symbolic/numeric mappings.
#
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    For a list of plot colors useful for matplotlib, see also utilities.PlotLineCode(...)
# Extended ShellMesh class with symbolic slope computation via vertexMapping.
# Copied from ANCFThinPlatePrecurved.py; intended to replace exudyn.shells.ShellMesh
# once merged into the Exudyn library.
#
# Differences vs. ANCFThinPlatePrecurved.py:
#   1. ApplyVertexMapping uses helper functions _eval/_diff instead of calling
#      .Evaluate()/.Diff() directly. This allows the vertexMapping to return
#      plain Python floats or numpy scalars (non-symbolic) for components that
#      do not depend on the symbolic variable.
#      ANCFThinPlatePrecurved.py assumes ALL components of vertexMapping's
#      return value are always exu.symbolic.Real objects.
#   2. vertexMapping docstring is more explicit: "must accept exu.symbolic.Real".
#   3. ApplyVertexMapping has an explanatory docstring.
#   4. Minor formatting/spacing differences; logic is otherwise identical.
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import exudyn as exu
import exudyn.itemInterface as eii
from exudyn.basicUtilities import Normalize


#**class: class for generation, representation of plate and shell meshes; creaton of Exudyn elements
class ShellMesh:
    #**classFunction: initialize rectangular shell mesh with geometry, discretization and physics parameters
    #**notes: x-axis is aligned with bottom (y=min) and top (y=max); y-axis is aligned with left (x=min) and right (x=max)
    #**input:
    #  vertices: list of four 3D vectors (numpy array or list), sorted [bottom-left, bottom-right, top-right, top-left];
    #            defining the reference positions of the corner nodes; if further transformations are added, use unit coordinates!
    #  numberOfElementsX: number of elements in x-direction
    #  numberOfElementsY: number of elements in y-direction
    #  youngsModulus: Young's modulus; used for calculation of membrane and bending stiffness
    #  poissonsRatio: Poisson's ratio for inplane shear deformation
    #  density: average density of plate/shell
    #  thickness: thickness of plate/shell
    #  massProportionalDamping: damping parameter which introduces damping proportional to distributed mass
    def __init__(self,
                 vertices=[[-1,-1,0],[ 1,-1,0],[ 1, 1,0],[-1, 1,0]],
                 numberOfElementsX=1,
                 numberOfElementsY=1,
                 youngsModulus=None,
                 poissonsRatio=None,
                 density=None,
                 thickness=None,
                 massProportionalDamping=0.,
                 thicknessAtNodes=None):

        # store mesh geometry corners (four 3D points, counter-clockwise)
        self.vertices = vertices
        # number of elements in each parametric direction
        self.numberOfElementsX = numberOfElementsX
        self.numberOfElementsY = numberOfElementsY
        # material parameters
        self.youngsModulus = youngsModulus
        self.poissonsRatio = poissonsRatio
        self.density = density
        self.thickness = thickness
        self.massProportionalDamping = massProportionalDamping
        # per-node thickness array, shape (nNodes,); None means constant physicsThickness.
        # Set before calling CreateANCFThinPlateElements, or pass as constructor argument.
        self.thicknessAtNodes = thicknessAtNodes

        # constitutive matrices, filled in CreateANCFThinPlateElements
        self.Dstrain = None     #computed when mesh is generated
        self.Dcurvature = None  #computed when mesh is generated

        # optional curved-geometry mapping F([x,y,z]) -> [x',y',z'];
        # must accept exu.symbolic.Real arguments so that automatic differentiation
        # can compute the tangent slopes dr/dx and dr/dy analytically.
        # ANCFThinPlatePrecurved.py has the same attribute but without this note.
        self.vertexMapping = None       #function F([x0,y0,z0]) -> [x1,y1,z1] transforming vertices; must accept exu.symbolic.Real

        # reserved for a future homogeneous-transformation feature (not yet implemented)
        self.vertexTransformation = None #homogeneous transformation (reserved for future use)

        # initialise all output lists to empty
        self.CreateReset()

    def CreateReset(self):
        """Reset all generated mesh data so CreateANCFThinPlateElements can be called again."""
        # boundary node numbers indexed by side name; 'all' = union of the four sides
        self.boundaryNodeNumbers = {'left':[], 'bottom':[], 'right':[], 'top':[], 'all':[]}
        # four corner node numbers in the same order as self.vertices
        self.vertexNodeNumbers = []     #sorted same as vertices
        # all (nx+1)*(ny+1) node numbers, row-major: inner loop over x, outer over y
        self.nodeNumbers = []           #sorted from left to right, bottom to top
        # all nx*ny element numbers, same row-major order
        self.elementNumbers = []        #sorted from left to right, bottom to top
        # reference positions AFTER mapping, one numpy(3,) per node
        self.nodeReferencePositions = []
        # dr/dx tangent slopes (unit vectors) per node — azimuthal direction for shells
        self.nodeSlopesX = []
        # dr/dy tangent slopes (unit vectors) per node — polar / width direction for shells
        self.nodeSlopesY = []

    def NumberOfNodes(self):
        """Return total number of nodes (including boundary nodes)."""
        return len(self.nodeReferencePositions)

    def SetVisualizationThicknessFactor(self, mbs, VthicknessFactor=1.0):
        """Set native visualization thickness scaling (VthicknessFactor) for all shell elements."""
        tf = float(VthicknessFactor)
        if tf < 0.0:
            raise ValueError("VthicknessFactor must be >= 0")
        for o in self.elementNumbers:
            mbs.SetObjectParameter(o, 'VthicknessFactor', tf)

    def ApplyVertexTransformation(self):
        """Apply a homogeneous vertex transformation. Reserved for future use; currently a no-op."""
        if self.vertexTransformation is None: return
        #reserved for future use

    def ApplyVertexMapping(self):
        """Apply vertexMapping using exu.symbolic.Real for exact analytical slope computation.

        For each node the unmapped parametric position [x, y, z] is wrapped into
        exu.symbolic.Real variables.  The user-supplied vertexMapping is then called
        with these symbolic values so that Exudyn's automatic differentiation can
        return exact partial derivatives dr/dx and dr/dy without finite differences.

        After this call:
          - nodeReferencePositions[i]  <- mapped 3D position (evaluated from symbolic)
          - nodeSlopesX[i]             <- unit tangent dr/dx  (analytically exact)
          - nodeSlopesY[i]             <- unit tangent dr/dy  (analytically exact)
          - nodeReferencePositionsUnmapped <- original parametric positions (saved for reference)

        Difference vs. ANCFThinPlatePrecurved.py:
          The original calls v[k].Evaluate() and v[k].Diff(x) directly, which raises
          AttributeError when a mapping component returns a plain float (e.g. a constant
          z=0 component).  Here, _eval/_diff helpers check for the attribute first and
          fall back to float() / 0.0, so mixed symbolic+numeric mappings work correctly.
        """
        if self.vertexMapping is None: return
        SymReal = exu.symbolic.Real

        # clear slopes; they will be recomputed from the mapping derivatives
        self.nodeSlopesX = []
        self.nodeSlopesY = []
        # save the unmapped (parametric) positions for potential later use
        self.nodeReferencePositionsUnmapped = []

        # --- helper: evaluate a symbolic Real or fall back to plain float ---
        # ANCFThinPlatePrecurved.py calls v[k].Evaluate() directly (no fallback).
        def _eval(val):
            return val.Evaluate() if hasattr(val, 'Evaluate') else float(val)

        # --- helper: differentiate a symbolic Real or return 0.0 for plain floats ---
        # ANCFThinPlatePrecurved.py calls v[k].Diff(var) directly (no fallback).
        def _diff(val, var):
            return val.Diff(var) if hasattr(val, 'Diff') else 0.0

        def _map_numeric(pos_values):
            mapped = self.vertexMapping([float(pos_values[0]), float(pos_values[1]), float(pos_values[2])])
            return np.array([float(mapped[0]), float(mapped[1]), float(mapped[2])], dtype=float)

        def _numeric_slope(pos_values, component):
            step = 1e-6 * max(1.0, abs(float(pos_values[component])))
            pos_minus = list(pos_values)
            pos_plus = list(pos_values)
            pos_minus[component] -= step
            pos_plus[component] += step
            return (_map_numeric(pos_plus) - _map_numeric(pos_minus)) / (2.0 * step)

        for i, posNumpy in enumerate(self.nodeReferencePositions):
            pos = posNumpy.tolist()
            # wrap the three parametric coordinates as named symbolic variables;
            # the name ("x","y","z") and the current float value are both stored
            # so that Evaluate() returns the numeric value and Diff() returns the derivative.
            x = SymReal("x", pos[0])   #use Exudyn symbolic for exact slope computation
            y = SymReal("y", pos[1])
            z = SymReal("z", pos[2])

            # evaluate the user mapping — returns a list of three symbolic (or plain) values
            v = self.vertexMapping([x, y, z])

            # evaluate mapped position numerically
            posEval = np.array([_eval(v[0]), _eval(v[1]), _eval(v[2])])

            # compute tangent along parametric x-direction: dr/dx = d(v)/d(x)
            slopeX   = np.array([_diff(v[0], x), _diff(v[1], x), _diff(v[2], x)])

            # compute tangent along parametric y-direction: dr/dy = d(v)/d(y)
            slopeY   = np.array([_diff(v[0], y), _diff(v[1], y), _diff(v[2], y)])

            if np.linalg.norm(slopeX) == 0.0:
                slopeX = _numeric_slope(pos, 0)
            if np.linalg.norm(slopeY) == 0.0:
                slopeY = _numeric_slope(pos, 1)

            # override parametric position with the physically mapped 3D position
            self.nodeReferencePositions[i] = posEval   #override with mapped position

            # normalize tangents to unit length (required by NodePointSlope12 convention)
            # NOTE: normalization means the reference slopes are unit tangent vectors,
            # NOT the actual partial derivatives dr/dx (which have magnitude |dr/dx|).
            # This is consistent with ANCFThinPlatePrecurved.py which also normalizes.
            self.nodeSlopesX.append(np.array(Normalize(slopeX)))
            self.nodeSlopesY.append(np.array(Normalize(slopeY)))

            # keep parametric (unmapped) position for debugging / seam-closure
            self.nodeReferencePositionsUnmapped.append(posNumpy)

    def CreateANCFThinPlateElements(self, mbs, VthicknessFactor=1.0):
        """Generate all Exudyn nodes (NodePointSlope12) and elements (ObjectANCFThinPlate)
        and add them to mbs.  Populates nodeNumbers, elementNumbers, boundaryNodeNumbers.
        """
        tf = float(VthicknessFactor)
        if tf < 0.0:
            raise ValueError("VthicknessFactor must be >= 0")

        # clear any previously generated data (allows calling this method more than once)
        self.CreateReset()

        #+++++++++
        # --- Step 1: generate node reference positions in parametric space ---
        # (nx+1)*(ny+1) nodes for nx*ny quadrilateral elements.
        # Positions are computed by bilinear interpolation of the four corner vertices.
        # Slopes are the edge-aligned unit tangents of the *unmapped* flat quad —
        # they will be overwritten by ApplyVertexMapping if a mapping is set.
        nx = self.numberOfElementsX
        ny = self.numberOfElementsY
        v = [np.array(self.vertices[0]),
             np.array(self.vertices[1]),
             np.array(self.vertices[2]),
             np.array(self.vertices[3])]

        for j in range(ny + 1):
            for i in range(nx + 1):
                # normalized parametric coordinates in [0,1]
                sx = i / nx
                sy = j / ny
                # bilinear interpolation: p = (1-sx)(1-sy)*v0 + sx(1-sy)*v1 + sx*sy*v2 + (1-sx)*sy*v3
                pos = (1-sx)*(1-sy)*v[0] + sx*(1-sy)*v[1] + sx*sy*v[2] + (1-sx)*sy*v[3]

                # flat-quad tangents: dr/dx ~ v1-v0, dr/dy ~ v3-v0 (constant for a parallelogram)
                slopeX = np.array(Normalize(v[1] - v[0]))
                slopeY = np.array(Normalize(v[3] - v[0]))
                self.nodeSlopesX.append(slopeX)
                self.nodeSlopesY.append(slopeY)
                self.nodeReferencePositions.append(pos)

        #+++++++++
        # --- Step 2: apply curved-geometry mapping (overrides positions and slopes) ---
        # If vertexMapping is set, positions become the mapped 3D coordinates and
        # slopes become the analytically-exact unit tangents dr/dx, dr/dy.
        self.ApplyVertexMapping()

        # apply optional homogeneous transformation (currently a no-op)
        self.ApplyVertexTransformation()

        #+++++++++
        # --- Step 3: create Exudyn NodePointSlope12 nodes ---
        # Each node stores: [r(3), dr/dx(3), dr/dy(3)] = 9 reference coordinates.
        for j in range(ny + 1):
            for i in range(nx + 1):
                index = i + j * (nx + 1)
                pos    = self.nodeReferencePositions[index]
                slopeX = self.nodeSlopesX[index]
                slopeY = self.nodeSlopesY[index]

                # NodePointSlope12: 9 reference coords = position + slopeX + slopeY
                node = eii.NodePointSlope12(referenceCoordinates=pos.tolist() + slopeX.tolist() + slopeY.tolist())
                nNum = mbs.AddNode(node)
                self.nodeNumbers.append(nNum)

                # classify boundary membership
                if i == 0:  self.boundaryNodeNumbers['left'].append(nNum)
                if i == nx: self.boundaryNodeNumbers['right'].append(nNum)
                if j == 0:  self.boundaryNodeNumbers['bottom'].append(nNum)
                if j == ny: self.boundaryNodeNumbers['top'].append(nNum)
                if i == 0 or i == nx or j == 0 or j == ny:
                    self.boundaryNodeNumbers['all'].append(nNum)

        # store the four corner node numbers in the same order as self.vertices
        self.vertexNodeNumbers = [self.boundaryNodeNumbers['bottom'][0],   # bottom-left
                                  self.boundaryNodeNumbers['bottom'][-1],  # bottom-right
                                  self.boundaryNodeNumbers['top'][-1],     # top-right
                                  self.boundaryNodeNumbers['top'][0]]      # top-left

        #+++++++++
        # --- Step 4: create ObjectANCFThinPlate elements ---
        # Each element spans four nodes (n0=bottom-left, n1=bottom-right,
        # n2=top-right, n3=top-left) in counter-clockwise order.
        for jy in range(ny):
            for ix in range(nx):
                # node indices follow the row-major layout: index = ix + iy*(nx+1)
                i0 = ix       + jy       * (nx + 1)
                i1 = (ix + 1) + jy       * (nx + 1)
                i2 = (ix + 1) + (jy + 1) * (nx + 1)
                i3 = ix       + (jy + 1) * (nx + 1)
                n0 = self.nodeNumbers[i0]  # bottom-left
                n1 = self.nodeNumbers[i1]  # bottom-right
                n2 = self.nodeNumbers[i2]  # top-right
                n3 = self.nodeNumbers[i3]  # top-left

                # --- per-element nodal thicknesses for variable thickness ---
                if self.thicknessAtNodes is not None:
                    hn = self.thicknessAtNodes
                    physicsThickness = [float(hn[i0]), float(hn[i1]), float(hn[i2]), float(hn[i3])]
                else:
                    physicsThickness = self.thickness

                #plate parameters:
                Em = self.youngsModulus
                nu = self.poissonsRatio

                thicknessList  = physicsThickness
                if isinstance(thicknessList, float):
                    thicknessList = [thicknessList] #make list for single float
        
                self.Dstrain = exu.Matrix3DList()
                self.Dcurvature = exu.Matrix3DList()
        
                for t in thicknessList:
                    # --- constitutive matrices (plane-stress Kirchhoff-Love) ---
                    # Membrane stiffness D_eps = E*t / (1-nu^2) * [[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]]
                    localDstrain = (Em * t / (1.0 - nu * nu)) * np.array([
                        [1.0, nu,  0.0],
                        [nu,  1.0, 0.0],
                        [0.0, 0.0, (1.0 - nu) / 2.0],
                    ], dtype=float)
            
                    # Bending stiffness D_kappa = E*t^3 / (12*(1-nu^2)) * [[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]]
                    localDcurvature = (Em * t**3 / (12.0 * (1.0 - nu * nu))) * np.array([
                        [1.0, nu,  0.0],
                        [nu,  1.0, 0.0],
                        [0.0, 0.0, (1.0 - nu) / 2.0],
                    ], dtype=float)
                    
                    self.Dstrain.Append(localDstrain)
                    self.Dcurvature.Append(localDcurvature)
                    


                
                oANCF = eii.ObjectANCFThinPlate(
                    nodeNumbers=[n0, n1, n2, n3],
                    physicsThickness=physicsThickness,
                    physicsStrainCoefficients=self.Dstrain,        # membrane stiffness
                    physicsCurvatureCoefficients=self.Dcurvature,  # bending stiffness
                    physicsDensity=self.density,
                    physicsMassProportionalDamping=self.massProportionalDamping,
                )
                o = mbs.AddObject(oANCF)
                self.elementNumbers.append(o)
