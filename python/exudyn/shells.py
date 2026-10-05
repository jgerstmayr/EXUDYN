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

from exudyn.misc.docmeta import docmeta
import numpy as np
import exudyn as exu
import exudyn.itemInterface as eii
from exudyn.basicUtilities import Normalize


#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'ShellMesh', 'SymSin', 'SymCos', 'MapSkewParallelogram', 'MapTrapezoid', 'MapCurvedEdge',
    'MapOutOfPlaneWarp', 'MapBezierStrip', 'MapConeFrustum', 'MapCylinder', 'MapHemisphericalShell',
    'MapToroidalPanel', 'ANCFThinPlateBuilder', 'AddNodeConstraints', 'AddSphericalJointToGround',
    'AddClampToGround', 'AddSlopeConformityConstraints', 'AddDistributedClampToEdge',
    'ApplyEdgeLoad', 'AddRotationalSpringDamper', 'AddEdgeSpringDamper',
    ]


class ShellMesh:
    """class for generation, representation of plate and shell meshes; creaton of Exudyn elements
    """
    def __init__(self,
                 vertices=[[-1,-1,0],[ 1,-1,0],[ 1, 1,0],[-1, 1,0]],
                 numberOfElementsX=1,
                 numberOfElementsY=1,
                 youngsModulus=None,
                 poissonsRatio=None,
                 density=None,
                 thickness=None,
                 massProportionalDamping=0.,
                 thicknessAtNodes=None,
                 thicknessFunction=None,
                 stiffnessProportionalDamping=0.):
        """initialize rectangular shell mesh with geometry, discretization and physics parameters

        Args:
            vertices: list of four 3D vectors (numpy array or list), sorted [bottom-left, bottom-right, top-right, top-left];
                      defining the reference positions of the corner nodes; if further transformations are added, use unit coordinates!
            numberOfElementsX: number of elements in x-direction
            numberOfElementsY: number of elements in y-direction
            youngsModulus: Young's modulus; used for calculation of membrane and bending stiffness
            poissonsRatio: Poisson's ratio for inplane shear deformation
            density: average density of plate/shell
            thickness: thickness of plate/shell
            massProportionalDamping: damping parameter which introduces damping proportional to distributed mass
            thicknessAtNodes: optional thickness at each node, in the order of the nodes; interpolated bilinearly in each element
            thicknessFunction: optional function f(x, y) of the thickness at the global x and y of a point, which must accept
                               exu.symbolic.Real; its value and gradients at the nodes give 12 thickness values per element;
                               correct for rectangular elements parallel to the x-y plane only
            stiffnessProportionalDamping: Kelvin-Voigt damping coefficient [s] of the membrane and the bending stiffness

        Note:
            x-axis is aligned with bottom (y=min) and top (y=max); y-axis is aligned with left (x=min) and right (x=max)
        """

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
        self.massProportionalDamping      = massProportionalDamping
        self.stiffnessProportionalDamping = stiffnessProportionalDamping
        # per-node thickness array, shape (nNodes,); None means constant thickness.
        # Set before calling CreateANCFThinPlateElements, or pass as constructor argument.
        self.thicknessAtNodes = thicknessAtNodes
        # optional callable thicknessFunction(x, y) -> scalar; must accept exu.symbolic.Real.
        # When set, enables ANCF-consistent cubic thickness interpolation (12 values per element):
        # [h, dh/dx, dh/dy] at each of the 4 corner nodes, computed via symbolic differentiation.
        # Takes priority over thicknessAtNodes and thickness if set.
        self.thicknessFunction = thicknessFunction

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

    @docmeta(public=False)
    def CreateReset(self):
        """Reset all generated mesh data so CreateANCFThinPlateElements can be called again."""
        # boundary node numbers indexed by side name; 'all' = union of the four sides
        self.boundaryNodeNumbers = {'left':[], 'bottom':[], 'right':[], 'top':[], 'all':[]}
        # per-node cubic thickness data: numpy array of shape (nNodes, 3) = [h, dh/dx, dh/dy];
        # populated by ComputeNodalThicknessGradients() when thicknessFunction is set
        self.thicknessGradientAtNodes = None
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

    @docmeta(public=False)
    def NumberOfNodes(self):
        """Return total number of nodes (including boundary nodes)."""
        return len(self.nodeReferencePositions)

    @docmeta(public=False)
    def SetVisualizationThicknessFactor(self, mbs, VthicknessFactor=1.0):
        """Set native visualization thickness scaling (VthicknessFactor) for all shell elements."""
        tf = float(VthicknessFactor)
        if tf < 0.0:
            raise ValueError("VthicknessFactor must be >= 0")
        for o in self.elementNumbers:
            mbs.SetObjectParameter(o, 'VthicknessFactor', tf)

    @docmeta(public=False)
    def ApplyVertexTransformation(self):
        """Apply a homogeneous vertex transformation. Reserved for future use; currently a no-op."""
        if self.vertexTransformation is None: return
        #reserved for future use

    @docmeta(public=False)
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

        A component of the mapping may also be a plain float or numpy scalar (e.g. a constant z=0).
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

    @docmeta(public=False)
    def ComputeNodalThicknessGradients(self):
        """Compute thickness and its physical x/y gradients at each node via symbolic differentiation.
        Requires self.thicknessFunction to be set to a callable f(x, y) that accepts
        exu.symbolic.Real values and returns the scalar thickness at position (x, y).
        The node's first two reference-position components (pos[0], pos[1]) are used as x and y.
        Populates self.thicknessGradientAtNodes as a numpy array of shape (nNodes, 3):
          column 0 = h        -- thickness value [m]
          column 1 = dh/dx    -- thickness gradient in local x-direction [m/m]
          column 2 = dh/dy    -- thickness gradient in local y-direction [m/m]
        These 3 values per node form the 12-component thickness vector per element that
        enables ANCF-consistent cubic thickness interpolation in ComputeThicknessAtPoint.
        """
        if self.thicknessFunction is None:
            return

        SymReal = exu.symbolic.Real
        numberOfNodes = len(self.nodeReferencePositions)
        self.thicknessGradientAtNodes = np.zeros((numberOfNodes, 3), dtype=float)

        # helper: evaluate a symbolic Real or fall back to plain float
        def _eval(val):
            return float(val.Evaluate()) if hasattr(val, 'Evaluate') else float(val)

        # helper: differentiate a symbolic Real or return 0.0 for plain floats
        def _diff(val, var):
            return float(val.Diff(var)) if hasattr(val, 'Diff') else 0.0

        for i, pos in enumerate(self.nodeReferencePositions):
            # wrap the node's x and y coordinates as named symbolic variables
            # so that Diff() returns exact partial derivatives of the thickness function
            xSymbolic = SymReal("x", float(pos[0]))
            ySymbolic = SymReal("y", float(pos[1]))

            # evaluate the thickness function symbolically at this node position
            hSymbolic = self.thicknessFunction(xSymbolic, ySymbolic)

            # extract thickness value and gradients from the symbolic result
            self.thicknessGradientAtNodes[i, 0] = _eval(hSymbolic)   # h(x, y)
            self.thicknessGradientAtNodes[i, 1] = _diff(hSymbolic, xSymbolic)  # dh/dx
            self.thicknessGradientAtNodes[i, 2] = _diff(hSymbolic, ySymbolic)  # dh/dy

    @docmeta(public=False)
    def CreateANCFThinPlateElements(self, mbs, VthicknessFactor=1.0, useReducedOrderIntegration=0):
        """Generate all Exudyn nodes (NodePointSlope12) and elements (ObjectANCFThinPlate)
        and add them to mbs.  Populates nodeNumbers, elementNumbers, boundaryNodeNumbers.
        Input:  mbs                         -- Exudyn multibody system
                VthicknessFactor            -- optional uniform thickness scaling factor (>= 0)
                useReducedOrderIntegration  -- integration mode passed to ObjectANCFThinPlate:
                                               0 = full Gauss (order 9, 5x5 pts), safe baseline
                                               1 = Lobatto SRI (Ntarladima/Pieber/Gerstmayr 2023):
                                                   membrane: Lobatto order 3 (3x3 pts at {-1,0,+1}),
                                                   bending:  Gauss order 3 (2x2 pts at {+-1/sqrt(3)});
                                                   eliminates membrane locking via disjoint point sets
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
        # Slopes are the exact unit tangents of the bilinear corner map at each node
        # (edge-aligned and constant only for parallelograms) —
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

                # CHANGED: exact per-node tangents of the bilinear corner map (Claude/MP, 2026-06-10):
                #   dp/dsx = (1-sy)*(v1-v0) + sy*(v2-v3),  dp/dsy = (1-sx)*(v3-v0) + sx*(v2-v1)
                # For parallelograms this equals the previous constant tangents v1-v0, v3-v0;
                # for trapezoidal/irregular vertex quads the tangent varies with the node position,
                # making the slope field consistent with the bilinear geometry.
                # OLD: slopeX = np.array(Normalize(v[1] - v[0]))
                #      slopeY = np.array(Normalize(v[3] - v[0]))
                slopeX = np.array(Normalize((1-sy)*(v[1] - v[0]) + sy*(v[2] - v[3])))
                slopeY = np.array(Normalize((1-sx)*(v[3] - v[0]) + sx*(v[2] - v[1])))
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
        # --- Step 2b: compute cubic thickness gradients if thicknessFunction is set ---
        # Evaluates thicknessFunction at every node position symbolically to obtain
        # [h, dh/dx, dh/dy] per node, which enables the 12-value ANCF-consistent
        # cubic interpolation in ComputeThicknessAtPoint.
        self.ComputeNodalThicknessGradients()

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

                # --- per-element thickness and constitutive matrices ---
                # Three modes, in priority order:
                #   thicknessGradientAtNodes set (from thicknessFunction) -> 12-value cubic ANCF scheme
                #   thicknessAtNodes set                                   -> 4-value bilinear scheme
                #   otherwise                                              -> constant thickness
                if self.thicknessGradientAtNodes is not None:
                    # ANCF-consistent cubic interpolation: thickness has 12 values
                    # [h_i, dh/dx_i, dh/dy_i] for i = 0..3 (one triple per element corner node)
                    g = self.thicknessGradientAtNodes
                    thickness = [
                        float(g[i0, 0]), float(g[i0, 1]), float(g[i0, 2]),
                        float(g[i1, 0]), float(g[i1, 1]), float(g[i1, 2]),
                        float(g[i2, 0]), float(g[i2, 1]), float(g[i2, 2]),
                        float(g[i3, 0]), float(g[i3, 1]), float(g[i3, 2]),
                    ]
                    # constitutive matrices use only the 4 nodal thickness values (no gradients)
                    constitutiveThicknesses = [float(g[i0, 0]), float(g[i1, 0]),
                                               float(g[i2, 0]), float(g[i3, 0])]
                elif self.thicknessAtNodes is not None:
                    # bilinear scheme: 4 nodal thickness values
                    hn = self.thicknessAtNodes
                    thickness = [float(hn[i0]), float(hn[i1]), float(hn[i2]), float(hn[i3])]
                    constitutiveThicknesses = list(thickness)
                else:
                    # constant thickness
                    thickness = self.thickness
                    constitutiveThicknesses = [float(self.thickness)]

                # --- constitutive matrices (plane-stress Kirchhoff-Love) ---
                Em = self.youngsModulus
                nu = self.poissonsRatio
                self.Dstrain = exu.Matrix3DList()
                self.Dcurvature = exu.Matrix3DList()

                for t in constitutiveThicknesses:
                    # membrane stiffness: D_eps = E*h / (1-nu^2) * [[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]]
                    localDstrain = (Em * t / (1.0 - nu * nu)) * np.array([
                        [1.0, nu,  0.0],
                        [nu,  1.0, 0.0],
                        [0.0, 0.0, (1.0 - nu) / 2.0],
                    ], dtype=float)
                    # bending stiffness: D_kappa = E*h^3 / (12*(1-nu^2)) * [[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]]
                    localDcurvature = (Em * t**3 / (12.0 * (1.0 - nu * nu))) * np.array([
                        [1.0, nu,  0.0],
                        [nu,  1.0, 0.0],
                        [0.0, 0.0, (1.0 - nu) / 2.0],
                    ], dtype=float)
                    self.Dstrain.Append(localDstrain)
                    self.Dcurvature.Append(localDcurvature)
                    


                
                oANCF = eii.ObjectANCFThinPlate(
                    nodeNumbers=[n0, n1, n2, n3],
                    thickness=thickness,
                    strainCoefficients=self.Dstrain,        # membrane stiffness
                    curvatureCoefficients=self.Dcurvature,  # bending stiffness
                    density=self.density,
                    massProportionalDamping=self.massProportionalDamping,
                    stiffnessProportionalDamping=self.stiffnessProportionalDamping,
                    useReducedOrderIntegration=useReducedOrderIntegration,
                )
                o = mbs.AddObject(oANCF)
                self.elementNumbers.append(o)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Geometry builder utilities
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# ── tolerance constants ──
_zeroTol   = 1e-14   # near-zero threshold for unit vectors and normals
_degenTol  = 1e-12   # mapping degeneracy threshold (|sx x sy| ~ 0)
_aspectTol = 1e-6    # minimum |sx x sy| / max(|sx x sy|) for ill-conditioned check

# ── vector / matrix helpers ──────────────────────────────────────────────────

def _UnitVector(v):
    """Return the unit vector of v; raises ValueError if v is near-zero."""
    v = np.array(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < _zeroTol:
        raise ValueError("vector must be non-zero")
    return v / n


def _RotAxisAngle(axis, angle):
    """Rodrigues rotation matrix: rotate by `angle` (radians) around `axis`."""
    a = _UnitVector(axis)
    x, y, z = float(a[0]), float(a[1]), float(a[2])
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    C = 1.0 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s,  x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,    y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s,  c + z*z*C  ],
    ], dtype=float)


def _RotZ(angle):
    """3x3 rotation matrix about the global z-axis."""
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


# ── symbolic math helpers ─────────────────────────────────────────────────────
# These allow mapping functions to work both numerically and with exu.symbolic.Real,
# which is required by ShellMesh.ApplyVertexMapping for automatic differentiation.

def SymSin(x):
    """sin() compatible with both plain float and exu.symbolic.Real."""
    if hasattr(x, 'Diff'):
        return exu.symbolic.sin(x)
    return np.sin(float(x))


def SymCos(x):
    """cos() compatible with both plain float and exu.symbolic.Real."""
    if hasattr(x, 'Diff'):
        return exu.symbolic.cos(x)
    return np.cos(float(x))


# ── geometry mapping helpers ──────────────────────────────────────────────────
# Each function returns a mapLocalPosition(x, y) callable for use with ANCFThinPlateBuilder.
# All mappings must accept exu.symbolic.Real arguments (use SymSin/SymCos, not np.sin/cos).

def MapSkewParallelogram(skewY=0.0, skewX=0.0):
    """Skew a rectangle into a parallelogram.
    skewY: adds x += skewY*y  (shear x with y).
    skewX: adds y += skewX*x  (shear y with x)."""
    def f(x, y):
        return [x + skewY * y, y + skewX * x, 0.0]
    return f


def MapTrapezoid(Lx, Ly, topScale=1.0, aboutMidline=True):
    """Trapezoid by scaling x-span linearly with y.
    topScale<1: narrower at top; topScale>1: wider at top.
    aboutMidline=True keeps the plate centreline fixed."""
    Lx = float(Lx); Ly = float(Ly)
    if Ly <= 0:
        raise ValueError("Ly must be > 0")
    topScale = float(topScale)
    def f(x, y):
        s  = 1.0 + (topScale - 1.0) * (y / Ly)
        x2 = s * x
        if aboutMidline:
            x2 += 0.5 * Lx * (1.0 - s)
        return [x2, y, 0.0]
    return f


def MapCurvedEdge(Lx, Ly, amp=0.0, mode='top', shape='sin', aboutMidline=True):
    """Curve one edge by an x-shift that varies with y (and x).
    mode:  'top' or 'bottom' — where the curvature reaches full amplitude.
    shape: 'sin' (sinusoidal in x) or 'parabola'.
    amp:   maximum x-shift [m]."""
    Lx = float(Lx); Ly = float(Ly); amp = float(amp)
    if Ly <= 0:
        raise ValueError("Ly must be > 0")
    def waviness(x):
        if shape == 'sin':
            return SymSin(np.pi * x / Lx)
        if shape == 'parabola':
            xi = x / Lx
            return 4.0 * xi * (1.0 - xi)
        raise ValueError("shape must be 'sin' or 'parabola'")
    def f(x, y):
        g  = (y / Ly) if mode == 'top' else (1.0 - y / Ly)
        dx = amp * g * waviness(x)
        x2 = x + dx
        if aboutMidline:
            x2 -= 0.5 * amp * g * (2.0 / np.pi)
        return [x2, y, 0.0]
    return f


def MapOutOfPlaneWarp(Lx, Ly, amp=0.0):
    """Initial out-of-plane warp vanishing at all four boundaries:
    z = amp * sin(pi*x/Lx) * sin(pi*y/Ly)."""
    Lx = float(Lx); Ly = float(Ly); amp = float(amp)
    if Lx <= 0 or Ly <= 0:
        raise ValueError("Lx and Ly must be > 0")
    def f(x, y):
        z = amp * SymSin(np.pi * x / Lx) * SymSin(np.pi * y / Ly)
        return [x, y, z]
    return f


def MapBezierStrip(P0, P1, P2, P3, Lx, Ly, up=(0, 0, 1), useArcLength=True, nArc=400):
    """Map a plate onto a cubic Bezier centerline strip.
    x in [0, Lx] follows the curve (optionally by arc-length).
    y in [0, Ly] offsets across the strip width."""
    P0 = np.array(P0, dtype=float).reshape(3)
    P1 = np.array(P1, dtype=float).reshape(3)
    P2 = np.array(P2, dtype=float).reshape(3)
    P3 = np.array(P3, dtype=float).reshape(3)
    Lx = float(Lx); Ly = float(Ly)
    if Lx <= 0 or Ly <= 0:
        raise ValueError("Lx and Ly must be > 0")
    up = _UnitVector(up)

    def C(t):
        u = 1.0 - t
        return (u**3)*P0 + 3.0*(u**2)*t*P1 + 3.0*u*(t**2)*P2 + (t**3)*P3

    def dCdT(t):
        u = 1.0 - t
        return 3.0*(u**2)*(P1-P0) + 6.0*u*t*(P2-P1) + 3.0*(t**2)*(P3-P2)

    if useArcLength:
        ts = np.linspace(0.0, 1.0, int(nArc))
        v  = np.array([np.linalg.norm(dCdT(t)) for t in ts], dtype=float)
        s  = np.zeros_like(ts)
        for i in range(1, len(ts)):
            s[i] = s[i-1] + 0.5*(ts[i]-ts[i-1])*(v[i]+v[i-1])
        sTot = float(s[-1])
        if sTot <= _zeroTol:
            raise ValueError("Bezier curve arc-length is ~0 (degenerate control points)")
        def tOfX(x):
            sTarget = (float(x) / Lx) * sTot
            if sTarget <= 0: return 0.0
            if sTarget >= sTot: return 1.0
            j  = max(1, min(int(np.searchsorted(s, sTarget)), len(ts)-1))
            s0, s1 = float(s[j-1]), float(s[j])
            t0, t1 = float(ts[j-1]), float(ts[j])
            a = 0.0 if abs(s1-s0) < _zeroTol else (sTarget-s0)/(s1-s0)
            return t0 + a*(t1-t0)
    else:
        def tOfX(x):
            return float(x) / Lx

    def f(x, y):
        t  = tOfX(x)
        p  = C(t)
        T  = dCdT(t)
        nT = float(np.linalg.norm(T))
        if nT < _zeroTol:
            t2 = min(1.0, max(0.0, t+1e-6)); T = dCdT(t2); nT = float(np.linalg.norm(T))
        if nT < _zeroTol:
            T = np.array([1.0, 0.0, 0.0]); nT = 1.0
        T  = T / nT
        N  = np.cross(up, T); nN = float(np.linalg.norm(N))
        if nN < _zeroTol:
            N = np.cross(np.array([0.0,1.0,0.0]), T); nN = float(np.linalg.norm(N))
        if nN < _zeroTol:
            N = np.array([0.0,1.0,0.0]); nN = 1.0
        N  = N / nN
        yc = float(y) - 0.5*Ly
        return (p + yc*N).tolist()
    return f


def MapConeFrustum(Lx, Ly, r0, r1, phiDeg=20.0, thetaCenterDeg=0.0,
                   centered=True, yMode="arclength"):
    """Map a rectangle onto a conical frustum mantle.
    x in [0,Lx] along the axis; y in [0,Ly] around the circumference.
    r0, r1: radii at x=0 and x=Lx. phiDeg: angular extent [deg]."""
    Lx=float(Lx); Ly=float(Ly); r0=float(r0); r1=float(r1)
    if Lx<=0 or Ly<=0: raise ValueError("Lx and Ly must be > 0")
    if r0<=0 or r1<=0: raise ValueError("r0 and r1 must be > 0")
    phi         = float(np.deg2rad(phiDeg))
    thetaCenter = float(np.deg2rad(thetaCenterDeg))
    if abs(phi) < _zeroTol: raise ValueError("phiDeg must be non-zero")
    yMode = str(yMode).lower()
    if yMode not in ("angle","arclength"): raise ValueError("yMode must be 'angle' or 'arclength'")
    def rOfX(x):
        return r0 + (r1-r0)*(float(x)/Lx)
    def thetaOfXy(x, y):
        if yMode == "angle":
            return thetaCenter + ((float(y)/Ly - 0.5)*phi if centered else (float(y)/Ly)*phi)
        rx = rOfX(x); yc = 0.5*Ly if centered else 0.0
        return thetaCenter + (float(y)-yc)/rx
    def f(x, y):
        r  = rOfX(x); th = thetaOfXy(x,y)
        return [float(x), float(r*np.cos(th)), float(r*np.sin(th))]
    return f


def MapCylinder(Lx, Ly, R, phiDeg=90.0, thetaCenterDeg=0.0, centered=False, yMode='arclength'):
    """Map a rectangle onto a cylindrical mantle (special case of MapConeFrustum with r0=r1=R)."""
    return MapConeFrustum(Lx=Lx, Ly=Ly, r0=R, r1=R,
                          phiDeg=phiDeg, thetaCenterDeg=thetaCenterDeg,
                          centered=centered, yMode=yMode)


def MapHemisphericalShell(Lx, Ly, R, alpha, center=(0.0,0.0,0.0), thetaOffset=0.0,
                           thetaRange=None):
    """Map a rectangle onto a hemispherical shell with a circular apex cutout.
    R: radius. alpha: cutout angle from +z axis [rad].
    x maps to azimuthal angle; y maps from cutout edge (y=0) to equator (y=Ly)."""
    Lx=float(Lx); Ly=float(Ly); R=float(R); alpha=float(alpha)
    thetaOffset=float(thetaOffset)
    center=np.array(center,dtype=float).reshape(3)
    if thetaRange is None: thetaRange=2.0*np.pi
    thetaRange=float(thetaRange)
    if Lx<=0 or Ly<=0: raise ValueError("Lx and Ly must be > 0")
    if R<=0:            raise ValueError("R must be > 0")
    if alpha<0 or alpha>=np.pi/2: raise ValueError("alpha must be in [0, pi/2)")
    phiBase=np.pi/2.0; phiCutout=alpha
    def f(x, y):
        theta = thetaOffset + (thetaRange/Lx)*x
        phi   = phiCutout + ((phiBase-phiCutout)/Ly)*y
        X = R*SymSin(phi)*SymCos(theta)
        Y = R*SymSin(phi)*SymSin(theta)
        Z = R*SymCos(phi)
        return [center[0]+X, center[1]+Y, center[2]+Z]
    return f


def MapToroidalPanel(Lx, Ly, r1=1.5, r2=0.5):
    """Map a rectangle onto a toroidal panel.
    x -> meridional angle q1 in [-pi/2, pi/2]; y -> azimuthal angle q2 in [0, pi/2]."""
    Lx=float(Lx); Ly=float(Ly); r1=float(r1); r2=float(r2)
    q1Min=-np.pi/2.0; q1Range=np.pi; q2Min=0.0; q2Range=np.pi/2.0
    def f(x, y):
        q1  = q1Min + (q1Range/Lx)*x
        q2  = q2Min + (q2Range/Ly)*y
        rho = r1 + r2*SymCos(q1)
        return [rho*SymCos(q2), -rho*SymSin(q2), r2*SymSin(q1)]
    return f


# ── main builder class ────────────────────────────────────────────────────────

class ANCFThinPlateBuilder:
    """Build an ANCF thin plate mesh from a high-level geometric description.

    Wraps ShellMesh with rotation, placement, and optional curved-geometry support.
    All geometry is described in local plate coordinates (x: length, y: width, z: thickness).

    Rotation: provide ONE of rotationMatrix, rotationAxis+rotationAngle, or rotationZ.
    Curved geometry: set mapLocalPosition(x, y) -> [x', y', z'] in local coordinates;
                     must accept exu.symbolic.Real arguments (use SymSin/SymCos).
    Variable thickness: set thicknessField(x, y) -> float (local coordinates).

    Returns from Build(): dict with keys nodes, elements, nodeRefs9, nodeId,
                          edgeNodes, edgeNodeNumbers, elementThicknesses.
    """

    def __init__(self,
                 mbs=None,
                 origin=(0.0, 0.0, 0.0),
                 Lx=1.0, Ly=1.0,
                 nx=1, ny=1,
                 rotationMatrix=None,
                 rotationAxis=None,
                 rotationAngle=0.0,
                 rotationZ=0.0,
                 mapLocalPosition=None,
                 validateMapping=True,
                 thicknessField=None,
                 E=2.1e11, nu=0.3, rho=7800,
                 thickness=1e-3,
                 massProportionalDamping=0.0,
                 stiffnessProportionalDamping=0.0,
                 massProportionalLoad=[0, 0, 0],
                 useReducedOrderIntegration=1):
        self.origin = np.array(origin, dtype=float).reshape(3)
        self.Lx = float(Lx); self.Ly = float(Ly)
        self.nx = int(nx);   self.ny = int(ny)
        self.rotationMatrix = rotationMatrix
        self.rotationAxis   = rotationAxis
        self.rotationAngle  = float(rotationAngle)
        self.rotationZ      = float(rotationZ)
        self.mapLocalPosition      = mapLocalPosition
        self.validateMapping       = bool(validateMapping)
        self.thicknessField        = thicknessField
        self.E   = float(E); self.nu = float(nu); self.rho = float(rho)
        self.thickness             = float(thickness)
        self.thicknessAtNodes      = None
        self.massProportionalDamping      = float(massProportionalDamping)
        self.stiffnessProportionalDamping = float(stiffnessProportionalDamping)
        self.massProportionalLoad         = list(massProportionalLoad)
        self.useReducedOrderIntegration = int(useReducedOrderIntegration)
        if self.Lx <= 0 or self.Ly <= 0:
            raise ValueError("Lx and Ly must be > 0")
        if self.nx < 1 or self.ny < 1:
            raise ValueError("nx and ny must be >= 1")
        # auto-build if mbs is provided — mirrors GenerateStraightLineANCFCable2D API
        self.built = self.Build(mbs) if mbs is not None else None

    def Rotation(self):
        """Return 3x3 rotation matrix from whichever rotation spec was provided."""
        if self.rotationMatrix is not None:
            R = np.array(self.rotationMatrix, dtype=float)
            if R.shape != (3,3): raise ValueError("rotationMatrix must be 3x3")
            return R
        if self.rotationAxis is not None:
            return _RotAxisAngle(self.rotationAxis, self.rotationAngle)
        return _RotZ(self.rotationZ)

    def SetVisualizationThicknessFactor(self, mbs, builtOrElements, VthicknessFactor=1.0):
        """Set VthicknessFactor on all plate elements (accepts Build() dict or element list)."""
        tf = float(VthicknessFactor)
        if tf < 0.0: raise ValueError("VthicknessFactor must be >= 0")
        elements = builtOrElements['elements'] if isinstance(builtOrElements, dict) else builtOrElements
        for elem in elements:
            mbs.SetObjectParameter(elem, 'VthicknessFactor', tf)

    def Build(self, mbs):
        """Build nodes and ObjectANCFThinPlate elements and add them to mbs.

        Returns dict:
          nodes, elements, nodeRefs9, nodeRefs12 (None),
          nodeId(iy,ix), edgeNodes, edgeNodeNumbers, elementThicknesses.
        """
        R  = self.Rotation()
        p0 = self.origin

        if self.mapLocalPosition is None:
            # flat plate: rotate corners directly
            vertices = [
                p0.tolist(),
                (p0 + R @ np.array([self.Lx, 0.0, 0.0])).tolist(),
                (p0 + R @ np.array([self.Lx, self.Ly, 0.0])).tolist(),
                (p0 + R @ np.array([0.0, self.Ly, 0.0])).tolist(),
            ]
            vertexMapping = None
        else:
            vertices = [[0.0,0.0,0.0],[self.Lx,0.0,0.0],
                        [self.Lx,self.Ly,0.0],[0.0,self.Ly,0.0]]
            _mapLocal = self.mapLocalPosition
            _p0 = p0.copy(); _R = R.copy()
            def vertexMapping(v):
                x, y = v[0], v[1]    #the mapping of the builder depends on x and y only
                local = _mapLocal(x, y)
                return [
                    _p0[0] + _R[0,0]*local[0] + _R[0,1]*local[1] + _R[0,2]*local[2],
                    _p0[1] + _R[1,0]*local[0] + _R[1,1]*local[1] + _R[1,2]*local[2],
                    _p0[2] + _R[2,0]*local[0] + _R[2,1]*local[1] + _R[2,2]*local[2],
                ]

        self.shellMesh = ShellMesh(
            vertices=vertices,
            numberOfElementsX=self.nx,
            numberOfElementsY=self.ny,
            youngsModulus=self.E,
            poissonsRatio=self.nu,
            density=self.rho,
            thickness=self.thickness,
            massProportionalDamping=self.massProportionalDamping,
            stiffnessProportionalDamping=self.stiffnessProportionalDamping,
        )
        self.shellMesh.vertexMapping = vertexMapping

        if self.thicknessField is not None:
            # evaluate thicknessField at each node's parametric position
            thicknessAtNodes = np.zeros((self.ny+1)*(self.nx+1), dtype=float)
            for iy in range(self.ny+1):
                for ix in range(self.nx+1):
                    thicknessAtNodes[iy*(self.nx+1)+ix] = float(
                        self.thicknessField(ix/self.nx*self.Lx, iy/self.ny*self.Ly))
            self.shellMesh.thicknessAtNodes = thicknessAtNodes

        self.shellMesh.CreateANCFThinPlateElements(
            mbs, useReducedOrderIntegration=self.useReducedOrderIntegration)

        nodes    = self.shellMesh.nodeNumbers
        elements = self.shellMesh.elementNumbers

        if self.thicknessField is not None:
            nt = self.shellMesh.thicknessAtNodes
            elementThicknesses = []
            for iy in range(self.ny):
                for ix in range(self.nx):
                    i0 = iy*(self.nx+1)+ix;   i1 = iy*(self.nx+1)+(ix+1)
                    i2 = (iy+1)*(self.nx+1)+(ix+1); i3 = (iy+1)*(self.nx+1)+ix
                    elementThicknesses.append(float(nt[i0]+nt[i1]+nt[i2]+nt[i3])/4.0)
        else:
            elementThicknesses = [self.thickness]*len(elements)

        nodeRefs9 = np.array([
            np.hstack([self.shellMesh.nodeReferencePositions[k],
                       self.shellMesh.nodeSlopesX[k],
                       self.shellMesh.nodeSlopesY[k]])
            for k in range(len(nodes))
        ], dtype=float)

        def nodeId(iy, ix):
            return iy*(self.nx+1)+ix

        if self.validateMapping:
            areas = [float(np.linalg.norm(np.cross(nodeRefs9[k,3:6], nodeRefs9[k,6:9])))
                     for k in range(nodeRefs9.shape[0])]
            aMin = float(np.min(areas)) if areas else 0.0
            aMax = float(np.max(areas)) if areas else 0.0
            flips = sum(1 for a in areas if a < _degenTol)
            if flips > 0 or (aMax > 0 and aMin/aMax < _aspectTol):
                print(f"WARNING(ANCFThinPlateBuilder): distorted reference mapping "
                      f"min|sx x sy|={aMin:.3e}, max={aMax:.3e}, nearZeroCount={flips}")

        edgeNodes = {
            'left':   [nodeId(iy, 0)        for iy in range(self.ny+1)],
            'right':  [nodeId(iy, self.nx)  for iy in range(self.ny+1)],
            'bottom': [nodeId(0,  ix)        for ix in range(self.nx+1)],
            'top':    [nodeId(self.ny, ix)   for ix in range(self.nx+1)],
        }
        edgeNodeNumbers = {k: [nodes[i] for i in v] for k,v in edgeNodes.items()}

        # distributed body force (e.g. gravity): same pattern as GenerateStraightBeam
        gravityLoads = []
        if np.linalg.norm(self.massProportionalLoad) != 0:
            for elem in elements:
                mMass = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=elem))
                lGrav = mbs.AddLoad(eii.Gravity(
                    markerNumber=mMass,
                    loadVector=self.massProportionalLoad,
                ))
                gravityLoads.append(lGrav)

        return {
            'nodes':              nodes,
            'elements':           elements,
            'nodeRefs9':          nodeRefs9,
            'nodeRefs12':         None,
            'nodeId':             nodeId,
            'edgeNodes':          edgeNodes,
            'edgeNodeNumbers':    edgeNodeNumbers,
            'elementThicknesses': elementThicknesses,
            'gravityLoads':       gravityLoads,
        }


# ── joint / constraint helpers ────────────────────────────────────────────────
# ANCF thin plate node DOFs:
#   0,1,2 = position r;  3,4,5 = x-slope sx;  6,7,8 = y-slope sy

def AddNodeConstraints(mbs, nodeNumber, dofs, refNodeNumber=None):
    """Constrain selected DOFs (0-8) of an ANCF plate node, to its reference value or to the same DOF of another node.

    Mirrors the GenerateStraightBeam pattern: one NodePointGround placed at the
    node's reference position, one shared ground marker, one CoordinateConstraint
    per DOF.  MarkerNodeCoordinate returns ODE2 displacement (without reference
    values), so default offset=0 pins each displacement to zero = node stays at
    its reference configuration.

    Args:
        mbs: the MainSystem
        nodeNumber: the NodePointSlope12 node
        dofs: the coordinates of the node to constrain, 0-2 position, 3-5 slope x, 6-8 slope y
        refNodeNumber: None - each selected coordinate keeps its reference value (ground); a node number - each
                       selected coordinate equals the same coordinate of this node (C1 node-to-node connection)

    Returns:
        list of CoordinateConstraint object indices.
    """
    from exudyn.utilities import NodePointGround, MarkerNodeCoordinate, CoordinateConstraint
    constraints = []
    if refNodeNumber is None:
        # place ground at the node's actual reference position (mirrors GenerateStraightBeam)
        refCoords = mbs.GetNode(nodeNumber)['referenceCoordinates']
        refPos    = list(refCoords[:3])
        groundNode = mbs.AddNode(NodePointGround(referenceCoordinates=refPos))
        mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=groundNode, coordinate=0))
        for dof in dofs:
            mN = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeNumber, coordinate=int(dof)))
            constraints.append(mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround, mN])))
    else:
        # couple dof-by-dof: q[nodeNumber][dof] == q[refNodeNumber][dof]
        for dof in dofs:
            mRef = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=refNodeNumber, coordinate=int(dof)))
            mN   = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeNumber,    coordinate=int(dof)))
            constraints.append(mbs.AddObject(CoordinateConstraint(markerNumbers=[mRef, mN])))
    return constraints


def AddSphericalJointToGround(mbs, nodeNumber):
    """Pin an ANCF node to ground: fix translations (DOF 0,1,2), slopes free."""
    return AddNodeConstraints(mbs, nodeNumber, dofs=[0,1,2])


def AddClampToGround(mbs, nodeNumber, fixedConstraints=None):
    """Clamp an ANCF plate node to ground using a 9-element binary constraint vector.

    fixedConstraints: list/array of 9 values (0 or 1), one per DOF in the order
        [x, y, z,  sx_x, sx_y, sx_z,  sy_x, sy_y, sy_z].
        A value of 1 fixes that DOF to its reference value; 0 leaves it free.
        Mirrors the fixedConstraintsNode0 convention of GenerateStraightLineANCFCable2D.

        Default (None): fix all 9 DOFs -- equivalent to [1,1,1, 1,1,1, 1,1,1].

        For a flat plate in the xy-plane (clamped left edge), the physically
        consistent choice that mirrors the 2D cable [1,1,0,1] convention is
        [1,1,1, 0,1,1, 1,0,1]: positions fixed, transverse slope components
        fixed (zero rotation), axial slope components (sx_x and sy_y, both = 1
        in reference) left free to allow boundary stretching.
    """
    if fixedConstraints is None:
        dofs = range(9)
    else:
        dofs = [j for j, flag in enumerate(fixedConstraints) if flag != 0]
    return AddNodeConstraints(mbs, nodeNumber, dofs=dofs)


def _ThinPlateShapeFunctionDerivatives(xi, eta, scaleX, scaleY):
    """First derivatives (d/dxi, d/deta) of the 12 ANCFThinPlate shape functions.

    Transcribed from CObjectANCFThinPlate::ComputeShapeFunctions_xy (C++), including
    the per-node slope scaling (slopesScalingX/Y parameters of the element).
    Node order: n0=(-1,-1), n1=(+1,-1), n2=(+1,+1), n3=(-1,+1); per node the SF
    order is [position, slopeX, slopeY].

    Returns (sf_xi, sf_eta): two numpy arrays of length 12.
    """
    xi2 = xi*xi;   xi3 = xi2*xi
    eta2 = eta*eta; eta3 = eta2*eta
    Lx1 = 0.5*(1.0 - xi);  Lx2 = 0.5*(1.0 + xi)
    Ly1 = 0.5*(1.0 - eta); Ly2 = 0.5*(1.0 + eta)
    hx_pos_1 = 0.25*(2.0 - 3.0*xi + xi3);   hx_pos_2 = 0.25*(2.0 + 3.0*xi - xi3)
    hy_pos_1 = 0.25*(2.0 - 3.0*eta + eta3); hy_pos_2 = 0.25*(2.0 + 3.0*eta - eta3)
    hx_slope_1 = 0.125*(1.0 - xi - xi2 + xi3);   hx_slope_2 = 0.125*(-1.0 - xi + xi2 + xi3)
    hy_slope_1 = 0.125*(1.0 - eta - eta2 + eta3); hy_slope_2 = 0.125*(-1.0 - eta + eta2 + eta3)
    dLx1 = -0.5; dLx2 = 0.5; dLy1 = -0.5; dLy2 = 0.5
    d_hx_pos_1 = 0.75*(xi2 - 1.0);  d_hx_pos_2 = 0.75*(1.0 - xi2)
    d_hy_pos_1 = 0.75*(eta2 - 1.0); d_hy_pos_2 = 0.75*(1.0 - eta2)
    d_hx_slope_1 = 0.125*(-1.0 - 2.0*xi + 3.0*xi2);   d_hx_slope_2 = 0.125*(-1.0 + 2.0*xi + 3.0*xi2)
    d_hy_slope_1 = 0.125*(-1.0 - 2.0*eta + 3.0*eta2); d_hy_slope_2 = 0.125*(-1.0 + 2.0*eta + 3.0*eta2)

    sf_xi  = np.zeros(12)
    sf_eta = np.zeros(12)
    # node 0 (-1,-1)
    sf_xi[0]  = d_hx_pos_1*Ly1 + hy_pos_1*dLx1 - dLx1*Ly1
    sf_eta[0] = hx_pos_1*dLy1 + d_hy_pos_1*Lx1 - Lx1*dLy1
    sf_xi[1]  = d_hx_slope_1*Ly1;  sf_eta[1]  = hx_slope_1*dLy1
    sf_xi[2]  = hy_slope_1*dLx1;   sf_eta[2]  = d_hy_slope_1*Lx1
    # node 1 (+1,-1)
    sf_xi[3]  = d_hx_pos_2*Ly1 + hy_pos_1*dLx2 - dLx2*Ly1
    sf_eta[3] = hx_pos_2*dLy1 + d_hy_pos_1*Lx2 - Lx2*dLy1
    sf_xi[4]  = d_hx_slope_2*Ly1;  sf_eta[4]  = hx_slope_2*dLy1
    sf_xi[5]  = hy_slope_1*dLx2;   sf_eta[5]  = d_hy_slope_1*Lx2
    # node 2 (+1,+1)
    sf_xi[6]  = d_hx_pos_2*Ly2 + hy_pos_2*dLx2 - dLx2*Ly2
    sf_eta[6] = hx_pos_2*dLy2 + d_hy_pos_2*Lx2 - Lx2*dLy2
    sf_xi[7]  = d_hx_slope_2*Ly2;  sf_eta[7]  = hx_slope_2*dLy2
    sf_xi[8]  = hy_slope_2*dLx2;   sf_eta[8]  = d_hy_slope_2*Lx2
    # node 3 (-1,+1)
    sf_xi[9]  = d_hx_pos_1*Ly2 + hy_pos_2*dLx1 - dLx1*Ly2
    sf_eta[9] = hx_pos_1*dLy2 + d_hy_pos_2*Lx1 - Lx1*dLy2
    sf_xi[10] = d_hx_slope_1*Ly2;  sf_eta[10] = hx_slope_1*dLy2
    sf_xi[11] = hy_slope_2*dLx1;   sf_eta[11] = d_hy_slope_2*Lx1

    for k in range(4):                       # apply per-node slope scaling
        sf_xi[3*k+1]  *= scaleX[k]; sf_xi[3*k+2]  *= scaleY[k]
        sf_eta[3*k+1] *= scaleX[k]; sf_eta[3*k+2] *= scaleY[k]
    return sf_xi, sf_eta


def _CreateMPCZeroMarker(mbs, nCoords=36):
    """Return a marker whose coordinate vector is clamped to zero.

    Used as marker0 in ObjectConnectorCoordinateVector MPCs so Exudyn does not
    warn about two identical markers (self-constraint via markerNumbers=[m, m]).
    One ground node can be shared by many element constraints in the same mbs.
    """
    from exudyn.utilities import NodeGenericODE2, MarkerNodeCoordinates
    nZero = mbs.AddNode(NodeGenericODE2(
        referenceCoordinates=[0.0] * nCoords,
        initialCoordinates=[0.0] * nCoords,
        initialCoordinates_t=[0.0] * nCoords,
        numberOfODE2Coordinates=nCoords,
        visualization={'show': False},
    ))
    AddNodeConstraints(mbs, nZero, dofs=list(range(nCoords)))
    return mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nZero))


def _AddSlopeDeviationConstraint(mbs, elem, rowFunc, groundMarker=None,
                                 nodalCoords=(-1.0, +1.0),
                                 components=(0, 1, 2)):
    """Add a linear MPC forcing the slope functional rowFunc(s) to equal the linear
    interpolation of its values at the edge ends, at the 2 interior Gauss points.

    rowFunc(s) must return the 12 shape-function-derivative coefficients of the
    constrained (transverse) slope at edge coordinate s in [-1, +1].
    Returns the constraint object index.
    """
    from exudyn.utilities import MarkerObjectODE2Coordinates, ObjectConnectorCoordinateVector
    gp = 1.0 / np.sqrt(3.0)
    nodes = mbs.GetObjectParameter(elem, 'nodeNumbers')
    qRef = np.concatenate([np.array(mbs.GetNodeParameter(n, 'referenceCoordinates'))
                           for n in nodes])                  # 36 reference coordinates
    sfdN0 = rowFunc(nodalCoords[0])
    sfdN1 = rowFunc(nodalCoords[1])
    rows = []
    for s in (-gp, +gp):
        sfd = rowFunc(s) - 0.5*(1.0 - s)*sfdN0 - 0.5*(1.0 + s)*sfdN1
        for c in components:
            r = np.zeros(36)
            for i in range(12):
                r[3*i + c] = sfd[i]
            rows.append(r)
    C = np.array(rows)
    offset = C @ qRef        # marker measures reference + current coordinates
    if groundMarker is None:
        groundMarker = _CreateMPCZeroMarker(mbs)
    mElem = mbs.AddMarker(MarkerObjectODE2Coordinates(objectNumber=elem))
    return mbs.AddObject(ObjectConnectorCoordinateVector(
        markerNumbers=[groundMarker, mElem],
        scalingMarker0=np.zeros(C.shape),
        scalingMarker1=C,
        offset=offset,
        visualization={'show': False, 'color': [-1., -1., -1., -1.]},
    ))


def AddSlopeConformityConstraints(mbs, plateMesh, direction='x', components=(0, 1, 2)):
    """Suppress the non-conforming transverse-slope modes of ALL plate elements.

    The blended (ACM-type) ANCFThinPlate basis interpolates the transverse slope
    r_,x along edge lines NON-conformingly: between the nodes it deviates from the
    linear nodal interpolation by contributions of the far-node DOFs.  This
    deviation is CONSTANT in xi (the eta-nonlinear terms of sf_,xi carry only
    constant-in-xi factors), so constraining it on ONE face per element removes it
    from the whole element; analogously for r_,y in eta.

    Effect: with direction='x', the membrane strain field of a beam-like strip
    (ny=1) reproduces ANCFCable2D EXACTLY (verified: identical L2/max membrane
    error vs fine cable reference), eliminating the spurious self-equilibrated
    strain oscillations across the width at clamps and inter-element lines.

    CAUTION: this stiffens the element (removes the non-conforming modes that
    contribute to 2D bending/twist softness).  Intended for beam-like
    verification/benchmark setups, not for general shell meshes.
    Do NOT combine with AddDistributedClampToEdge on the same elements
    (duplicate equations -> singular Jacobian).
    LIMITATION: direction='both' creates REDUNDANT constraints (singular
    Jacobian) on meshes with ny>=2 (rank check: the y-direction rows of stacked
    element rows become dependent); use 'both' only for ny=1 (and 'y'+nx>=2
    analogously untested) -- for the beam-like strip use the default 'x'.

    Args:
        mbs: the MainSystem
        plateMesh: the ShellMesh whose elements are constrained
        direction:  'x' (constrain r_,x along eta), 'y' (r_,y along xi), or 'both'
        components: slope vector components to constrain (default all)

    Returns:
        list of constraint object indices.
    """
    constraints = []
    groundMarker = _CreateMPCZeroMarker(mbs)
    for elem in plateMesh.elementNumbers:
        scaleX = mbs.GetObjectParameter(elem, 'slopesScalingX')
        scaleY = mbs.GetObjectParameter(elem, 'slopesScalingY')
        if direction in ('x', 'both'):
            def rowFunc(s, scaleX=scaleX, scaleY=scaleY):
                return _ThinPlateShapeFunctionDerivatives(-1.0, s, scaleX, scaleY)[0]
            constraints.append(_AddSlopeDeviationConstraint(mbs, elem, rowFunc,
                                                            groundMarker=groundMarker,
                                                            components=components))
        if direction in ('y', 'both'):
            def rowFunc(s, scaleX=scaleX, scaleY=scaleY):  # noqa: F811 - the row function of the y-direction
                return _ThinPlateShapeFunctionDerivatives(s, -1.0, scaleX, scaleY)[1]
            constraints.append(_AddSlopeDeviationConstraint(mbs, elem, rowFunc,
                                                            groundMarker=groundMarker,
                                                            components=components))
    return constraints


def AddDistributedClampToEdge(mbs, plateMesh, edgeKey, components=(0, 1, 2)):
    """Constrain the TRANSVERSE slope field along a clamped edge at the 2 interior Gauss points of every edge element ('distributed clamp').

    Background: nodal clamps (AddClampToGround) fix the edge POSITION field
    completely (it is conforming), but the transverse slope r_,n along the edge is
    the classic ACM-type NON-CONFORMING field: between the nodes it receives
    contributions from the interior/far-node DOFs of the edge element, so the
    clamp condition is violated mid-edge.  This produces a spurious
    self-equilibrated strain oscillation across the clamped edge (boundary layer).

    Constraint formulation (per edge element, per Gauss point +-1/sqrt(3), per
    component): the transverse slope at the interior edge point must equal the
    LINEAR interpolation of the two nodal transverse slopes,
        r_,n(s*) - [(1-s*)/2 r_,n(-1) + (1+s*)/2 r_,n(+1)] = 0.
    This kills exactly the non-conforming deviation while leaving the nodal slope
    DOFs (e.g. free axial stretch sx_x of a clamp like [1,1,1, 0,1,1, 1,0,1])
    untouched -- mirroring an ideal 1D (cable-like) clamp.  Since the transverse
    slope is cubic along the edge, 2 interior points reduce it exactly to the
    linear nodal interpolation.  Each constraint is a LINEAR multipoint constraint
    on the element's 36 ODE2 coordinates (MarkerObjectODE2Coordinates +
    ObjectConnectorCoordinateVector with exact constant Jacobian).

    NOTE: this makes the strain AT the clamp face exact (matches an ideal 1D
    clamp), but the suppressed non-conforming mode reappears at the next
    inter-element line (overall L2 error barely changes).  To suppress the
    non-conformity in the WHOLE mesh (e.g. to reproduce ANCFCable2D exactly with
    a beam-like strip), use AddSlopeConformityConstraints instead -- but do not
    combine both on the same elements (duplicate equations -> singular Jacobian).

    Args:
        mbs: the MainSystem
        plateMesh: the ShellMesh of the edge
        edgeKey:    'left'/'right'/'bottom'/'top' -- should also be nodally clamped
        components: vector components (0=x, 1=y, 2=z) of the transverse slope to
                    constrain; default all three.

    Returns:
        list of constraint object indices.
    """
    nx = plateMesh.numberOfElementsX
    ny = plateMesh.numberOfElementsY
    if edgeKey == 'left':
        edgeElems = [plateMesh.elementNumbers[iy * nx] for iy in range(ny)]
    elif edgeKey == 'right':
        edgeElems = [plateMesh.elementNumbers[iy * nx + nx - 1] for iy in range(ny)]
    elif edgeKey == 'bottom':
        edgeElems = [plateMesh.elementNumbers[ix] for ix in range(nx)]
    elif edgeKey == 'top':
        edgeElems = [plateMesh.elementNumbers[(ny - 1) * nx + ix] for ix in range(nx)]
    else:
        raise ValueError("AddDistributedClampToEdge: invalid edgeKey")

    constraints = []
    groundMarker = _CreateMPCZeroMarker(mbs)
    for elem in edgeElems:
        scaleX = mbs.GetObjectParameter(elem, 'slopesScalingX')
        scaleY = mbs.GetObjectParameter(elem, 'slopesScalingY')

        def transverseSlopeRow(s):
            """sf-derivative row of the transverse slope at edge coordinate s."""
            if edgeKey in ('left', 'right'):
                xi = -1.0 if edgeKey == 'left' else +1.0
                return _ThinPlateShapeFunctionDerivatives(xi, s, scaleX, scaleY)[0]  # d/dxi
            eta = -1.0 if edgeKey == 'bottom' else +1.0
            return _ThinPlateShapeFunctionDerivatives(s, eta, scaleX, scaleY)[1]      # d/deta

        constraints.append(_AddSlopeDeviationConstraint(mbs, elem, transverseSlopeRow,
                                                        groundMarker=groundMarker,
                                                        components=components))
    return constraints


def ApplyEdgeLoad(mbs, plateMesh, edgeKey, loadVector=None, torqueVector=None,
                  distributed=False):
    """Apply a uniformly distributed force/torque along one plate edge.

    Accepts the TOTAL load/torque; the force is distributed in one of two ways:

    distributed=False (default, nodal):
        Trapezoidal rule on the edge nodes: corner nodes receive 1x the
        per-element base, interior nodes receive 2x, so the sum equals the
        total exactly.  NOTE: nodal point forces are NOT work-equivalent to a
        uniform line load on the cubic (Hermite) edge interpolation -- the
        slope-conjugate load components are missing, which excites a spurious
        Saint-Venant boundary layer (local membrane/bending oscillation across
        the width) at the loaded edge.

    distributed=True (consistent line load):
        Work-equivalent uniform line load: per edge element, the force is
        applied at the 2 Gauss points of the edge (local coordinate
        +-1/sqrt(3)) via MarkerBodyPosition with weight 1/2 each.  Since the
        edge field is cubic and the load direction is constant, 2-point Gauss
        integration of f = int S^T(edge) q ds is EXACT -- including the
        Hermite slope-conjugate components that nodal forces miss.  Use this
        to reproduce ideal beam-like load introduction (e.g. matching an
        ANCFCable2D tip load on a beam-like strip).

    Torques are always applied nodally (trapezoidal rule).

    Args:
        mbs: the MainSystem
        plateMesh: the ShellMesh of the edge
        edgeKey: the edge, 'left', 'right', 'bottom' or 'top' of plateMesh.boundaryNodeNumbers
        loadVector:   3-component total force  [N]   or None
        torqueVector: 3-component total torque [N·m] or None
        distributed:  if True, apply force as consistent line load (see above)
    """
    from exudyn.utilities import MarkerNodeRigid, Force, Torque
    edgeNodes   = plateMesh.boundaryNodeNumbers[edgeKey]
    cornerNodes = set(plateMesh.vertexNodeNumbers)
    nEdge       = len(edgeNodes) - 1              # elements along this edge
    fBase = np.zeros(3) if loadVector  is None else np.array(loadVector,  dtype=float) / (2.0 * nEdge)
    tBase = np.zeros(3) if torqueVector is None else np.array(torqueVector, dtype=float) / (2.0 * nEdge)
    doForce  = np.any(fBase != 0)
    doTorque = np.any(tBase != 0)

    if doForce and distributed:
        from exudyn.utilities import MarkerBodyPosition
        nx = plateMesh.numberOfElementsX
        ny = plateMesh.numberOfElementsY
        # element object numbers along the edge and the fixed parametric coordinate
        if edgeKey == 'left':
            edgeElems = [plateMesh.elementNumbers[iy * nx] for iy in range(ny)]
            fixedXi = -1.0
        elif edgeKey == 'right':
            edgeElems = [plateMesh.elementNumbers[iy * nx + nx - 1] for iy in range(ny)]
            fixedXi = +1.0
        elif edgeKey == 'bottom':
            edgeElems = [plateMesh.elementNumbers[ix] for ix in range(nx)]
            fixedXi = None  # eta fixed at -1
        elif edgeKey == 'top':
            edgeElems = [plateMesh.elementNumbers[(ny - 1) * nx + ix] for ix in range(nx)]
            fixedXi = None
        else:
            raise ValueError("ApplyEdgeLoad: edgeKey must be 'left'/'right'/'bottom'/'top'")
        fixedEta = {'bottom': -1.0, 'top': +1.0}.get(edgeKey)

        gp = 1.0 / np.sqrt(3.0)               # 2-pt Gauss, weights 1 (scaled by 1/2 below)
        fPerElem = np.array(loadVector, dtype=float) / len(edgeElems)
        for elem in edgeElems:
            for xiG in (-gp, +gp):
                if fixedXi is not None:
                    locPos = [fixedXi, xiG, 0.0]
                else:
                    locPos = [xiG, fixedEta, 0.0]
                mG = mbs.AddMarker(MarkerBodyPosition(bodyNumber=elem,
                                                      localPosition=locPos))
                mbs.AddLoad(Force(markerNumber=mG,
                                  loadVector=(0.5 * fPerElem).tolist()))
        doForce = False  # force handled; nodal loop below only handles torque

    for node in edgeNodes:
        if not (doForce or doTorque):
            break
        fact  = 1 if node in cornerNodes else 2
        mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
        if doTorque:
            mbs.AddLoad(Torque(markerNumber=mNode, loadVector=(tBase * fact).tolist()))
        if doForce:
            mbs.AddLoad(Force(markerNumber=mNode,  loadVector=(fBase * fact).tolist()))


def AddRotationalSpringDamper(mbs, nodeA, nodeB, dofs, stiffness, damping=0.0):
    """Add a CoordinateSpringDamper on each selected DOF between nodeA and nodeB.

    Used to add a rotational spring/damper at an ANCF plate hinge:
      dofs=[3,4,5]  resists relative rotation about the y-edge-axis (sx slopes).
      dofs=[6,7,8]  resists relative twist about the x-edge-axis (sy slopes).

    Args:
        mbs: the MainSystem
        nodeA: the first node
        nodeB: the second node
        dofs: the node coordinates to couple
        stiffness: the stiffness of each CoordinateSpringDamper
        damping: the damping of each CoordinateSpringDamper

    Returns:
        list of CoordinateSpringDamper object indices.
    """
    from exudyn.utilities import MarkerNodeCoordinate, CoordinateSpringDamper
    springs = []
    for dof in dofs:
        mA = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeA, coordinate=int(dof)))
        mB = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeB, coordinate=int(dof)))
        springs.append(mbs.AddObject(CoordinateSpringDamper(
            markerNumbers=[mA, mB],
            stiffness=stiffness,
            damping=damping,
        )))
    return springs


def AddEdgeSpringDamper(mbs, nodesA, nodesB, dofs, totalStiffness, totalDamping=0.0):
    """Distribute a total rotational spring-damper along matching edge node lists.

    Mirrors ApplyEdgeLoad: distributes totalStiffness via the trapezoidal rule so
    that the effective stiffness per unit length is uniform regardless of mesh
    refinement.  Corner nodes (first/last) receive 1x the base weight; interior
    nodes receive 2x, giving the correct trapezoidal integration.

    Args:
        mbs:             the MainSystem
        nodesA:          list of node numbers on side A of the hinge (e.g. plate1 right edge)
        nodesB:          list of node numbers on side B of the hinge (e.g. plate2 left edge)
        dofs:            DOF indices to couple, e.g. [3,4,5] for x-slope (bending hinge)
        totalStiffness:  total rotational stiffness [Nm/rad] of the whole hinge
        totalDamping:    total rotational damping   [Nm·s/rad] of the whole hinge

    Returns:
        list of all CoordinateSpringDamper object indices.
    """
    nElem  = len(nodesA) - 1          # number of segments along the hinge
    kBase  = totalStiffness / (2.0 * nElem)
    dBase  = totalDamping   / (2.0 * nElem)
    springs = []
    for i, (nA, nB) in enumerate(zip(nodesA, nodesB)):
        fact = 1 if (i == 0 or i == len(nodesA) - 1) else 2  # corner=half, interior=full
        springs += AddRotationalSpringDamper(mbs, nA, nB, dofs=dofs,
                                             stiffness=kBase * fact,
                                             damping=dBase * fact)
    return springs
