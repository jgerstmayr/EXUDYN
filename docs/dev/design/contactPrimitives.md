# Contact between primitives: cylinders and convex shapes (revision2026c step RG9.12)

A design note of open steps, for the maintainer to check the approach, the item names and the parameter names before
any code. It leaves the repository when the steps close (WORKFLOW.md §8.4). No issues are raised before section 6 is
decided.

## 1. The request (maintainer 2026-11-06)

- Further contact primitives: **cylinder-cylinder, cylinder-sphere, cylinder-triangle, convex shape - convex shape**
  (both defined by triangles).
- Built like ObjectContactSphereSphere (without its adhesion) and ObjectContactSphereTriangle.
- They are prototypes for the future contact module (search trees, parallelization); they need not be efficient.
- They also serve the path planning of RG21 (robotPlanning.md): an **output of the (minimum) distance**, available
  while the contact is deactivated, or computed without any item in a simulation.
- Steps under RG9 or RG11, no new group.

## 2. What exists

| item | geometry | contact law | data node | outputs |
|---|---|---|---|---|
| ObjectContactSphereSphere | two spheres at the markers, the second optionally hollow | penalty, `impactModel` 0 (linear, with the adhesive elasto-plastic model), 1 (Hunt-Crossley), 2 (Gonthier / Carvalho-Martins); regularized Coulomb friction | 4: gap, tangential velocity, impact velocity, plastic overlap | Position, Displacement, DisplacementLocal (the gap), Velocity, Force, Director1, Torque |
| ObjectContactSphereTriangle | sphere at marker 0, triangle `trianglePoints` in the frame of marker 1, `includeEdges` | as above without adhesion | 4 | as above |
| ObjectContactSphereTorus | sphere, torus | as above | 4 | as above |
| GeneralContact | spheres, triangles of rigid bodies, ANCF cables | penalty, friction | internal | per contact |

`src/Linalg/Geometry.h` has the point-segment and point-triangle distances (`MinDistTP`, `MinDistTrigEdges`) and the
plane functions. The connector force is computed by `ComputeConnectorForceRigid(markers, t, forces, torques)` (#2745),
which also gives the Jacobian by differentiation.

## 3. The approach

### 3.1 One distance algorithm for convex shapes: GJK and EPA

All four pairs are pairs of **convex** shapes (a cylinder, a sphere, a triangle, the convex hull of points). Exact
closed forms exist for sphere-cylinder, but cylinder-cylinder (finite, with caps) and cylinder-triangle have no simple
closed form, and convex-convex has none. The standard method is:

- **GJK** (Gilbert-Johnson-Keerthi) for the distance of two convex shapes and the closest points (witness points),
  from a support function per shape - the point of the shape farthest in a direction: for a sphere the center plus
  r d, for a cylinder the cap rim point in that direction (or the cap center if d is along the axis), for a triangle
  and a convex point set the vertex with the largest dot product;
- **EPA** (expanding polytope algorithm) for the penetration depth and normal when GJK finds an overlap;
- a sphere or a capsule as a point or segment with a **margin** (radius): GJK of the cores, then the distance minus the
  radii - exact and robust for spheres, and it keeps EPA away from the round shapes in most contacts.

One implementation serves all pairs and later shapes (capsule, box, cone) by a further support function; it is what
the future contact module will use in its narrow phase (Bullet, ODE, FCL and MuJoCo use GJK/EPA or MPR). The
analytic sphere-cylinder distance is implemented as well, as a check of GJK in the tests and as the faster path.

Accuracy: GJK on a curved shape converges by iteration; the tolerance 1e-10 relative to the shape size gives distances
to about 1e-10 m for shapes of 1 m; EPA on curved shapes approximates by a polytope (tolerance 1e-8 relative). A
smooth contact force needs a continuous normal: for a cylinder the support function is not unique along the axis and
across the cap - the witness point then moves discontinuously; the force uses the normal of GJK/EPA, the point is
chosen as the midpoint of the witness points.

### 3.2 One contact point or several

A single point (the deepest one) is exact for sphere-sphere and nearly right for curved contacts, but a **flat contact**
- a cylinder lying on its cap on a triangle, a box on a box - gets the whole force at one point that jumps between the
corners: the body rocks and the contact chatters. Two ways:

- phase 1: one point, as the existing items; fine for curved contacts (cylinder side on a plane is a line contact:
  one point at the deepest end, so it rocks as well);
- phase 2: a **contact manifold** of up to 4 points for face-face and face-edge contacts: the face of each shape most
  aligned with the normal, the incident face clipped against the reference face (Sutherland-Hodgman), the points
  below the reference face as contacts with the force of each by its own penetration; for a cylinder the cap as a
  polygon of 8 to 16 points and the line contact of its side by the two ends of the contact segment.

Proposed: phase 1 for all four items, phase 2 for ConvexConvex and the cylinder caps and sides as a step of its own.

### 3.3 The contact law

As ObjectContactSphereTriangle: normal force $f_n = k_c \delta^{n_\mathrm{exp}} + f_d$ by `impactModel` 0 (linear
damping), 1 (Hunt-Crossley) or 2 (Gonthier / Carvalho-Martins); regularized Coulomb friction
($\mu_d$, `frictionProportionalZone`) on the relative tangential velocity of the two bodies at the contact point; no
adhesion and no plastic overlap. The active set by the data node (the gap and the tangential velocity of the last
PostNewton, the impact velocity), with **3 data coordinates**; with the manifold of phase 2, 3 per contact point
(the number of points fixed per item, e.g. `numberOfContactPoints` = 1 or 4).

### 3.4 The distance output and deactivation

- New output **`OutputVariableType.Distance`**: the signed minimum distance of the two shapes, negative for a
  penetration (then the penetration depth by EPA), from the current configuration, **also with
  `activeConnector = False`** - then the item adds no force and no Jacobian entries, but its output is computed on
  request. The same output added to ObjectContactSphereSphere, SphereTriangle and SphereTorus (their gap is in
  DisplacementLocal today; Distance says the same under the name the planner uses).
- Without an mbs: the distance kernels exported in **exudyn.cUtilities** - shape classes and
  `cUtilities.Distance(shape0, HT0, shape1, HT1)` returning the distance, the witness points and the normal - the same
  C++ functions as the items, for RG21 and for tests.

## 4. The items (names and parameters proposed)

Common to all four, with the names and meaning of ObjectContactSphereTriangle:
`markerNumbers` [m0, m1] (rigid markers: position and orientation, e.g. MarkerBodyRigid with `localHT`),
`nodeNumber` (NodeGenericData with 3 coordinates), `contactStiffness`, `contactDamping`, `contactStiffnessExponent`,
`dynamicFriction`, `frictionProportionalZone`, `restitutionCoefficient`, `minimumImpactVelocity`, `impactModel`,
`activeConnector`; VItem `show`, `color`. Shape parameters in the frame of their marker:

| item | marker 0 | marker 1 | shape parameters |
|---|---|---|---|
| `ObjectContactSphereCylinder` | sphere | cylinder | `sphereRadius`, `cylinderRadius`, `cylinderLength` |
| `ObjectContactCylinderCylinder` | cylinder | cylinder | `cylindersRadii` [r0, r1], `cylindersLengths` [L0, L1] (as `spheresRadii`) |
| `ObjectContactCylinderTriangle` | cylinder | triangle | `cylinderRadius`, `cylinderLength`, `trianglePoints` (as SphereTriangle) |
| `ObjectContactConvexConvex` | convex shape | convex shape | `points0`, `points1` (Vector3DList), `triangles0`, `triangles1` (the triangles of the surfaces, for drawing and the manifold) |

- The cylinder is **centered at the marker, its axis the local z axis** of the marker frame, length L from -L/2 to
  L/2 - the frame given by `localHT` of the marker. (Alternative: an axis vector parameter; the marker frame is the
  Exudyn way, as for joints.)
- The sphere is marker 0 where a sphere is involved, as in SphereTriangle; the triangle is marker 1.
- The convex shapes: the points are assumed convex (their convex hull is used); the triangles of a closed convex
  surface, checked at Assemble (closed, outward normals, convex within a tolerance). Their type: there is no list of
  index triples among the item parameter types; proposed `TArrayIndex` with 3 entries per triangle, or a new type
  `Index3List` in the generator.
- Create functions: `CreateSphereCylinderContact`, `CreateCylinderCylinderContact`, `CreateCylinderTriangleContact`,
  `CreateConvexConvexContact` (the data node and markers created, as CreateSphereSphereContact), and
  `graphics.Cylinder`/`FromPointsAndTrigs` with the same geometry for the bodies.
- Outputs: `Distance` (new), `Position` (the contact point, midpoint of the witness points), `Director1` (the normal
  from shape 0 to shape 1), `DisplacementLocal` (the gap, as the others), `Velocity` (relative, at the contact point),
  `Force`, `Torque` (on marker 0).
- Drawing: the shapes and, with `contact.showContactForces`-like settings, the contact point and normal; the existing
  `contact.showSpheres`, `contact.showTriangles` and new `contact.showCylinders`, `contact.colorCylinders`.

## 5. C++ structure

- `src/Linalg/GeometryConvex.h` (new): support functions of `ConvexSphere`, `ConvexCylinder`, `ConvexCapsule`,
  `ConvexTriangle`, `ConvexPoints`; `GJKDistance(shape0, HT0, shape1, HT1, result)` and `EPAPenetration(...)` with
  the closest points, the normal and the simplex for warm starting (the simplex of the last step, kept per item -
  a speed-up the future module will need);
- `src/ImplObjects/CObjectContact<...>.cpp` - the four items on one helper for the law and the friction shared with
  SphereTriangle (`ComputeContactForces` moved into a common function);
- `src/Pymodules/` - the shapes and `Distance` in exudyn.cUtilities.

## 6. Decisions for the maintainer

1. GJK + EPA for all pairs, with the analytic sphere-cylinder as check and fast path.
2. One contact point first; the contact manifold of up to 4 points as a later step.
3. The names: ObjectContactSphereCylinder, CylinderCylinder, CylinderTriangle, ConvexConvex; the parameters of
   section 4; the cylinder axis along the local z axis of its marker.
4. The type of the triangle list (`TArrayIndex` flat or a new `Index3List`).
5. 3 data coordinates (no plastic overlap).
6. `OutputVariableType.Distance` for all primitive contact items, computed when inactive; `cUtilities.Distance`.

## 7. Proposed steps (RG9.12)

| step | what |
|---|---|
| RG9.12.1 | this design, decided by the maintainer |
| RG9.12.2 | GeometryConvex.h: support functions, GJK, EPA; cUtilities shapes and Distance; tests against analytic distances (sphere-sphere, sphere-cylinder, parallel and crossed cylinders, box-box) |
| RG9.12.3 | the Distance output for SphereSphere, SphereTriangle, SphereTorus, computed when inactive |
| RG9.12.4 | ObjectContactSphereCylinder, with a test against the analytic case and a drop test |
| RG9.12.5 | ObjectContactCylinderCylinder and ObjectContactCylinderTriangle |
| RG9.12.6 | ObjectContactConvexConvex |
| RG9.12.7 | the contact manifold for flat contacts (cylinder caps and sides, convex faces) |
| RG9.12.8 | examples and the theory section of the contact manual |
