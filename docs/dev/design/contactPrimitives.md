# Contact between primitives: cylinders and convex shapes (revision2026c step RG9.12)

A design note of open steps: the approach, the item names and the parameter names, decided by the maintainer on
2026-11-07 (section 6). It leaves the repository when the steps close (WORKFLOW.md §8.4).

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

Measured in RG9.12.5 (#3329): with one point, a cylinder lying on a plane or a triangle, a cap on a cylinder and two
parallel cylinders fail within 0.2 s in every case - the deepest point jumps between the ends of the line, and the
discontinuous iteration does not converge. Decided (maintainer 2026-11-07): the contact points of the cylinders in
RG9.12.5, up to 4 with a data block of 3 each (12 data coordinates):

- a cylinder against a face (the plane or the triangle, the cap of the other cylinder): the deepest point of the
  lower rim and the other end of the side line through it, clipped to the face, and two points of the lower rim at
  +-120 degrees (the scheme of MuJoCo's plane-cylinder collision); a rim point beyond the face moves along its radius
  to the border of the face; below a tilt of 1e-3 rad the radial direction goes over into the local x axis of the
  marker, so that the points of a standing cylinder do not jump;
- side on side: the closest point of the axes and the two ends of the interval of penetration along axis 0; the
  closest point carries only the penetration beyond the line of the ends, so crossed (1 point) and parallel (2 points)
  cylinders go over into each other continuously; a gap within 1e-10 of zero counts as apart;
- otherwise (rim on rim, rim on side) the one point of GJK and EPA.

ObjectContactConvexConvex (RG9.12.6/.7, #3331) has up to 8 points: the points of one shape over the face of the other
whose normal is the contact normal, from both for a face on a face. The cones (RG9.12.11, #3332) use the face points
with the radius of each rim and, at the side, the side line of cone 0 with the exact signed distance to cone 1: its
minimum by golden section and the ends of the penetration by bisection.
The inside contacts (RG9.12.12, #3334) carry the rims of the inner shape, 3 points each; ObjectJournalBearing (RG9.12.13,
#3335) has the short and the long bearing built in, finite bearings by its user function.

### 3.3 The contact law

As ObjectContactSphereTriangle: normal force $f_n = k_c \delta^{n_\mathrm{exp}} + f_d$ by `impactModel` 0 (linear
damping), 1 (Hunt-Crossley) or 2 (Gonthier / Carvalho-Martins); regularized Coulomb friction
($\mu_d$, `frictionProportionalZone`) on the relative tangential velocity of the two bodies at the contact point; no
adhesion and no plastic overlap. The active set by the data node (the gap and the tangential velocity of the last
PostNewton, the impact velocity), with **3 data coordinates** per contact point.

Common codes keep no plastic state per contact: Bullet's persistent manifold stores per point the witness points, the
normal, the distance and the impulses of the last step (for warm starting), MuJoCo and ODE a soft constraint
(stiffness and damping); adhesion and plasticity are Exudyn's own (ObjectContactSphereSphere). The data coordinates
are therefore laid out to be extended without a change of the items' interface:

- a parameter `numberOfContactPoints` (1 in phase 1, up to 4 with the manifold of phase 2);
- the data node holds `numberOfContactPoints` blocks of equal size, the block of point i at i times the block size;
  the block size is the node's number of coordinates divided by `numberOfContactPoints`;
- block size 3 (gap, tangential velocity, impact velocity): the elastic law of this section; block size 4 adds the
  plastic overlap and enables the adhesive elasto-plastic law of ObjectContactSphereSphere as a later option, for the
  shapes where it is meaningful; any other size is refused in CheckPreAssembleConsistency.

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
  surface, checked at Assemble (closed, outward normals, convex within a tolerance). The points are a `NumpyMatrix`
  of n x 3 and the triangles a `NumpyMatrixI` of m x 3 (the type of `triangleMesh` of the FFRF objects), both set
  from 2D numpy arrays or lists of lists, without a new parameter type; CheckPreAssembleConsistency checks the
  3 columns and the indices.
- Create functions (done with #3342, also for the cones; a cylinder or a cone placed by `localPosition` and `localAxis`):
  `CreateSphereCylinderContact`, `CreateCylinderCylinderContact`, `CreateCylinderTriangleContact`,
  `CreateConvexConvexContact` (the data node and markers created, as CreateSphereSphereContact), and
  `graphics.Cylinder`/`FromPointsAndTrigs` with the same geometry for the bodies.
- Outputs: `Distance` (new), `Position` (the contact point, midpoint of the witness points), `Director1` (the normal
  from shape 0 to shape 1), `DisplacementLocal` (the gap, as the others), `Velocity` (relative, at the contact point),
  `Force`, `Torque` (on marker 0).
- Drawing: the shapes and, with `contact.showContactForces`-like settings, the contact point and normal; the existing
  `contact.show.spheres`, `contact.show.triangles` and new `contact.show.cylinders`, `contact.color.cylinders`.

## 5. C++ structure

- `src/Linalg/GeometryConvex.h` (new): support functions of `ConvexSphere`, `ConvexCylinder`, `ConvexCapsule`,
  `ConvexBox`, `ConvexTriangle`, `ConvexPoints` (the box for the URDF shapes of RG21); `GJKDistance(shape0, HT0, shape1, HT1, result)` and `EPAPenetration(...)` with
  the closest points, the normal and the simplex for warm starting (the simplex of the last step, kept per item -
  a speed-up the future module will need);
- `src/ImplObjects/CObjectContact<...>.cpp` - the four items on one helper for the law and the friction shared with
  SphereTriangle (`ComputeContactForces` moved into a common function);
- `src/Pymodules/` - the shapes and `Distance` in exudyn.cUtilities.

## 6. Decisions (maintainer 2026-11-07)

1. GJK + EPA for all pairs, with the analytic sphere-cylinder as check and fast path: yes.
2. One contact point first, the contact manifold of up to 4 points as a later step: yes, with data coordinates that
   extend smoothly to several points (section 3.3: blocks per point, `numberOfContactPoints`).
3. The names ObjectContactSphereCylinder, CylinderCylinder, CylinderTriangle, ConvexConvex, the parameters of
   section 4, the cylinder axis along the local z axis of its marker: yes.
4. The points and triangles of the convex shapes as numpy arrays (`NumpyMatrix`, `NumpyMatrixI`) of 3 columns,
   checked in CheckPreAssembleConsistency, instead of a new type `Index3List`.
5. The data coordinates: what common codes keep (no plastic state); 3 per point, a block of 4 enabling plasticity and
   adhesion as a later option (section 3.3).
6. `OutputVariableType.Distance` for all primitive contact items, computed when inactive; `cUtilities.Distance`: yes.

## 7. Proposed steps (RG9.12)

| step | what |
|---|---|
| RG9.12.1 | this design, decided by the maintainer (2026-11-07) |
| RG9.12.2 | GeometryConvex.h: support functions (sphere, capsule, cylinder, box, triangle, points), GJK, EPA; cUtilities shapes and Distance; tests against analytic distances (sphere-sphere, sphere-cylinder, parallel and crossed cylinders, box-box) |
| RG9.12.3 | the Distance output for SphereSphere, SphereTriangle, SphereTorus, computed when inactive |
| RG9.12.4 | ObjectContactSphereCylinder, with a test against the analytic case and a drop test |
| RG9.12.5 | ObjectContactCylinderCylinder and ObjectContactCylinderTriangle |
| RG9.12.6 | ObjectContactConvexConvex |
| RG9.12.7 | the contact manifold for flat contacts of ConvexConvex (the cylinders have their points since RG9.12.5) |
| RG9.12.8 | examples and the theory section of the contact manual |
| RG9.12.9 | the plastic overlap and the adhesion as an option with a data block of 4 per point (section 3.3) |
| RG9.12.10 | the option `infinitePlane` of the triangle contacts (section 8.1) |
| RG9.12.11 | ObjectContactConeCone and ObjectContactConeTriangle, truncated cones (section 8.2) |
| RG9.12.12 | the contact inside a hollow cylinder and a hollow cone: ObjectContactCylinderInCylinder, the inner cone (section 8.3) |
| RG9.12.13 | ObjectJournalBearing on the geometry of RG9.12.12 (section 8.4) |
| RG9.12.14 | user functions of the contact force for all primitive contacts, also symbolic (section 8.5) |

## 8. Further primitives (maintainer 2026-11-07)

Requested: a cylinder inside a cylinder ring, cone-cone (truncated cones, also a cone inside a cone), cone-triangle
(less relevant), the infinite plane of the triangle without its boundaries, a journal bearing on the cylinder in the
ring, and user functions of the contact force.

### 8.1 The infinite plane

For SphereTriangle, CylinderTriangle and ConeTriangle a parameter `infinitePlane` (False by default): the plane of
the triangle, its normal by the order of the points, without edges and boundaries. The distance is closed-form and
one-sided - the support point of the other shape against the normal: $d = \nv\tp(\pv_c - \pv_0) - r$ for a sphere,
$d = \nv\tp(\sv(-\nv) - \pv_0)$ for a cylinder or cone with its support point $\sv$; no GJK, no EPA, penetrations of
any depth, the contact point the support point. It is the floor of most models (wheels, rolling cylinders, dropped
parts), cheaper and smoother than a triangle, whose edges switch the normal. The existing `includeEdges` of
SphereTriangle stays; `infinitePlane=True` ignores it. (An item ObjectContactSpherePlane would repeat the parameters
of the triangle item, so an option is proposed.)

### 8.2 Truncated cones

`ConvexShapeType::Cone`: a frustum along the local z axis, centered, with `radius0` at $z=-L/2$, `radius1` at
$z=+L/2$ and the length $L$ (a radius 0 gives a pointed cone, equal radii a cylinder). Support: the one of its two
rims farther in the direction - convex, so GJK and EPA apply as for the cylinder, with the same refinements (the
closest point to a point: in the half-plane through the axis, the closest point of the trapezoid). Items:

- `ObjectContactConeCone`: two frustums from the outside; parameters `conesRadii0`, `conesRadii1`, `conesLengths`
  (as `cylindersRadii`);
- `ObjectContactConeTriangle`: a frustum and a triangle, with `infinitePlane`.

### 8.3 Inside a hollow cylinder or cone

A shaft in a bore, a pin in a ring, a taper in a conical seat: the inner shape is **inside** the outer one, and the
gap is the distance to the inner surface of the outer shape - not a pair of convex shapes, so neither GJK nor EPA. It
has a closed form: the inner cylinder (radius $r$, length $L$) in the bore (radius $R \ge r$, length $L_b$), axes
misaligned and offset; the contact is where the inner surface comes closest to the bore, at the ends of the inner
cylinder within the bore (edge loading when tilted), or along a line when aligned:

- the rim circle of each end of the inner cylinder against the bore: the radial distance of a rim point from the bore
  axis is a trigonometric function of the angle on the rim, its maximum by Newton from the direction of the
  eccentricity; the gap $R - \rho_{max}$;
- the parts of the inner cylinder outside the bore (`boreLength` exceeded) are clipped: there the contact is at the
  edge of the bore, its rim against the side of the inner cylinder;
- up to 2 contact points (one per end), with the data blocks of section 3.3 per point; aligned axes give 2 points of
  equal gap, a line contact by its ends.

The bore is the hollow cylinder of marker 1: its radius and length, `cylindersRadii[1]` and `cylindersLengths[1]`, are those of
the bore, with an edge at each end; the inner cylinder is that of marker 0. The flag `isHollowCylinder1` of
ObjectContactCylinderCylinder selects this geometry and keeps the law of section 3.3 and the outputs (decided, 8.6).
The cone inside a cone (a taper seat): the same with the radii of section 8.2, the gap normal to the cone surface, by
the flag `isHollowCone1` of ObjectContactConeCone.

### 8.4 The journal bearing

The cylinder in the bore is the geometry of a lubricated revolute joint. The force models of multibody codes are
closed forms of the dynamic Reynolds equation: the infinitely short bearing of Ocvirk (length/diameter below about
0.5) and the infinitely long bearing of Sommerfeld, both as derived by Pinkus and Sternlicht, and approximations of
finite bearings (correction functions of the short or long bearing, or the analytic approximation of Bastani and de
Queiroz). Flores, Ambrosio, Claro, Lankarani and Koshy (2009) model the lubricated revolute joint as a force element:
the hydrodynamic force while the film carries, a contact force where it breaks down; Machado, Costa, Seabra and Flores
(2012) compare the force models in planar multibody systems. Simpack has a journal bearing element with nonlinear
stiffness and damping, and an elastohydrodynamic element that solves the Reynolds equation. Proposed:

- **two items**: the dry contact ObjectContactCylinderInCylinder (section 8.3), and `ObjectJournalBearing` on the same
  geometry function (eccentricity vector, its rate, the relative angular velocity about the axis), with the
  hydrodynamic force of a selectable model (`bearingModel`: short, long, finite) and the contact force of section 3.3
  where the eccentricity exceeds a fraction of the clearance (Flores et al.); with its own data coordinates where a
  model needs them, which the dry contact does not carry;
- parameters: `journalRadius`, `bearingRadius` (the clearance is their difference), `bearingLength`, `viscosity`,
  `contactEccentricityRatio`, the contact law; outputs: the eccentricity, the force, the minimum film thickness;
- a test against the closed-form steady load of the short bearing (the Sommerfeld number), a rotor example.

### 8.5 User functions of the contact force

Each primitive contact gets `contactForceUserFunction(mbs, t, itemNumber, gap, gap_t, tangentialVelocity,
impactVelocity, dataCoordinates)`, returning the contact force in the contact frame [normal, tangential 1,
tangential 2]; it replaces the law of section 3.3, while the geometry and the active set stay in C++. As a symbolic
user function (exudyn.symbolic) it is evaluated in C++ and costs about as much as the built-in law. The same signature
for all primitive items; per contact point with the manifold.

### 8.6 Decisions (maintainer 2026-11-07)

1. `infinitePlane` as an option of the triangle items, not an item of its own: yes; it costs nothing significant.
2. The truncated cone as `ConvexShapeType::Cone`, the items ConeCone and ConeTriangle with these parameter names: yes.
3. The inside contacts: if possible as an option of the outside item, as ObjectContactSphereSphere covers a sphere
   inside a hollow sphere; checked below. Decided after the check: the flags `isHollowCylinder1` and `isHollowCone1`,
   with a check at Assemble that the inner shape fits the hollow one.
4. The journal bearing as a separate item on the geometry of the inside contact, with the models short, long and
   finite and the contact force of Flores et al.: yes; further bearing models will follow, and the force user
   function adds others - its example implements one of the built-in models.
5. The signature of `contactForceUserFunction`: yes.
6. The order: 8.1 and 8.5 with the first items (RG9.12.4/.5), 8.2 after RG9.12.6, then 8.3 and 8.4: yes.

On 3: ObjectContactSphereSphere has no negative radius but the flag `isHollowSphere1`, which flips the sign $h_1$ of
the second sphere: $g = h_1 d - (r_0 + h_1 r_1)$ and the normal $h_1 \Delta\pv/d$. That works because the distance of
two spheres is a function of the distance of their centers alone, for both cases. For two cylinders or two cones it
is not: the outside contact is GJK and EPA of two convex shapes, the inside one the closed form of section 8.3 - two
geometry functions in any case. The hollow shape keeps its length and the edges at both ends, as the bore of section
8.3; a negative radius could carry that as well, but would turn the check of the radii into one of their signs.
Decided (maintainer 2026-11-07): the inside contact as the flag `isHollowCylinder1` of ObjectContactCylinderCylinder
and `isHollowCone1` of ObjectContactConeCone, named as in SphereSphere, which select the geometry of section 8.3 and
keep the parameters, the law and the outputs; no item ObjectContactCylinderInCylinder or ConeInCone. The check at
Assemble: with `isHollowCylinder1`, `cylindersRadii[0]` $\le$ `cylindersRadii[1]`; with `isHollowCone1` the radii of a
cone vary along the axis, so the check is that the inner cone fits through the seat somewhere - its smaller radius
not above the larger radius of the seat -, made exact with the geometry in RG9.12.12.

References: P. Flores, J. Ambrosio, J.C.P. Claro, H.M. Lankarani, C.S. Koshy, Lubricated revolute joints in rigid
multibody systems, Nonlinear Dynamics 56 (2009); M. Machado, J. Costa, E. Seabra, P. Flores, The effect of the
lubricated revolute joint parameters and hydrodynamic force models on the dynamic response of planar multibody
systems, Nonlinear Dynamics 69 (2012); O. Pinkus, B. Sternlicht, Theory of Hydrodynamic Lubrication, McGraw-Hill
(1961); Y. Bastani, M. de Queiroz, A new analytic approximation for the hydrodynamic forces in finite-length journal
bearings, ASME Journal of Tribology 132 (2010).

## 9. Mesh collision shapes, polytopes against primitives, capsules (maintainer 2026-11-08, RG9.13)

Decided 2026-11-08 (maintainer) as recommended, and done (#3364 to #3366), with these differences: a capsule cannot be combined with a hollow shape at all, and the check of the convex hull at the URDF import goes with #3374.

### 9.1 Are the collision meshes of a URDF convex?

No: the URDF format allows any mesh in `<collision>`, and convexity is a convention of the file's author. Measured
on the three robots of CommonData_git, the volume of each collision mesh divided by the volume of its convex hull
(script of RG9.13.1; 1 is convex):

| robot | collision meshes | volume / hull |
|---|---|---|
| Staubli TX2-90L | 7 STL | 0.993 to 1.000: convex |
| Franka Emika Panda | 9 STL and the primitives of the fingers | 1.000: convex |
| UR5 | 7 STL | 0.645 to 0.945: **not convex** |

Other codes: MuJoCo and PyBullet (by default) replace a collision mesh by its convex hull, FCL (MoveIt) handles any
triangle mesh by a bounding volume hierarchy. Recommended for Exudyn: the check of the volume against the hull at the
import, and a convex hull with a warning giving this ratio where it is below 0.98 (`scipy.spatial.ConvexHull`,
behind the optional import); a decomposition into several convex parts is not part of this step.

### 9.2 A polytope against a sphere, a cylinder and a cone

`ObjectContactConvexConvex` couples only two polytopes. The kernel is ready: `EGeometry::ConvexShape` has Sphere,
Capsule, Cylinder, Box, Triangle, Points and Cone, and GJK/EPA take any pair. What is missing is the item and the
contact points.

Options:

| | how | for | against |
|---|---|---|---|
| A | one item per pair: `ObjectContactSphereConvex`, `ObjectContactCylinderConvex`, `ObjectContactConeConvex` | the naming of the existing items, each page says exactly what it is | about 2000 lines of definitions (the items have 13 functions and a page each) and three times the Create function, tests, drawing, for one implementation |
| B | **one item `ObjectContactPrimitiveConvex`**, marker 0 the primitive by `primitiveType` (`Sphere`, `Capsule`, `Cylinder`, `Cone`) with `radius`, `radius1` (cone), `length`, marker 1 the polytope by `points1`, `triangles1` | one implementation, one page; a further shape is an enum value | one item with parameters that depend on the type |
| C | options in `ObjectContactConvexConvex`, a primitive instead of `points0` | no new item | the parameters of both kinds in every polytope contact |

**Recommended: B**, with one Create function `CreatePrimitiveConvexContact`. The contact points of it:

- sphere: the one closest point; a data block of 3, the size of the data node 3;
- cylinder and cone: on a face of the polytope, the scheme of `ObjectContactCylinderTriangle` and `ConeTriangle` (the
  deepest rim point, the other end of the side line, two rim points at +-120 degrees), clipped to the **polygon of
  the face** of the polytope, which `ContactConvex` finds already for the face on face case; otherwise one point
  by GJK and EPA. Up to 4 points, 12 data coordinates, the same for capsules, which have only the line of their side.

### 9.3 The capsule in ObjectContactCylinderCylinder

The item has no capsule; `ConvexShape` has it. Recommended: as the option `isHollowCylinder1`, the options
**`isCapsule0`, `isCapsule1`**, False by default: a capsule of cylinder $i$ with its length `cylindersLengths[i]`
between the centers of its spherical ends, so of the total length $L_i + 2 r_i$ - the same axis and marker frame as the
cylinder.

- Capsule on capsule: the distance of the two segments, closed form, minus $r_0 + r_1$, no EPA at any penetration
  short of the axes crossing. The contact points are those of the side on side case of section 3.2 (the closest point,
  the two ends of the interval along the axis), clipped to the segments, without caps and rims: parallel capsules on
  two points, crossed on one.
- Capsule on cylinder: GJK and EPA of the capsule's segment core against the cylinder, with the radius as margin; the
  side-on-side and on-a-cap points of the item apply with the end of the capsule beyond the rim replaced by its
  sphere.
- Together with the hollow option: refused in `CheckPreAssembleConsistency` for the capsule that is the hollow one.
- The capsules for robots (RG21.2) are the first users; a capsule against a triangle or the infinite plane is
  `primitiveType Capsule` of section 9.2 with a polytope, or a later option of `ObjectContactCylinderTriangle`.

### 9.4 graphics.Capsule

`graphics.Capsule(pAxis=[0,0,0], vAxis=[0,0,1], radius=0.1, color=[0.,0.,0.,1.], nTiles=16)`, the arguments of
`graphics.Cylinder`: the centers of the spherical ends are `pAxis` and `pAxis + vAxis`. It is one
`SolidOfRevolution` of a contour of two quarter circles and a line (smooth normals, closed at both poles), so that it
needs no new drawing code. A `length` of 0 gives a sphere.

### 9.5 Steps (RG9.13)

| step | issue | what |
|---|---|---|
| RG9.13.1 | #3364 | `ObjectContactPrimitiveConvex` (9.2) with `CreatePrimitiveConvexContact`, the check of the hull at the import (9.1) |
| RG9.13.2 | #3365 | `isCapsule0`, `isCapsule1` of ObjectContactCylinderCylinder and of its Create function (9.3) |
| RG9.13.3 | #3366 | `graphics.Capsule` (9.4) |
