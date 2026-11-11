# The primitive contacts inside

How the contact items between primitive shapes - spheres, cylinders and capsules, truncated cones, triangles and their
planes, convex polytopes - compute the distance of their shapes, the contact points, the forces and the state of the
data node, and how the journal bearing computes its film force. What a user sets is {ref}`sec-contact-primitives`;
this page is how it is built and why.

## 1. The files

| file | what |
|---|---|
| `src/Linalg/GeometryConvex.h` | `EGeometry::ConvexShape` and its support functions, GJK, EPA, the refinement of curved sides, the closed form of sphere and cylinder; `ConvexDistance` is the entry point (#3317) |
| `src/ImplObjects/ContactPrimitiveForce.h` | `ContactPrimitiveForce`: the normal force of the three impact models, the adhesive elasto-plastic option, the regularized friction - the law of one contact point (#3320, #3336) |
| `src/ImplObjects/ContactPrimitivePoints.h` | `ContactPoints`: the contact points of an item, the force of a point by the law or the user function, the generalized forces, the outputs, the post Newton step, the drawing of the forces; the polytope class (#3329, #3331) |
| `src/ImplObjects/CObjectContactCylinderCylinder.cpp` | the point generators of the cylinders, capsules, cones, triangles and of a primitive on a polytope (namespace `ContactCylinder`), and the items CylinderCylinder, CylinderTriangle, ConeCone, ConeTriangle, PrimitiveConvex |
| `src/ImplObjects/CObjectContactConvexConvex.cpp` | ObjectContactConvexConvex (#3331) |
| `src/ImplObjects/CObjectContactSphereCylinder.cpp` | ObjectContactSphereCylinder, one point, the closed form (#3320) |
| `src/ImplObjects/CObjectJournalBearing.cpp` | ObjectJournalBearing (#3335) |
| `src/System/checkPreAssembleConsistenciesObjects.cpp` | the checks at Assemble: `CheckPreAssembleContactCylinder` (data node, markers, law), `CheckConvexShape`, `CheckConeShape` |
| `src/Pymodules/PyConvexShape.h`, `PyCollisionModel.h` | `exudyn.cUtilities.ConvexShape`, `Distance` (#3317) and `CollisionModel` (#3380), bound in `definitions/pybindCUtilities.py` |
| `python/exudyn/misc/mainSystemExtensions.py` | the Create functions of the contacts (#3342) |

The items are defined in `definitions/itemDefsObjects.py`. Every one uses the rigid connector interface with
`jacobian='numerical'` and `velocityDerivative=True` (#2745): its forces are a function of the two marker frames and
their velocities, and the Jacobian is the local numerical differentiation of that function.

## 2. The items

| item | shape 0 / shape 1 | distance | points | data coordinates |
|---|---|---|---|---|
| ObjectContactSphereCylinder | sphere / cylinder | closed form | 1 | 3 |
| ObjectContactCylinderCylinder | cylinder or capsule / cylinder or capsule, or hollow cylinder | GJK, EPA; capsule on capsule closed form; hollow closed form | 4; 6 hollow | 12; 24 hollow |
| ObjectContactCylinderTriangle | cylinder / triangle or its plane | GJK, EPA; plane by the support point | 4 | 12 |
| ObjectContactConeCone | truncated cone / truncated cone, or hollow cone | GJK, EPA; hollow closed form | 4; 6 hollow | 12; 24 hollow |
| ObjectContactConeTriangle | truncated cone / triangle or its plane | as CylinderTriangle | 4 | 12 |
| ObjectContactConvexConvex | polytope / polytope | GJK, EPA | 8 | 24 |
| ObjectContactPrimitiveConvex | sphere, capsule, cylinder or cone (`primitiveType`) / polytope | GJK, EPA | 1 sphere, else 4 | 12 |

- A cylinder, capsule or cone is centered at its marker along the local z axis of the marker, as the joints take their
  axis from the marker frame; a cone has `radius` at $z=-L/2$ and `radius1` at $z=+L/2$, one of them may be 0. A
  capsule's length is that of its segment, between the centers of its half spheres.
- The data coordinates are blocks of 3 per point (gap, norm of the tangential velocity, impact velocity of the last post
  Newton step), of 4 with `contactPlasticityRatio > 0` (the plastic overlap): `GetDataVariablesSize` is the number of
  points of the item times the block size, and the data node must have exactly that many coordinates. The number of
  points is fixed per item and option, not a parameter.
- The older sphere contacts SphereSphere, SphereTriangle and SphereTorus keep their own data (one point) and their own
  geometry; they share the output `Distance` (#3319) and SphereTriangle the option `infinitePlane` (#3323) and the law
  of `ContactPrimitiveForce.h`.

## 3. The distance: GJK and EPA

All shapes are convex, and one algorithm serves every pair: GJK (Gilbert, Johnson, Keerthi) gives the distance and the
closest points of two convex shapes from a support function per shape, EPA (expanding polytope algorithm) the
penetration depth where they overlap. A further shape is a further support function; Bullet, ODE, FCL and MuJoCo use
the same family of algorithms in their narrow phase.

- **Shapes.** `ConvexShapeType`: Sphere, Capsule, Cylinder, Box, Triangle, Points, Cone, in the local frame of the
  shape. The sphere is a point and the capsule a segment - the **core** - with the radius as **margin**
  (`Margin()`): GJK and EPA run on the cores, and `ConvexDistance` subtracts the margins and moves the closest points
  out along the normal, exact for spheres and capsules, and EPA is not needed until the cores overlap. Points are read
  from the rows of the item's matrix through `pointRows`, without a copy (#3331); a triangle is a fixed array of 3
  points, so the items allocate nothing.
- **Support.** `SupportCore(d)`: the point of the core farthest in the local direction d. Cylinder: the rim point of the
  cap in the direction, the cap center if d is along the axis; cone: the farther of its two rims; box: the corner by
  the signs; triangle and points: the vertex of the largest dot product.
- **GJK** (`ConvexGJK`): starts from the support in the direction between the frame origins, keeps a simplex of up to 4
  vertices of the Minkowski difference C = A - B and its point v closest to the origin (the Voronoi regions of a
  triangle after Ericson, Real-Time Collision Detection 5.1.5). It stops at $|v|^2 - v \cdot w \le 10^{-12} |v|^2$,
  when a support point repeats, when $|v|$ no longer decreases, or after 128 iterations; the cores overlap where $|v|$
  falls below $10^{-14}$ of the size (the bounding radii plus the distance of the frames). Each call starts afresh,
  without the simplex of the last step.
- **EPA** (`ConvexEPA::Compute`): the start polytope is the tetrahedron of GJK where it is not flat, else the largest
  tetrahedron of the simplex and the supports along the axes; the polytope grows by the support in the normal of its
  face nearest to the origin until that support is on the face within $10^{-10}$ of the size, at most 128 times. A new
  point removes the faces it sees that are connected to the one it sees most, and is refused if it is a vertex
  already, if a new face would be degenerate, or if a new face would have the origin outside (#3338). A flat C - two
  cores without common volume - gives the depth 0 and the normal of its plane.
- **Curved sides.** GJK stalls on the side of a cylinder or cone, whose points are combinations of the two rims at
  slightly different angles: separated, the closest points are refined by alternating projections onto the cores
  (`ClosestPointCore`, closed forms per shape, GJK for a point set; at most 64). EPA converges slowly on a cylinder, and
  where it stops early its face is not one of C (`converged` false, #3346): shape 1 is moved out by $(d + \delta) n$,
  GJK of the separated shapes gives the boundary point p of C with the normal m, $p \cdot m$ is an upper bound of the
  depth; the smallest bound is kept (at most 32 moves). This refinement runs for every penetration with a cylinder or
  cone, and for every other one where EPA did not converge.
- **Sphere and cylinder** have the closed form `DistanceSphereCylinder` (side, cap, rim, center inside), the check of
  GJK in the tests and the path of ObjectContactSphereCylinder and of `ConvexDistance(analytic=True)`.

Measured (#3317, #3338, #3346), on random pairs of all combinations moved to contact or apart:

| case | result |
|---|---|
| separated, without cylinder | to rounding, 1e-14 |
| separated, with cylinder: GJK alone | 4e-8 off at a distance of 0.08; with the alternating projections to rounding |
| with a cylinder, within about 1e-6 of contact | about 1e-7 of the size; the normal at the side to 1e-7 |
| penetrating cylinder: EPA alone, 128 iterations | depth errors up to 0.15; the move-out refinement converges linearly, factor about 0.3 |
| polytopes resting on a face, 20000 poses near rest | 439 wrong normals (+z for -z) before the guard of #3338, 0 after, depth to 9e-10 |
| polytope resting on a face, normals of EPA | 2 of 5000 off by 0.0115 rad before #3346, 0 of 20000 after |
| cost | 15 us per call on average |

## 4. The contact points

One point at the deepest place is exact for a sphere and fails for a line or a face of contact: with one point, a
cylinder lying on a plane or a triangle, a cap on a cylinder and two parallel cylinders stopped the discontinuous
iteration within 0.2 s in every case, as the deepest point jumps between the ends of the line (#3329). The items
therefore carry up to 4 points (8 for polytopes and hollow shapes), each a `ContactPoints::Point` with its point on
each shape, its normal from shape 0 to shape 1 and its gap; `Points<N>` also holds the result of `ConvexDistance`,
from which the outputs Distance, Displacement and Director1 come. The generator is chosen by the case:

- **A cylinder or cone on a face** (`FacePoints`; the plane, a triangle, a cap, a face of a polytope): the deepest point
  of the lower rim and the other end of the side line through it, clipped to the face, and two points of the lower rim
  at $\pm 120$ degrees - the scheme of MuJoCo's plane-cylinder collision. A rim point beyond the face moves along its
  radius of the cap to the border of the face, so a cap over the border rests on the part over it. The radial
  direction of a standing cylinder has no tilt to follow: it gets the local x axis of the marker with the share
  $\epsilon^2/(\epsilon + \text{tilt})$, $\epsilon = 10^{-3}$, so the points turn with the body below a tilt of about
  1e-3 rad instead of jumping at the first tilt, where Newton diverged (#3329). A tip has no rim points. The gap of each
  point is its height over the plane of the face.
- **A capsule on a face** (`CapsuleFacePoints`): the two ends of its segment over the face, each lowered by the radius.
- **Cylinder side on side** (`SidePoints`), where the closest points of the two axes lie within both cylinders, or the
  axes are parallel ($1 - (a_0 \cdot a_1)^2 \le 10^{-12}$) and the normal is normal to both within 1e-3: the ends of
  the interval of axis 0 where the sides penetrate, clipped to both cylinders, and the closest point of the axes, which
  carries only the penetration beyond the line between the ends - crossed cylinders touch at one point, parallel ones
  at two, and the two go over into each other continuously. The ends and the closest point of parallel sides have the
  gap 0 by construction; their gaps get the offset $10^{-10}(r_0 + r_1)$ added, continuously, so that rounding does not
  switch the contact state - a switch at that threshold was a jump of 1e-5 N inside Newton (#3332).
- **A cap** (`CapPoints`), where the GJK normal is along an axis within 1e-3: the other shape against the cap as a disc.
- **Otherwise** - a rim on a rim, a rim on a side - the closest points of GJK and EPA, one point.
- **Two capsules** (`CapsuleCapsulePoints`, #3365): the closest points of the segments in closed form, the distance
  minus $r_0 + r_1$, then the side points clipped to the segments; no GJK or EPA. A capsule against a cylinder takes the
  distance of GJK and EPA (the segment with its margin), the side points, and against a cap the capsule face points.
- **Cone side on side** (`ConeConePoints`, #3332): along the side line of cone 0 in the half-plane through the point of
  the axis of cone 1 closest to the center of cone 0, the exact signed distance to cone 1 (`Cylinder::SignedDistance`,
  the trapezoid of the half-plane); it is convex along the line, so its minimum by golden section and the ends of the
  penetration by bisection (60 steps each), the minimum carrying the penetration beyond the line of the ends. The
  half-plane is not taken from the closest points of EPA, whose noise gave 1e-6 N, nor from the closest points of the
  axes, which jump where the axes intersect (a rotation of 1e-8 gave 5300 N).
- **A triangle or its plane** (`CylinderTrianglePoints`, also for cones): with `infinitePlane`, the face points on the
  unbounded plane, one-sided along the triangle normal by the order of its points, and the Distance as the height of
  the support point of the shape against the normal - no GJK. Otherwise GJK and EPA, and the face points clipped to the
  triangle where the normal is along the triangle normal within 1e-3.
- **Inside a hollow cylinder or cone** (`HollowPoints`, #3334): not a pair of convex shapes, so neither GJK nor EPA. On
  each rim of the inner shape - where the inner shape leaves the hollow one, the circle at its end plane, so a long
  shaft through a ring rests on the ring's edges - the point in the direction of the eccentricity of the rim and the
  points at $\pm 120$ degrees, so a centered shape (a cone in its seat) rests on all sides; the eccentricity goes over
  into the local x axis of marker 0 as for the standing cylinder. The gap $(R - \rho)/\sqrt{1 + R'^2}$ is normal to the
  wall of radius $R$ and slope $R'$, the normal that of the wall. The rims carry the contact because the distance of a
  point of a side line from the axis is largest at its ends. Up to 6 points; the data node has room for 8.
- **Two polytopes** (`ContactConvex::ComputePoints`, #3331): the face of shape 1 whose outward normal is $-n$ within
  1e-6 (the triangles with that normal) and the points of shape 0 over it, projected into one of its triangles; the
  face of shape 0 with the normal $n$ and the points of shape 1 over it; points of both shapes within 1e-10 of the size
  of each other count once. Of more than 8, the 8 deepest. They are given to the data blocks deepest first, equal gaps
  by their index (#3349): an icosahedron resting flat has 12 points over the face, 3 per layer of equal height, and an
  order by index alone moved the points in contact to other blocks from one Newton iteration to the next, so the solver
  stopped in the first step. Without a face, the closest point.
- **A primitive on a polytope** (`ComputePoints` of ObjectContactPrimitiveConvex, #3364): a sphere by its closest point;
  a capsule, cylinder or cone, where $-n$ is the normal of a face of the polytope within 1e-3, by its face points
  clipped to the **polygon** of the face - the union of its coplanar triangles (`Face::Polygon`, at most 64 triangles);
  else the closest point.

The contact point of a point is the middle of its points on both shapes.

## 5. The force, the data node and the active set

- **Force of a point** (`ContactPoints::PointForce`): the relative velocity of the body of marker 1 to that of marker 0
  at the contact point; in contact - by the gap of its data block during Newton, by its current gap in the post Newton
  step and the drawing - the force on marker 0 by `ContactPrimitiveForce` or by the user function; nothing while
  `activeConnector` is False. The force on marker 0 acts at the contact point with its torque about the marker position,
  its negative on marker 1 (`ForcesRigid`); the stiffness is per point, so a line on two points is twice as stiff as one
  point.
- **The law** (`ContactPrimitiveForce`): $k_c \delta^{n_\mathrm{exp}}$ and the damping, with the term of
  `ContactImpactModel` HuntCrossley or GonthierCarvalhoMartins (#3327); regularized Coulomb friction, proportional below
  `frictionProportionalZone`, its regime from the data block during Newton. With `contactPlasticityRatio` $\lambda_P >
  0$ and the linear impact model, the adhesive elasto-plastic model of ObjectContactSphereSphere: loading with $k_c$,
  unloading and reloading with $K_2 = k_c/(1 - \lambda_P)$ from the plastic overlap, adhesion and a constant pull-off
  force (#3336). Common contact codes keep no plastic state per contact (Bullet stores per point the witness points,
  normal, distance and impulses for warm starting; MuJoCo and ODE a soft constraint); the fourth coordinate of the
  block is Exudyn's own, there only when it is used.
- **User function** (#3324): `contactForceUserFunction(mbs, t, itemNumber, gap, gap_t, normal, tangentialVelocity,
  impactVelocity)` returns the global force on marker 0 of one point in contact, the convention of the output Force -
  the items have no tangential axes for a force in a contact frame, and the impact velocity is the one value of the
  data block a law needs. It replaces the law; the geometry and the active set stay in C++. The symbolic version comes
  from the generator, one member of `PySymbolicUserFunctionSet.h` for the shared signature.
- **Post Newton** (`ContactPoints::PostNewtonStep`): for every block of the data node, the gap, the tangential velocity
  and the impact velocity of the current point - a block without a point is apart, gap 1 -; the error of a switch of
  contact ($|g\, k_c|$) or of stick and slip, the step size to the switch from the gap at the start of the step and its
  rate, and the plastic overlap, the largest $\lambda_P^{1/n} \delta$ of the current contact kept from the start of the
  step. The Create functions start every block apart (gap 1).
- **Outputs** (`ContactPoints::OutputVariable`): Distance and DisplacementLocal the signed distance of the shapes,
  computed from the configuration whether the item is active or not; Displacement and Director1 from the closest
  points; Position the center of the contact points weighted by their normal forces, the middle of the closest points
  without force; Force and Torque the sums on marker 0. The Distance of a deactivated item is what the path planning
  reads.
- **ObjectContactSphereCylinder** keeps the structure of ObjectContactSphereTriangle: one point by the closed form, 3
  data coordinates, marker 0 rigid only with friction, the torque output $((r + g/2)\, n) \times f$ on marker 0 (#3328).

## 6. The journal bearing

ObjectJournalBearing (#3335) is a separate item, not an option of the hollow contact: its force depends on the film, not
on contact points, and it has no data node. Its kinematics are the eccentricity of the journal center normal to the axis
(the local z axis of marker 1), its rate with the bearing turning with its marker, and the sum of the angular velocities
about the axis; tilt is not considered.

- The film force in the frame of the eccentricity after Flores, Ambrosio, Claro, Lankarani and Koshy (2009), the
  infinitely short bearing of Ocvirk and the infinitely long bearing of Sommerfeld (`JournalBearingModel` Short, Long),
  with their squeeze terms; $\varepsilon$ limited by `maximumEccentricityRatio` (below 1, where the forces are
  singular). Beyond the clearance a linear penalty contact $k_c (e - c) + d_c \dot e$ is added. The force acts on the
  line through the journal center; there is no friction torque of the film.
- The formulas assume the film converging in the direction of rotation, $w = \omega - 2\dot\gamma > 0$. The wedge term of
  $F_r$ takes $|w|$, $F_t$ is unchanged and changes its sign with $w$ (#3345): the steady equilibrium is the exact mirror
  for the other direction, and $F_r$ stays continuous. The exact mirror of the whole force made the squeeze term of
  $F_t$ jump where $w$ changes its sign, which it does at the start from the center, and Newton failed.
- Finite bearings are not built in: `forceUserFunction(mbs, t, itemNumber, eccentricity, eccentricity_t,
  angularVelocity, axis)` replaces model and contact.
- Measured: the film force at the closed-form equilibrium of the short bearing is $m g$ to 1e-12; a rigid rotor whirls
  at eccentricity ratios below about 0.8 (oil whip), so the tests run at 0.86 (short) and 0.93 (long).

## 7. Checks, Create functions, drawing

- **At Assemble**: the data node a NodeGenericData of `GetDataVariablesSize` coordinates, both markers rigid,
  `restitutionCoefficient` in [0, 1], `impactModel` in 0..2 (`CheckPreAssembleContactCylinder`); radii and lengths
  positive (a cone: radii not negative, not both 0); a triangle with an area; with `isHollowCylinder1`
  $r_0 \le r_1$, with `isHollowCone1` the smaller radius of cone 0 not above the larger radius of cone 1; a capsule
  option together with `isHollowCylinder1` is refused for either shape. A polytope (`CheckConvexShape`): rows of 3,
  indices in range, every triangle with an area, every point on the inner side of every triangle plane within 1e-8 of
  the size - convex with outward normals; that the surface is closed is not checked, and a polytope without triangles
  is accepted and then has only the closest point.
- **Create functions** (#3342): `CreateSphereCylinderContact` ... `CreatePrimitiveConvexContact` add the rigid markers
  (a cylinder or cone placed by `localPosition` and `localAxis`, its marker z axis along the axis, `AxisHT`), the data
  node of the size the item needs, apart at the start, and the item; the shared helpers are `CheckPrimitiveContact` and
  `AddPrimitiveContact`.
- **Drawing**: the shapes of an item with `show=True` by the switches of `contact.show` in the colors of
  `contact.color` (#3340), whose default alpha 0.8 keeps the force arrows inside the shapes visible (#3376); with
  `contact.showContactForces` two arrows per point meeting at the contact point, from the visualization state with the
  contact state from the gaps (`ContactPoints::DrawForces`, #3339). An item with a force user function draws no forces:
  the render thread cannot call a Python function. Cylinders and cones by `EXUvis::DrawCylinder` with `radiusEnd`
  (#3330), a hollow shape as a tube 0.2 of its radii thick; `graphics.Capsule` is one `SolidOfRevolution` (#3366).

## 8. Without a system

- `exudyn.cUtilities.ConvexShape(shapeType, radius, length, size, points, radius1)` and
  `cUtilities.Distance(shape0, HT0, shape1, HT1, analytic)` return (distance, point0, point1, normal) from the same
  `ConvexDistance` as the items (#3317); a parameter the type does not use is refused.
- `cUtilities.CollisionModel` (#3380) checks the shapes on the links of a KinematicChain against obstacles and each
  other by `ConvexDistance`, filtered by bounding spheres.
- Collision meshes become polytopes through their convex hull, `robotics.urdfImport.ConvexHullOfMesh` (scipy, optional),
  which warns where the volume of the mesh is below 0.98 of its hull (#3374): a URDF allows any mesh, and its convexity
  is the author's convention. Measured on three robots, volume over hull: Staubli TX2-90L 0.993 to 1.000, Franka Emika
  Panda 1.000, UR5 0.645 to 0.945. MuJoCo and PyBullet replace a collision mesh by its hull as well; FCL handles any
  mesh by a bounding volume hierarchy.

## 9. Tests

| test model | what |
|---|---|
| `convexDistanceTest.py` | the distance kernels against closed forms (boxes, capsules, parallel and crossed cylinders, sphere-triangle), analytic sphere-cylinder against GJK/EPA, random pairs at contact and apart, refusals |
| `contactDistanceOutputTest.py` | Distance of inactive sphere contacts against closed forms |
| `sphereCylinderContactTest.py`, `sphereTrianglePlaneTest.py` | the closed form in all regions, at rest, rolling at 5/7 v0; the infinite plane |
| `cylinderContactTest.py` | Distance of deactivated contacts against cUtilities.Distance; resting heights of line, cap and crossed contacts within 6e-9; the roller at 2/3 v0; the user function against the law |
| `coneContactTest.py` | cones standing, upside down, on a cone, lying on a plane and on a cylinder |
| `hollowContactTest.py` | a shaft in a bore and through a ring, a cone in its seat, to 1e-17; the refusal of a shaft larger than the bore |
| `convexConvexContactTest.py` | cubes at rest and sliding; 2000 resting poses (#3338), 5000 poses of a resting icosahedron (#3346), an icosahedron dropped flat (#3349) |
| `capsulePrimitiveContactTest.py` | capsules parallel, crossed, on a cap, on a side; sphere, capsule, cylinder, cone on a box on 1, 2, 3, 3 points; graphics.Capsule |
| `contactPlasticityTest.py` | elastic and plastic cube with pull-off force against closed forms |
| `contactUserFunctionTest.py` | Python and symbolic user functions against the law |
| `journalBearingTest.py` | the closed-form equilibrium of the short bearing, floating journals in both directions, the user function, the dry contact |
| `createPrimitiveContactsTest.py`, `contactPrimitivesTableTest.py` | the Create functions; all primitive contacts on a shaken table |
