# The superelement matrices as sparse triplets for larger meshes

A design note of revision2026c step RG5.4 (#2182, #2183, #2184): the analysis (Sections 1-6) and what was done first
(Section 7).

## 1. What exists

### 1.1 The superelements

`CObjectSuperElement` (src/ImplObjects/CObjectSuperElement.cpp) is the base of three objects:

| object | coordinates | matrices (definitions/itemDefsObjects.py) | element mass matrix | ODE2 Jacobian |
|---|---|---|---|---|
| ObjectGenericODE2 | the nodes' | massMatrix, stiffnessMatrix, dampingMatrix: `PyMatrixContainer` (1623-1633) | `CopyOrAddTriplets(parameters.massMatrix, ltg)` - **sparse triplets in global indices already** (CObjectGenericODE2.cpp:75) | analytic, triplets or dense as stored (CObjectGenericODE2.cpp:121-170) |
| ObjectFFRF | rigid node + **all mesh nodes** (3 per node) | massMatrixFF, stiffnessMatrixFF, dampingMatrixFF: `PyMatrixContainer` (3908-3918), stored as triplets ("internally data is stored in triplet format", CObjectFFRF.h:40-42); PHItTM: dense `TNumpyMatrix` 3 x nFF | **dense nODE2 x nODE2** (CObjectFFRF.cpp:170 `GetInternalDenseMatrix()`), Mff by `massMatrixFF.GetEXUdenseMatrix()` (CObjectFFRF.cpp:270): a dense **copy of nFF^2 allocated on every call** | **numerical only** (CObjectFFRF.h:263-265: ODE2_ODE2 without `_function`): nODE2 evaluations of the LHS, a dense nODE2 x nODE2 block |
| ObjectFFRFreducedOrder | rigid node + nModes | massMatrixReduced, stiffnessMatrixReduced, dampingMatrixReduced: `PyMatrixContainer` (4499-4509); modeBasis, outputVariableModeBasis: dense `TNumpyMatrix` (3 nNodes x nModes) | dense (nODE2Rigid+nModes)^2; Mff from triplets by `AddToDenseMatrix` (CObjectFFRFreducedOrder.cpp:194-202) | analytic for EP/Rxyz without force user function (#3212, CObjectFFRFreducedOrder.cpp:477, 623-632), dense |

### 1.2 How an object hands its matrices to the system

`EXUmath::MatrixContainer` (src/Linalg/MatrixContainer.h:21-200) holds a dense `ResizableMatrix` **or** a `SparseTripletMatrix`, switched by `useDenseMatrix`.

- **Mass matrix** (`CSystem::ComputeMassMatrix`, src/Main/CSystem.cpp:53-180): the container is set dense before each object
  (CSystem.cpp:66, 112); an object may switch it to sparse and append **triplets in global indices** (the `ltg` argument
  is "only used in sparse mode", CSystem.cpp:69). Both system matrix kinds accept that today:
  - dense system matrix: the triplets are added entry by entry (CSystem.cpp:76-85);
  - sparse system matrix: the triplets are kept, and a **dense** element matrix is scanned entry by entry, n^2, its
    nonzeros appended (CSystem.cpp:117-129).
- **Jacobian** (`CSystem::AddLocalJacobianODE2` / `AppendLocalJacobianODE2`, src/Main/CSystemJacobians.cpp:300-365):
  the same - `jacobianODE2Container` dense (AddSubmatrix with ltg, or the nonzeros of every entry) or triplets
  (`AddSparseTriplets` / `AppendArray`), for analytic Jacobians. A numerical Jacobian is always a dense block.

So **the interface for sparse element matrices exists**; only ObjectGenericODE2 uses it. Nothing in CSystem has to
change for a superelement to return triplets.

### 1.3 SparseTripletMatrix (src/Linalg/BasicLinalg.h:768-970)

Has `AddTriplet`, `MultiplyWithFactor`, `MultMatrixVector(Add)`, `MultMatrixDenseMatrix`,
`MultDenseMatrixTransposedMatrix`, `AddToDenseMatrix(dense, rowOffset, colOffset, factor)` (903), `GetEXUdenseMatrix()`
(937, by value). Missing for #2183: appending a **dense submatrix** (and its transpose) as triplets with a row/column
offset or an ltg map, and appending another triplet matrix with offset and factor (`AddSparseTripletMatrix`).

## 2. Where the work scales with the mesh

| where | cost | per |
|---|---|---|
| ObjectFFRF mass matrix: dense nODE2^2 set to zero, `GetEXUdenseMatrix()` copy of Mff (nFF^2 allocation), AddSubmatrix | O(nFF^2) memory and time | each mass matrix evaluation (each Jacobian in implicit solvers, each step in explicit ones) |
| the same in the sparse system path: n^2 scan for nonzeros (CSystem.cpp:117-129) | O(nFF^2) | same |
| ObjectFFRF numerical Jacobian | nODE2 LHS evaluations, each O(nnz(K)+nFF), plus the dense n^2 block | each Jacobian |
| ObjectFFRF LHS (stiffness, damping, quadratic velocity: `MultMatrixVectorAdd` and `MultDenseMatrixTransposedMatrix` on triplets, PHItTM 3 x nFF) | O(nnz + nFF) - fine | each RHS |
| ObjectFFRFreducedOrder mass matrix, LHS | O((nRigid+nModes)^2), independent of the mesh; the reduced matrices are already triplets - **#2182 gains nothing** | - |
| **MarkerSuperElementRigid on ObjectFFRFreducedOrder** (#2184): `GetWeightedRotations` / `GetWeightedAngularVelocity` (CMarkerSuperElementRigid.cpp:67-160) loop twice over the interface nodes; `GetMeshNodeLocalPosition` costs O(nModes) per node (CObjectFFRFreducedOrder.cpp:1121-1138); the constant `factorMatrix`/`factor` of the reference positions is recomputed every call (CMarkerSuperElementRigid.cpp:91-102) | O(nIF * nModes), several times per marker evaluation (position, velocity, rotation, angular velocity) | every RHS and every Jacobian |
| the access functions `TranslationalVelocity_qt`/`AngularVelocity_qt + SuperElement` of ObjectFFRFreducedOrder (CObjectFFRFreducedOrder.cpp:866-1010): `value(j, nODE2rigid+m) += jac(j,k)*modeBasis(offset+k,m)` over all interface nodes, plus the weighted reference positions again | O(9 nIF nModes) | every marker Jacobian (CMarkerSuperElementRigid.cpp:300-335) |
| MarkerSuperElementPosition: the same position/velocity sums and the translational access function | O(nIF nModes) | every RHS / Jacobian |

For ObjectFFRFreducedOrder all interface sums are **linear in the modal coordinates** with constant weights, so they
can be precomputed at Assemble:

- `PsiW = sum_i w_i Psi_i` (3 x nModes) and `pW = sum_i w_i xRef_i`: the weighted position is `pW + PsiW zeta`, the
  weighted velocity `PsiW zeta_t`, the translational Jacobian block `A PsiW`;
- `pRef0`, `factorMatrix` (or `factor`) and its inverse: constant;
- `RW = sum_i w_i (pRef_i - pRef0)~ (Psi_i - PsiW)` (3 x nModes): the weighted rotations are `RW zeta`, the weighted
  angular velocity `RW zeta_t`, and the rotational Jacobian block follows from it.

Then a marker costs O(nModes) instead of O(nIF nModes): for an interface of 1000 nodes, a factor of about 1000 on
the marker work. (For ObjectFFRF and ObjectGenericODE2 the interface nodes are coordinates themselves; the sums are
O(nIF) with no modes - only the constant factorMatrix is worth precomputing there.)

## 3. The Python side

- **ObjectFFRF** (`ObjectFFRFinterface.AddObjectFFRF`, python/exudyn/FEM.py:1019-1080): the scipy CSR mass and
  stiffness matrices go in by `exu.MatrixContainer().SetWithSparseMatrix(..., useDenseMatrix=False)` (FEM.py:1057-1070),
  i.e. **already as triplets**.
- **ObjectFFRFreducedOrder** (`AddObjectFFRFreducedOrder` and `...WithUserFunctions`, FEM.py:1567-1730, 1879-): the
  reduced matrices go in by `SetWithDenseMatrix(..., useDenseMatrix=False)` (FEM.py:1667-1684), which drops exact
  zeros and stores triplets (src/Pymodules/PyMatrixContainer.cpp:118-150). `modeBasis` and `outputVariableModeBasis`
  are dense numpy matrices - inherently dense, they stay so.
- **ObjectGenericODE2** (FEM.py:3505-3630 and user models): triplets or dense, as the user chooses.

**Answer to the maintainer's question: storing the matrices as triplets needs no change in the initialization of
the Python objects** - the interfaces already transport them as triplets. What is dense is the hand-over inside C++
(Section 2). The one case to handle is a user who gives a dense container (`useDenseMatrix=True`) to ObjectFFRF:
the sparse path then reads it through `GetEXUdenseMatrix`/`AddToDenseMatrix`-style accessors or falls back to the
dense path for that object - no transporting function is needed.

## 4. The models that use them

- python/PerformanceModels/perfObjectFFRFreducedOrder.py: the rotor of testData/rotorDiscTest - **115 nodes**
  (345 coordinates, lumped mass: 345 nonzeros), 8 modes, two MarkerSuperElementPosition of **9 nodes** each, 20000
  steps. Its time (1.13.548: 2.84 s; ODE2RHS 1.35 s, jacobianODE2 1.08 s, massMatrix 0.30 s) is not dominated by
  the mesh at all; **it cannot show a gain of RG5.4.**
- Test models: objectFFRFTest.py, objectFFRFTest2.py, objectFFRFforceVectorTest.py (ObjectFFRF);
  objectFFRFreducedOrderTest.py, ...Accelerations.py, ...StressModesTest.py, ...ShowModes.py, superElementRigidJointTest.py,
  NGsolveCMStest.py, abaqusImportTest.py, ACFtest.py (reduced order, MarkerSuperElementRigid/Position);
  objectGenericODE2Test.py, linearFEMgenericODE2.py. All with small meshes.
- A **new performance case** is needed: a block or plate meshed with some thousand nodes (`FEMinterface.CreateMesh` /
  the internal Hex8 mesh generator used by the NGsolve-free tests), (a) ObjectFFRFreducedOrder with 20 modes and two
  MarkerSuperElementRigid of 500-2000 interface nodes, (b) ObjectFFRF on a mesh of 500-2000 nodes with the sparse solver.

## 5. Proposal

### 5.1 First change, recommended: #2184, the precomputed interface of the markers on ObjectFFRFreducedOrder

- `CObjectFFRFreducedOrder`: a function computing, for a weight vector and node list, `pW`, `PsiW`, `pRef0`, `RW`
  and the constant factor matrix (from `modeBasis`, `referencePositions`); used by the access functions.
- `CMarkerSuperElementRigid`, `CMarkerSuperElementPosition`: a cache of these (members, filled at Assemble, read-only
  afterwards - thread safe in the parallel loops, unlike a lazy mutable cache). This needs a hook: a new virtual
  `CMarker::InitializeMarker(const CSystemData&)` (default empty), called in the marker loop of
  `CSystem::Assemble` (src/Main/CSystemAssemble.cpp:1248-1255). A marker whose object is not reduced order keeps its
  loops; the constant factor matrix is cached for all.
- `GetWeightedRotations`, `GetWeightedAngularVelocity`, `GetPosition`, `GetVelocity` and the two access functions use
  the cache: O(nModes).
- **Fallback**: an experimental flag `exu.special.solver.superElementPrecomputedInterface` (default True once the
  test passes; experimental, so `exudyn.special` by the convention of #3446, to be removed after a release) - or the
  existing marker parameter-free automatic choice; both paths stay in the code.
- **Test**: a test model (name to be checked free, e.g. `superElementSparseTest.py`) with a mesh of some hundred
  nodes, a reduced-order body on two MarkerSuperElementRigid and a MarkerSuperElementPosition, solved with the flag
  on and off: positions and the sensor values equal to 1e-13 relative; its result registered in runTestSuiteRefSol.py.
- Expected gain: the marker work from O(nIF nModes) to O(nModes) - large for realistic interfaces (hundreds to
  thousands of nodes), nothing for the 9-node interfaces of the existing performance model.

### 5.2 Second change: #2183 + the sparse mass matrix of ObjectFFRF

- `SparseTripletMatrix` / `MatrixContainer` (BasicLinalg.h, MatrixContainer.h): `AddDenseSubmatrixAsTriplets(matrix,
  ltg, rowOffset, columnOffset, transposed, factor)` (exact zeros dropped) and `AddSparseTripletMatrix(other, ltg,
  rowOffset, columnOffset, factor)`.
- `CObjectFFRF::ComputeMassMatrix`: with the flag, switch `massMatrixC` to triplets and append: Mtt (3x3), Mtr/Mrt
  (3 x nRot), Mrr, the dense rows Mtf/Mft (3 x nFF, `A PHItTM`) and Mrf/Mfr (nRot x nFF), and Mff as the stored
  triplets of `massMatrixFF` through ltg. nnz = nnz(Mff) + about 14 nFF + 49 instead of nODE2^2; no allocation of
  nFF^2. The mass matrix user function keeps the dense path (fallback, and when `massMatrixFF` is dense).
- Fallback: the same kind of experimental flag (`superElementSparseMassMatrix`), or one flag for both 5.1 and 5.2.
- Test: objectFFRFTest.py / objectFFRFTest2.py run with the flag on and off (EigenDense and EigenSparse): equal to 1e-13.
- Expected gain: for ObjectFFRF on 1000+ nodes with EigenSparse, the mass matrix from O(nFF^2) to O(nnz); the
  numerical Jacobian of ObjectFFRF stays O(nODE2 * (nnz + nFF)) and dense - **an analytic Jacobian of ObjectFFRF**
  (K, D and the FFRF terms, as #3212 did for the reduced order) is the next bottleneck and a step of its own.

### 5.3 #2182: no change

The reduced matrices of ObjectFFRFreducedOrder are already triplets (Section 3) and the element matrix is
(nRigid+nModes)^2; returning it as triplets would not pay. Proposed: close #2182 as done by the existing storage
(or merge it into 5.2, which is what "larger meshes" needs).

## 6. Uncertain

- The exact cost split of a realistic large-interface model is a guess until the new performance case runs.
- `useAlternativeApproach` of MarkerSuperElementRigid (and `alternativeRotationMode` of the object, which "must be
  same", CObjectFFRFreducedOrder.cpp:862) changes the rotation formula - both variants need their precomputed form;
  the factor/factorMatrix inverse may be singular for degenerate interfaces (an existing check).
- Whether a marker's parameters can be changed after Assemble without a new Assemble (then the cache would be stale):
  `SetMarkerParameter` should be checked; the safe rule is "the cache is rebuilt at every Assemble, and a change of
  meshNodeNumbers/weightingFactors requires Assemble", as for the object matrices.
- ObjectFFRF stores Mff as triplets but may hold duplicates (summed by Eigen when the sparse matrix is built - fine for
  the system matrix, to be kept in mind for the dense fallback).

## 7. Done first: the mass matrix of ObjectFFRF as triplets (#2183)

- `EXUmath::MatrixContainer::AppendSubmatrixAsTriplets(matrix, ltg, rowOffset, columnOffset, transposed)` and
  `AppendTripletsWithOffset(tripletMatrix, ltg, rowOffset, columnOffset)` (`MatrixContainer.h`).
- `CObjectFFRF::ComputeMassMatrix` appends the blocks Mtt, Mtr/Mrt, Mtf/Mft, Mrf/Mfr, Mrr as their nonzeros and Mff as
  the stored triplets of `massMatrixFF`, in the global indices of `ltg`, when its parameter `sparseMassMatrix` is True
  (the default; False is the fallback, #3449) and neither a
  mass matrix user function nor a dense `massMatrixFF` is given. The dense path adds Mff from the triplets directly
  instead of a dense copy of nFF x nFF on every call.
- `mbs.ComputeItem(body, exu.ComputeItemType.MassMatrix)` turns the triplets of a body that returns a sparse mass matrix
  into its dense matrix (before, such a body - also ObjectGenericODE2 with a sparse mass matrix - gave an empty one).
- Test model `objectFFRFsparseMassMatrixTest.py`: the rotor of 115 nodes; the mass matrix of both paths is the same to
  the bit, the motion with EigenDense the same to the bit, with EigenSparse within 2e-12.

Measured (scratch study: an NGsolve block 1 x 0.1 x 0.1 m of linear tetrahedra, ObjectFFRF on a spherical joint at one
end, 20 steps of GeneralizedAlpha, modified Newton; regular module; solver timers in s):

| nodes | linear solver | massMatrix dense | sparse | total dense | sparse |
|---:|---|---:|---:|---:|---:|
| 166 | EigenSparse | 0.024 | 0.007 | 0.186 | 0.168 |
| 166 | EigenDense | 0.038 | 0.010 | 0.208 | 0.179 |
| 486 | EigenSparse | 0.185 | 0.022 | 1.787 | 1.645 |
| 486 | EigenDense | 0.327 | 0.076 | 2.160 | 1.879 |
| 1525 | EigenSparse | 1.571 | 0.080 | 24.07 | 22.53 |

The mass matrix is 8-20 times faster; the coordinates agree to 1e-16. The total gains 6-13 %, because the rest stays:
the numerical Jacobian of ObjectFFRF (7.9 s of 24 s at 1525 nodes, nODE2 evaluations of the element and a dense block)
and the remaining 14 s, not split in this study (the right-hand side and the overhead). Next for ObjectFFRF: an analytic Jacobian (Section 5.2); for
ObjectFFRFreducedOrder: the precomputed interfaces of the markers (Section 5.1, #2184), with a performance case of a
large interface.

## 8. The interfaces of the superelement markers (#3450, #2184)

The interface between the markers and the superelements (`CObjectSuperElement`, `src/System/CObjectBody.h`):

- `SuperElementMarkerInterface` (`src/System/SuperElementMarkerInterface.h`) holds what a marker needs of its mesh nodes
  that depends only on the reference configuration: p_0 = sum_i w_i p_i^ref, the weighting W of the rotation (scalar, or
  its inverse matrix for `useAlternativeApproach`), and for `ObjectFFRFreducedOrder` the matrices Pm = sum_i w_i Psi_i
  and Rm = W^{-1} (sum_i w_i tilde(p_i) Psi_i - tilde(sum_i w_i p_i) Pm), p_i = p_i^ref - p_0.
- The superelement computes it in `InitializeMarkerInterface`, which the marker calls in its new `PostAssemble`
  (`CMarker::PostAssemble`, called after the objects by Assemble). A changed parameter of a marker
  (`CMarker::ParametersHaveChanged`, now called by the generated `SetParameter` of every marker) makes the marker compute
  the part of the mesh nodes on every evaluation until the next Assemble.
- The marker asks the superelement for the weighted local position, velocity, rotation and angular velocity
  (`GetMarkerLocalPosition`, ...) and for the Jacobians (`GetAccessFunctionSuperElement` with the interface); the frame
  comes from `CObjectSuperElement::GetReferenceFrame`. The base class sums over the mesh nodes, `ObjectFFRFreducedOrder`
  multiplies Pm or Rm with the modal coordinates (`precomputeMarkerModes`, default True). The weighting matrix of n x 3
  weights, which no marker used, is gone.
- The rigid marker no longer computes W, p_0 and the frame per call in four places; the check of the regularity of W
  uses the same p_0 as the marker (it added the offset of `localHT` before).

Measured (scratch markerModesBench.py: an NGsolve block 0.2 x 0.4 x 0.4 m, Hurty-Craig-Bampton modes with RBE2
interfaces at both ends, clamped at one end by MarkerSuperElementRigid, a force and a torque on the other end;
200 steps, modified Newton, EigenSparse; regular module):

| nodes | interface nodes | modes | precomputeMarkerModes False | True | factor |
|---:|---:|---:|---:|---:|---:|
| 676 | 136 / 135 | 10 | 0.102 s | 0.003 s | 33 |
| 676 | 136 / 135 | 22 | 0.236 s | 0.006 s | 38 |
| 2203 | 327 / 329 | 10 | 0.242 s | 0.003 s | 76 |
| 2203 | 327 / 329 | 22 | 0.565 s | 0.006 s | 90 |
| 5234 | 613 / 618 | 10 | 0.484 s | 0.003 s | 148 |
| 5234 | 613 / 618 | 22 | 1.102 s | 0.006 s | 171 |

The positions agree to 2e-15. The precomputation at Assemble takes less than a millisecond. Test model
objectFFRFreducedOrderMarkerModesTest.py: the rotor with RBE2 and RBE3 interfaces and 0-11 eigenmodes, both settings
agree to 6e-13, also with weights changed after Assemble.

## 9. The analytic Jacobian of ObjectFFRF (#3447)

`CObjectFFRF::ComputeJacobianODE2_ODE2` gives the ODE2 Jacobian as triplets in the global indices, as ObjectFFRFreducedOrder
does densely (#3212). With the local positions rF and velocities cF_t of the mesh nodes, omegaTilde = I x tilde(omega):

- the stiffness and damping matrices K and D;
- by the mesh coordinates analytic: the mesh rows M (I x (tilde(omega)^2 dq + 2 tilde(omega) dq_t)), from the triplets of
  M; the translation rows A tilde(omega) PHItTM omegaTilde dq + 2 A PHItTM omegaTilde dq_t; the rotation rows per node i
  -G^T ((tilde(omega) (tilde((M a)_i) + E_i) + 2 tilde((M b)_i)) dq_i + 2 E_i dq_t_i), a = omegaTilde rF,
  b = omegaTilde cF_t, E = rfTilde^T M omegaTilde;
- by the rotation parameters and their velocities by local differences of `ComputeFrameTerms` (the quadratic velocity
  vector and the forceVector, which ComputeODE2LHS uses too), with A and G of the node's parametrization at the changed
  parameters.

It is used on Euler parameters and Tait-Bryan angles without a forceUserFunction; `numericalJacobian = True` lets the
system differentiate the object as before. The mass matrix term M(q) q_tt is not differentiated, as for all bodies.

Measured (scratch ffrfJacStudy.py: the NGsolve block of section 7, 20 steps, modified Newton, EigenSparse):

| nodes | jacobianODE2 numerical | analytic | total numerical | analytic |
|---:|---:|---:|---:|---:|
| 166 | 0.073 s | 0.001 s | 0.167 s | 0.095 s |
| 486 | 0.713 s | 0.004 s | 1.596 s | 0.958 s |
| 1525 | 8.193 s | 0.015 s | 24.09 s | 15.96 s |

The coordinates agree to 1.6e-15. At 1525 nodes the rest is the reaction forces of the spherical joint on the
MarkerSuperElementPosition, 12.3 of 14.2 s (0.14 s per Newton iteration, #3452).
