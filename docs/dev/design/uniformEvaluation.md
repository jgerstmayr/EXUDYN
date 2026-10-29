# Uniform evaluation of a symbolic system on the CPU and the GPU: design (revision2026c group RG14, #3136)

A system of objects with recorded symbolic user functions - the composed recordings of `SymbolicSystem`
([symbolicSystem.md](symbolicSystem.md)) - evaluated as a fixed sequence of **uniform array operations**: the same
operation on many values at once, as numpy does it on the CPU and a GPU does it in its threads. The parallelism comes
from the objects of the system and, for ensembles, from many states of the same system. This note collects the ideas,
an experiment and the steps. It lives while RG14 is open (WORKFLOW.md §8.4).

## 1. Two ways to the GPU

**One kernel per kind.** The `numpy` writer of `exudyn.symbolics.codeExport` gives scalar Python code per program;
`numba.njit` compiles it, and a loop over the objects of a kind (`prange`) evaluates the kind for all objects, reading
the coordinates through the LTG table and the parameters from the parameter table - one compiled function per kind.
Numba is optional (rule 6): imported at the point of use, with a clear failure. The same structure maps to
`numba.cuda` or CuPy kernels later: one thread per object, the scatter by atomic adds or by a coloring of the objects.
This is the simplest way and fits systems of few kinds of many objects - particles, a mesh of one element type.

**One step of all objects by levels.** It does not fit a system of many different trees: each kind is a launch of its
own, and the threads of a warp that run different programs diverge and wait for each other. The maintainer's idea
(2026-10-26): evaluate **one time step of the whole system in parallel, whatever the trees are**, by making all trees
regular - sections 2 to 4, and the experiment of section 5.

## 2. Levels

The flat programs of all kinds and objects are one directed acyclic graph of scalar operations - the flat programs with
vectors and matrices written as scalars. Each operation gets a **level**, the length of its longest path from the
inputs (as soon as possible); all operations of one level depend only on lower levels and are independent of each
other. A value needed at a higher level than the next is carried up by a **copy** (a no-op $x \to x$), so that every
operation reads only from the level directly below. The evaluation is then a sequence of levels, each a wide, uniform,
element-wise operation over arrays: the values of level k are an array, the operands of level k+1 are gathered from it
by index arrays, and the results form the array of level k+1 - for all objects of all kinds at once.

The values that do not change during one evaluation - the numbers of the programs and the inputs t, q, q_t - are a
**block** that every level reads (constant memory on a GPU), so that they are not carried by copies.

## 3. Uniform operations

Two ways to keep the threads of a warp on the same instruction:

1. **Sorted by operation**: within a level, the operations are sorted by their kind - all `+`, then all `*`, all
   `sin`, ... -, so a warp runs mostly one operation; divergence only at the borders of the groups. No extra levels.
2. **A fixed cycle of operation layers**, the maintainer's proposal: the levels follow a fixed sequence of operation
   kinds, e.g. `+`, `*`, `/`, `sin`, `cos`, `pow`, `-x`, repeated; an operation is placed in the next layer of its kind,
   and every other value passes the layer as a copy. Each layer is one operation for all its entries, without any
   divergence - the cost is the padding: a tree of depth d becomes at most (number of kinds) x d layers, and the
   copies move values. The rare kinds can share one layer of each cycle.

Comparisons and `IfThenElse` become a select of both branches computed (predication), the usual form on a GPU.

Instead of carrying values by copies, all values can be kept in **one array**, one entry per node of the graph: a level
reads from anywhere below it. That costs the memory of every value - for $10^4$ nodes and $10^3$ states 80 MB in double
precision, which a GPU has - and saves all copies.

## 4. A step

Per stage of the explicit integrator (symbolicSystem.md §10), one pass over the levels gives the forces of all objects
and the mass matrices of the bodies; the scatter by the LTG tables is a further level of sums - by atomic adds, or by a
fixed reduction tree, which gives the same bits in every run -; then the batched solve with the mass matrix of each body
and the update of q and q_t, element-wise. A whole step is a fixed sequence of kernels on arrays that stay on the GPU;
only the output steps copy back. With the Jacobian program of option D (symbolicSystem.md §5) the same holds for the
Newton matrix of an implicit step. The composed recordings (#3131) already contain the scatter: their results are the
global forces and the blocks of the bodies.

The width of a level is the parallelism: a system of $10^5$ objects gives levels of $10^5$ to $10^6$ entries, which
fills a GPU. A small system fills it only as an **ensemble** - many states of the same system, a second dimension of
every array: parameter studies, uncertainty, the training data of a surrogate model, sampling-based control.

## 5. The experiment (RG14.2.1, #3137)

[uniformEvaluationExperiment.py](uniformEvaluationExperiment.py) does sections 2 to 4 in about 400 lines of Python on
the system of `python/Examples/symbolicSystemRigidBodyPendulum.py` - a rigid bar on spring-dampers and a torsional
spring-damper, a mass point, gravity and a load of the time - with a chain of `nBars` bars:

1. `Scalarize` reads `GetFlatProgramData()` of the two composed recordings (forces and the mass matrices of the bodies)
   and writes each operation on vectors and matrices as scalar operations, into one `ScalarGraph`: a node with the same
   operation and operands is reused, numbers are combined, `x*1`, `x+0`, `0*x` are dropped. Dot products and matrix
   products are sums in the order of the C++ code.
2. Schedules: by levels (as soon as possible), by levels as late as possible (each operation just below its first use),
   by a cycle of all kinds, by a cycle of the four most frequent kinds and one layer for the rest; and one array of
   all values. `BuildLayers` gives per layer a slice and the index arrays of the operands per kind, `Evaluate` one
   numpy ufunc call per kind and layer, for a batch of states (an array per layer of nodes x states).
3. Checked against `SymbolicSystem.ODE2RHS` and `MassMatrixBlocks` at 20 random states.
4. An ensemble of 100 states integrated by RK44 with the mass matrix of each body solved by `np.linalg.solve` on all
   bodies and states at once; member 0 against `SolveExplicit`.

Measured 2026-10-28 (Windows, one core, numpy 2):

| | 1 bar (9 coordinates) | 10 bars (63 coordinates) |
|---|---|---|
| scalar operations | 523 (`*` 294, `+` 157, `neg` 34, `-` 22, the others 16) | 7732 |
| levels | 29 | 33 |
| entries per operation: by levels | 3.43 | 3.38 |
| as late as possible | 2.48 | 2.91 |
| a cycle of all 9 kinds | 19.9 (182 layers) | 19.1 (209 layers) |
| a cycle of 4 kinds + the rest | 11.2 (102 layers) | 10.8 (117 layers) |
| one array | 1 (573 values) | 1 (7957 values) |
| relative difference to SymbolicSystem | 0 | 1.1e-16 |
| time per state: SymbolicSystem (C++ flat program, one state per call) | 11 us | 57 us |
| one array, 1 state | 162 us | 315 us |
| one array, 1000 states | 1.1 us | 29 us |
| as late as possible, 1000 states | 4.2 us | 83 us |
| RK44, 200 steps, member 0 against SolveExplicit | 1.3e-15 | 2.2e-15 |
| 100 states by RK44 / SolveExplicit of 1 state | 0.21 s / 0.028 s | 1.5 s / 0.10 s |

What it shows:

- **The values are those of Exudyn**, to the last bit for the bar: the same operations in the same order, on the CPU.
- **Copies dominate the levelled form**: 2 to 3.5 entries per operation, mostly values computed early and used late -
  sines and cosines of the angles, the rotation matrices. Placing operations as late as possible removes a third.
- **The fixed cycle costs a factor of 11 to 20** in entries for these programs: `*` and `+` alternate along the products
  and sums, and every other kind in the cycle is a layer of copies between them. On a GPU, a warp with a few kinds of
  operation diverges for a few instructions; a factor of 10 in memory traffic is worse. The cycle pays only if the
  kinds of a layer are many and evenly used, which these programs are not.
- **One array of all values is the fastest on the CPU** and needs no copies: 10 times faster than SymbolicSystem per
  state for the bar with 1000 states, 2 times for the chain; for a single state the about 100 numpy calls per
  evaluation cost more than the C++ program.
- An ensemble of 100 states integrates 7 times faster than 100 runs of SolveExplicit for the chain.

## 6. Limits

- **Not the bits of Exudyn on the GPU.** `sin`, `cos`, `sqrt`, `exp` of CUDA are not the C library's functions of the
  CPU, and the compiler fuses `a*b + c` into one rounding (FMA); results differ in the last bits, and the differences
  grow along a trajectory. A comparison against Exudyn is by a tolerance, not by equality. A sum by atomic adds is not
  even the same from run to run; a fixed reduction tree is.
- **Double precision** is slow on consumer GPUs (1/32 to 1/64 of single precision); single precision changes the
  results by much more than the last bits and needs its own check.
- **Small systems** fill a GPU only as ensembles.
- **Data-dependent branches** (`IfThenElse`) compute both branches.
- **Jacobians, constraints and implicit solvers** come later: the forward derivative of a flat program (#3116) gives
  programs that can be levelled the same way (option D).

## 7. Steps (group RG14)

- RG14.1 (#3136): this design.
- RG14.2 (#3137): the evaluation by numpy on the CPU - RG14.2.1 the experiment above in docs/dev/design; RG14.2.2
  into `exudyn.misc` once the form is chosen: scalarizing, schedule and evaluation of a `SymbolicSystem` or of any
  recording, ensembles in `SolveExplicit`.
- RG14.3 (#3138): the same arrays by CuPy on the GPU, optional (rule 6): the evaluation uses only gathers and ufuncs,
  which CuPy has with the numpy interface; checked by a tolerance against numpy (section 6).
- RG14.4 (#3139): one compiled function per kind by Numba (section 1), with explicit integration and a constant mass
  matrix or the matrix-free product.

## 8. Related work

From memory, not checked against the sources; to be confirmed before RG14.2.2.

- *Levelized evaluation of logic circuits on GPUs*: gate-level simulators sort the gates of a netlist into levels and
  evaluate each level in parallel, with a small set of gate types - the closest analogy; e.g. Chatterjee, DeOrio and
  Bertacco, "GCS: high-performance gate-level simulation with GP-GPUs", DATE 2009.
- *Genetic programming on GPUs*: many different expression trees evaluated in parallel, by an interpreter per thread
  or by evaluating one tree on many data; e.g. Langdon and Banzhaf, "A SIMD interpreter for linear genetic programming
  on GPU graphics cards", EuroGP 2008.
- *Cartesian genetic programming* (Miller, *Cartesian Genetic Programming*, Springer 2011): programs as a fixed grid of
  nodes in columns, with a function set and connections only to earlier columns - the layered form of section 3, way 2.
- *Level scheduling* of sparse triangular solves (Anderson and Saad, 1989): the same leveling of a dependency graph.
- *Wavefront path tracing* (Laine, Karras and Aila, "Megakernels considered harmful", HPG 2013): sorting the work by
  its kind instead of one kernel with branches - section 3, way 1.
- The tensor compilers (XLA, TVM) fuse and schedule graphs of array operations, but for one program on many data, not
  for many different scalar programs.
