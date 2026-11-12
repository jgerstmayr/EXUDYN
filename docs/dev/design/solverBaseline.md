# The solver baseline

A design note of revision2026c step RG5.8 (#3002, RG5.8.1): the time of the solvers on the models of
`python/TestModels/modelLibrary.py` and larger variants of them, as the baseline for the decision on total coordinates
(RG5.8.2) and for other performance steps. Produced by `python/testing/solverBaseline.py` (the CSV and Markdown files go
to `python/logs/performance`, not tracked); `EXUDYN_MODULE=fast` measures the fast module.

## 1. The run of 2026-11-12

Exudyn 1.13.577.dev1, one machine (16 cores / 32 logical processors, Windows, MSVC), Python 3.13, the regular and the
fast module. 420 cases per module (13 models; GeneralizedAlpha, TrapezoidalIndex2, ExplicitEuler, RK44, DOPRI5;
EigenDense and EigenSparse; modified and full Newton; two step sizes; 1 and 4 threads), none failed, 804 s per module.
Each case is repeated at least three times and until 0.5 s have passed; the smallest time counts. Explicit solvers are
skipped for models with constraints; the drift check of the index-2 solver is off (time, not accuracy, is measured).

The base configuration is EigenSparse, modified Newton, one thread, the larger step; the ratios are the time of the
other configuration over the base time of the regular module.

| model | coordinates | solver | base [s] regular | fast | steps | Newton steps | Jacobians | dense | full | h/4 | 4 threads |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| massOscillator | 3 | GeneralizedAlpha | 0.000351 | 0.000317 | 501 | 500 | 1 | 0.99 | 4.42 | 3.73 | 1.35 |
| massOscillator | 3 | TrapezoidalIndex2 | 0.000347 | 0.000309 | 501 | 500 | 1 | 1.01 | 4.45 | 3.71 | 1.53 |
| massOscillator | 3 | ExplicitEuler | 0.000291 | 0.000273 | 501 |  |  |  |  | 3.77 | 1.70 |
| massOscillator | 3 | RK44 | 0.000645 | 0.000592 | 501 |  |  |  |  | 3.92 | 1.34 |
| massOscillator | 3 | DOPRI5 | 0.001057 | 0.000973 | 501 |  |  |  |  | 4.01 | 1.15 |
| nPendulum | 16 | GeneralizedAlpha | 0.008013 | 0.007394 | 501 | 3347 | 96 | 1.03 | 1.40 | 3.65 | 1.07 |
| nPendulum | 16 | TrapezoidalIndex2 | 0.008205 | 0.007563 | 501 | 3358 | 112 | 1.02 | 1.51 | 3.63 | 1.03 |
| rigidBodyJoints | 39 | GeneralizedAlpha | 0.005774 | 0.004967 | 251 | 1805 | 30 | 1.11 | 1.89 | 3.60 | 1.02 |
| rigidBodyJoints | 39 | TrapezoidalIndex2 | 0.00662 | 0.005542 | 251 | 1834 | 62 | 1.10 | 3.04 | 3.52 | 1.06 |
| rigidBodySpringDampers | 24 | GeneralizedAlpha | 0.00655 | 0.005817 | 251 | 1738 | 31 | 1.01 | 5.49 | 3.16 | 1.03 |
| rigidBodySpringDampers | 24 | TrapezoidalIndex2 | 0.006506 | 0.005781 | 251 | 1737 | 30 | 1.02 | 5.95 | 3.20 | 1.03 |
| sliderCrank3D | 42 | GeneralizedAlpha | 0.00523 | 0.00456 | 201 | 1331 | 4 | 1.13 | 1.33 | 3.96 | 1.18 |
| sliderCrank3D | 42 | TrapezoidalIndex2 | 0.005573 | 0.00474 | 201 | 1333 | 4 | 1.13 | 1.27 | 3.96 | 1.05 |
| kinematicTree | 3 | GeneralizedAlpha | 0.001999 | 0.001734 | 251 | 2168 | 2 | 0.96 | 1.30 | 3.60 | 1.09 |
| kinematicTree | 3 | TrapezoidalIndex2 | 0.001986 | 0.001726 | 251 | 2171 | 2 | 0.98 | 1.32 | 3.62 | 1.10 |
| kinematicTree | 3 | ExplicitEuler | 0.001489 | 0.001303 | 251 |  |  |  |  | 3.91 | 1.12 |
| kinematicTree | 3 | RK44 | 0.00361 | 0.003205 | 251 |  |  |  |  | 3.97 | 1.05 |
| kinematicTree | 3 | DOPRI5 | 0.01031 | 0.008745 | 418 |  |  |  |  | 2.42 | 1.04 |
| beam3D | 36 | GeneralizedAlpha | 0.016506 | 0.013572 | 201 | 1055 | 2 | 1.02 | 1.52 | 3.08 | 1.05 |
| beam3D | 36 | TrapezoidalIndex2 | 0.01649 | 0.013585 | 201 | 1051 | 2 | 1.03 | 1.53 | 3.09 | 1.02 |
| FFRFreducedOrder | 18 | GeneralizedAlpha | 0.004314 | 0.003319 | 101 | 497 | 5 | 0.98 | 1.19 | 3.95 | 1.05 |
| FFRFreducedOrder | 18 | TrapezoidalIndex2 | 0.004374 | 0.003324 | 101 | 497 | 5 | 0.98 | 1.18 | 3.94 | 1.05 |
| genericDAE | 3 | GeneralizedAlpha | 0.013071 | 0.013011 | 201 | 1202 | 22 | 1.00 | 0.79 | 3.80 | 1.04 |
| genericDAE | 3 | TrapezoidalIndex2 | 0.013056 | 0.013087 | 201 | 1209 | 22 | 1.00 | 1.08 | 3.81 | 1.18 |
| rigidBodyJoints50 | 650 | GeneralizedAlpha | 0.09063 | 0.081764 | 251 | 1373 | 104 | 11.35 | 2.46 | 2.96 | 0.86 |
| rigidBodyJoints50 | 650 | TrapezoidalIndex2 | 0.096207 | 0.084139 | 251 | 1340 | 116 | 11.29 | 2.53 | 2.93 | 0.86 |
| rigidBodySpringDampers50 | 400 | GeneralizedAlpha | 0.082046 | 0.072295 | 251 | 1655 | 10 | 2.57 | 8.31 | 3.61 | 0.64 |
| rigidBodySpringDampers50 | 400 | TrapezoidalIndex2 | 0.083907 | 0.073246 | 251 | 1653 | 11 | 2.55 | 8.43 | 3.57 | 0.66 |
| beam3D60 | 372 | GeneralizedAlpha | 0.244588 | 0.201458 | 201 | 1150 | 3 | 1.35 | 1.39 | 3.65 | 0.49 |
| beam3D60 | 372 | TrapezoidalIndex2 | 0.225926 | 0.187335 | 201 | 1049 | 3 | 1.35 | 1.50 | 3.99 | 0.48 |
| contactSpheres | 324 | ExplicitEuler | 0.087576 | 0.079496 | 1001 |  |  |  |  | 3.99 | 0.63 |
| contactSpheres | 324 | RK44 | 0.219654 | 0.197896 | 1001 |  |  |  |  | 4.03 | 0.61 |
| contactSpheres | 324 | DOPRI5 | 0.437835 | 0.396902 | 1174 |  |  |  |  | 3.36 | 0.61 |

## 2. What it says

- **Cost per step.** The small models take 0.7-8 us per step (`massOscillator` 0.7 us, `nPendulum` 16 us with 6.7
  Newton iterations per step); the largest, `beam3D60` (372 coordinates), 1.2 ms per step, and `contactSpheres` (324
  coordinates, explicit) 87 us per step with ExplicitEuler and 220 us with RK44.
- **The fast module** is 5-25 % faster (most 10-15 %); `genericDAE`, whose time is in Python user functions, not at all.
- **Threads.** Four threads make every model up to 42 coordinates slower (1.02-1.70 times) and pay only from about 300
  coordinates on (0.48-0.86): the threads are worth it for the large systems only; `parallel.numberOfThreads = 1` stays
  the right default.
- **Dense versus sparse.** Up to 42 coordinates the two are equal within 13 %; the rigid-body chains of 400-650 coordinates
  are 2.6-18 times slower with EigenDense (`rigidBodyJoints50`: 11 times in the regular, 17-18 times in the fast
  module), `beam3D60` (372 coordinates, three Jacobians in the run) 1.35 times.
- **Full Newton** costs 1.2-8.6 times modified Newton; most for the chains of spring-dampers, whose Jacobian is
  expensive and changes little; `genericDAE` is the exception (0.79 with GeneralizedAlpha).
- **A quarter of the step size** costs 2.4-4.0 times: the implicit solvers need fewer Newton iterations per step with the
  smaller step, DOPRI5 fewer step rejections.

## 3. For the total coordinates (RG5.8.2)

Total coordinates would replace the addition of reference and current coordinates in the items by one addition per
evaluation of the system. A vector addition costs about 0.1 ns per coordinate in the regular and 0.05 ns in the fast
module (`linalgBenchmark.md` §3.6, `Vector +=` at n = 500). For the cheapest case per coordinate, `contactSpheres` with
RK44 (four evaluations per step), that is 4 x 324 x 0.1 ns = 0.13 us per step against 220 us: 0.06 %. The additions
inside the items cost of the same order, since every item adds only its own few coordinates. Total coordinates can
therefore save at most a fraction of a percent of the solver time in these models; RG5.8.2 decides on this basis.
