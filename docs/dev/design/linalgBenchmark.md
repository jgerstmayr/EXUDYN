# Linear algebra benchmark: results (revision2026c step RG5.1)

A design note of open steps: the measurements of `exudyn.special.RunLinalgBenchmark` (#3410) that steer the open
sub-steps of RG5.1 (RG5.1.2 merge `ResizableVector` into `Vector`, RG5.1.4 inline `Vector::operator[]`) and RG5.2
(compile-time sizes, vectorization). It leaves the repository when the steps close (WORKFLOW.md §8.4); a step that
changes `src/Linalg` adds its run before and after here.

## 1. How to run it

```python
import exudyn as exu
results = exu.special.RunLinalgBenchmark(minimumTime=0.05, numberOfThreads=4)  #list of dicts:
#operation, size, threads, nsPerCall, checkSum
```

- Each operation runs in batches that double until `minimumTime` has passed; the result is the time per call in ns.
  The check sum keeps the compiler from removing the work.
- `numberOfThreads > 1` adds the `ResizableVectorParallel` operations multithreaded for sizes from
  `ResizableVectorParallelThreadingLimit` (20000) on; the task manager is entered and left as in the solvers.
- `EXUDYN_MODULE=fast` measures the fast module (`exudev build --fast` first).
- The source is `src/Linalg/linalgBenchmark.h`; the solver-level counterpart is `tools/benchmarks/avx2Benchmark.py`.

## 2. Results 2026-11-11

Machine: AMD Ryzen 9 9950X (16 cores, 32 threads), Windows 11, MSVC 14.44 with the options of `setup.py`, regular
module (with range checks), Exudyn 1.13.546.dev1; `RunLinalgBenchmark(0.05, 4)` and `(0.05, 16)`. `RVP` is
`ResizableVectorParallel`. Times in ns unless marked us or ms.

### 2.1 Vectors, single-threaded: ns per call

| n | `Vector +=` | `Vector *=` | `Vector MultAdd` | `RVP +=` | `RVP MultAdd` | `RVP CopyFrom` |
|---:|---:|---:|---:|---:|---:|---:|
| 3 | 2.7 | 2.5 | 2.3 | 2.3 | 2.3 | 3.4 |
| 10 | 3.2 | 2.9 | 4.4 | 2.7 | 3.9 | 4.7 |
| 100 | 22.9 | 7.6 | 38.1 | 12.4 | 28.0 | 19.6 |
| 1000 | 240 | 64.7 | 382 | 115 | 274 | 215 |
| 10000 | 2440 | 741 | 3925 | 1324 | 2845 | 2363 |
| 20000 | 4863 | 1481 | 7758 | 2696 | 5627 | 4732 |
| 200000 | 52.2 us | 15.9 us | 87.6 us | 27.9 us | 57.0 us | 93.0 us |
| 1000000 | 254.1 us | 78.1 us | 397.8 us | 138.2 us | 289.2 us | 465.9 us |

### 2.2 Vectors, single-threaded: ns per element

| n | `Vector +=` | `Vector *=` | `Vector MultAdd` | `RVP +=` | `RVP MultAdd` | `RVP CopyFrom` |
|---:|---:|---:|---:|---:|---:|---:|
| 3 | 0.888 | 0.826 | 0.753 | 0.760 | 0.778 | 1.121 |
| 10 | 0.318 | 0.289 | 0.444 | 0.271 | 0.387 | 0.473 |
| 100 | 0.229 | 0.076 | 0.381 | 0.124 | 0.280 | 0.196 |
| 1000 | 0.240 | 0.065 | 0.382 | 0.115 | 0.274 | 0.215 |
| 10000 | 0.244 | 0.074 | 0.393 | 0.132 | 0.285 | 0.236 |
| 20000 | 0.243 | 0.074 | 0.388 | 0.135 | 0.281 | 0.237 |
| 200000 | 0.261 | 0.080 | 0.438 | 0.140 | 0.285 | 0.465 |
| 1000000 | 0.254 | 0.078 | 0.398 | 0.138 | 0.289 | 0.466 |

### 2.3 ResizableVectorParallel, multithreaded: ns per call

| n | `RVP +=` 1 | 4 | 16 | `RVP MultAdd` 1 | 4 | 16 | `RVP CopyFrom` 1 | 4 | 16 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20000 | 2696 | 1816 | 2406 | 5627 | 2290 | 2691 | 4732 | 1671 | 2176 |
| 200000 | 27.9 us | 20.7 us | 9128 | 57.0 us | 30.4 us | 11.9 us | 93.0 us | 18.8 us | 9983 |
| 1000000 | 138.2 us | 103.5 us | 45.8 us | 289.2 us | 149.7 us | 52.8 us | 465.9 us | 86.2 us | 42.8 us |

The threaded times scatter between runs: a first run with 4 threads (`minimumTime=0.02`) gave 52.0 us for `RVP +=`
at 10^6 and 1501 ns at 20000, against 103.5 us and 1816 ns here. Compare threaded runs only within one session.

### 2.4 Small fixed-size types, transformations, matrix times vector

| operation | size | ns per call |
|---|---:|---:|
| `SlimVector<3> +` | 3 | 5.6 |
| `SlimVector<3> cross product` | 3 | 9.8 |
| `Matrix3D * Vector3D` | 3 | 7.5 |
| `Matrix3D * Matrix3D` | 3 | 20.5 |
| `ConstSizeMatrix<36> * ConstSizeVector<6>` | 6 | 42.5 |
| `ConstSizeMatrix<36> * ConstSizeMatrix<36>` | 6 | 125 |
| `HT * v, rotation` | 3 | 8.3 |
| `HT * v, translation` | 3 | 10.0 |
| `HT * HT, rotation` | 3 | 13.4 |
| `HT * HT, translation` | 3 | 11.6 |
| `HT inverse, rotation` | 3 | 8.5 |
| `HT inverse, translation` | 3 | 9.0 |
| `Matrix * Vector` | 10 | 66.1 |
| `Matrix * Vector` | 100 | 14.5 us |
| `Matrix * Vector` | 1000 | 2.08 ms |

The translation cases of `HT * v` and `HT * HT` contain a scaling by 0.5 more than the rotation cases, which keeps the
values bounded; they are not a measure of the no-rotation flag (that is RG5.1.1, #3405).

## 3. What the results say for the open steps

- **RG5.1.2, RG5.1.4**: `Vector` is slower than `ResizableVectorParallel` for the same operation from 100 elements
  on, single-threaded: += 1.8 to 1.9 times, `MultAdd` 1.4 times. Below 10 elements both are at the cost of the call
  (2-5 ns). The loops differ: `VectorBase::operator+=` iterates over the other vector and adds by `GetUnsafe`, one
  element at a time; `ResizableVectorParallelBase::operator+=` works in blocks of the AVX width
  (`nItems >> AVXRealShift`). The merge would give `Vector` the block loops; the fast module (AVX2) shows how much
  more the blocks gain there.
- **`CopyFrom`** of `ResizableVectorParallel` costs 0.47 ns per element from 200000 elements on, twice as much as below
  and three times `+=`, which reads two vectors and writes one; not explained yet.
- **Threads**: at the limit of 20000 elements 4 threads gain 1.5 to 2.8 times, 16 threads less than 4; at 10^6, 16
  threads gain 3 to 11 times. A higher limit, or a number of threads that depends on the size, would fit these
  numbers; RG5.1.2 decides together with the merge.
- **RG5.2**: the dense `Matrix * Vector` costs 0.66 ns per multiply-add at n = 10, 1.45 ns at 100 and 2.1 ns at 1000,
  against 0.1 to 0.4 ns per element of the vector operations; 6 x 6 times 6 x 6 (216 multiply-adds) takes 125 ns,
  `Matrix3D * Matrix3D` (27) 20.5 ns. These are the candidates for compile-time sizes and vectorization.

## 4. Open runs

- The fast module (`EXUDYN_MODULE=fast`, AVX2, no range checks), for the block loops of 2.1 with the wider AVX.
- Linux (gcc) and macOS (clang), for the same table.
