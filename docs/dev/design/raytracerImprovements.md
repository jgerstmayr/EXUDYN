# Raytracer: the cost of shadows and options to reduce it (design note)

Scope: the software raytracer in `src/Graphics/Raytracing.cpp` / `Raytracing.h` and its uniform grid
`SearchTreeBase<RTfloat>` in `src/Linalg/SearchTree.h`; threads of its own since #3438, scaling measured in #3439,
`numberOfThreads = 0` as default since #3445. Line numbers refer to the working tree of 2026-11-12.

## 1. How shadows are computed

### 1.1 One pixel

`ComputePixelColor` (Raytracing.cpp:321) casts the primary ray with `IntersectRayWithTriangles` (closest hit,
Raytracing.cpp:227-317) and shades the hit. For each active light (Raytracing.cpp:451):

- `nLightPoints` (Raytracing.cpp:457-463) is `lightRadiusVariations` (N) only if the light casts a shadow
  (`light.shadow > 0`), is a point light (`position[3] != 0`), has `lightRadius > 0`, `N > 1`, the call is not
  `shadowMode == 2`, and the hit is a primary hit (reflection and transparency depth 0). Otherwise 1.
- Each of the `nLightPoints` light positions is a point on a disk perpendicular to the light direction:
  sample `np` has radius `sqrt(np/N)·lightRadius/lightDistance` and angle `2π·(np/N)·sqrt(N)` (a fixed spiral,
  Raytracing.cpp:499-513), sample 0 is the centre. The same pattern is used for every pixel.
- Per sample: the light position is transformed into the model frame (Raytracing.cpp:479-480, the 3x3 matrix
  extracted from `modelViewRM` again for every sample), an orthogonal basis is built (Raytracing.cpp:510), and
  one shadow ray `IsInShadow` (Raytracing.cpp:196-203, called at Raytracing.cpp:520) is cast from
  `hitPoint + 0.01·lightDir`.
- `IsInShadow` calls `IntersectRayWithTriangles` with `returnOnSingleHit = true`: the traversal returns at the
  first triangle hit with `0 <= t < lightDistance` (Raytracing.cpp:297-302). Shadow rays are therefore already
  any-hit rays with early termination; an occluded ray stops at its first occluder, an unoccluded ray walks the
  grid until it leaves the box (the traversal has no `tmax`, SearchTree.h:727, 744, 796-814 commented out).
- A blocked sample contributes nothing; each unblocked sample adds `1/N` of diffuse and specular light
  (Raytracing.cpp:537-561). A transparent occluder blocks fully (the alpha of the hit is not used in
  `IsInShadow`).

Reflection (Raytracing.cpp:588-595) and refraction (Raytracing.cpp:602-611) recurse into `ComputePixelColor`
with `shadowMode = 0`; there `nLightPoints = 1`, i.e. every reflected or refracted hit casts one hard shadow ray
per shadowed light.

### 1.2 The precomputed shadow at reduced resolution

`preComputeShadow = N > 1 && shadowScalingFactor·AAfactor > 1` (Raytracing.cpp:1650). It does not check
whether any light qualifies for soft shadows (radius, point light, shadow flag).

1. Shadow pass (Raytracing.cpp:1652-1696, parallel): an image of `W·A/(S·A) × H·A/(S·A)` = `W/S × H/S` pixels
   (W, H after `imageSizeFactor`, A = `multiSampling`, S = `shadowScalingFactor`). Per pixel a primary ray and
   `ComputePixelColor(..., shadowMode = 1)`: the light loop with N samples per qualifying light, no shading, no
   reflection or transparency (return at Raytracing.cpp:565). The result is a `ShadowValue` (4 floats, the lit
   fraction per light; 1 for pixels whose primary ray misses, Raytracing.cpp:411).
2. `shadowSmoothingSteps` passes of `SmoothImageValues` at low resolution (Raytracing.cpp:1697-1702,
   Raytracing.h:315-365): a 3x3 box filter per shadowed light, skipped where the 3x3 neighbourhood contains both an
   exact 0 and an exact 1. Each pass copies the whole array twice (Raytracing.h:319 and Raytracing.cpp:1700).
3. `ScaleImageValues` (Raytracing.h:246-269): nearest-neighbour upsampling to the full `W·A × H·A` resolution.
4. `shadowScalingFactor` passes of the 3x3 box filter at full resolution, without the 0/1 test
   (Raytracing.cpp:1707-1711), again with two full copies per pass.

Steps 2-4 run on the calling thread only.

5. Final pass (Raytracing.cpp:1719-1780, parallel), `shadowMode = 2`: the primary ray again at full resolution;
   for each light the precomputed lit fraction replaces the shadow rays (Raytracing.cpp:526-529), one light sample
   (the centre). Reflections and refraction still cast their hard shadow rays.

The filter does not know about depth, normal or object: at a silhouette the shadow value of the floor and of the
object in front are averaged, and a missing primary ray counts as lit (1), so light leaks into the edges of
objects in front of the background.

### 1.3 Cost model

Notation: P = `(A·W/f)·(A·H/f)` rendered pixels (f = `imageSizeFactor`), L = lights with `shadow > 0`,
n_l = N for a light that qualifies for soft shadows and 1 otherwise, R = the secondary rays (per reflective or
transparent hit: one ray plus L hard shadow rays, recursively up to the depths).

| case | rays per image |
|---|---|
| N = 1 | P·(1 + L) + R |
| N > 1, S·A > 1 (precomputed) | P/(S·A)²·(1 + Σ n_l) + P + R, plus serial filtering O(P·S) |
| N > 1, S·A ≤ 1 (S = 0, or S = 1 with A = 1) | P·(1 + Σ n_l) + R |

The shadow pass has `W·H/(f·S)²` pixels, independent of A: with multi-sampling the soft shadow does not become
more expensive, the full-resolution filtering does (A² more pixels).

## 2. Measurements

Scene: `python/TestModels/raytracerNOGLFWtest.py` at 1200 x 1000, perspective, the ten bodies placed on their
circle as the model's `PreStepUserFunction` does at t = 1 (21172 triangles, grid 84 x 84 x 12, box 10 x 10 x 2.9),
light0 at [8, -6, 10, 1], `lightRadius = 0.5`, reflection and transparency depth 2, chrome floor with
reflectivity 0.12. 16 cores / 32 logical processors, `numberOfThreads = 0`, best of 3 images,
`RedrawAndGetImage(True)` (wall time including Python overhead). Script: `python/PerformanceModels/raytracerShadowBenchmark.py`
(#3468; light0 there at the position of this scene).
The installed module 1.13.592.dev1 (copied, so a parallel rebuild did not disturb the run).

The difference column is the mean absolute difference in 0..255 to a reference with N = 240 and S = 0 (240 shadow
rays per pixel, 5.23 s), over the RGB channels of the upper 900 rows; the last column the 99.5 % quantile of the
largest channel difference per pixel. Rows without reflections or with a directional light differ from the
reference by construction; their difference columns are not a quality measure.

| configuration | time [s] | mean diff. | 99.5 % diff. |
|---|---:|---:|---:|
| no shadow | 0.088 | 10.95 | 125 |
| no shadow, no reflection/transparency | 0.052 | - | - |
| hard shadow N = 1 | 0.106 | 1.30 | 43 |
| hard shadow N = 1, no reflection/transparency | 0.061 | - | - |
| N = 2, S = 3, smooth 2 | 0.124 | 0.97 | 31 |
| N = 5, S = 3, smooth 2 | 0.135 | 0.41 | 14 |
| **N = 13, S = 3, smooth 2 (test model settings)** | **0.151** | 0.18 | 6 |
| N = 25, S = 3, smooth 2 | 0.181 | 0.17 | 6 |
| N = 50, S = 3, smooth 2 | 0.239 | 0.11 | 4 |
| N = 13, S = 3, smooth 0 / 8 | 0.146 / 0.161 | 0.21 / 0.17 | 7 / 5 |
| N = 13, S = 0 (per pixel) | 0.355 | 0.29 | 8 |
| N = 13, S = 1 (per pixel, A = 1) | 0.362 | 0.29 | 8 |
| N = 13, S = 2 / 4 / 6 | 0.200 / 0.160 / 0.152 | 0.19 / 0.17 / 0.16 | 6 / 5 / 5 |
| N = 50, S = 0 (per pixel) | 1.116 | 0.17 | 5 |
| N = 13, S = 3, `lightRadius = 0` | 0.131 | 1.21 | 40 |
| N = 1, `lightRadius = 0` | 0.108 | 1.30 | 43 |
| N = 13, S = 3, directional light | 0.127 | - | - |
| N = 1, directional light | 0.124 | - | - |
| A = 2: no shadow / N = 1 / N = 13, S = 3 / N = 13, S = 1 | 0.281 / 0.349 / 0.441 / 0.655 | | |

Single thread against 32 threads:

| configuration | 1 thread [s] | 32 threads [s] | speed-up |
|---|---:|---:|---:|
| no shadow | 1.452 | 0.091 | 16.0 |
| N = 1 | 1.769 | 0.104 | 17.0 |
| N = 13, S = 3, smooth 2 | 1.968 | 0.149 | 13.2 |
| N = 13, S = 6, smooth 8 | 1.850 | 0.144 | 12.8 |

What the numbers say:

- **One shadow sample per full-resolution pixel costs 0.0206 s** at 1.2 Mpixel on 32 threads (linear in N: S = 0,
  N = 13 and N = 50 both give 0.0206 s per sample beyond N = 1), about 17 ns wall time per shadow ray or ~3.5 M
  shadow rays per second per core. Per pixel, the soft shadow without precomputation therefore costs N/4 times a
  complete primary pass with reflections (0.088 s). **This per-pixel path - S = 0, or S = 1 without
  multi-sampling - is the inefficient one: N = 50 takes 10.5 times the time of a hard shadow.**
- With the precomputation (S = 3) a sample costs 1/9 of that (0.0024 s); N = 13 adds 0.045 s (+42 %) to the hard
  shadow, N = 50 adds 0.13 s. The quality at S = 3 is as good as per pixel at the same N (0.18 against 0.29;
  the per-pixel spiral bands, the filtered low resolution does not).
- **About 30 ms of the 149 ms at N = 13, S = 3 is serial** (an Amdahl fit of the two thread counts, with the
  speed-up 16.5 of the parallel part): the smoothing, upsampling and copying of steps 2-4, single-threaded. That
  is 2/3 of what the soft shadow costs over the hard one.
- With `lightRadius = 0` or a directional light and N > 1, the precomputation still runs and costs 24 % (0.131 against
  0.106 s) for no soft shadow; the hard shadow is then computed at 1/S resolution and blurred.
- Reflections and transparency take 40 % of the image without shadows (0.088 against 0.052 s); their hard shadow
  rays are part of every shadowed configuration (0.106 against 0.061 s for N = 1).
- The silhouette artefact ("saw teeth" at the shadow terminator of the faceted ellipsoids) is present at N = 1
  as well; it is the shadow terminator problem of interpolated normals on flat triangles, not an effect of the
  precomputation.

The scaling table of #3439 used the same test model without its `PreStepUserFunction`: all ten bodies at the
origin, one ellipsoid filling the image and the light above it, so almost no visible shadow. It measures the
scaling of the threads correctly but is not representative for shadows; the scene above is.

## 3. Inefficiencies in the code

Gains are estimates for the scene of section 2 unless measured.

| # | where | what | estimated gain | image |
|---|---|---|---|---|
| I1 | Raytracing.cpp:1697-1711, Raytracing.h:246-365 | Smoothing and upsampling single-threaded; each pass copies the full array twice (`smoothedValues = imageValues`, `RTS.shadowValues = RTS.smoothShadowValues`); S passes at full resolution with 4 channels even if one light casts shadows; nearest-neighbour upsampling followed by S box filters. Parallel rows with `RaytracerParallelFor`, ping-pong buffers instead of copies, separable filter, one channel per shadowed light. | 20-25 ms of 150 ms at 32 threads (15 %) | bitwise if the sums keep their order |
| I2 | Raytracing.cpp:1650 | Precomputation for N > 1 even when no light qualifies (radius 0, directional, no shadow). Condition on "some light has `shadow > 0`, `position[3] != 0`, `lightRadius > 0`". | 24 % in those configurations (measured) | changes those images (hard shadow at full resolution instead of blurred) |
| I3 | Raytracing.cpp:98-145 | `IntersectRayWithTriangle` interpolates and normalizes the normal, interpolates the colour and looks up the material for every geometric hit, also for hits farther than the current closest one and for shadow rays, which only need t. Split into a geometric test (t, u, v) and a shading step for the final closest hit (and for the texture-mask test). | 10-20 % of the intersection time | bitwise |
| I4 | GraphicsData.h:114-122, SearchTree.h:411, 461, 491 | The triangle test reads `GLTriangle` (128 bytes: itemID, points, normals, colours) through an index into a per-bin `ResizableArray` (one heap block per bin, a pointer chase per bin visited). A compact array of the 36 bytes of points (or v0, e1, e2) in item order and the bins as one flat index array with offsets (CSR) keep the traversal in cache. | 1.3-2x on traversal and tests | bitwise |
| I5 | Raytracing.cpp:284-311 | No mailboxing: a triangle that overlaps several bins along the ray is tested again in each (triangles and bins have a similar size here, 0.1-0.25). A per-thread "last ray id" stamp per item avoids it. | 10-30 % of the triangle tests | bitwise |
| I6 | Raytracing.cpp:284, SearchTree.h:727-814 | The traversal ignores the ray length: a lit shadow ray walks to the boundary of the grid, also past a light inside the box; the commented `tmax` checks would stop at `min(tMax) > rayLength`. Likewise, the closest-hit cut-off (Raytracing.cpp:287-292) computes a bin box and a squared distance per bin, where `min(tMaxX, tMaxY, tMaxZ) > closestHit.distance` is exact and cheaper. | small for lights outside the scene box; more for lights inside | bitwise (the cut-off: same hits) |
| I7 | Raytracing.cpp:479-480, 499-513 | Per hit and per sample: the 3x3 matrix extracted from `modelViewRM`, the light transformed, `sqrt`, `sin`, `cos`, an orthogonal basis. The light position and the N disk offsets (radius and angle) can be computed once per image and light; the basis once per hit. | a few % at large N | bitwise if the float operations are the same |
| I8 | Raytracing.cpp:499-513 | The same spiral for every pixel: correlated errors that form bands and contours, which is why the description asks to "avoid squares of integers" and why smoothing is needed. | quality per sample (see O3) | - |
| I9 | Raytracing.cpp:1662-1696, 1719-1771 | The primary ray is cast twice for the precomputation (low and full resolution). Small (1/(S·A)²) and the low-resolution hits are not the full-resolution hits anyway. | < 5 % | - |
| I10 | Raytracing.cpp:592, 607 | Reflected and refracted hits cast hard shadow rays at every depth; with a slightly reflective floor every floor pixel adds a reflection ray and a shadow ray although the reflection is weighted by 0.12. | up to 40 % in such scenes (measured share of reflection/transparency) | changes the image |
| I11 | Raytracing.cpp:1511, SearchTree.h:877-912 | A uniform grid with about 4 bins per item (`2·sx, 2·sy, sz`): fine for evenly distributed triangles, poor for a large floor with small detailed objects or one fine mesh in a large scene ("teapot in a stadium"); `searchTreeFactor` is a global knob. | scene-dependent | - |

Not inefficiencies, for completeness: the arithmetic is already `float` (`RTfloat`, Raytracing.h:35); there are no
per-ray heap allocations in the traversal path used (`useFullLineSearch = false`, Raytracing.cpp:233; the per-thread
`tempSearchTreeBins` are only filled by the unused path); shadow rays already terminate at the first hit.

Correctness observations seen on the way (for issues, not for this step): the shadow ray offset `0.01·lightDir`
(Raytracing.cpp:520) is absolute, not relative to the scene size, so a scene in millimetres or kilometres gets
acne or leaks; transparent occluders block the light fully; a directional light never gets soft shadows
(Raytracing.cpp:458), while its description of `lightRadius` does not say so.

## 4. Options for shadows

### O1. Cheap and bitwise identical: I1, I3, I5, I6, I7

Parallel and copy-free filtering removes most of the serial 30 ms; the split intersection test and mailboxing
speed up every ray, shadow rays included. No reference value changes. Effort: 1-2 days together.
Expected: N = 13, S = 3 from 0.151 to about 0.11-0.12 s; the per-pixel paths by 15-30 %.

### O2. Flat grid and compact triangle data (I4)

CSR bins and a triangle array of 36-48 bytes (optionally the precomputed edges) for the test; the `GLTriangle`
only for shading the closest hit. Bitwise as long as the order of the tests is kept. Effort: 2-3 days.
Expected 1.3-2x for all rays.

### O3. Adaptive soft shadows: many samples only in the penumbra

Most pixels are fully lit or in the umbra. Per pixel and light, first k samples (e.g. 4-6, well spread: centre and
the rim of the disk); if they agree, the pixel takes that value; only otherwise the remaining N - k samples. With
the low-resolution shadow pass, a second form: a full-resolution pixel whose low-resolution neighbourhood is all 0
or all 1 takes that value, the others (the penumbra band, typically a few % of the pixels) are traced at full
resolution with N samples - this also removes the upsampling and the blurring at silhouettes, and replaces steps
3-4 of 1.2.

Gain: in the scene of section 2 the average number of samples drops from N to roughly k + (N - k)·p, p the penumbra
fraction (measured in this scene: 6 % of the pixels differ by more than 3/255 between the hard shadow and the
N = 240 reference, 13.5 % are in the hard shadow; with a margin p ≈ 10 %): for k = 5, N = 13 about 6 samples
instead of 13, for N = 50 about 9-10 instead of 50; it makes the per-pixel
path (S = 0/1) affordable, e.g. N = 50 at S = 0 from 1.12 s to an estimated 0.3 s. Changes the image: penumbra
features smaller than the spacing of the first k samples are missed (thin occluders far from the receiver). A
reference test keeps its scene but needs a new checksum; a tolerance-based comparison against a stored low-
resolution image would make such changes reviewable. Effort: 1-2 days.

### O4. Better sample positions: stratified disk with a per-pixel rotation

Concentric (Shirley-Chiu) or Fibonacci-spiral points on the disk, rotated per pixel by a hash or a small blue-noise
tile (Cranley-Patterson rotation). The error becomes uncorrelated noise with no bands, which a small filter removes;
the advice "avoid squares of integers" goes away, and N = 6-8 with a filter looks like N = 13-20 today. Changes the
image; deterministic (no random state), so the test stays reproducible with a new checksum. Effort: half a day.
Combines with O3 (the first k samples are the strata) and O5.

### O5. Edge-aware filter of the shadow term

Replace the 3x3 box filters by a cross-bilateral filter (weights from the depth, already in `RTS.zDepths`, and the
normal or the hit item of each pixel, stored by the passes) and the nearest-neighbour upsampling by joint-bilateral
upsampling. Stops shadow from bleeding across silhouettes and the "missed ray counts as lit" leak, and allows smaller
N or larger S. Changes the image. Effort: 1-2 days; runs in parallel rows.

### O6. Last-occluder cache per thread and light

Before traversing, test the triangle that blocked the previous shadow ray of this thread and light; neighbouring
pixels and neighbouring samples are mostly blocked by the same triangle, so an umbra sample costs one test. Only the
boolean of `IsInShadow` is used, so the result is the same as long as the cached test uses the same
intersection function (theoretically, a triangle the grid would not have listed in a crossed bin could differ at
the tolerance `epsilon5`). Gain: the umbra share of the shadow rays becomes nearly free; in this scene perhaps
10-20 % of the shadow time. Effort: hours.

### O7. A BVH instead of the uniform grid

A binned-SAH BVH (4-wide nodes, AABB slab tests) adapts to non-uniform scenes and is the standard for CPU ray
tracing; with O2's compact data 2-5x faster traversal than the grid in scenes with large size differences, similar
in uniform ones. Build cost O(n log n) per image (the scene changes every frame), comparable to today's grid build for
up to ~1e6 triangles if binned. Order of tests changes, so hits at equal distance (coincident faces) may be resolved
differently: nearly but not exactly bitwise. Effort: 1-2 weeks including tests. Worth it only after O2 if scenes of
users (large FE meshes, STL robots on a large floor) show the grid as the limit.

### O8. Ray packets / SIMD for coherent shadow rays

Shadow rays of neighbouring pixels to the same light are coherent; 4-8 rays per packet through the grid or BVH
with AVX. 2-4x on shadow rays, but a large change and tied to the vector width (the `exudynCPPfast` module is the
AVX2 build). Effort: weeks. Not recommended now.

### O9. Shadow maps or precomputed visibility per vertex - not recommended

A shadow map needs a rasterizer from the light (a cube map for a point light), a depth bias and filtering (PCF/PCSS)
- a second renderer inside the raytracer, with its own artefacts. Visibility per vertex fails for large triangles
(two triangles per floor tile here). Neither fits the raytracer's exact visibility.

### O10. Cheaper secondary shadows (I10)

Skip shadow rays for reflection hits whose accumulated weight (product of reflectivities) is below a threshold, or
use the precomputed lit fraction for them. Changes the image slightly where the reflection is strong; a setting
would keep the current behaviour available. Effort: hours. Gain up to the 0.045 s of reflections and their shadows
in this scene.

### Ranking

| order | option | gain | effort | image |
|---|---|---|---|---|
| 1 | O1 (I1 serial filter, I3, I5, I6, I7) and I2 | 20-30 % for soft shadows, 10-20 % for all | 1-2 days | bitwise; I2 changes configurations without a soft light |
| 2 | O3 adaptive penumbra sampling | 2-5x on the soft shadow term, makes S = 0/1 practical | 1-2 days | changes |
| 3 | O4 stratified rotated samples | same quality at half N | 0.5 day | changes |
| 4 | O6 last-occluder cache | 10-20 % of shadow time | hours | bitwise (practically) |
| 5 | O2 flat grid, compact triangles | 1.3-2x all rays | 2-3 days | bitwise |
| 6 | O5 edge-aware filter and upsampling | quality, allows larger S | 1-2 days | changes |
| 7 | O10 secondary shadows by weight | up to 30 % in reflective scenes | hours | changes (setting) |
| 8 | O7 BVH | 2-5x in non-uniform scenes | 1-2 weeks | nearly bitwise |
| - | O8 packets, O9 shadow maps | - | large | not now |

Recommended order: first the bitwise group (O1, then O6 and O2) behind the unchanged test checksum, measured with the
scene of section 2; then the image-changing group in one go (O4 + O3 + O5, and I2), with the reference value of
`raytracerNOGLFWtest.py` updated once and the images compared by eye and against the N = 240 reference by the
difference measure of section 2 (`rtPenumbra.py` gives the penumbra fraction); O7 only if a user scene shows the grid as the limit.

## 5. Proposed sub-steps

1. Benchmark scene and measure: the test model's scene with the bodies placed and an oblique light, timings and the difference to an N = 240 reference, as a script under `python/PerformanceModels`.
2. The shadow filter and upsampling in parallel rows, without full copies, one channel per shadowed light (I1); bitwise.
3. Precompute shadows only if a light qualifies for soft shadows (I2).
4. Split the triangle test into geometry and shading of the closest hit; hoist the per-sample light terms (I3, I7); bitwise.
5. Mailboxing, the traversal limited by the ray length and an exact closest-hit cut-off (I5, I6); bitwise.
6. Last-occluder cache per thread and light for shadow rays (O6).
7. Flat (CSR) grid and compact triangle array for the traversal (O2); bitwise.
8. Stratified disk samples with a per-pixel rotation (O4).
9. Adaptive penumbra sampling: k samples first, the rest only where they disagree; full-resolution refinement of the low-resolution penumbra (O3).
10. Depth/normal-guided filtering and upsampling of the shadow term (O5); update the reference value of raytracerNOGLFWtest.py once for steps 3, 8-10.
11. Optional: shadow rays of secondary hits by their weight (O10); evaluate a BVH (O7) on large user scenes.

## 6. Done: the image-preserving group and the precomputation (2026-11-12, #3468, #3469)

The plan steps are RG6.26.2-RG6.26.16: the sub-steps of section 5 in their order (RG6.26.2-RG6.26.12), the three
correctness points (RG6.26.13-RG6.26.15) and blurred reflections (RG6.26.16).

- **Benchmark** (RG6.26.2): `python/PerformanceModels/raytracerShadowBenchmark.py` renders the scene of section 2 in six
  configurations and prints the best of three times and an MD5 of each image; `--out`/`--compare` compare two builds,
  `--reference` adds the difference to N = 240. Not in the performance tests (the images differ between platforms).
- **I1** (RG6.26.3): `SmoothShadowValues` and `ScaleShadowValues` in Raytracing.cpp work in blocks of rows on the
  threads of the raytracer, from one array into the other, which are swapped; the old serial functions of
  Raytracing.h are removed.
- **I3, I7** (RG6.26.5): `IntersectRayWithTriangleGeometry` gives t, u, v; `ShadeTriangleHit` (normal, colour,
  material, side) and the texture only for a hit closer than the closest one; the direction and distance of the light
  and the basis of its disk once per light and hit instead of per sample.
- **I5, I6** (RG6.26.6): a mailbox of the last 32 items (by index modulo 32) per ray; the traversal stops when the next
  cell begins beyond the closest hit or the ray length (a relative margin of 1e-4 for the round-off of t), with the
  entry parameter of the grid from `GetBinsCrossedByLineInit`; the bounding-sphere test per cell is gone.
- **I2** (RG6.26.4): the precomputation runs only if an active light with a shadow and a position has `lightRadius > 0`.

Measured with the benchmark, the same machine, best of three (before: 1.13.602 as committed):

| configuration | before [s] | after [s] | change | image |
|---|---:|---:|---:|---|
| no shadow | 0.072 | 0.060-0.062 | -14 to -17 % | same |
| hard shadow | 0.100 | 0.084-0.087 | -14 to -16 % | same |
| N = 13, S = 3 (test model) | 0.143 | 0.111-0.112 | -21 to -23 % | same |
| N = 50, S = 3 | 0.220 | 0.170-0.171 | -22 % | same |
| N = 13, per pixel | 0.335 | 0.264-0.268 | -20 to -21 % | same |
| N = 13, S = 3, `lightRadius = 0` | 0.115 | 0.088 | -23 % | the hard shadow (I2) |

The test suite is unchanged (the checksum of raytracerNOGLFWtest.py and the raytracer cases of pytest). The serial
part of section 2 is gone; what remains at N = 13, S = 3 over the hard shadow is the shadow rays of the low-resolution
pass. The per-pixel path is still N/4 primary passes: that is for the adaptive sampling (RG6.26.10) and the
stratified samples (RG6.26.9).
