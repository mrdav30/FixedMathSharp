# Benchmark Hotspot Optimization Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before changing runtime code for any measured
> hotspot, use superpowers:test-driven-development for behavior-preserving
> optimizations, and use superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Done

**Goal:** Convert the 2026-06-04 full benchmark baseline into focused,
correctness-first optimization work for measured hotspots only.

**Architecture:** Keep benchmark findings separate from regression guardrail
work. This plan tracks candidate runtime/design optimizations; the benchmark
regression plan tracks repeatability, reporting, and CI posture.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, BenchmarkDotNet,
xUnit, MemoryPack, System.Text.Json, `Release`, and `ReleaseLean`.

---

## Phase 1: Frustum Construction Allocation

**Status:** Complete on 2026-06-04.

**Why:** `BoundsBenchmarks.FrustumCreateFromMatrix` was the only
non-serialization benchmark with managed allocation in the full baseline:
933,380.6 ns and 165,888 B per benchmark operation.

**Likely cause:** `FixedBoundFrustum` is a sealed reference type and owns
`Vector3d[8]` plus `FixedPlane[6]` arrays per instance.

- [x] Decide whether frustum construction is expected to be frequent enough to
  justify a public allocation-free construction path.
- [x] Investigate internal fixed-field storage, caller-provided buffers, pooling,
  or a value-type/lightweight frustum variant without breaking existing public
  API expectations.
- [x] Preserve `GetCorners`/`GetPlanes` array overloads and existing allocation
  regression tests.
- [x] Add focused benchmarks for constructor-only cost, matrix-set cost, and
  repeated contains/intersects reuse.

Result:

- Converted `FixedBoundFrustum` from a sealed reference type with owned plane
  and corner arrays into a value type with fixed plane/corner fields. This is an
  intentional major-version breaking change that removes per-frustum object and
  array allocation instead of adding a pooled or parallel lightweight API.
- Added allocation regression tests for `new FixedBoundFrustum(matrix)` and
  `FixedBoundSphere.CreateFromFrustum(frustum)`.
- Added benchmark probes for `FrustumConstructOnly` and `FrustumSetMatrix`; the
  existing frustum containment/intersection benchmarks continue to cover reused
  frustum scenarios.
- Removed obsolete null-frustum tests and null guards at call sites, because a
  value-type frustum is no longer nullable.

Focused `ShortRun` evidence:

| Benchmark | Before | After |
| --- | ---: | ---: |
| `FrustumCreateFromMatrix` | 881.994 us, 165,888 B | 853.613 us, 0 B |
| `FrustumConstructOnly` | not captured | 858.154 us, 0 B |
| `FrustumSetMatrix` | not captured | 829.007 us, 0 B |
| `FrustumContainsPoint` | 17.958 us, 0 B | 17.179 us, 0 B |
| `FrustumIntersectsSphere` | 29.102 us, 0 B | 28.128 us, 0 B |
| `FrustumIntersectsRay` | 122.210 us, 0 B | 118.882 us, 0 B |
| `FrustumGetCornersIntoArray` | 2.370 us, 0 B | 1.880 us, 0 B |
| `FrustumGetPlanesIntoArray` | 2.202 us, 0 B | 1.140 us, 0 B |

Note: `FrustumIntersectsBox` measured 33.620 us before and 38.621 us after in
the focused short runs. The allocation fix does not change box/frustum
algorithmic complexity, so treat this as a follow-up candidate only if a full
baseline confirms a real regression outside short-run noise.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedBoundFrustum"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds --filter "*Frustum*" --exporters json
```

## Phase 2: Scalar Transcendental Speed

**Status:** Complete on 2026-06-04.

**Why:** The slowest allocation-free scalar methods were `Pow` (136.806 us),
`Acos` (124.290 us), `Tan` (103.154 us), `Atan2` (93.539 us), `Atan`
(92.729 us), and `Asin` (77.112 us).

- [x] Review algorithms in `FixedMath` and `FixedTrigonometry` for iteration
  counts, range reduction, table lookup, division count, and branch shape.
- [x] Add correctness sweeps before optimization for edge cases, singularities,
  sign handling, and deterministic raw-value expectations.
- [x] Benchmark each candidate independently before and after changes.
- [x] Prefer deterministic algorithmic wins over lookup-table growth unless the
  table size, cache behavior, and `netstandard2.1` impact are justified.

Result:

- Factored the reduced sine polynomial into an aggressively inlined helper so
  `Sin` and other already-reduced callers can share the same polynomial without
  repeating expression shape.
- Replaced `Tan`'s continued-fraction path with deterministic principal-range
  reduction followed by `sin(x) / cos(x)` using the reduced sine polynomial.
  This removes the per-call variable-depth fraction loop and was the largest
  scalar win in this phase.
- Added a large-input `Atan` identity for `|z| > 1`, reducing expensive Taylor
  work before the existing `z > 0.5` identity path.
- Added contract-level reference sweeps for tangent principal-range values and
  arctangent values above and below one, and removed a stale tangent test that
  asserted details of the old continued-fraction implementation.
- Benchmarked but rejected a direct `Cos` rewrite and an alternate `Asin`
  half-angle identity because the focused `ShortRun` measurements did not show
  a clear improvement.
- Left `Pow`, `Log2`, and `Ln` unchanged in this pass. The follow-up scalar
  pow/log pass was captured and completed as Phase 2b below.

Focused `ShortRun` evidence from this phase:

| Benchmark | Before | After |
| --- | ---: | ---: |
| `Sin` | 37.282 us, 0 B | 37.431 us, 0 B |
| `Cos` | 44.898 us, 0 B | 42.402 us, 0 B |
| `Tan` | 95.667 us, 0 B | 59.946 us, 0 B |
| `Acos` | 127.092 us, 0 B | 117.660 us, 0 B |
| `Asin` | 80.325 us, 0 B | 82.613 us, 0 B |
| `Atan` | 86.645 us, 0 B | 66.957 us, 0 B |
| `Atan2` | 89.267 us, 0 B | 86.025 us, 0 B |
| `Pow` | 122.432 us, 0 B | 123.863 us, 0 B |
| `Log2` | 34.097 us, 0 B | 34.382 us, 0 B |
| `Ln` | 42.336 us, 0 B | 41.323 us, 0 B |

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Fixed64|FullyQualifiedName~FixedTrigonometry"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i --exporters json
```

## Phase 2b: Scalar Pow And Log Follow-Through

**Status:** Complete on 2026-06-05.

**Files:**

- Review: `src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Review: `tests/FixedMathSharp.Tests/Core/FixedTrigonometry.Tests.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`

- [x] Capture focused baseline numbers for `Pow`, `Pow2`, `Log2`, and `Ln`.
- [x] Add correctness sweeps for powers of two, fractional exponents, log
  identities, rounding behavior, and invalid inputs before runtime changes.
- [x] Prefer removing repeated work and iteration count over lookup-table growth
  unless a table proves a clear cache-friendly win on `netstandard2.1`.
- [x] Re-benchmark each changed method and record the before/after result.

Context: Phase 2 of the hotspot optimization plan improved trig-heavy scalar
paths, but deliberately left `Pow`, `Pow2`, `Log2`, and `Ln` unchanged because
they need a focused algorithm pass. `Pow` composes `Log2` and `Pow2`, so it is
the clearest end-user hotspot, while `Ln` is a thin `Log2` derivative. This
phase should keep the work measurement-driven and avoid changing approximation
semantics unless tests prove the new raw results stay within accepted
deterministic tolerances.

Phase 2b result on 2026-06-05: completed. `Pow2` now uses compact Q32.32
fractional-bit lookup tables for positive and negative exponents instead of a
division-heavy Taylor loop. `Log2` now normalizes by integer bit position instead
of repeated shifting, and `Ln` no longer rounds the final result to an integer.
This also fixed two correctness issues discovered during the phase:
`Pow2(31)` wrapped negative instead of saturating, and `Ln` discarded fractional
log results such as `ln(0.125)`.

Focused `ShortRun` evidence:

| Benchmark | Before | After |
| --- | ---: | ---: |
| `Pow` | 118.184 us, 0 B | 61.974 us, 0 B |
| `Pow2` | 75.779 us, 0 B | 9.349 us, 0 B |
| `Log2` | 33.442 us, 0 B | 31.224 us, 0 B |
| `Ln` | 41.089 us, 0 B | 33.225 us, 0 B |

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedTrigonometry"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i --filter "*Pow*" "*Log2*" "*Ln*" --exporters json
```

## Phase 2c: Scalar Sqrt Follow-Through

**Status:** Complete on 2026-06-05.

**Files:**

- Review: `src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Review: `tests/FixedMathSharp.Tests/Core/FixedTrigonometry.Tests.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Vector4dBenchmarks.cs`

- [x] Add focused `Sqrt` baseline plus consumer probes where useful.
- [x] Strengthen correctness tests for tiny raw values, fractional inputs,
  perfect squares, huge raw values, and invalid negatives.
- [x] Investigate deterministic integer/Newton-style approaches against the
  current bit-by-bit method.
- [x] Keep only measured wins with no allocation and no precision drift beyond
  existing tolerance.

Context: `Sqrt` was not changed during Phase 2 or Phase 2b, but it remains a
core scalar primitive and feeds vector normalization, hypotenuse, bound/ray
queries, quaternion normalization/conversion, and `Acos`. Treat this as the last
scalar-primitives cleanup pass before moving into quaternion and vector-heavy
phases.

Result:

- Kept the deterministic bit-by-bit integer square-root algorithm rather than
  switching to a Newton/division-based method. The measured win came from
  removing setup work inside the existing integer path, so there was no need to
  introduce fixed-point division or new convergence behavior.
- Added a zero fast path and derived the first candidate bit directly from the
  input's top set bit via `FloorLog2(num) & ~1`, replacing the repeated
  top-down scan from bit 62.
- Added focused correctness coverage for tiny raw values, fractional inputs,
  perfect squares, huge raw values, `Fixed64.MaxValue`, zero, and invalid
  negatives.
- Ran consumer probes for vector and quaternion normalization. Short-run
  consumer measurements were noisy, so only the direct scalar `Sqrt` result is
  treated as the phase's performance claim.

Focused `ShortRun` evidence:

| Benchmark | Before | After |
| --- | ---: | ---: |
| `Sqrt` | 30.767 us, 0 B | 23.037 us, 0 B |

Consumer smoke evidence before the runtime change:

| Benchmark | Before |
| --- | ---: |
| `Vector2d.Normal` | 54.742 us, 0 B |
| `Vector2d.NormalizeInPlace` | 57.415 us, 0 B |
| `Vector2d.GetNormalized` | 56.478 us, 0 B |
| `Vector3d.Normal` | 65.445 us, 0 B |
| `Vector3d.NormalizeInPlace` | 65.888 us, 0 B |
| `Vector3d.GetNormalized` | 64.320 us, 0 B |
| `Vector4d.Normal` | 81.110 us, 0 B |
| `Vector4d.NormalizeInPlace` | 77.945 us, 0 B |
| `Vector4d.GetNormalized` | 78.070 us, 0 B |
| `Quaternion.Normalize` | 81.909 us, 0 B |

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Sqrt|FullyQualifiedName~Vector2d|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d|FullyQualifiedName~FixedQuaternion"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic vector2d vector3d vector4d quaternion -j Short -i --filter "*Sqrt*" "*Normal*" "*Normalize*" --exporters json
```

## Phase 3: Quaternion Direction And Euler Paths

Status: Completed on 2026-06-05.

**Why:** Quaternion conversion-heavy paths are among the slowest allocation-free
runtime operations: `FromEulerAngles` (468.24 us), `FromDirection` (443.02 us),
`ToEulerAngles` (297.75 us), `Slerp` (256.60 us), and `FromAxisAngle`
(216.72 us).

- [x] Separate cost inherited from scalar trig from quaternion-specific
  normalization, branch, and multiplication work.
- [x] Reuse coordinate-convention tests to keep forward/up handedness stable.
- [x] Add focused benchmarks for common identity, cardinal-axis, near-parallel,
  and anti-parallel inputs.
- [x] Avoid changing public convention semantics while optimizing.

Implementation notes:

- `FromDirection` now uses the deterministic unit-vector-to-quaternion form for
  the common path, while keeping explicit identity, opposite-direction, and
  near-opposite fallbacks so forward/up handedness and near-180 degree precision
  stay stable.
- `FromAxisAngle` now checks squared axis length before normalizing, avoiding
  the extra square root for already-normalized axes and skipping result
  normalization when the axis/half-angle construction already produces the
  rotation.
- A direct `ToEulerAngles` term rewrite was measured and rejected because it did
  not beat the matrix-based path in the short benchmark run.
- Removing `FromEulerAngles` result normalization was measured and rejected
  because it improved construction in isolation but caused downstream
  quaternion normalization work to regress.

Measured short-run result:

| Benchmark | Before | After |
| --- | ---: | ---: |
| `Quaternion.FromAxisAngle` | 196.100 us, 0 B | 150.884 us, 0 B |
| `Quaternion.FromEulerAngles` | 429.983 us, 0 B | 427.959 us, 0 B |
| `Quaternion.Normalize` | 90.859 us, 0 B | 88.443 us, 0 B |
| `Quaternion.ToEulerAngles` | 292.361 us, 0 B | 291.418 us, 0 B |
| `Quaternion.FromDirection` | 428.156 us, 0 B | 107.838 us, 0 B |
| `Quaternion.FromAxisAngleCardinal` | not captured | 127.005 us, 0 B |
| `Quaternion.FromDirectionCardinal` | not captured | 1.251 us, 0 B |
| `Quaternion.FromDirectionNearParallel` | not captured | 22.505 us, 0 B |
| `Quaternion.FromDirectionNearAntiParallel` | not captured | 60.774 us, 0 B |

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedQuaternion"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll quaternion -j Short -i --filter "*FromAxisAngle*" "*FromDirection*" "*FromEulerAngles*" "*ToEulerAngles*" "*Normalize*" --exporters json
```

## Phase 4: Matrix Rotation, Transform Composition, And Inversion

**Status:** Complete on 2026-06-05.

**Why:** Matrix-heavy methods are high in the baseline without managed
allocation: `Matrix3x3.CreateRotation` (467.73 us), `Matrix4x4.InvertFull`
(390.78 us), `Matrix4x4.TranslateRotateScale` (287.10 us),
`Matrix4x4.ScaleRotateTranslate` (267.88 us), `Matrix4x4.InvertAffine`
(215.20 us), and `Matrix4x4.CreateTransform` (201.88 us).

- [x] Distinguish trig-driven rotation cost from matrix multiplication and
  inversion cost.
- [x] Confirm affine inversion stays on the affine path and does not regress
  full inversion correctness.
- [x] Add focused correctness tests for singular, near-singular, affine, and
  non-affine matrices before optimizing.
- [x] Benchmark composition order variants with the same deterministic fixture
  set.

Implementation notes:

- `Fixed4x4.CreateTransform` now constructs the transform directly from
  `FixedQuaternion.ToMatrix3x3()` with scaled basis rows and translation,
  removing temporary `Fixed4x4` construction, rotation-basis normalization, and
  scale/translation helper calls from the hot path.
- `Fixed4x4.ScaleRotateTranslate` now delegates to the direct transform builder
  because its explicit `Scale * Rotation * Translation` result matches the same
  matrix layout.
- `Fixed4x4.TranslateRotateScale` now builds its `Translation * Rotation *
  Scale` result directly instead of allocating work to three temporary matrices
  and two matrix multiplications.
- Affine inversion keeps the existing public point-transform convention but now
  computes the inverse translation directly, avoiding temporary `Fixed3x3` and
  `Vector3d` values.
- Added `Fixed4x4.FuzzyEqualAbsolute` and `Fixed4x4.FuzzyEqual` extension
  methods to match the existing `Fixed3x3` comparison API and support matrix
  regression tests.
- `Matrix3x3.CreateRotation` remains primarily trig-bound and was not changed
  in this phase.
- Captured `FMS-Issue-009` in `docs/feature-work/issue-tracker.md` for the
  deeper `Fixed4x4` convention mismatch between public point transforms and
  affine matrix composition identity proofs.

Focused `ShortRun` evidence:

| Benchmark | Before | After |
| --- | ---: | ---: |
| `Matrix3x3.CreateRotation` | 449.886 us, 0 B | 431.061 us, 0 B |
| `Matrix3x3.Invert` | 146.526 us, 0 B | 145.205 us, 0 B |
| `Matrix4x4.CreateRotation` | 113.116 us, 0 B | 111.021 us, 0 B |
| `Matrix4x4.CreateTransform` | 181.935 us, 0 B | 152.313 us, 0 B |
| `Matrix4x4.ScaleRotateTranslate` | 249.063 us, 0 B | 152.532 us, 0 B |
| `Matrix4x4.TranslateRotateScale` | 265.813 us, 0 B | 185.691 us, 0 B |
| `Matrix4x4.InvertAffine` | 212.357 us, 0 B | 195.912 us, 0 B |
| `Matrix4x4.InvertFull` | 435.059 us, 0 B | 385.024 us, 0 B |

Note: `Matrix4x4.InvertFull` was not changed directly, so the lower focused
short-run result should be treated as noise rather than a claimed algorithmic
win. A combined `matrix3x3 matrix4x4` short run hit a one-off in-process
BenchmarkDotNet `SEHException` during `Matrix3x3.Invert`; the isolated
`Matrix3x3.Invert` rerun passed, so no issue was tracked unless it reproduces.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Fixed3x3|FullyQualifiedName~Fixed4x4"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix3x3 matrix4x4 --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix4x4 -j Short -i --filter "*CreateRotation*" "*CreateTransform*" "*ScaleRotateTranslate*" "*TranslateRotateScale*" "*Invert*" --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix3x3 -j Short -i --filter "*Invert*" --exporters json
```

## Phase 5: Serialization Cost Review

**Why:** Serialization dominates absolute time and allocation, especially JSON:
`JsonRoundTripCurve` (4.115 ms, 3,041,448 B), `JsonSerializeBoundsAndCurves`
(1.908 ms, 1,362,672 B), and `JsonSerializeMathTypes` (1.498 ms,
1,223,096 B). MemoryPack is much cheaper but still allocates payload buffers.

- [x] Treat JSON and MemoryPack payload allocation as intentional until a focused
  benchmark proves avoidable FixedMathSharp-side work.
- [x] Keep serialization benchmarks conditional under
  `FIXEDMATHSHARP_DISABLE_MEMORYPACK` for lean builds.
- [x] Consider documentation or sample guidance before runtime changes if the
  measured cost is primarily serializer behavior.
- [x] Only open runtime work if source inspection finds avoidable intermediate
  objects or repeated metadata setup inside FixedMathSharp-owned code.

Phase 5 result:

- Confirmed scalar/vector MemoryPack deserialization remains effectively
  allocation-free; serialization allocations mostly come from requested payload
  buffers.
- Found one FixedMathSharp-owned curve cost: `FixedCurve` construction used LINQ
  `OrderBy(...).ToArray()` for every multi-key curve, including serializer
  roundtrips where keyframes are already materialized.
- Replaced `FixedCurve` construction with in-place `Array.Sort` and replaced the
  remaining runtime LINQ equality path with a counted loop. This intentionally
  changes the constructor contract for v5: the supplied keyframe array is sorted
  and retained instead of defensively cloned.
- Removed LINQ usage from the benchmark catalog/CLI and the affected random test
  while the context was fresh, keeping the repo's performance examples aligned
  with the runtime direction.
- Release serialization rerun after the change showed
  `MemoryPackRoundTripCurve` at 60.252 us / 145,408 B, down from the Phase 5
  pre-change pass of 90.561 us / 370,688 B. JSON curve roundtrip allocation also
  dropped from 3,041,452 B to 2,816,172 B, but JSON time remains dominated by
  `System.Text.Json` behavior and should be treated as noisy unless a future
  serializer-options/source-generation investigation is opened.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~DeterministicRandomTests"
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll serialization -j Short -i --filter "*RoundTripCurve*" --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll serialization -j Short -i --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/ReleaseLean/net8.0/FixedMathSharp.Benchmarks.dll serialization -j Short -i --exporters json
```

## Phase 6: Vector Normalization Follow-Through

**Status:** Complete on 2026-06-05.

**Why:** Vector arithmetic is allocation-free and fast, but normalization and
magnitude paths scale with fixed-point sqrt cost: `Vector4d.GetNormalized`
(89.416 us), `Vector4d.NormalizeInPlace` (87.263 us), `Vector4d.Normal`
(86.907 us), `Vector3d.GetNormalized` (72.913 us), and `Vector2d.GetNormalized`
(58.084 us).

- [x] Revisit vector normalization after scalar sqrt/transcendental work, since
  many costs may be inherited.
- [x] Confirm zero-vector and near-zero-vector behavior before any fast-path
  changes.
- [x] Benchmark squared-length alternatives only where public API semantics do
  not require magnitude.
- [x] Keep `out` overloads and in-place overloads allocation-free.

Result:

- Captured a fresh full vector `ShortRun`. The earlier `Sqrt` work already
  pulled normalization down without vector-specific changes:
  `Vector2d.GetNormalized` now measured 52.171 us, `Vector3d.GetNormalized`
  67.265 us, and `Vector4d.GetNormalized` 76.879 us. All vector benchmarked
  hot paths remained allocation-free.
- Rejected the tempting `invMag = Fixed64.One / mag` normalization rewrite. It
  reduced repeated fixed divisions in theory, but changed raw rounding enough to
  fail exact behavior: `Vector2d(3,4).Normal` drifted by a few raw units and
  `Vector3d.SpeedLerp_DoesNotOvershoot` overshot to `5.000000004656613`.
- Kept the existing per-component division path for normalization because it
  preserves current deterministic rounding and exact no-overshoot behavior.
- Updated `Vector3d.IsNormalized()` to use `SqrMagnitude` instead of
  `Magnitude`, matching the existing `Vector4d` shape and avoiding an
  unnecessary square root where boolean semantics only require squared length.
- Added focused `IsNormalized` benchmark coverage for `Vector3d` and `Vector4d`.
  `Vector3d.IsNormalized` measured 3.422 us versus the old `Magnitude` path's
  30.649 us proxy from the same fresh run; `Vector4d.IsNormalized` measured
  4.480 us.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Vector2d|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d"
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d"
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d vector3d vector4d -j Short -i --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector3d vector4d -j Short -i --filter "*IsNormalized*" --exporters json
```
