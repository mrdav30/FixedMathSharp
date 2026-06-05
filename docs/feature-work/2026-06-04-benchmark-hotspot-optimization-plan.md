# Benchmark Hotspot Optimization Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before changing runtime code for any measured
> hotspot, use superpowers:test-driven-development for behavior-preserving
> optimizations, and use superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Active

**Goal:** Convert the 2026-06-04 full benchmark baseline into focused,
correctness-first optimization work for measured hotspots only.

**Architecture:** Keep benchmark findings separate from regression guardrail
work. This plan tracks candidate runtime/design optimizations; the benchmark
regression plan tracks repeatability, reporting, and CI posture.

**Baseline:** `docs/feature-work/benchmark-baselines/2026-06-04/`

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
- Left `Pow`, `Log2`, and `Ln` unchanged in this pass. `Pow` remains a real
  scalar candidate, but it likely needs a dedicated `Log2`/`Pow2` algorithm pass
  with tighter correctness coverage rather than being mixed into the trig
  range-reduction work.

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

## Phase 3: Quaternion Direction And Euler Paths

**Why:** Quaternion conversion-heavy paths are among the slowest allocation-free
runtime operations: `FromEulerAngles` (468.24 us), `FromDirection` (443.02 us),
`ToEulerAngles` (297.75 us), `Slerp` (256.60 us), and `FromAxisAngle`
(216.72 us).

- [ ] Separate cost inherited from scalar trig from quaternion-specific
  normalization, branch, and multiplication work.
- [ ] Reuse coordinate-convention tests to keep forward/up handedness stable.
- [ ] Add focused benchmarks for common identity, cardinal-axis, near-parallel,
  and anti-parallel inputs.
- [ ] Avoid changing public convention semantics while optimizing.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedQuaternion"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll quaternion --exporters json
```

## Phase 4: Matrix Rotation, Transform Composition, And Inversion

**Why:** Matrix-heavy methods are high in the baseline without managed
allocation: `Matrix3x3.CreateRotation` (467.73 us), `Matrix4x4.InvertFull`
(390.78 us), `Matrix4x4.TranslateRotateScale` (287.10 us),
`Matrix4x4.ScaleRotateTranslate` (267.88 us), `Matrix4x4.InvertAffine`
(215.20 us), and `Matrix4x4.CreateTransform` (201.88 us).

- [ ] Distinguish trig-driven rotation cost from matrix multiplication and
  inversion cost.
- [ ] Confirm affine inversion stays on the affine path and does not regress
  full inversion correctness.
- [ ] Add focused correctness tests for singular, near-singular, affine, and
  non-affine matrices before optimizing.
- [ ] Benchmark composition order variants with the same deterministic fixture
  set.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Fixed3x3|FullyQualifiedName~Fixed4x4"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix3x3 matrix4x4 --exporters json
```

## Phase 5: Serialization Cost Review

**Why:** Serialization dominates absolute time and allocation, especially JSON:
`JsonRoundTripCurve` (4.115 ms, 3,041,448 B), `JsonSerializeBoundsAndCurves`
(1.908 ms, 1,362,672 B), and `JsonSerializeMathTypes` (1.498 ms,
1,223,096 B). MemoryPack is much cheaper but still allocates payload buffers.

- [ ] Treat JSON and MemoryPack payload allocation as intentional until a focused
  benchmark proves avoidable FixedMathSharp-side work.
- [ ] Keep serialization benchmarks conditional under
  `FIXEDMATHSHARP_DISABLE_MEMORYPACK` for lean builds.
- [ ] Consider documentation or sample guidance before runtime changes if the
  measured cost is primarily serializer behavior.
- [ ] Only open runtime work if source inspection finds avoidable intermediate
  objects or repeated metadata setup inside FixedMathSharp-owned code.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --force
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0 --force
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll serialization --exporters json
```

## Phase 6: Vector Normalization Follow-Through

**Why:** Vector arithmetic is allocation-free and fast, but normalization and
magnitude paths scale with fixed-point sqrt cost: `Vector4d.GetNormalized`
(89.416 us), `Vector4d.NormalizeInPlace` (87.263 us), `Vector4d.Normal`
(86.907 us), `Vector3d.GetNormalized` (72.913 us), and `Vector2d.GetNormalized`
(58.084 us).

- [ ] Revisit vector normalization after scalar sqrt/transcendental work, since
  many costs may be inherited.
- [ ] Confirm zero-vector and near-zero-vector behavior before any fast-path
  changes.
- [ ] Benchmark squared-length alternatives only where public API semantics do
  not require magnitude.
- [ ] Keep `out` overloads and in-place overloads allocation-free.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Vector2d|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d vector3d vector4d --exporters json
```
