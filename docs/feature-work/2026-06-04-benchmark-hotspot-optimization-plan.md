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

**Why:** `BoundsBenchmarks.FrustumCreateFromMatrix` was the only
non-serialization benchmark with managed allocation in the full baseline:
933,380.6 ns and 165,888 B per benchmark operation.

**Likely cause:** `FixedBoundFrustum` is a sealed reference type and owns
`Vector3d[8]` plus `FixedPlane[6]` arrays per instance.

- [ ] Decide whether frustum construction is expected to be frequent enough to
  justify a public allocation-free construction path.
- [ ] Investigate internal fixed-field storage, caller-provided buffers, pooling,
  or a value-type/lightweight frustum variant without breaking existing public
  API expectations.
- [ ] Preserve `GetCorners`/`GetPlanes` array overloads and existing allocation
  regression tests.
- [ ] Add focused benchmarks for constructor-only cost, matrix-set cost, and
  repeated contains/intersects reuse.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedBoundFrustum"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds --filter "*Frustum*" --exporters json
```

## Phase 2: Scalar Transcendental Speed

**Why:** The slowest allocation-free scalar methods were `Pow` (136.806 us),
`Acos` (124.290 us), `Tan` (103.154 us), `Atan2` (93.539 us), `Atan`
(92.729 us), and `Asin` (77.112 us).

- [ ] Review algorithms in `FixedMath` and `FixedTrigonometry` for iteration
  counts, range reduction, table lookup, division count, and branch shape.
- [ ] Add correctness sweeps before optimization for edge cases, singularities,
  sign handling, and deterministic raw-value expectations.
- [ ] Benchmark each candidate independently before and after changes.
- [ ] Prefer deterministic algorithmic wins over lookup-table growth unless the
  table size, cache behavior, and `netstandard2.1` impact are justified.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Fixed64"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic --exporters json
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
