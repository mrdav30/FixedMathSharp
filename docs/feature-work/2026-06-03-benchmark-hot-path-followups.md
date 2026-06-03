# Benchmark Hot-Path Optimization Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before changing runtime code for any
> benchmark-discovered issue, use superpowers:test-driven-development for
> correctness fixes, and use superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Proposed

**Goal:** Turn the first benchmark smoke-run findings into measured,
deterministic, allocation-conscious runtime improvements without weakening
FixedMathSharp semantics.

**Architecture:** Keep `tests/FixedMathSharp.Benchmarks` as the evidence lab:
expand coverage, isolate allocation sources, make one runtime optimization at a
time, and verify each change with focused correctness tests plus before/after
benchmarks. Treat short in-process BenchmarkDotNet runs as smoke checks only;
use full `Release` benchmark artifacts for performance claims.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, .NET 8
BenchmarkDotNet runner, xUnit, `Fixed64`, `FixedMath`, `Vector2d`, `Vector3d`,
`Vector4d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`, MemoryPack,
System.Text.Json.

---

## Context

The first benchmark pass added deterministic BenchmarkDotNet coverage for:

- `Fixed64` arithmetic, division, square root, `Sin`/`Cos`, and `Atan2`.
- `Vector3d` arithmetic, dot, cross, normalization, distance, and interpolation.
- `FixedQuaternion` creation, multiplication, interpolation, and vector rotation.
- `Fixed4x4` transform creation, multiplication, point transforms, and affine
  inversion.
- `FixedBoundBox` containment, intersection, sphere intersection, and point
  clamping.

Verification used `Release` and `ReleaseLean` builds plus a short in-process
BenchmarkDotNet smoke run. That run exposed useful allocation signals, but its
numbers are not stable enough for published claims or performance gates.

## Smoke-Run Signals To Validate

Allocation-free in the first pass:

- `FixedMath.Sqrt`
- `Vector3d.Dot`
- `Vector3d.Cross`
- `Vector3d.Distance`
- `FixedQuaternion` multiplication
- `FixedQuaternion` vector rotation
- `Fixed4x4` multiplication
- `Fixed4x4.TransformPoint`
- `FixedBoundBox` containment/intersection/clamping benchmarks

Allocating in the first pass:

- `Fixed64` add/subtract/multiply loop: 28,672 B per benchmark operation.
- `Fixed64` division loop: 46,488 B.
- `FixedMath.Sin` + `FixedMath.Cos` loop: 327,153 B.
- `FixedMath.Atan2` loop: 390,193 B.
- `Vector3d.AddScale` and `Vector3d.Lerp`: 28,672 B.
- `Vector3d.Normalize`: 113,744 B.
- `FixedQuaternion.FromEulerAngles`: 1,715,261 B.
- `FixedQuaternion.Slerp`: 649,307 B.
- `Fixed4x4.CreateTransform` and `Fixed4x4.InvertAffine`: 28,672 B.

Treat these as investigation leads. Do not change runtime code until a focused
diagnostic run identifies the allocation source.

## Phase 1: Establish Baseline Discipline

**Files:**

- Review: `tests/FixedMathSharp.Benchmarks/README.md`
- Review: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/QuaternionBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Matrix4x4Benchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Review: `tests/FixedMathSharp.Benchmarks/Support/BenchmarkFixtures.cs`
- Modify only if needed: `tests/FixedMathSharp.Benchmarks/README.md`

- [ ] Build the benchmark project in `Release` and `ReleaseLean`.
- [ ] Run `list` and confirm the expected aliases are visible:
  `bounds`, `fixed64-arithmetic`, `matrix4x4`, `quaternion`, and `vector3d`.
- [ ] Run the short in-process suite only as a smoke check.
- [ ] Run a full `Release` baseline with JSON export before runtime
  optimization begins.
- [ ] Archive or preserve the baseline artifact path in the working notes or a
  follow-up doc before making performance changes.
- [ ] Confirm benchmark fixture setup is outside measured methods or is
  intentionally part of the measured scenario.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

## Phase 2: Diagnose Scalar And Trigonometry Allocations

**Files:**

- Modify as needed: `tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`
- Modify as needed: `tests/FixedMathSharp.Benchmarks/Support/BenchmarkFixtures.cs`
- Review: `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- Review: `src/FixedMathSharp/Core/FixedMath.cs`
- Review: `src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Core/FixedTrigonometry.Tests.cs`

- [ ] Split `Fixed64ArithmeticBenchmarks.SinCos` into separate `Sin` and `Cos`
  benchmark methods.
- [ ] Add focused `FixedMath.Tan`, `Acos`, `Asin`, `Atan`, `Pow`, `Log2`, and
  `Ln` benchmark methods.
- [ ] Run focused allocation diagnostics for `fixed64-arithmetic`.
- [ ] Identify whether allocations come from lookup creation, span/array
  patterns, exception-helper shape, decimal/string conversion, or operator
  helper behavior.
- [ ] If an allocation source is confirmed, add a targeted correctness test
  before changing runtime code.
- [ ] Preserve guarded overflow, saturation, rounding, and deterministic
  approximation behavior.
- [ ] Re-run focused benchmarks before and after each runtime change.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Fixed64Tests|FullyQualifiedName~FixedMathTests|FullyQualifiedName~FixedTrigonometryTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic --exporters json
```

## Phase 3: Diagnose Vector And Quaternion Allocations

**Files:**

- Modify: `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/QuaternionBenchmarks.cs`
- Create: `tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Create: `tests/FixedMathSharp.Benchmarks/Vector4dBenchmarks.cs`
- Review: `src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- Review: `src/FixedMathSharp/Numerics/Vectors/Vector3d.cs`
- Review: `src/FixedMathSharp/Numerics/Vectors/Vector4d.cs`
- Review: `src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector4d.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Rotations/FixedQuaternion.Tests.cs`

- [ ] Split `Vector3d.Normalize` into `Magnitude`, `Normal`,
  `NormalizeInPlace`, and `GetNormalized` benchmark cases.
- [ ] Add `Vector2d` and `Vector4d` benchmarks for arithmetic, dot,
  normalization, distance, and interpolation.
- [ ] Add quaternion benchmarks for `FromAxisAngle`, `AngleAxis`,
  `FromEulerAngles`, `Lerp`, `Slerp`, `Normalize`, `ToEulerAngles`, and
  `FromDirection`.
- [ ] Diagnose whether quaternion allocations are inherited from
  trigonometry/vector normalization or from quaternion-specific code.
- [ ] Compare existing value-returning overloads against existing `out`
  overloads where both are already part of the public API.
- [ ] Keep coordinate-convention fixes separate from performance changes unless
  a benchmark directly proves a convention-related cost.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector2dTests|FullyQualifiedName~Vector3dTests|FullyQualifiedName~Vector4dTests|FullyQualifiedName~FixedQuaternionTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector3d quaternion -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector3d quaternion --exporters json
```

## Phase 4: Expand Matrix And Bounds Coverage

**Files:**

- Modify: `tests/FixedMathSharp.Benchmarks/Matrix4x4Benchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Create: `tests/FixedMathSharp.Benchmarks/Matrix3x3Benchmarks.cs`
- Review: `src/FixedMathSharp/Numerics/Matrices/Fixed3x3.cs`
- Review: `src/FixedMathSharp/Numerics/Matrices/Fixed4x4.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundFrustum.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundSphere.cs`
- Review: `src/FixedMathSharp/Geometry/Primitives/FixedPlane.cs`
- Review: `src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Matrices/Fixed3x3.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Matrices/Fixed4x4.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundArea.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundBox.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundFrustum.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundSphere.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedPlane.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedRay.Tests.cs`

- [ ] Split `Fixed4x4.CreateTransform` into `CreateTranslation`,
  `CreateRotation`, `CreateScale`, `CreateTransform`, `ScaleRotateTranslate`,
  and `TranslateRotateScale` benchmark cases.
- [ ] Split inversion benchmarks into affine and full inversion paths.
- [ ] Add `Fixed3x3` coverage for rotation creation, multiplication,
  transform direction, determinant, and inversion.
- [ ] Add `FixedBoundSphere`, `FixedBoundArea`, `FixedBoundFrustum`,
  `FixedRay`, and `FixedPlane` benchmarks.
- [ ] Add mixed shape dispatch benchmarks only after deciding which public
  dispatch shape must remain stable.
- [ ] Preserve explicit cross-type overloads unless a measured replacement
  proves faster and clearer.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Fixed3x3Tests|FullyQualifiedName~Fixed4x4Tests|FullyQualifiedName~FixedBound|FullyQualifiedName~FixedPlaneTests|FullyQualifiedName~FixedRayTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix4x4 bounds -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix4x4 bounds --exporters json
```

## Phase 5: Add Serialization, Curves, Ranges, And RNG Coverage

**Files:**

- Create: `tests/FixedMathSharp.Benchmarks/SerializationBenchmarks.cs`
- Create: `tests/FixedMathSharp.Benchmarks/CurveBenchmarks.cs`
- Create: `tests/FixedMathSharp.Benchmarks/FixedRangeBenchmarks.cs`
- Create: `tests/FixedMathSharp.Benchmarks/DeterministicRandomBenchmarks.cs`
- Review: `src/FixedMathSharp/Numerics/Curves/FixedCurve.cs`
- Review: `src/FixedMathSharp/Numerics/Curves/FixedCurveKey.cs`
- Review: `src/FixedMathSharp/Numerics/Scalars/FixedRange.cs`
- Review: `src/FixedMathSharp/Random/DeterministicRandom.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Curves/FixedCurveTests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Numerics/Scalars/FixedRange.Tests.cs`
- Test if runtime changes are made: `tests/FixedMathSharp.Tests/Random/DeterministicRandom.Tests.cs`

- [ ] Add standard-build MemoryPack serialization and deserialization
  benchmarks for `Fixed64`, vectors, quaternions, matrices, bounds, and curves.
- [ ] Add JSON roundtrip benchmarks where JSON support is part of the tested
  public behavior.
- [ ] Ensure serialization benchmarks are excluded or conditionally compiled in
  `ReleaseLean` when MemoryPack APIs are unavailable.
- [ ] Add curve evaluation benchmarks for common key counts and interpolation
  modes.
- [ ] Add deterministic random benchmarks for `Next`, `NextFixed64`, seeded
  streams, and feature-derived streams.
- [ ] Validate that fixtures are deterministic and do not depend on benchmark
  execution order.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all -j Short -i
```

## Phase 6: Regression Guardrails And Reporting

**Files:**

- Modify: `tests/FixedMathSharp.Benchmarks/README.md`
- Modify if adopted: `.github/workflows/build-and-test.yml`
- Create or modify if adopted: benchmark baseline documentation under
  `docs/feature-work` or `docs/wiki`

- [ ] Document how to compare current branch benchmark results against a stored
  baseline.
- [ ] Decide whether CI should only compile benchmarks or also run a small
  smoke benchmark job.
- [ ] Avoid raw timing thresholds in CI until runner variance is understood.
- [ ] Prefer artifact comparison or explicit BenchmarkDotNet comparison support
  over single-run timing gates.
- [ ] Add release-note guidance for performance improvements that includes
  environment, baseline, and measured delta.
- [ ] Confirm optimized paths remain deterministic across `Release` and
  `ReleaseLean`.

Verification:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
git diff --check
```

## Recommended First Implementation Slice

Start with Phases 1 and 2. They provide the highest-signal payoff because the
first smoke run showed large allocations in scalar/trigonometry paths, and many
vector, quaternion, and matrix costs likely inherit from those same helpers.

Do not combine broad benchmark expansion with runtime optimization in the same
commit. A clean sequence is:

1. Add focused benchmark coverage.
2. Capture baseline artifacts.
3. Diagnose one allocation source.
4. Add correctness coverage.
5. Optimize one runtime path.
6. Re-run focused benchmarks and full tests.
