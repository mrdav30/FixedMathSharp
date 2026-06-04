# Benchmark Hot-Path Optimization Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before changing runtime code for any
> benchmark-discovered issue, use superpowers:test-driven-development for
> correctness fixes, and use superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Active

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

- [x] Build the benchmark project in `Release` and `ReleaseLean`.
- [x] Run `list` and confirm the expected aliases are visible:
  `bounds`, `fixed64-arithmetic`, `matrix4x4`, `quaternion`, and `vector3d`.
- [x] Run the short in-process suite only as a smoke check.
- [x] Run a full `Release` baseline with JSON export before runtime
  optimization begins.
- [x] Archive or preserve the baseline artifact path in the working notes or a
  follow-up doc before making performance changes.
- [x] Confirm benchmark fixture setup is outside measured methods or is
  intentionally part of the measured scenario.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

Phase 1 result on 2026-06-03: completed. `Release` and `ReleaseLean` builds
passed, `list` showed all expected aliases, the short in-process smoke suite
ran 23 benchmarks, and the full `Release` baseline ran 23 benchmarks with JSON
exports. Baseline artifacts are gitignored and preserved locally at:

```text
BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.BoundsBenchmarks-report-full-compressed.json
BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.Fixed64ArithmeticBenchmarks-report-full-compressed.json
BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.Matrix4x4Benchmarks-report-full-compressed.json
BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.QuaternionBenchmarks-report-full-compressed.json
BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.Vector3dBenchmarks-report-full-compressed.json
```

Fixture review: `BenchmarkFixtures` uses static deterministic arrays and factory
methods to prepare shared scalar, vector, quaternion, and matrix data outside
measured benchmark methods. No Phase 1 benchmark code changes were needed.

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

- [x] Split `Fixed64ArithmeticBenchmarks.SinCos` into separate `Sin` and `Cos`
  benchmark methods.
- [x] Add focused `FixedMath.Tan`, `Acos`, `Asin`, `Atan`, `Pow`, `Log2`, and
  `Ln` benchmark methods.
- [x] Run focused allocation diagnostics for `fixed64-arithmetic`.
- [x] Identify whether allocations come from lookup creation, span/array
  patterns, exception-helper shape, decimal/string conversion, or operator
  helper behavior.
- [x] If an allocation source is confirmed, add a targeted correctness test
  before changing runtime code.
- [x] Preserve guarded overflow, saturation, rounding, and deterministic
  approximation behavior.
- [x] Re-run focused benchmarks before and after each runtime change.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Fixed64Tests|FullyQualifiedName~FixedMathTests|FullyQualifiedName~FixedTrigonometryTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic --exporters json
```

Phase 2 result on 2026-06-03: completed. The benchmark suite now isolates
`Sin`, `Cos`, `Tan`, `Acos`, `Asin`, `Atan`, `Atan2`, `Pow`, `Log2`, and `Ln`.
`Tan` uses a tangent-specific angle fixture inside `[-pi/3, pi/3)` so the
benchmark measures the hot path without sampling tangent singularities.

Focused allocation diagnostics confirmed eager exception-message construction
as the scalar/trigonometry allocation source:

- `Fixed64.operator /` built an interpolated divide-by-zero message even when
  the divisor was valid.
- `FixedMath.Asin` and `FixedMath.Acos` concatenated domain-error messages even
  when inputs were in range.
- `FixedMath.Pow10Lookup` returned a fresh array for every `ReadOnlySpan<int>`
  access.

Targeted allocation regression tests were added before runtime changes. The
fixes preserve existing exception types/messages on error paths and only defer
message/table allocation on valid paths. A short focused benchmark after the
fix reported no managed allocations for all 13 `fixed64-arithmetic` benchmarks.
A full focused `Release` benchmark export also reported no managed allocations
and wrote:

```text
BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.Fixed64ArithmeticBenchmarks-report-full-compressed.json
```

Verification notes: focused scalar/trigonometry tests passed with coverage
disabled for the final local verification loop, full solution Debug tests
passed, and `Release`/`ReleaseLean` `netstandard2.1` build verification passed.
Follow-up tooling/package warnings discovered during verification are tracked
in `docs/feature-work/issue-tracker.md` as `FMS-Issue-005` and
`FMS-Issue-006`.

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

- [x] Split `Vector3d.Normalize` into `Magnitude`, `Normal`,
  `NormalizeInPlace`, and `GetNormalized` benchmark cases.
- [x] Add `Vector2d` and `Vector4d` benchmarks for arithmetic, dot,
  normalization, distance, and interpolation.
- [x] Add quaternion benchmarks for `FromAxisAngle`, `AngleAxis`,
  `FromEulerAngles`, `Lerp`, `Slerp`, `Normalize`, `ToEulerAngles`, and
  `FromDirection`.
- [x] Diagnose whether quaternion allocations are inherited from
  trigonometry/vector normalization or from quaternion-specific code.
- [x] Diagnose the residual 1 B short-run allocation signal in
  `QuaternionBenchmarks.FromEulerAngles` and `QuaternionBenchmarks.Slerp` after
  `FixedThrowHelper` removal.
- [x] Compare existing value-returning overloads against existing `out`
  overloads where both are already part of the public API.
- [x] Keep coordinate-convention fixes separate from performance changes unless
  a benchmark directly proves a convention-related cost.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector2dTests|FullyQualifiedName~Vector3dTests|FullyQualifiedName~Vector4dTests|FullyQualifiedName~FixedQuaternionTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d vector3d vector4d quaternion -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d vector3d vector4d quaternion --exporters json
```

Phase 3 note from the `FixedThrowHelper` removal side quest on 2026-06-03:
`fixed64-arithmetic` remained allocation-free in a short run, while
`QuaternionBenchmarks.FromEulerAngles` and `QuaternionBenchmarks.Slerp` dropped
to a small residual `1 B` allocation signal. Treat that as a Phase 3 diagnostic
target rather than a completed quaternion allocation fix.

Phase 3 result on 2026-06-03: benchmark coverage was expanded for
`Vector2d`, `Vector3d`, `Vector4d`, and `FixedQuaternion`. Vector fixtures now
include deterministic 2D and 4D arrays plus normalized 3D axes for quaternion
creation paths. `Vector3d` normalization is split into `Magnitude`, `Normal`,
`NormalizeInPlace`, and `GetNormalized`. Value-returning vector additions were
compared against existing `out` overloads across 2D, 3D, and 4D; `Vector2d` and
`Vector3d` also compare value-returning `Lerp` against `Lerp(..., out result)`.

Short in-process BenchmarkDotNet diagnostics reported no managed allocations
for all vector benchmarks and all quaternion benchmarks, including
`QuaternionBenchmarks.FromEulerAngles` and `QuaternionBenchmarks.Slerp`. The
prior residual `1 B` quaternion signal did not reproduce after the benchmark
expansion and current scalar/trigonometry fixes, so no runtime allocation
change was made in Phase 3. The current evidence points to the large original
quaternion allocation coming from shared scalar/trigonometry guard-message and
lookup paths fixed in Phase 2, not from quaternion-specific heap allocation.

Verification notes: `Release` benchmark build passed with no warnings, `list`
showed `vector2d`, `vector3d`, `vector4d`, and `quaternion`, and short
in-process smoke runs completed for the vector/quaternion benchmark set. Timing
comparisons between value-returning and `out` overloads should not be used as
API guidance until a full exported benchmark run is captured; the short runs
are sufficient only for compile, selection, and allocation diagnostics.

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

- [x] Split `Fixed4x4.CreateTransform` into `CreateTranslation`,
  `CreateRotation`, `CreateScale`, `CreateTransform`, `ScaleRotateTranslate`,
  and `TranslateRotateScale` benchmark cases.
- [x] Split inversion benchmarks into affine and full inversion paths.
- [x] Add `Fixed3x3` coverage for rotation creation, multiplication,
  transform direction, determinant, and inversion.
- [x] Add `FixedBoundSphere`, `FixedBoundArea`, `FixedBoundFrustum`,
  `FixedRay`, and `FixedPlane` benchmarks.
- [x] Add mixed shape dispatch benchmarks only after deciding which public
  dispatch shape must remain stable.
- [x] Preserve explicit cross-type overloads unless a measured replacement
  proves faster and clearer.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Fixed3x3Tests|FullyQualifiedName~Fixed4x4Tests|FullyQualifiedName~FixedBound|FullyQualifiedName~FixedPlaneTests|FullyQualifiedName~FixedRayTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix3x3 matrix4x4 bounds -j Short -i
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix3x3 matrix4x4 bounds --exporters json
```

Phase 4 result on 2026-06-04: benchmark coverage was expanded for `Fixed3x3`,
matrix transform construction variants, affine versus full `Fixed4x4`
inversion, and the remaining hot bounds/primitive shapes. `BenchmarkFixtures`
now provides deterministic translation, scale, 3x3 matrix, and perspective
matrix fixtures so measured methods do not build ad-hoc setup data.

Short in-process diagnostics completed 43 benchmark cases across `matrix3x3`,
`matrix4x4`, and `bounds`. Bounds and primitive operations were allocation-free
except `FrustumCreateFromMatrix`, which allocated about 165 KB per benchmark
operation because each measured loop intentionally constructs new
`FixedBoundFrustum` instances and each frustum owns its plane/corner arrays.
The reusable `GetCorners(Vector3d[])` and `GetPlanes(FixedPlane[])` frustum
paths stayed allocation-free.

The in-process run reported tiny residual `1 B` allocation signals in
`Matrix3x3Benchmarks.CreateRotation`,
`Matrix4x4Benchmarks.TranslateRotateScale`, and
`Matrix4x4Benchmarks.InvertFull`. A focused normal BenchmarkDotNet short run
with `--memory` did not reproduce those allocations, so treat the `1 B` entries
as in-process diagnoser noise unless a full exported benchmark run reproduces
them. No runtime optimization was made in Phase 4.

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
