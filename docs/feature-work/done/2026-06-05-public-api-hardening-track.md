# Public API Hardening Track

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:brainstorming before changing public API shape,
> superpowers:test-driven-development before production or test changes,
> performance-optimization-engineer before accepting performance-motivated
> runtime changes, and superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Done

**Goal:** Make FixedMathSharp's public API easier to discover, harder to
misuse, and friendlier for v5.0.0 consumers while preserving deterministic,
low-allocation, high-performance behavior.

**Architecture:** Keep one canonical implementation for every operation, then
offer static, instance, operator, and extension entry points only when each form
has a clear consumer value. Treat this as a breaking-change hardening track:
remove legacy shapes that obscure intent, split large files by responsibility,
and benchmark any API form that exists mainly for hot-path performance.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, xUnit,
BenchmarkDotNet, MemoryPack-gated `Release`/`ReleaseLean` builds, `Fixed64`,
`Vector2d`, `Vector3d`, `Vector4d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`,
and engine-agnostic FixedMathSharp core APIs.

---

## Context

FixedMathSharp is preparing for a major release after a broad correctness,
benchmarking, and convention-hardening pass. Breaking changes are acceptable
when they make the library more deterministic, faster, cleaner, or easier to
use. This plan groups the public API cleanup work into nine focused tracks:

1. Replace legacy vector `out` helper shapes with clearer mutating/static APIs.
2. Clarify ownership between `FixedMath`, `FixedMath.Trigonometry`, `Fixed64`,
   and `Fixed64.Extensions`.
3. Curate static/instance/extension parity across the numeric types.
4. Split files above roughly 1000 lines into logical partials.
5. Remove unnecessary `this.` qualifiers.
6. Normalize inconsistent public naming.
7. Review public `Fast*` and special-case helper APIs.
8. Add span-oriented overloads where enumerable APIs sit on hot paths.
9. Replace distance checks with squared-distance checks where semantics allow.

## Guardrails

- Keep core runtime engine-agnostic. Do not introduce Unity, XNA, MonoGame, or
  other engine conditionals into public numeric APIs.
- Do not create alternate implementations in extension classes. Extensions
  should forward to canonical static or instance implementations.
- Do not accept a performance-motivated API unless it has benchmark evidence,
  no managed allocations, and correctness tests covering representative fixed
  raw values.
- Do not preserve legacy API shapes solely for backwards compatibility. v5.0.0
  can break consumers when the new shape is meaningfully clearer or faster.
- Keep `netstandard2.1` compatibility in mind; avoid APIs that would block
  Unity package support unless guarded or isolated.
- Track newly discovered correctness defects in
  `docs/feature-work/issue-tracker.md` instead of burying them in this plan.

## Phase 1: Public API Inventory And Ownership Map

**Files:**

- Review: `src/FixedMathSharp/Core/FixedMath.cs`
- Review: `src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Review: `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- Review: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Extensions.cs`
- Review: `src/FixedMathSharp/Numerics/Vectors/*.cs`
- Review: `src/FixedMathSharp/Numerics/Rotations/*.cs`
- Review: `src/FixedMathSharp/Numerics/Matrices/*.cs`
- Modify: this plan and, if needed, `docs/feature-work/issue-tracker.md`

- [x] Inventory public static, instance, operator, and extension methods for
  scalar, vector, quaternion, and matrix types.
- [x] Classify each method as `canonical`, `forwarding convenience`,
  `legacy/remove`, `rename`, `benchmark before deciding`, or `issue tracker`.
- [x] Define ownership rules before runtime edits. Recommended baseline:
  `FixedMath` owns deterministic math algorithms, `Fixed64` owns
  representation/operators/parsing/conversion, and extension types own only
  fluent forwarding wrappers.
- [x] Identify methods with unclear semantic ownership, especially scalar
  interpolation, angle conversion, trigonometry wrappers, formatting helpers,
  and `Fast*` helpers.
- [x] Record any suspected bugs in `docs/feature-work/issue-tracker.md`.

Phase 1 result on 2026-06-06: completed.

### Ownership Map

Use these ownership rules for the remaining phases unless a focused design
review changes them:

- `FixedMath` is the canonical home for deterministic scalar algorithms:
  rounding, clamping, absolute/min/max, powers, logarithms, square root,
  trigonometry, angle conversion, and low-level special-case math helpers.
- `FixedMath.Trigonometry.cs` should remain a partial implementation file for
  `FixedMath`, not a separate public API identity. Consumers should discover
  one scalar algorithm surface: `FixedMath`.
- `Fixed64` is the canonical home for Q32.32 representation, constants,
  constructors/conversions, parsing, operators, raw-value creation, comparison,
  equality, and serialization layout.
- `Fixed64.Extensions` should contain fluent forwarding wrappers only. It should
  not define alternate scalar algorithms, and every method intended to be
  fluent must be extension-shaped.
- `Vector2d`, `Vector3d`, and `Vector4d` own vector constants, fields,
  constructors, indexers, component properties, operators, instance mutation,
  static vector algorithms, conversion/deconstruction, comparison, and
  serialization layout.
- Vector extension files should contain only curated fluent wrappers and fuzzy
  comparison helpers. They should not preserve legacy `out` APIs or duplicate
  vector algorithms.
- `FixedQuaternion` owns rotation representation, quaternion operators,
  normalization, rotation factories, quaternion/vector rotation, interpolation,
  matrix conversion, direction conversion, and angular velocity helpers.
- `FixedQuaternion.Extensions` should stay small: fuzzy comparison plus any
  receiver-shaped convenience wrappers that clearly improve call sites.
- `Fixed3x3` and `Fixed4x4` own matrix storage, row-vector transform
  semantics, factory methods, extraction/decomposition, inversion,
  composition, operators, and matrix/vector transform algorithms.
- Matrix extension files should remain ref-returning or receiver-shaped
  conveniences that forward to canonical matrix methods. Keep factory methods
  off the extension surface.
- Geometry bounds and primitives own their own containment, intersection,
  projection, transform, and construction workflows. Span overload work belongs
  in those geometry types, with `IEnumerable<T>` wrappers kept only for
  interoperability when useful.

### Classification Summary

Canonical:

- `FixedMath`: `Clamp`, `Clamp01`, `ClampOne`, `Abs`, `Ceil`, `Floor`, `Round`,
  `RoundToPrecision`, `Min`, `Max`, `Squared`, `MoveTowards`, `Pow`, `Pow2`,
  `Log2`, `Ln`, `Sqrt`, `Sin`, `Cos`, `Tan`, `Asin`, `Acos`, `Atan`, `Atan2`,
  `RadToDeg`, and `DegToRad`.
- `Fixed64`: constants, raw/value construction, explicit primitive
  conversions, arithmetic/comparison operators, `Parse`, `TryParse`, `FromRaw`,
  `ToInt`, `FromDouble`, `FromFraction`, and equality/comparison.
- Vectors: constants, fields, constructors, `Normalized`, `Magnitude`,
  `MagnitudeSquared`, indexers, `Set`, `*InPlace`, `NormalizeInPlace`,
  `Distance`, `DistanceSquared`, `Dot`, `Cross`,
  static interpolation/geometry helpers,
  operators, conversion/deconstruction, and equality/comparison.
- Quaternions: constants, fields, constructors, `Normalized`, `Magnitude`,
  `EulerAngles`, `NormalizeInPlace`, `Conjugate`, `Inverse`, `Rotate`, factories,
  interpolation, angle/dot helpers, operators, matrix/direction conversions,
  and equality.
- Matrices: fields, constructors, basis/transform properties, determinant,
  reset/set/decompose/extract methods, transform factories, projection
  factories, inversion, transform methods, operators, and equality.

Forwarding convenience:

- Existing scalar, vector, quaternion, and matrix extension files are intended
  to be forwarding wrappers.
- `Fixed3x3Extensions.SetScale`, `Fixed3x3Extensions.SetGlobalScale`,
  `Fixed4x4Extensions.SetGlobalScale`, `Fixed4x4Extensions.SetTranslation`,
  `Fixed4x4Extensions.SetRotation`, and
  `Fixed4x4Extensions.NormalizeRotationMatrix` are already `this ref`
  extension shapes and should inform the Phase 2 vector `this ref` benchmark
  spike.

Legacy/remove:

- Public static vector `out` helpers should be treated as legacy unless Phase 2
  benchmarks prove a clear reason to keep a specific one. Current public
  vector `out` helpers include `Add`, `Subtract`, `Scale`, `Lerp`,
  `SmoothStep`, `Clamp`, `Cross`, `Max`, `Min`, `Negate`, `Reflect`, and
  `Transform` variants across `Vector2d`, `Vector3d`, and `Vector4d`.

Rename/review:

- `Fixed64Extensions.ToDegree` should become `ToDegrees` if the extension
  remains public.
- `Vector2d.Lerped` should be reviewed against `Lerp`, `LerpTo`, or removed if
  it duplicates clearer static/instance forms.
- `GetMagnitude` methods should be reviewed against existing `Magnitude`
  properties; static forms remain intentionally because C# cannot expose a
  static `Magnitude(...)` method beside the instance `Magnitude` property, and
  they pair with `GetNormalized`.
- `Scale`/`ScaleInPlace` was resolved in Phase 2 for vectors; matrix scale
  factories/properties remain because they describe transform scale.
- `CheckDistance` should be reviewed for naming and implementation; it likely
  wants squared-distance semantics when callers only need threshold checks.
- `FromFloatPoint`, `RawToString`, and `RawToInt` were resolved in Phase 7 as
  `FromDouble`, `ToRawString`, and `ToInt`; `ToFormatted*` remains a later
  formatting/conversion ownership review target.
- `GetHypotenuse` and `SinToCos` should be reviewed for naming and whether they
  belong on the public `FixedMath` surface or as internal implementation
  helpers with curated extension aliases.

Benchmark before deciding:

- `this ref` vector extension methods for chained mutating workflows.
- Public exposure and real consumer use of `FastAdd`, `FastSub`, `FastMul`,
  and `FastMod`.
- `ReadOnlySpan<T>` overloads for bounds construction, especially
  `FixedBoundSphere.CreateFromPoints(IEnumerable<Vector3d>)`.
- Replacing distance-threshold helpers with squared-distance comparisons.

Issue tracker:

- `FMS-Issue-010` tracks suspected `Fixed64` arithmetic overload defects where
  `long` operands appear to be treated as raw fixed values instead of integer
  values.

### Phase Handoff Notes

- Phase 2 should start with vectors and keep the `this ref` extension question
  evidence-driven. The existing matrix `this ref` extension shape proves the
  language/API style is already present in the repo, but vectors still need
  benchmark and call-site proof.
- Phase 3 should resolve scalar ownership before moving methods. `FixedMath`
  should remain the algorithm surface because it mirrors `System.Math` and
  keeps algorithm changes independent from `Fixed64` representation.
- Phase 4 should curate extension parity instead of adding wrappers for every
  public static method. Extension methods should read naturally with the first
  argument as the receiver.
- `FMS-Issue-010` should be fixed before or during Phase 3, but as its own
  correctness slice with focused tests.

Verification:

```bash
rg -n "public (static )?.*\\(" src/FixedMathSharp/Core src/FixedMathSharp/Numerics
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Debug -f net8.0
```

Verification result on 2026-06-06: inventory command completed, the Debug
`net8.0` library build passed with zero warnings/errors, placeholder scan
found no stale plan text, and `git diff --check` passed.

## Phase 2: Vector Mutating API And Legacy `out` Shape Cleanup

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector3d.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector4d.cs`
- Review/no change: `src/FixedMathSharp/Numerics/Vectors/Vector2d.Extensions.cs`
- Review/no change: `src/FixedMathSharp/Numerics/Vectors/Vector3d.Extensions.cs`
- Review/no change: `src/FixedMathSharp/Numerics/Vectors/Vector4d.Extensions.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector4d.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/Vector4dBenchmarks.cs`

- [x] Replace legacy public `Add(vector, vector, out vector)`,
  `Subtract(vector, vector, out vector)`, `Scale(vector, vector, out vector)`,
  and similar vector `out` helpers with canonical return-by-value statics and
  instance `*InPlace` methods.
- [x] Add or standardize `MultiplyInPlace` and `DivideInPlace` naming if the
  current `ScaleInPlace` terminology is incomplete for component-wise math.
- [x] Decide whether `ScaleInPlace` remains as an alias, is renamed, or is
  removed for v5.0.0.
- [x] Benchmark return-by-value statics, instance `*InPlace`, and `this ref`
  extension methods for representative chained vector workflows.
- [x] Include chained mutation benchmarks such as add-subtract-scale-normalize
  and multiply-divide-project style loops, not just single-operation
  microbenchmarks.
- [x] Accept public `this ref` extension methods only if they improve either
  measurable speed or meaningful call-site ergonomics without confusing value
  type copy semantics.
- [x] Remove rejected experiments cleanly and keep only the API shape that has
  the best clarity/performance tradeoff.

Phase 2 result on 2026-06-06: completed.

- Added return-by-value `Add`, `Subtract`, `Multiply`, and `Divide` static
  helpers across `Vector2d`, `Vector3d`, and `Vector4d`.
- Added `MultiplyInPlace` and `DivideInPlace` overloads across all vector
  dimensions, then removed `ScaleInPlace` aliases so vector component-wise
  multiplication has one public verb.
- Added the missing `Fixed64 * Vector2d` operator so `Vector2d` matches the
  scalar-left multiply shape already present on `Vector3d` and `Vector4d`.
- Renamed instance `Vector2d.Lerped` to `Vector2d.Lerp`.
- Removed redundant static scalar-left `Multiply(Fixed64, Vector*d)` overloads;
  callers can use `scalar * vector` for that order.
- Removed vector-result `out Vector*d` helper overloads from the public runtime
  surface. `Normalize(out Fixed64 magnitude)` remains because it returns a
  second scalar result, not a legacy vector-result destination.
- Converted `Vector3d` spline, clamp, cross, min/max, negate, reflect, and
  transform helpers to return-value-only public shapes where applicable.
- Updated vector benchmarks to cover operator add, static add/subtract,
  in-place add/subtract, static/in-place multiply, static/in-place divide, and
  chained return-by-value versus chained in-place assignment workflows, with
  stale `AddScale` benchmark names moved to `AddMultiply`.
- Measured a temporary benchmark-only `this ref` extension experiment and
  removed it after the run. Short-run chained results were:
  `Vector2d` return `30.119 us`, in-place assignment `28.361 us`, ref
  extension `28.042 us`; `Vector3d` return `36.595 us`, in-place assignment
  `36.060 us`, ref extension `36.889 us`; `Vector4d` return `48.156 us`,
  in-place assignment `45.504 us`, ref extension `49.266 us`. All reported
  zero managed allocations. The ref shape was not accepted because its only
  win was a tiny 2D short-run result while 3D and 4D were slower, and public
  `this ref` extensions would add value-type mutation ambiguity.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector2d|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d"
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d vector3d vector4d -j Short -i --filter "*AddMultiply*" "*MultiplyStatic*" "*MultiplyInPlace*" --exporters json
rg -n "out Vector[234]d result|public static .*out Vector[234]d|Vector[234]d\.[A-Za-z]+\([^\n]*out Vector[234]d|AddOut|LerpOut|RefInPlace|ChainedRefExtensions|Lerped|ScaleInPlace|public static Vector[234]d Scale|AddScale|Multiply\(Fixed64 factor, Vector[234]d" src tests
git diff --check
```

Verification result on 2026-06-06: focused vector tests passed with 238 tests,
the full Debug solution test run passed with 944 tests, Release and ReleaseLean
`netstandard2.1` builds passed with zero warnings/errors, the Release `net8.0`
benchmark build passed with zero warnings/errors, the short multiply benchmark
subset completed 9 probes with zero managed allocations, the source scan found
no remaining public vector-result `out Vector*d` helpers or stale `AddOut`,
`LerpOut`, `Lerped`, `ScaleInPlace`, `Scale`, `AddScale`, `RefInPlace`, or
static scalar-left `Multiply(Fixed64, Vector*d)` probes, and `git diff --check`
passed.

## Phase 3: FixedMath, Trigonometry, Fixed64, And Scalar Extensions Cleanup

**Files:**

- Modify: `src/FixedMathSharp/Core/FixedMath.cs`
- Modify: `src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Extensions.cs`
- Modify: `tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Core/FixedTrigonometry.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`

- [x] Make `FixedMath` the canonical public surface for deterministic scalar
  algorithms such as `Abs`, `Clamp`, `Round`, `Sqrt`, `Sin`, `Cos`, `Pow`,
  `Log2`, `Pow2`, and `Ln`.
- [x] Keep `FixedMath.Trigonometry.cs` as a partial implementation file only;
  do not create a second public class unless a later API review proves it is
  clearer.
- [x] Keep `Fixed64` focused on value representation, constants, operators,
  parsing, formatting, conversion, and type-specific helpers.
- [x] Convert scalar extensions into consistent fluent forwarding wrappers.
  Fix wrappers that are not actually extension-shaped, such as missing `this`
  on scalar `Sin`.
- [x] Decide whether formatting helpers belong on `Fixed64`, extensions, or a
  separate diagnostics/formatting surface.
- [x] Preserve raw-result correctness for all moved or renamed scalar methods.

Phase 3 result on 2026-06-06: completed.

- `FixedMath` now owns scalar interpolation algorithms in addition to the
  existing rounding, clamp, square root, power, logarithm, trigonometry, and
  angle conversion algorithms.
- `Fixed64` no longer exposes public static scalar algorithm helpers for
  `Lerp`, `SmoothStep`, `CubicInterpolate`, `CatmullRom`, `HermiteSpline`, or
  `BarycentricCoordinate`; it stays focused on Q32.32 representation,
  constants, construction/conversion, operators, parsing, comparison, equality,
  and serialization layout.
- `Fixed64.Extensions` remains the fluent wrapper surface. Wrappers forward to
  `FixedMath`, scalar `Sin` is now extension-shaped, missing fluent wrappers
  were added for tangent/inverse tangent/power/log flows, and `ToDegree` was
  renamed to `ToDegrees`.
- Human-readable formatting helpers remain in `Fixed64.Extensions` for now.
  This keeps them out of the representation type while preserving convenient
  call sites; a separate diagnostics/formatting surface can be reconsidered if
  Phase 4 finds broader formatting API pressure.
- Barycentric scalar and vector tests now verify the actual weight contract:
  `amount1` weights the second value/vertex, `amount2` weights the third
  value/vertex, and the first value/vertex receives the remaining weight.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedMath|FullyQualifiedName~FixedTrigonometry|FullyQualifiedName~Fixed64"
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedMath|FullyQualifiedName~FixedTrigonometry|FullyQualifiedName~Fixed64|FullyQualifiedName~Vector3d"
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
rg -n "Fixed64\\.(Lerp|SmoothStep|CubicInterpolate|CatmullRom|HermiteSpline|BarycentricCoordinate)|ToDegree\\(" src tests docs
git diff --check
```

Verification result on 2026-06-06: focused scalar tests passed with 943 tests,
focused scalar plus `Vector3d` tests passed with 945 tests, the full Debug
solution test run passed with 945 tests, Release and ReleaseLean
`netstandard2.1` builds passed with zero warnings/errors, the stale scalar API
scan found no remaining `Fixed64` interpolation calls or old singular degree
extension calls,
and `git diff --check` passed.

## Phase 4: Curated Static, Instance, And Extension Parity

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Vectors/*.Extensions.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- Modify: `src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.Extensions.cs`
- Modify: `src/FixedMathSharp/Numerics/Matrices/Fixed3x3.Extensions.cs`
- Modify: `src/FixedMathSharp/Numerics/Matrices/Fixed4x4.Extensions.cs`
- Modify: matching numeric test files under `tests/FixedMathSharp.Tests/`

- [x] Add extension wrappers only for receiver-shaped operations where fluent
  syntax is clearly readable, such as `value.Sqrt()`, `vector.Dot(other)`,
  `matrix.TransformPoint(point)`, and `quaternion.Rotate(vector)`.
- [x] Avoid extension wrappers for factories and convention-heavy operations
  such as `FromEulerAngles`, `CreateLookAt`, `CreatePerspective`, or
  `LookRotation`.
- [x] Ensure every extension wrapper forwards to the canonical implementation
  and does not repeat algorithm logic.
- [x] Add tests only where the wrapper shape has meaningful behavior or guards;
  otherwise rely on canonical implementation tests and compile coverage.
- [x] Update XML docs so consumers can discover the canonical method from the
  wrapper.

Phase 4 result on 2026-06-06: completed.

- Added curated receiver-shaped wrappers for vector clamp/interpolation,
  selected `Vector3d` geometry operations, `Vector4d` interpolation/transform
  helpers, quaternion interpolation/log/dot/angle helpers, and matrix
  extraction/transform/interpolation helpers.
- Removed legacy non-ref `ClampOneInPlace(this Vector*d)` extension shapes.
  They were misleading because value-type extension receivers are copied unless
  declared `this ref`, and the old helpers duplicated component logic instead
  of forwarding through a canonical vector implementation.
- Added `Vector2d.Clamp(Vector2d, Vector2d, Vector2d)` so 2D clamp parity has a
  canonical static implementation matching `Vector3d` and `Vector4d`.
- Changed `Fixed4x4.Decompose` to return out parameters in
  `translation, rotation, scale` order for both static and extension entry
  points. This intentionally breaks from the System.Numerics/XNA-style
  `scale, rotation, translation` ordering because FixedMathSharp's transform
  construction APIs already accept `translation, rotation, scale`, and matching
  construction/decomposition order is easier for v5 users to read correctly.
- Added reflection tests to keep quaternion and matrix factory/convention-heavy
  APIs off the extension surface.

Verification:

```bash
rg -n "public static .*\\(this " src/FixedMathSharp/Numerics
rg -n "ClampOneInPlace|Vector[234]dExtensions\\.Sign|public static .*\\(this .*(FromEulerAngles|CreateLookAt|CreatePerspective|LookRotation)" src tests
rg -n "Decompose\\([^\\n]*out (var |Vector3d )?(scale|decomposedScale)[^\\n]*out (var |FixedQuaternion )?(rotation|decomposedRotation)[^\\n]*out (var |Vector3d )?(translation|decomposedTranslation)" src tests
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector2d|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d|FullyQualifiedName~FixedQuaternion|FullyQualifiedName~Fixed3x3|FullyQualifiedName~Fixed4x4"
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release -f net8.0 --no-restore
git diff --check
```

Verification result on 2026-06-06: focused vector/quaternion/matrix tests
passed with 519 tests, the full Debug solution test run passed with 951 tests,
Release and ReleaseLean `netstandard2.1` builds passed with zero
warnings/errors, the Release `net8.0` benchmark project build passed with zero
warnings/errors, the extension surface scan completed, the stale source/test
scan found no `ClampOneInPlace`, static vector-extension `Sign` calls,
factory/convention-heavy extension wrappers, or old `Decompose` out-parameter
ordering, and `git diff --check` passed.

## Phase 5: Large File Partial Split

**Files:**

- Modify or split: `src/FixedMathSharp/Numerics/Vectors/Vector3d.cs`
- Modify or split: `src/FixedMathSharp/Numerics/Matrices/Fixed4x4.cs`
- Modify or split: `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- Modify or split: `src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- Modify or split: `src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.cs`
- Consider later: test files above roughly 1000 lines

- [x] Split only files that have meaningful responsibility boundaries; avoid
  one-method or two-method partial files.
- [x] Prefer partials such as `*.Constants.cs`, `*.Properties.cs`,
  `*.Instance.cs`, `*.Statics.cs`, `*.Operators.cs`, `*.Conversions.cs`,
  `*.Serialization.cs`, and `*.Equality.cs` when those boundaries fit the type.
- [x] Move code mechanically first, with no behavior changes in the same commit.
- [x] Preserve XML docs, attributes, regions only where they still help, and
  MemoryPack/System.Text.Json attributes exactly.
- [x] Keep namespace and `partial` declarations consistent across all new files.

Implementation result on 2026-06-06: split the oversized production structs
into responsibility-based partials while leaving the front-door type files with
fields, constructors, properties, and instance methods. New partials cover
static operations, factories, decomposition, operators, conversions, equality,
and focused helpers where those boundaries fit. The five original files now sit
below the roughly 1000-line review threshold; test-file splitting remains a
later consideration. `AGENTS.md` now captures the production-source rule so
future large files are split into meaningful partials instead of drifting past
the threshold.

Verification:

```bash
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
```

Verification result on 2026-06-06: the full Debug solution test run passed
with 951 tests, Release and ReleaseLean `netstandard2.1` builds passed with
zero warnings/errors, the touched production source files and their new
partials were all below the roughly 1000-line threshold, and `git diff --check`
passed.

## Phase 6: Remove Redundant `this.` Qualifiers

**Files:**

- Modify: `src/FixedMathSharp/**/*.cs`
- Modify only if already touched: `tests/FixedMathSharp.Tests/**/*.cs`

- [x] Remove `this.` where it only adds noise.
- [x] Keep explicit member qualification when it prevents constructor parameter
  confusion, especially around `m_rawValue`.
- [x] Do not mix this purely mechanical cleanup with semantic API changes.

Implementation result on 2026-06-07: removed redundant source `this.`
qualifiers from constructors and simple member calls while preserving
`this.m_rawValue` in the internal `Fixed64(long m_rawValue)` constructor,
where the qualifier disambiguates the parameter from the readonly field.

Verification:

```bash
rg -n "\\bthis\\." src/FixedMathSharp tests/FixedMathSharp.Tests
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
```

## Phase 7: Public Naming Consistency Review

**Files:**

- Review and modify: `src/FixedMathSharp/Numerics/**/*.cs`
- Review and modify: `src/FixedMathSharp/Core/*.cs`
- Modify: matching tests
- Modify: README or wiki docs if public names change

- [x] Normalize singular/plural and tense mismatches such as `ToDegree` versus
  `ToDegrees`, `Lerped` versus `Lerp`, and `GetMagnitude` versus `Magnitude`.
- [x] Prefer names that make mutability explicit: `NormalizeInPlace`,
  `GetNormalized`, and `IsNormalized` should remain semantically distinct.
- [x] Decide whether `Scale` means scalar multiply, component-wise multiply, or
  both; rename if that ambiguity hurts public discoverability.
- [x] Document any intentional deviations where a common game-math name is kept
  for discoverability.
- [x] Update tests and benchmark names with the public API changes so future
  reports read cleanly.

Implementation result on 2026-06-07: renamed the public value-returning
normalization surface from `Normal` to `Normalized`, mutating normalization
overloads from `Normalize` to `NormalizeInPlace`, static plane/matrix
normalization from `Normalize` to `GetNormalized`, squared APIs from `Sqr*` to
`*Squared`, double-based factories from `FromFloatPoint`/`FromFloat` to
`FromDouble`, and scalar raw helpers from `RawToString`/`RawToInt` to
`ToRawString`/`ToInt`. `FixedPlane.Normal` remains unchanged because it is a
real plane normal field, not a normalized-value convenience API. `GetMagnitude`
remains intentionally because it mirrors `GetNormalized` and avoids a C#
member-name conflict with instance `Magnitude` properties.

Verification:

```bash
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release -f net8.0 --no-restore
```

Verification result on 2026-06-07: the full Debug solution test run passed
with 951 tests, the Release `net8.0` benchmark project build passed with zero
warnings/errors, Release and ReleaseLean `netstandard2.1` builds passed with
zero warnings/errors, the `this.` scan now only reports the intentional
`Fixed64(long m_rawValue)` field/parameter disambiguation, the stale public API
name scan only reports old names in Phase 7 rename-history notes, and
`git diff --check` passed.

## Phase 8: `Fast*` And Special-Case Helper API Review

**Files:**

- Modify: `src/FixedMathSharp/Core/FixedMath.cs`
- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.Extensions.cs`
- Modify: `tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs`
- Modify: relevant benchmark files under `tests/FixedMathSharp.Benchmarks/`

- [x] Decide whether public `FastAdd`, `FastSub`, `FastMul`, and `FastMod`
  should remain public, become internal, or move behind clearer naming.
- [x] Document that `Fast*` helpers skip overflow/saturation checks and are not
  drop-in replacements for public operators.
- [x] Keep exact special-case helpers such as positive-divisor paths internal
  unless a public API has a compelling consumer story.
- [x] Benchmark each accepted `Fast*` use in real consumers; do not swap
  operators for `Fast*` based on theoretical speed alone.
- [x] Add tests proving accepted fast paths preserve intended deterministic raw
  results or explicitly document where they intentionally do not.

**Implementation Result:**

Kept `FastAdd`, `FastSub`, `FastMul`, and `FastMod` public because they are
useful expert APIs when callers can prove the raw-value invariants. Strengthened
their XML docs so the unchecked raw-payload behavior is explicit, including the
important `FastMul` precision caveat: it skips the guarded operator's full-width
saturation path and truncates discarded fractional bits instead of using
round-half-to-even.

Added public `FixedMath.FastDiv` plus the `Fixed64.FastDiv` extension. The
consumer story is narrower than the other `Fast*` helpers: it preserves `/`
semantics and uses a positive-divisor fast path only when `y > 0`, falling back
to the guarded operator for zero or negative divisors. The earlier internal
`Fixed64.DivideByPositive` helper was retired so `FastDiv` is the single
searchable division fast path.

No internal call sites were swapped to `Fast*` purely on theory. A short
division-only benchmark showed `FastDiv` slightly ahead of `/` for the
positive-divisor fixture (`5.637 us` vs `5.723 us`, zero allocation), but the
margin is small enough that wider call-site changes should still be justified by
local benchmarks.

A fast follow added an integer-operand shortcut to `FastMul`. The branch checks
the raw fractional mask directly rather than routing through `Fixed64.IsInteger`
so the hot path avoids an extra call shape while preserving the same invariant.
Short benchmark results improved in all measured `FastMul` fixtures:
fractional operands moved from `656.4 ns` to `599.3 ns`, integer-left operands
from `569.2 ns` to `383.9 ns`, and integer-right operands from `698.4 ns` to
`520.5 ns`, all with zero allocation. Focused tests lock the unchecked raw
semantics for integer-left and integer-right operands.

**Verification Result:**

- Focused scalar tests passed: 958 passed.
- Benchmark project Release/net8.0 build passed.
- Short division benchmark passed with `Divide` and `FastDiv`.
- Short multiplication benchmark passed with fractional, integer-left, and
  integer-right `FastMul` fixtures.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedMath|FullyQualifiedName~Fixed64"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i --filter '*Div*' --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i --filter '*FastMul*' --exporters json
```

## Phase 9: Span Overloads And Squared-Distance Hot Path Cleanup

**Files:**

- Review and modify: `src/FixedMathSharp/Geometry/Bounds/*.cs`
- Review and modify: `src/FixedMathSharp/Numerics/Vectors/*.cs`
- Modify: matching tests under `tests/FixedMathSharp.Tests/`
- Modify: matching benchmarks under `tests/FixedMathSharp.Benchmarks/`

- [x] Add `ReadOnlySpan<T>` overloads where public APIs currently accept
  `IEnumerable<T>` and can benefit from countable, allocation-free iteration,
  starting with bounds construction APIs such as sphere-from-points workflows.
- [x] Keep `IEnumerable<T>` overloads only where they remain useful
  interoperability entry points, forwarding to optimized shared internals where
  possible.
- [x] Replace distance-threshold helpers with squared-distance comparisons
  where callers only need `distance <= threshold`.
- [x] Preserve existing behavior for invalid inputs, empty collections, and
  degenerate geometry.
- [x] Benchmark span overloads and squared-distance changes independently so
  collection-shape wins and math wins are not conflated.

Result:

- Added `FixedBoundSphere.CreateFromPoints(ReadOnlySpan<Vector3d>)` for
  allocation-free sphere construction from countable point data.
- Added an exact `FixedBoundSphere.CreateFromPoints(Vector3d[])` overload so
  array call sites stay unambiguous now that both `IEnumerable<Vector3d>` and
  `ReadOnlySpan<Vector3d>` overloads exist.
- Kept `FixedBoundSphere.CreateFromPoints(IEnumerable<Vector3d>)` as the broad
  interoperability entry point. It preserves the existing null/empty behavior,
  uses `IReadOnlyList<Vector3d>` directly when available, and only materializes
  streaming enumerables when there is no countable shape to index.
- Reworked `Vector2d.CheckDistance` and `Vector3d.CheckDistance` to compare
  squared distances after a negative-threshold guard, removing an unnecessary
  square root from threshold checks.
- Swept the runtime public surface for additional `IEnumerable<T>` span
  candidates and distance-threshold helpers. No other obvious production
  candidates were found in this phase.

Measured short-run results:

| Benchmark | Mean | Allocated |
| --- | ---: | ---: |
| `BoundsBenchmarks.SphereCreateFromPointsArray` | 7.643 us | - |
| `BoundsBenchmarks.SphereCreateFromPointsSpan` | 7.461 us | - |
| `BoundsBenchmarks.SphereCreateFromPointsEnumerable` | 10.286 us | 12,456 B |
| `Vector2dBenchmarks.DistanceThreshold` | 14.289 us | - |
| `Vector2dBenchmarks.CheckDistance` | 3.262 us | - |
| `Vector3dBenchmarks.DistanceThreshold` | 24.079 us | - |
| `Vector3dBenchmarks.CheckDistance` | 4.494 us | - |

Benchmark commands:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i --filter '*SphereCreateFromPoints*' --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d -j Short -i --filter '*Distance*' --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector3d -j Short -i --filter '*Distance*' --exporters json
```

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Bounds|FullyQualifiedName~Vector2d|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds vector2d vector3d vector4d -j Short -i --exporters json
```

## Final Regression And Documentation Pass

**Files:**

- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: relevant docs under `docs/`
- Review: `docs/feature-work/issue-tracker.md`

- [x] Update README API guidance after the final public shape is known.
- [x] Update `AGENTS.md` if the canonical ownership rules or public API policy
  should guide future agents.
- [x] Defer migration notes for removed or renamed v4 APIs until the release
  note pass can review the full v4.0.1-to-v5.0.0 commit range.
- [x] Move unresolved follow-up ideas into a new feature-work plan or the issue
  tracker before marking this plan done.
- [x] Move this plan to `docs/feature-work/done/` after all phases are
  completed and verified.

Result:

- Added concise README guidance for the final public API shape: canonical
  scalar algorithms live on `FixedMath`, representation and conversion live on
  `Fixed64`, mutation uses explicit `*InPlace` APIs, extensions are curated
  receiver-shaped conveniences, `Fast*` helpers are expert APIs, and span/array
  overloads are preferred for countable hot-path data.
- Updated `AGENTS.md` so future work preserves the same ownership model,
  avoids reintroducing vector-result `out` helpers, keeps convention-heavy
  factories on owning types, and treats public `Fast*` helpers as documented
  expert APIs that need benchmarked call-site proof.
- Skipped migration notes by design. They should be generated later from the
  full release commit range rather than only this feature plan.
- Extracted the remaining diagnostics and formatting ownership question into
  `docs/feature-work/2026-06-07-diagnostics-formatting-surface-plan.md`.

Verification:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release -f net8.0 --no-restore
git diff --check
```

Verification result on 2026-06-07: `dotnet restore`, the Debug solution build,
the full Debug solution test run, the Release `netstandard2.1` build, the
ReleaseLean `netstandard2.1` build, and the Release `net8.0` benchmark project
build all passed with zero warnings/errors. The full test run reported 963
passed tests.

## Recommended First Slice

Start with Phase 1 and produce the ownership map before touching runtime code.
Then execute Phase 2 as the first implementation slice because vector mutation
is highly visible, already benchmarked, and a good proving ground for the
`this ref` extension-method question. If `this ref` chaining does not produce a
measured win or clearly better call sites, keep the public surface simpler and
prefer canonical return-by-value statics plus instance `*InPlace` methods.
