# FixedMathSharp Agent Instructions

## Purpose

FixedMathSharp is a high-performance, engine-agnostic, deterministic fixed-point
math library for .NET simulations, games, tooling, procedural generation,
replays, and lockstep-style systems. The library centers on `Fixed64`
([`src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`](src/FixedMathSharp/Numerics/Scalars/Fixed64.cs))
with Q32.32 representation (`SHIFT_AMOUNT_I = 32` in
[`src/FixedMathSharp/Core/FixedMath.cs`](src/FixedMathSharp/Core/FixedMath.cs)).

This repo grew out of an older Unity prototype and was abstracted into an
engine-neutral math package over time. Treat Unity-originated assumptions as
historical context, not as source of truth. When a convention, algorithm, data
layout, or API shape looks copied from Unity, XNA, MonoGame, or another engine,
verify whether it is still optimal for FixedMathSharp's goals before extending
it.

Current priorities:

1. Preserve deterministic behavior across supported platforms and target
   frameworks.
2. Prefer the fastest, lowest-allocation, lowest-complexity solution that is
   still correct and explainable.
3. Keep the core runtime engine-agnostic; engine-specific conversions belong in
   adapter packages or docs.
4. Use benchmarks to guide hot-path redesigns and to demonstrate why the
   library is worth using.
5. Avoid band-aid layers that preserve weak translations from the Unity
   prototype when a clean deterministic design is the right move.

## Start Here

Read these in order before making non-trivial changes:

1. [`README.md`](README.md) for package orientation, supported packages, and
   current positioning.
2. [`src/FixedMathSharp/FixedMathSharp.csproj`](src/FixedMathSharp/FixedMathSharp.csproj),
   [`tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj`](tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj),
   and [`tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj`](tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj).
3. The relevant source area under [`src/FixedMathSharp`](src/FixedMathSharp).
4. The matching test file under [`tests/FixedMathSharp.Tests`](tests/FixedMathSharp.Tests).
5. [`tests/FixedMathSharp.Benchmarks/README.md`](tests/FixedMathSharp.Benchmarks/README.md)
   before changing measured hot paths or adding benchmark cases.
6. [`docs/complexity-exceptions.md`](docs/complexity-exceptions.md) before
   refactoring high-complexity deterministic hot paths only to satisfy a metric.
7. Existing feature plans under [`docs/feature-work`](docs/feature-work) when
   the task touches an active design thread, such as coordinate conventions.

## Source Of Truth

When code, README text, generated docs, and copied benchmark scaffolding
disagree, prefer current source, project files, tests, and workflows. Keep docs
honest as old prototype assumptions are corrected.

Keep these aligned whenever behavior, public API, package shape, performance
claims, serialization layout, or developer workflow changes:

- [`README.md`](README.md)
- [`AGENTS.md`](AGENTS.md)
- [`docs`](docs)
- [`tests/FixedMathSharp.Tests`](tests/FixedMathSharp.Tests)
- [`tests/FixedMathSharp.Benchmarks`](tests/FixedMathSharp.Benchmarks)
- [`.github/workflows`](.github/workflows)

## Repository Map

| Path | Purpose | Notes |
| --- | --- | --- |
| [`src/FixedMathSharp`](src/FixedMathSharp) | Main math library | Multi-targets `netstandard2.1` and `net8.0`. |
| [`src/FixedMathSharp/Numerics/Scalars`](src/FixedMathSharp/Numerics/Scalars) | `Fixed64` and scalar helpers | Preserve guarded overflow and deterministic rounding. |
| [`src/FixedMathSharp/Core`](src/FixedMathSharp/Core) | Shared fixed math and trigonometry algorithms | Hot deterministic backbone. Measure before redesigning. |
| [`src/FixedMathSharp/Numerics/Vectors`](src/FixedMathSharp/Numerics/Vectors) | `Vector2d`, `Vector3d`, `Vector4d`, and vector extensions | Watch coordinate-convention assumptions. |
| [`src/FixedMathSharp/Numerics/Rotations`](src/FixedMathSharp/Numerics/Rotations) | `FixedQuaternion` and rotation helpers | High-risk for convention, normalization, and determinism changes. |
| [`src/FixedMathSharp/Numerics/Matrices`](src/FixedMathSharp/Numerics/Matrices) | `Fixed3x3`, `Fixed4x4`, and matrix extensions | Keep transform storage and basis semantics explicit. |
| [`src/FixedMathSharp/Geometry`](src/FixedMathSharp/Geometry) | Bounds and primitive geometry | Includes 2D `FixedBoundArea`, 3D `FixedBoundBox`, `FixedBoundSphere`, `FixedBoundFrustum`, `FixedRay`, and `FixedPlane`. |
| [`src/FixedMathSharp.FluentAssertions`](src/FixedMathSharp.FluentAssertions) | Test assertion helpers package | Keep helpers aligned with core API semantics. |
| [`tests/FixedMathSharp.Tests`](tests/FixedMathSharp.Tests) | xUnit v3 test project | Add focused deterministic, edge-case, serialization, and regression coverage. |
| [`tests/FixedMathSharp.Benchmarks`](tests/FixedMathSharp.Benchmarks) | BenchmarkDotNet project | Experimental performance lab and showcase for hot-path wins. |
| [`docs/feature-work`](docs/feature-work) | Active and completed implementation plans | Use for multi-step design and optimization efforts. |
| [`docs/wiki`](docs/wiki) | Deeper package documentation | Add focused pages when README would become crowded. |

Ignore generated output when reviewing structure:

- `bin/`
- `obj/`
- `TestResults/`
- `artifacts/`
- `BenchmarkDotNet.Artifacts/`
- `.vs/`

## Determinism Rules

Any change that affects rounding, overflow, iteration order, equality,
normalization, transforms, serialization layout, random streams, or benchmarked
hot paths is high risk.

Always prefer:

- `Fixed64`, `Vector2d`, `Vector3d`, `Vector4d`, and `FixedQuaternion` over
  `float`, `double`, and `System.Numerics` in deterministic runtime logic.
- Explicit, stable ordering when iterating collections, bounds, query results,
  or generated data.
- Deterministic seeds and reproducible fixtures for random-dependent behavior.
- `Fixed64` constants (`Fixed64.Zero`, `Fixed64.One`, `Fixed64.Pi`) over
  primitive literals in math-heavy code.
- Source-level explanations and tests for numerical thresholds, saturation, and
  approximation behavior.

Avoid:

- Ambient time, ambient randomness, process-global conventions, hidden engine
  state, or platform-specific floating-point shortcuts in deterministic paths.
- Silent clamping that hides an algorithmic flaw without tests explaining the
  invariant.
- Using Unity, XNA, or MonoGame naming as proof that a convention is right for
  core FixedMathSharp.

## Performance And Benchmarking Bar

FixedMathSharp should not do something merely because engines or common math
libraries do it that way. Optimizations, new algorithms, and unusual data
layouts are welcome when they are deterministic, measurable, and maintainable.

For hot-path changes:

- Establish or update a BenchmarkDotNet baseline in
  [`tests/FixedMathSharp.Benchmarks`](tests/FixedMathSharp.Benchmarks) before
  broad optimization work.
- Compare against the current implementation and, when useful, against a simple
  reference implementation that proves correctness.
- Capture allocation impact with `[MemoryDiagnoser]` unless there is a specific
  reason not to.
- Use deterministic fixtures and fixed seeds; benchmark setup must not depend
  on ambient randomness or previous benchmark cases.
- Prefer algorithmic wins, fewer branches, fewer allocations, better cache
  locality, and lower time complexity over micro-optimizations that make the
  code fragile.
- Do not accept a faster result that weakens determinism, public semantics, or
  serialization compatibility.

Benchmark workflow:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all -j Short -i
```

Use short in-process runs only as smoke checks. Treat full `Release` benchmark
runs and exported BenchmarkDotNet artifacts as the evidence for performance
claims.

## Experimental Design And Evidence Bar

The library is allowed to be better than its prototype lineage. If old code
looks like a translation artifact from Unity or another engine, investigate it
instead of preserving it automatically.

Accept novel approaches when they:

- preserve deterministic replay for the same inputs.
- have clear invariants, units, ordering rules, and failure modes.
- improve or preserve time complexity and allocation behavior.
- are covered by focused tests and benchmarked when they touch hot paths.
- keep engine-specific assumptions outside the core package.

Reject clever changes that cannot be explained, tested, benchmarked, or made
deterministic.

## Code Conventions Specific To This Repo

- Preserve public API shape unless the task explicitly requests API changes.
- Match existing style: regions, XML docs, explicit namespaces, no implicit
  usings in project files.
- Shared runtime code must remain compatible with both `netstandard2.1` and
  `net8.0`. Do not use APIs that are unavailable to `netstandard2.1` unless
  they are guarded, isolated to a compatible target, or intentionally placed in
  tooling/benchmark code.
- Keep operations allocation-light; many hot methods use `m_rawValue` and
  `[MethodImpl(MethodImplOptions.AggressiveInlining)]`.
- Keep production source files under roughly 1000 lines. When a file grows past
  that threshold, split it into meaningful partials such as `*.Statics.cs`,
  `*.Operators.cs`, `*.Conversions.cs`, or `*.Equality.cs`; avoid one-method or
  two-method partials that make navigation worse.
- `FixedMath` and fixed trigonometry code are shared algorithm backbones.
  Extension classes are thin forwarding wrappers, not alternate implementations.
- Public API ownership should stay intentional: `FixedMath` owns deterministic
  scalar algorithms, `Fixed64` owns Q32.32 representation/conversion/operators,
  vectors/quaternions/matrices own their domain operations, and extensions
  expose only curated receiver-shaped conveniences.
- Prefer return-by-value statics/operators plus explicit `*InPlace` methods for
  mutation. Do not reintroduce vector-result `out` helper shapes unless a
  measured design proves they are clearer and faster.
- Keep factories and convention-heavy APIs, such as Euler, look-at, projection,
  and transform construction, on the owning type rather than extension classes.
- Treat public `Fast*` helpers as expert APIs. Their XML docs must state the
  skipped checks or precision caveats, and runtime call sites should use them
  only after local benchmarks prove the invariant and speed win.
- Preserve saturating and guarded semantics in operators and math helpers.
- When touching bounds logic, maintain the explicit cross-type `Contains` and
  `Intersects` overloads among boxes, areas, spheres, frustums, planes, and
  rays; do not replace them with a generic abstraction unless a measured design
  proves it is faster and clearer.
- For coordinate conventions, treat `+Z` forward as the current core convention
  until an approved plan changes it. Engine-specific forward/back naming belongs
  at adapter boundaries.

## Serialization And Package Variants

Serialization compatibility is intentional.

- MemoryPack attributes on serializable structs, such as `[MemoryPackable]` and
  `[MemoryPackOrder]`, are the source of truth for MemoryPack layout.
- Tests should use MemoryPack-based roundtrips and `System.Text.Json` where
  appropriate.
- `Release` builds the standard package with MemoryPack support.
- `ReleaseLean` sets `FIXEDMATHSHARP_DISABLE_MEMORYPACK` and excludes
  `*.MemoryPack.cs` files to build `FixedMathSharp.Lean`.
- Do not add direct MemoryPack dependencies to code that must compile in Lean
  mode.

## Build, Test, And Coverage Workflows

Solution: [`FixedMathSharp.slnx`](FixedMathSharp.slnx), with the core library,
FluentAssertions package, test project, and benchmark project.
`global.json` selects the .NET 10 SDK for `.slnx` tooling consistency while the
runtime projects continue to target `netstandard2.1` and `net8.0`.

Typical local workflow:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug
```

Release validation:

```bash
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
```

Coverage uses `tests/FixedMathSharp.Tests/coverlet.runsettings`. CI runs
Release and ReleaseLean on Linux and Windows through
[`.github/workflows/build-and-test.yml`](.github/workflows/build-and-test.yml).
The coverage workflow publishes coverage reports after `build-and-test`
completes on `main`.

Packaging/versioning comes from
[`src/FixedMathSharp/FixedMathSharp.csproj`](src/FixedMathSharp/FixedMathSharp.csproj):
GitVersion variables are consumed when present, otherwise version falls back to
`0.0.0`.

## Testing Patterns To Mirror

- Tests are xUnit v3 under [`tests/FixedMathSharp.Tests`](tests/FixedMathSharp.Tests).
- Keep one feature area per test file, such as `Vector3d.Tests.cs` or
  `Geometry/Bounds/FixedBoundBox.Tests.cs`.
- Use helper assertions from
  [`tests/FixedMathSharp.Tests/Support/FixedMathTestHelper.cs`](tests/FixedMathSharp.Tests/Support/FixedMathTestHelper.cs)
  for tolerance and range checks instead of ad-hoc epsilon logic.
- For deterministic RNG changes, validate same-seed reproducibility and
  bounds/argument exceptions like `DeterministicRandom.Tests.cs`.
- For serialization changes, update MemoryPack and JSON roundtrip tests.
- For performance-sensitive changes, pair correctness tests with benchmark
  coverage rather than relying on benchmark output alone.
- Cyclomatic complexity exceptions are documented in
  [`docs/complexity-exceptions.md`](docs/complexity-exceptions.md); update that
  register when touching methods above the review threshold instead of
  refactoring hot deterministic paths only to lower a metric.

## Agent Editing Guidance

- Make focused edits in the relevant numeric, geometry, serialization, test, or
  benchmark area.
- Prefer optimized, low time-complexity code. No band-aid solutions.
- Read the matching tests and benchmarks before changing hot-path behavior.
- If a docs or benchmark scaffold looks copied from another library, adapt it to
  FixedMathSharp instead of preserving irrelevant examples.
- Do not revert unrelated dirty files. Work with user changes and keep your diff
  scoped to the request.
