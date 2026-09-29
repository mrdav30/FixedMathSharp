# FixedMathSharp Benchmarks

This project contains the BenchmarkDotNet runner, alias catalog, deterministic
fixtures, and focused measurements for FixedMathSharp hot paths. The suite
covers scalar arithmetic, fixed trigonometry, vector operations, quaternion
rotations, matrix transforms, geometry, bounds, formatting, and serialization.

## Requirements

- .NET 10 SDK selected by `global.json`
- .NET 8 runtime for the benchmark target
- `Release` configuration for meaningful measurements

Avoid measuring `Debug` builds except when diagnosing benchmark setup failures.

## Running

Build the benchmark runner first, then execute the compiled DLL through the
configured `dotnet` host:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
```

On Linux/WSL this avoids `dotnet run` launching a generated apphost that does
not inherit capabilities such as `cap_sys_nice`. When the `dotnet` host is
configured for elevated process priority, run the built DLL so BenchmarkDotNet
can use that capability.

### List available benchmark selections

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
```

### Run all benchmarks

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all
```

### Run a selection by alias

Aliases are derived from benchmark class names. `Benchmarks` or `Benchmark` is
stripped, and the remaining words are joined with `-`.

For a class named `Vector3dBenchmarks`, the selection alias is `vector3d`:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector3d
```

Multiple aliases can run together:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic vector3d
```

### Forward BenchmarkDotNet arguments

Arguments after the selection are forwarded to BenchmarkDotNet:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --list flat
```

The launcher returns a nonzero exit code for malformed arguments, unmatched
benchmark filters, critical validation errors, failed
reports, or reported executions with missing workload results or nonzero child
exits. A successful earlier launch does not hide a later failure. Valid listing,
help, version and information commands return zero. This checks the results exposed by BenchmarkDotNet;
retain logs and check expected launches before using a capture as performance
evidence, including any separate diagnoser processes.

Run the launcher regression checks with PowerShell 7:

```powershell
pwsh -NoProfile -File tests/FixedMathSharp.Benchmarks/Verify-ExitCodes.ps1
```

The script builds the real launcher with small synthetic benchmarks in an
isolated directory under `artifacts/benchmark-exit-codes`. It checks child and
partial-launch failures, a nonzero exit after results, critical validation,
malformed arguments, empty selections, successful execution and informational
routing. It uses the benchmark project's
BenchmarkDotNet version and adds no fixtures to the regular benchmark catalog.
Its generated files and logs are disposable; the script recreates them.

### Fast development check

Use BenchmarkDotNet's short out-of-process job for broad local smoke runs. This
verifies benchmark code compiles and produces plausible output while preserving
row-level process isolation:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all -j Short
```

For narrow diagnostics, a filtered in-process run is still useful because it is
fast and keeps setup noise low:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i --filter "*FrustumCreateFromMatrix*"
```

Do not treat short-run numbers as canonical measurements, and do not use broad
`-i` runs as release validation evidence.

## Suggested Benchmark Areas

Start with hot paths that can be isolated and repeated deterministically:

- `Fixed64` arithmetic, division, square root, and trigonometry.
- `Vector2d`, `Vector3d`, and `Vector4d` arithmetic, dot/cross products,
  normalization, distance, interpolation, and transforms.
- `FixedQuaternion` creation, multiplication, interpolation, and vector
  rotation.
- `Fixed3x3` and `Fixed4x4` creation, multiplication, inversion, and
  point/vector transforms.
- Bounds containment/intersection checks and projection/clamping helpers.
- Serialization roundtrips for standard builds when payload size and allocation
  behavior matter.
- Diagnostics formatting when repeated display, logging, or editor polling needs
  the allocation-free `TryFormat` path.

## Authoring Guidelines

- Put benchmark classes in the `FixedMathSharp.Benchmarks` namespace.
- Prefer one benchmark class per subsystem or scenario group.
- Apply `[MemoryDiagnoser]` to benchmark classes unless there is a specific
  reason not to.
- Use deterministic fixtures and fixed seeds. Do not use ambient randomness in
  measured paths.
- Reset or dispose context between benchmark cases so measurements do not depend
  on previous cases.
- Capture both throughput and allocation impact when changing hot-path
  arithmetic, normalization, transforms, bounds dispatch, serialization, or
  fixture generation.
- For loop-style bounds benchmarks that iterate over
  `BenchmarkFixtures.SampleCount`, use the local `SampledBenchmarkAttribute` so
  BenchmarkDotNet reports cost per fixture operation instead of the whole
  sampled batch. Keep end-to-end API benchmarks, such as point-cloud
  construction, as normal `[Benchmark]` rows.

Keep support helpers specific. Remove copied template helpers when they stop
serving a FixedMathSharp benchmark scenario.

`convex-point-containment` measures exact inside/outside queries against
translated four- and six-vertex footprints, with zero and 30-degree rotations.
Setup verifies the intended classifications. This isolates the public convex
containment path; use a consuming library's end-to-end benchmark before claiming
a simulation-frame improvement from its results.

`planar-capsule-sweep` measures the strict whole-body planar relation for a
middle crossing, side tangency, rounded-corner miss and stationary overlap.
Setup checks the classifications; these are individual geometry queries, not
controller-frame measurements.

`centered-capsule-convex` measures closed contact-depth queries for side
overlap, rotated overlap, exact 3–4–5 corner tangency and a one-raw-unit radius
miss. Its `SideVertexContact` row measures contact-anchor construction against
an off-center vertex. Setup verifies each classification and the side witness.
The matching `Strict*` rows measure classification alone, including a positive
sub-raw overlap. Exact touch is intentionally false in strict rows and true in
closed-contact rows.

`oriented-box-anchor` includes paired `SpherePrimary`/`SphereStrict`,
`CapsulePrimary`/`CapsuleStrict`, and `CylinderManifold`/`CylinderStrict` rows
on the same geometry. The `rigid-finite-shape-relation` selection similarly
pairs ordinary hull/capsule, cylinder/capsule, and cylinder/cylinder queries
with their `Strict` counterparts. These isolate the cost avoided when a caller
needs only penetration truth, not normals, ranked depths, or contact witnesses.
They are not end-to-end posture-transaction or simulation-frame measurements.
Ordinary overlap fixtures can exit as soon as a center/core witness proves
intrusion; their results do not estimate the cost of difficult negative queries.

The box/cylinder rows also distinguish parallel cap manifolds, general oblique
contacts, a zero-radius segment, and cap-clipped radial separation. Setup checks
both primary and manifold rejection for the separated fixture; the matching
`FixedOrientedBoxCylinder*` tests supply exact feature and boundary regressions.
An older contact solver returned a false positive for that miss and nonminimum
depths for some positive contacts. Its timings are historical implementation
costs, not an equivalent-correctness performance target. A strict Boolean
predicate remains a different workload from complete contact construction.

The cylinder/capsule contact rows also cover endpoint rims, oblique interior
rims, an intersecting capsule core, zero cylinder radius, zero capsule-core
length, and full-domain arithmetic. Their geometric expectations live in the
matching `CenteredCylinderCapsule*` tests. Compare each fixture unchanged across
revisions; timings from an implementation that returns an incorrect contact
are historical costs, not equivalent-result speed comparisons. These rows
measure one contact query, not a complete physics frame.

Additional strict rows exercise cone/capsule, cylinder/cone, separated finite
rims, and triangle intrusion found only after clipping against the cylinder
caps. Setup validates each expected classification. Separated-rim rows are
correctness-sensitive workloads, not equivalent-result speed comparisons with
an ordinary contact query that misclassifies that geometry.

`PenetratingRimsCylinderCylinder`, `SeparatedRimsCylinderCylinder`, and
`TangentRimsCylinderCylinder` measure overlapping, disjoint, and exactly touching
finite cap rims. Setup validates those classifications;
`CenteredCylinderPairStrictRegressionTests` supplies the independent geometry,
minimum-depth bounds, and exact boundary checks. The penetrating geometry was
previously named `ReducedCoreAxisCylinderCylinder`; its historical selected-axis
depth was not the global minimum. Timings from that incomplete implementation
remain incorrect-answer costs, not equivalent-result performance baselines.

`triangle-capsule-slab-contact` measures complete triangle contacts against a
planar stadium extruded through a flat Y slab. The cases cover cap faces,
straight sides, odd-raw rounded ends, oblique rims, a certified rim gap, and
anchors beyond the scalar world-coordinate range. Setup validates overlap and
separation. Compare these fixtures unchanged; the historical incomplete axis
set misclassifies the certified gap, so its timings are not a baseline for an
equally complete solver. This does not imply that every historical positive
fixture had a wrong result. The `FixedTriangleCapsuleSlab*` tests provide exact
geometry and paired-witness regressions.

## Baseline Artifacts

Before starting optimization work, capture a baseline:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

BenchmarkDotNet writes results to `BenchmarkDotNet.Artifacts/results/` by
default. Archive the JSON or markdown reports before changing algorithms so
regressions can be compared against known results.

### Comparing Results

Use full `Release` BenchmarkDotNet artifacts for performance claims. Short
out-of-process runs are useful for broad smoke checks and alias validation.
Filtered in-process runs are useful for quick allocation diagnostics, but broad
in-process batches are not stable enough for release notes or regression gates.

When comparing a branch against a baseline:

1. Build the benchmark runner in `Release`.
2. Run the selected benchmark group with JSON export before changing runtime
   code.
3. Preserve the baseline artifact outside `BenchmarkDotNet.Artifacts/` if it
   needs to survive cleanup.
4. Make one focused runtime change.
5. Re-run the same benchmark group with the same command, SDK, OS, CPU power
   mode, and configuration.
6. Compare mean/median timing, error range, allocation bytes, GC counts, and
   benchmark method semantics.

Do not treat a single faster local run as proof. Prefer a repeatable trend,
especially for very small methods where noise can be larger than the measured
delta.

### Review Checklist

Use this checklist before turning a benchmark result into an optimization task
or release note:

- **Scenario:** the benchmark represents a real public API path or a documented
  synthetic stress case.
- **Environment:** OS, CPU, .NET SDK/runtime, configuration, and BenchmarkDotNet
  job are recorded.
- **Timing:** mean, error, and standard deviation are compared against the same
  baseline command.
- **Allocation:** allocated bytes and GC counts are explained, including
  intentional payload allocations such as serialization output.
- **Complexity:** operation count, loop shape, branch shape, duplicated work,
  data movement, and algorithmic complexity are considered.
- **Determinism:** optimized code preserves fixed-point semantics across
  `Release` and `ReleaseLean`.
- **Correctness:** focused tests cover any runtime behavior touched by the
  optimization.
- **Scope:** public API changes are avoided unless the measured benefit is large
  enough to justify the compatibility cost.
- **Reporting:** release notes include measured deltas and environment, not
  unsourced claims like "faster" or "zero allocation."

### Release Notes

Performance release notes should be proportional and evidence-backed. Include:

- The affected API or scenario.
- The baseline and optimized commands or artifact names.
- The measured timing/allocation delta.
- The environment used for measurement.
- Any compatibility or determinism caveat.

Example:

```text
Improved FixedMath.Sin valid-input hot path by removing eager exception-message
allocation. In a .NET 8 Release BenchmarkDotNet run on <CPU/OS>, the focused
fixed64-arithmetic suite reported 0 B allocated for Sin/Cos/Tan after the
change, with correctness tests passing for FixedMath and FixedTrigonometry.
```

## CI Guidance

CI should at minimum compile the benchmark project in `Release`. The normal
`FixedMathSharp.slnx` build already includes `tests/FixedMathSharp.Benchmarks`;
use this direct command when isolating benchmark compilation locally:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release
```

Full benchmark execution in CI is optional because raw timing thresholds are
sensitive to runner hardware. Any CI performance gate should use BenchmarkDotNet
comparison support or stored baseline artifacts. A smoke job should compile the
benchmark project first and use a short alias-scoped run only to verify
selection and basic execution, not to claim a performance delta.
