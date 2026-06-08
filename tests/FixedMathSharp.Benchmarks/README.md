# FixedMathSharp Benchmarks

This project is the BenchmarkDotNet scaffold for FixedMathSharp hot paths.

The runner, alias catalog, deterministic fixture helpers, and first-pass hot-path benchmark classes are in place. The initial suite covers scalar arithmetic, fixed trigonometry, vector operations, quaternion rotations, matrix transforms, and bounds checks.

## Requirements

- .NET 8 SDK
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

Aliases are derived from benchmark class names. `Benchmarks` or `Benchmark` is stripped, and the remaining words are joined with `-`.

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

### Fast development check

Use BenchmarkDotNet's short in-process job for quick local smoke runs. This verifies benchmark code compiles and produces plausible output without a full run:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all -j Short -i
```

Do not treat short-run numbers as canonical measurements.

## Suggested Benchmark Areas

Start with hot paths that can be isolated and repeated deterministically:

- `Fixed64` arithmetic, division, square root, and trigonometry.
- `Vector2d`, `Vector3d`, and `Vector4d` arithmetic, dot/cross products, normalization, distance, interpolation, and transforms.
- `FixedQuaternion` creation, multiplication, interpolation, and vector rotation.
- `Fixed3x3` and `Fixed4x4` creation, multiplication, inversion, and point/vector transforms.
- Bounds containment/intersection checks and projection/clamping helpers.
- Serialization roundtrips for standard builds when payload size and allocation behavior matter.
- Diagnostics formatting when repeated display, logging, or editor polling
  needs the allocation-free `TryFormat` path.

## Authoring Guidelines

- Put benchmark classes in the `FixedMathSharp.Benchmarks` namespace.
- Prefer one benchmark class per subsystem or scenario group.
- Apply `[MemoryDiagnoser]` to benchmark classes unless there is a specific reason not to.
- Use deterministic fixtures and fixed seeds. Do not use ambient randomness in measured paths.
- Reset or dispose context between benchmark cases so measurements do not depend on previous cases.
- Capture both throughput and allocation impact when changing hot-path arithmetic, normalization, transforms, bounds dispatch, serialization, or fixture generation.

Keep support helpers specific. Remove copied template helpers when they stop serving a FixedMathSharp benchmark scenario.

## Baseline Artifacts

Before starting optimization work, capture a baseline:

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

BenchmarkDotNet writes results to `BenchmarkDotNet.Artifacts/results/` by default. Archive the JSON or markdown reports before changing algorithms so regressions can be compared against known results.

### Comparing Results

Use full `Release` BenchmarkDotNet artifacts for performance claims. Short
in-process runs are useful for smoke checks, alias validation, and quick
allocation diagnostics, but they are not stable enough for release notes or
regression gates.

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
`FixedMathSharp.slnx` build already includes `tests/FixedMathSharp.Benchmarks`; use this
direct command when isolating benchmark compilation locally:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release
```

Running full benchmarks in CI is optional until local variance is understood.
When performance gates are introduced, prefer BenchmarkDotNet comparison support
or stored baseline artifacts over raw timing thresholds, which are sensitive to
runner hardware. A future smoke job should compile benchmarks first and, if it
runs benchmarks at all, use a short alias-scoped job that verifies selection and
basic execution without claiming a performance delta.
