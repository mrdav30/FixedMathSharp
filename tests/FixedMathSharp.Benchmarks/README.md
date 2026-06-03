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

## CI Guidance

CI should at minimum compile the benchmark project in `Release`. The normal
`FixedMathSharp.slnx` build already includes `tests/FixedMathSharp.Benchmarks`; use this
direct command when isolating benchmark compilation locally:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release
```

Running full benchmarks in CI is optional until local variance is understood. When performance gates are introduced, prefer BenchmarkDotNet comparison support or stored baseline artifacts over raw timing thresholds, which are sensitive to runner hardware.
