# FixedMathSharp Benchmark Baseline: 2026-06-04

This folder preserves the first full `Release` BenchmarkDotNet export after the
benchmark scaffold was expanded across scalar math, vectors, quaternions,
matrices, bounds, serialization, curves, ranges, and deterministic random APIs.

## Environment

- Date: 2026-06-04
- OS: Linux Ubuntu 24.04.1 LTS (Noble Numbat)
- CPU: Intel Core i7-9700K CPU 3.60GHz, 1 CPU, 8 logical cores, 8 physical cores
- SDK: .NET SDK 10.0.203
- Runtime: .NET 8.0.26, X64 RyuJIT x86-64-v3
- GC: Concurrent Workstation
- Power mode: BenchmarkDotNet reported power plan id `8c5e7fda-e8bf-4a96-9a85-a6e23a8c635c`; no additional WSL host power pinning was recorded.

## Commands

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --force
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

Result: completed successfully in 00:52:05 and executed 136 benchmarks.

## Archived Artifacts

The archive keeps each benchmark class's compressed JSON report and GitHub
markdown summary. CSV and HTML reports remain in the local
`BenchmarkDotNet.Artifacts/results/` output folder when the run is present, but
are intentionally not copied here to keep the committed baseline compact.

## Baseline Notes

- Serialization benchmarks intentionally allocate payload buffers and serializer
  outputs. Treat those as serialization costs, not core hot-path allocation
  defects, unless a future benchmark isolates avoidable library-side work.
- Non-serialization hot paths were allocation-free except
  `BoundsBenchmarks.FrustumCreateFromMatrix`, which allocated 165,888 B per
  benchmark operation because `FixedBoundFrustum` is currently a sealed reference
  type with internal corner and plane arrays.
- This baseline was captured on a local WSL environment. Use it for trend
  context and hotspot prioritization, not as a raw timing threshold for CI.
