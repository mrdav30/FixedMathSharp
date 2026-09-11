# Convex Point Containment Performance Follow-up

## Scope and decision

Measured on 2026-09-11 (UTC) while investigating Trailblazer's
`TRB-Benchmark-005`. Guided navigation spends substantial time in exact convex
containment through GridForge. Keep this policy-neutral geometry optimization
in FixedMathSharp; no navigation-specific API is required.

The four callers of the internal containment implementation all supplied an
already-world-space point and a zero local offset. Previously, every polygon
edge repeated the general identity-frame transform of that point. The new
implementation scales the point into the same wide product representation once
per query. It removes the unused internal offset argument and unrotated
forwarding overload. Both public containment overloads and both public segment
first-intersection overloads retain their signatures and behavior.

The deleted transform terms were exactly zero. `Scale(point.X/Y)` is signed
raw value multiplied by `2^32`, with no rounding or scalar saturation; the full
Fixed64 domain fits comfortably in `Signed192`. Polygon rotation, conceptual
vertices outside the scalar domain, edge order, sign tests, closed boundaries,
validation and early-exit behavior are unchanged. Gravitas does not consume the
removed internal signatures.

## Isolated measurements

Environment: Windows 11 25H2, Intel Core i7-9700K, .NET SDK 10.0.302,
.NET 8.0.29 x64 RyuJIT, BenchmarkDotNet 0.15.8, Release. Run serially with
normal runtime settings and no concurrent builds, tests or profilers.

```powershell
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll convex-point-containment --launchCount 3 --exporters json --keepFiles --artifacts artifacts/convex-containment-baseline
```

Repeat with `convex-containment-candidate` after the change. The eight cases
use translated quadrilateral/hexagonal footprints, zero/30-degree rotation,
and a verified inside or outside point. These are default adaptive,
out-of-process jobs with three fresh launches per case, not ShortRun jobs.

| Query | Vertices | Rotation | Mean before (ns) | Mean after (ns) | Reduction |
| --- | ---: | ---: | ---: | ---: | ---: |
| Inside | 4 | 0 degrees | 2,005.279 | 1,295.087 | 35.4% |
| Outside | 4 | 0 degrees | 1,029.240 | 685.186 | 33.4% |
| Inside | 4 | 30 degrees | 2,305.237 | 1,596.784 | 30.7% |
| Outside | 4 | 30 degrees | 1,806.693 | 1,281.625 | 29.1% |
| Inside | 6 | 0 degrees | 3,061.183 | 1,929.500 | 37.0% |
| Outside | 6 | 0 degrees | 1,062.202 | 701.616 | 33.9% |
| Inside | 6 | 30 degrees | 3,323.699 | 2,220.563 | 33.2% |
| Outside | 6 | 30 degrees | 1,315.366 | 969.042 | 26.3% |

These are standard BDN overhead-adjusted means with its default outlier
handling. All raw samples remain available: 360 before and 362 after, with the
adaptive job selecting two extra actual iterations in one candidate launch.
BDN retains 330/345 result observations. An independent calculation using
**every raw actual observation**, without overhead subtraction or exclusions,
also finds 26.2-36.9% lower means. All 24 corresponding launch-level raw
medians decrease. Every case reports zero bytes/op and zero Gen0/1/2 collections.
All 48 child executions complete successfully.

This is a normal rebuilt comparison, not a single-DLL swap. Frozen child
manifests have the same 51 paths: 45 hashes match; FixedMathSharp, benchmark
and generated-runner DLL/PDB pairs differ. Generated source/project/build
scripts are byte-identical. Full JSON retains mean, 99.9% confidence interval,
standard deviation, launch identity and every raw measurement.
Portable-PDB checksums also verify that all 26 benchmark source documents are
unchanged; only the two intended FixedMathSharp source documents differ.

The original captures and executed-child/source snapshots are retained in the
coordinating Trailblazer checkout under `artifacts/benchmark005`, with prefixes
`fms-containment-baseline` and `fms-containment-candidate`. The baseline runtime
source is FixedMathSharp `839bbdcd45593f608c523c4e0b5c717288931a4a`; the
candidate adds the two-file containment change described above.

## Correctness and downstream verification

Eight literal boundary cases cover both windings, both scalar-domain faces,
zero/quarter-turn rotation, exact corner inclusion and a one-raw-unit miss
past either face. They pass on both implementations. Deliberately replacing
one coordinate's product scaling with unscaled raw bits makes all eight fail;
restoring the implementation returns all 38 focused relation tests to green.
Independent source/proof review finds no actionable issues.

Full local Release/ReleaseLean verification passes against the unreleased
stack. Both FixedMathSharp solution builds cover both library targets with
zero warnings/errors:

| Configuration | Passing core tests | Covered lines | Covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 2,708 | 47,858/47,858 | 8,828/8,828 | 3,452/3,452 |
| ReleaseLean | 2,687 | 47,951/47,951 | 8,828/8,828 | 3,448/3,448 |

Coverage includes the core and FluentAssertions assemblies. Each configuration
also passes eight separate Chronicler tests, outside the core coverage gate.
Downstream full solution builds and tests pass in both configurations:
GridForge 844/844 tests; Gravitas 3,950/3,895; Trailblazer 2,467/2,405 core plus
65 adapter tests each. Every downstream coverage report retains exact 100%
line, branch and fully covered method coverage. These are local Windows checks,
not Linux CI or released-package validation. Logs and reports are retained
under the coordinating capture's `verification` directory.

Trailblazer's benchmark tracker owns the guided-frame results and remaining
frame-budget work. A microbenchmark improvement is not a universal simulation
speedup. The separate `TRB-Issue-119` setup exception remains unexplained;
removing redundant helper calls is not evidence that it is fixed.
