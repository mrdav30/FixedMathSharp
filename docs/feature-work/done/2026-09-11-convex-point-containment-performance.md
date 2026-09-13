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

## 2026-09-13: reuse rotated polygon endpoints

Keep a second, one-method containment simplification, measured against
`e8a2ab5fa92bea967fe60d8215bdfbf26768b795`. Widen point-minus-origin once,
rotate the first vertex, then carry each rotated endpoint into the next edge.
This uses only existing wide primitives and local values; no cache, public API,
caller change or navigation policy is added. A complete n-edge traversal now
performs n+1 vertex rotations rather than three rotation-helper calls per edge,
including the former zero-offset transform. The closing vertex is deliberately
rotated again, retaining the existing wrap conditional without another cache.

At the integer product scale, `R(end) - R(start) == R(end - start)` exactly.
Likewise, `(Scale(point) - Scale(origin)) - R(start)` equals the previous
world-point subtraction. Both operands are widened before subtraction; no
Fixed64 saturation or rotation rounding is introduced. Conservative full-domain
bounds keep rotated components at most 2^127 in magnitude, edge and point-relative
components below 2^128 and the determinant below 2^257. Signed192/Signed320 retain
the exact values. All four callers validate at least three vertices before the
internal method. Edge order, zero determinants, boundaries and earliest rejection
are unchanged, including repeated vertices.

Eight new literal cases exercise both windings, closing-edge rejection, a
quarter turn, an oblique exact boundary with a repeated vertex, one-raw misses,
full-width edges and point/origin differences wider than Fixed64. The unchanged
implementation and candidate each pass all 46 focused relation tests; candidate
ReleaseLean also passes. Temporarily skipping the closing edge makes all six
closing-edge cases fail. That mutation is removed. Independent proof and
Ponytail reviews find no actionable issue.

### Measurement limits and practical result

The two new isolated microbaselines are **incomplete and excluded**, not a new
eight-case speedup result. Both fail in `Outside(4 vertices, 30 degrees)`, launch
2, before the optimization: first a native access violation, then an NRE in
`GetWorldPoint`. Each executes only 23/24 planned children. Their partial
statistics are not used, and no candidate microcomparison is claimed.
[`FMS-Issue-018`](../issue-tracker.md#fms-issue-018-convex-containment-benchmark-child-fails-during-measurement)
retains both failures and the bounded no-dump diagnostic replay. Eliminating
the reported helper from this call path does not establish a crash fix.

The keep decision instead uses complete matched **Trailblazer guided-frame**
captures on the same environment described above. A*/100, Flow/100 and Flow/500
64-frame block medians fall by 7.6%, 8.3% and 9.8%; periodic recheck medians fall
from 118.644 / 96.029 / 392.543 ms to 107.127 / 86.388 / 353.334 ms.
Every corresponding launch-level block and recheck median decreases; ordinary
A* is effectively unchanged in aggregate and mixed by launch. All 18 children
complete. All 45 exported blocks per version retain exact cross-version frame
checksums and zero frame allocation/collections; timing summaries use the 27
actual blocks per version. These are stress-fixture results, still above the
31.25 ms full-frame budget, not a universal math speedup or gameplay guarantee.

The coordinating Trailblazer tracker owns the full table, frame tails and next
profile boundary. Evidence lives there under `artifacts/benchmark005`:
`guided-vertex-*`, `vertex-source-audit.json`, `vertex-profile-*` and
`fms-vertex-baseline*`. Executed-child hashes and portable-PDB checks confirm
that the only authored runtime source change is `WideConvex2dRelations.cs`;
benchmark and downstream source inputs match. Both failed microcaptures also
match the guided baseline's authored FMS sources. These checks identify the
executed artifacts and source inputs, not identical native compilation.

### Verification and allocation-guard correction

The first full Lean coverage run failed an **unchanged** product-comparison
allocation guard at 2,208 bytes. Its warmup discarded results in a separate
loop while measurement accumulated them. The guard now uses the existing
`FixedMathTestHelper.MeasureWarmedAllocations` boundary with the same operation
and 64 iterations in each phase. Exact zero bytes and the result checksum remain
mandatory. A temporary 64-byte allocation per iteration makes the corrected
guard fail at 5,632 bytes; it is removed before final validation.
[`FMS-Issue-019`](../issue-tracker.md#fms-issue-019-product-comparison-allocation-guard-reported-an-unexplained-burst)
keeps the original burst unattributed. This is test-protocol alignment, not a
production allocation fix or proof that flakiness is cured.

Fresh final solution builds and coverage runs pass, including both library
targets with zero build warnings/errors:

| Configuration | Core + Chronicler tests | Covered lines | Covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 2,716 + 8 | 47,856/47,856 | 8,828/8,828 | 3,452/3,452 |
| ReleaseLean | 2,695 + 8 | 47,949/47,949 | 8,828/8,828 | 3,448/3,448 |

Core coverage still includes FluentAssertions; Chronicler's eight tests per
configuration remain outside that coverage gate. The failed first matrix is
retained separately in `vertex-verification`; final evidence is under
`vertex-verification-final`. These are local Windows/source-stack checks,
not Linux CI or released-package validation.

Downstream final Release/Lean checks also pass with exact 100% line, branch and
fully covered method coverage: GridForge 852/852 tests, Gravitas 3,950/3,895,
Trailblazer 2,467/2,405 core plus 65 adapter tests in each configuration. Their
source remains unchanged; these validate the shared math change through the
unreleased local stack.
