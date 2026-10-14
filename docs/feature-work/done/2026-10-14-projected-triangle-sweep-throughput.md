# Finite-Slab Projected Circle/Triangle Sweep Throughput

## Scope And Baseline

The 2026-10-12 signal came from Gravitas issue 098. Its direct math probe swept
128 / 512 / 2,048 vertical triangles in 13.80 / 52.63 / 205.58 ms, with zero
reported allocations. This was a diagnostic Stopwatch probe, not a matched
BenchmarkDotNet capture. The maintained consumer and immutable comparison paths
remain under Gravitas `artifacts/grv-issue-098`: `query-refined-benchmarks/`,
`query-moving-{before,after}.txt`, `query-moving-timing/` and `query-math-only.txt`.
The original immutable consumer comparison measured dense 2,048-triangle sweeps
at 228.46 / 214.37 ms and proxy misses at 214.72 / 203.57 ms. Those captures
measure a separate consumer admission refinement; use the fresh matched math
captures below to attribute this change.

The fresh source baseline is FixedMathSharp `6842809` and Gravitas `10cf5e8`.
The existing projected triangle benchmark now includes the literal vertical grid:
subdivide the Y/Z square `[-1/2,1/2]^2` at X zero, with two triangles per cell,
using `(a,b,c)` / `(c,b,d)` along the same diagonal as the direct probe. Sweep
from X/Z `(-3,0)` along Right, maximum distance 6, radius 1/2, slab Y zero and
half-thickness 1/2. Every triangle hits. A separate 128-triangle control clips
against half-thickness 1/8 and retains 64 hits, including boundary touches.
Setup verifies these counts. Grid rows report whole-batch cost.

Captures use Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29 and BenchmarkDotNet
0.15.8. All builds use `UseLocalLsfStack=true`, `BuildInParallel=false`,
`UseSharedCompilation=false` and two logical processors. Timing runs disable
tiered compilation and concurrent GC, use affinity 3 and a below-normal launcher,
and are serialized with builds, tests and other measurements. Full captures use
two launches, five warmups, fifteen measurements and 250-ms target iterations.
Means and errors below are microseconds and half the 99.9% confidence interval.

The baseline profile attributes about 50-53% of sampled benchmark activity to
the rational point sweep and 40-42% to the rational segment sweep. The line
comparison occupies 28-31%; the separate closest-line bisection occupies 5-6%.
Clipping is not the dominant cost. These inclusive owners overlap and must not
be added. Traces and the activity-filtered summary use
`artifacts/projected-sweep-baseline-profile` and its sibling summary.

## Retained Arithmetic

Both sweep owners now construct their exact entering root and delegate its
nearest-even physical-distance rounding to the existing
`ContactQuadratic.RoundRatio` owner:

- Point entry is `(-B-sqrt(B²-A*C))*TimeScale/A`. Earlier start, approach and
  intersection checks establish A positive, B negative, C positive and a
  nonnegative discriminant. Two complete 832-bit products fit 27 limbs.
- Line entry is `(|cross|-R*D*sqrt(L))*TimeScale/|velocity|`, where D is the
  first rational point's denominator and L the squared retained integer edge
  vector (including its cross-denominator scale). Existing
  37-word fields cover every retained product, including authored trajectories.

Existing admission establishes a strictly positive numerator and denominator.
If the closest point or line crossing lies within the maximum distance, entry
precedes it; otherwise the exact endpoint-inside check bounds entry by that
maximum. The shared rounding owner therefore receives a proven interval
`[0, maximumDistance]`. It already owns root enclosures, exact integer and
half-raw comparisons, and nearest-even parity. This is a raw distance ratio;
no extra Q32 shift belongs here. Tangency and maximum-distance endpoints keep
their existing admission semantics.

Separate borrowed quadratic banks avoid aliasing scaling; all partially written
banks are cleared. Three obsolete private helpers and both local entry searches
are removed, including rounded closest-parameter materialization. Exact
finite-foot witness construction remains unchanged and uses the unrounded
entering event. Other expanded-box-edge callers retain their half-step helper.
The two runtime files shrink by 190 lines overall, with no new runtime method,
cache, dependency or public API. Fixed borrowed storage is approximately 1.5 KB
for the point block and 1.8 KB for the line block, plus existing shared arithmetic
scratch; the bounded-stack probe below checks the complete public path.

The narrow probes use one launch, three warmups, eight measurements, 100-ms
iterations and unroll factor one. They isolate the sequence, but are not the
protocol used for final before/after claims:

| Narrow stage | Clipped 128 | Full 128 | Full 512 | Full 2,048 |
| --- | ---: | ---: | ---: | ---: |
| Raw closest-line ratio | 5,947 | 12,917 | 50,884 | 200,625 |
| Also enclose line entry | 4,369 | 9,245 | 36,431 | 146,617 |
| Also enclose point entry | 2,127 | 3,984 | 15,802 | 62,755 |
| Round both entries in the shared owner | 1,522 | 3,042 | 11,844 | 47,134 |

Full probe statistics and raw logs use `artifacts/projected-sweep-` with
`ratio-probe`, `line-bounds-probe`, `point-bounds-probe` and `direct-probe` suffixes. These
separately captured stages do not establish additive isolated percentages.
The enclosure-only implementation also has full intermediate captures under
`projected-sweep-final` in both repositories: its full 2,048-triangle math row is
65.316 ms. It is superseded by the smaller direct-rounding implementation below.

## Correctness And Validation

Ten new characterization rows preserve positive-thickness clipping with no
surviving original vertex, both windings, both sweep overloads, reverse travel,
an exact quarter-turn, translated frames and an unmaterializable scalar-face
anchor. The clipped boundary has an analytic distance and authored local point.
The before-contact raw neighbor must miss and the exact boundary must hit.
An equivalent clipped polygon independently checks distance. Existing tangency,
full-domain displacement, tiny physical mappings, odd-raw clipping, half-even
ties, degenerate features, nearest-slab-center witness and warmed-allocation
tests remain gates. All 97 focused rows pass before and after each arithmetic
refinement.

The independent polygon comparison exposed a pre-existing normal/witness
defect, recorded with literal inputs and root cause as
[FMS-Issue-030](2026-10-14-convex-sweep-normal-witness.md).
The polygon's first distance agrees, but an arbitrary authored origin can flip
its normal and select the opposite support point. The triangle's analytic
anchor remains correct. This issue was separate from throughput and is resolved in the linked correctness pass.

## Final Measurements And Decision

The full matched FixedMathSharp group measures:

| Fixture | Baseline, us | Retained, us | Lower cost |
| --- | ---: | ---: | ---: |
| Ordinary projected triangle | 137.262 +/- 1.055 | 45.919 +/- 0.375 | 66.5% |
| Scalar-face projected triangle | 140.405 +/- 1.062 | 45.523 +/- 0.285 | 67.6% |
| Clipped 128 triangles | 6,215.792 +/- 53.728 | 1,544.392 +/- 6.851 | 75.2% |
| Circle-slab contact control | 12.488 +/- 0.163 | 12.498 +/- 0.087 | -0.1% |
| Capsule-slab contact control | 30.462 +/- 0.321 | 31.250 +/- 0.175 | -2.6% |
| Full 128 triangles | 13,881.095 +/- 70.227 | 3,043.764 +/- 19.130 | 78.1% |
| Full 512 triangles | 55,119.046 +/- 354.839 | 11,750.603 +/- 94.071 | 78.7% |
| Full 2,048 triangles | 220,133.393 +/- 1,526.365 | 46,143.082 +/- 357.236 | 79.0% |

The full grids remain linear at approximately 23 us per triangle, down from
108 us. These are the same triangles and measured public calls, not fewer
candidate queries. Contact rows are unchanged controls, not target improvement
claims; the capsule control is 0.788 us (2.6%) higher in this capture and does
not call the modified sweep owners. Captures and sibling logs use
`artifacts/projected-sweep-baseline-grid` and `artifacts/projected-sweep-direct-final`.
Each has sixteen successful child launches;
all final raw allocation/GC rows are exactly zero.

The final sampled profile is recorded separately under
`artifacts/projected-sweep-direct-final-profile` and its sibling summary.
Across full grids, point selection takes 43-50% and segment selection 31-36%
of sampled benchmark activity. Shared rounding takes 34-41%, including 21-26%
in ratio-bound refinement; exact finite witnesses take 8-9%. These inclusive
shares overlap. Remaining cost includes full-width coefficients, root bounds,
exact boundary comparisons and finite witnesses. This pass removes duplicate
root-search ownership rather than expanding the shared arithmetic surface.

The unchanged Gravitas consumers measure:

| Query / triangles | Baseline, us | Retained, us | Lower cost |
| --- | ---: | ---: | ---: |
| Dense hit / 128 | 12,929.719 +/- 101.210 | 3,133.844 +/- 34.948 | 75.8% |
| Proxy miss / 128 | 13,732.457 +/- 135.558 | 4,217.200 +/- 46.670 | 69.3% |
| Dense hit / 512 | 50,791.482 +/- 375.379 | 12,034.997 +/- 84.480 | 76.3% |
| Proxy miss / 512 | 54,046.922 +/- 387.582 | 16,655.199 +/- 167.359 | 69.2% |
| Dense hit / 2,048 | 202,518.830 +/- 1,562.210 | 47,183.308 +/- 481.271 | 76.7% |
| Proxy miss / 2,048 | 216,255.302 +/- 1,328.988 | 66,617.172 +/- 787.686 | 69.2% |

Both captures have twelve successful children with zero raw allocated bytes and
collections. Reports and logs are under `artifacts/projected-sweep-baseline-captured`
and `artifacts/projected-sweep-direct-final` in Gravitas. The consumer source, fixtures,
candidate count and query work are unchanged; only the math implementation changes.

Gravitas's first two capture attempts fail before any child launches because
BenchmarkDotNet discovers preserved `Gravitas.Benchmarks.csproj` experiment copies
under `artifacts`. The successful captures temporarily rename those files with a
`.capture` extension, then restore every original name and verify its content hash.
No preserved capture, runner or project is removed or modified. The failed
`projected-sweep-baseline` and `baseline-verified` logs remain supporting evidence;
they are not timing results.

Both repositories' solution configurations build both target frameworks without
warnings or errors. All 49 Chronicler tests pass in
each configuration, with no test failures or skips. Owning core/FluentAssertions
reports retain the existing generated-code filters without new exclusions:

| Repository / configuration | Core tests | Covered lines | Covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 5,747 | 59,800 / 59,800 | 17,018 / 17,018 | 4,542 / 4,542 |
| FixedMathSharp ReleaseLean | 5,726 | 59,893 / 59,893 | 17,018 / 17,018 | 4,538 / 4,538 |
| Gravitas Release | 6,026 | 61,041 / 61,041 | 20,144 / 20,144 | 6,042 / 6,042 |
| Gravitas ReleaseLean | 5,960 | 61,041 / 61,041 | 20,144 / 20,144 | 6,042 / 6,042 |

Reports, raw coverage and test/build logs use the `artifacts/projected-sweep-direct-`
prefix with the configuration name in each repository. The point and segment
owners have complexity 10 and 20 respectively, with 100% line/branch coverage;
the point owner no longer exceeds the exception threshold. Executable source
is unchanged throughout final captures and validation. Two comment-encoding
repairs follow the math capture; the final SHA-256 checkpoint is retained as
`artifacts/projected-sweep-direct-final-source.json`, with the comment-only
change recorded separately. An isolated resource probe passes in Release and Lean
on a requested 256-KiB worker stack with 64 KiB of live caller headroom. It
checks exact clipped anchors, a full-domain authored displacement and zero
warmed calling-thread allocations. The probe and logs are under
`artifacts/projected-sweep-stack` and `projected-sweep-direct-<configuration>-stack.log`.
The out-of-process broad smoke completes all 479 cases with 479 successful
children, populated statistics and retained raw GC rows. Thirteen rows allocate
as expected: one enumerable control, four string-formatting controls and eight
serialization controls. All other raw rows are zero; no zero-summary/raw-counter
discrepancy warnings occur. The smoke uses the Short job with 10-ms target
iterations, affinity 3 and preserved children, under
`artifacts/projected-sweep-direct-smoke`; it is a regression gate, not timing
evidence. Both repositories' DocFX builds pass with warnings treated as errors,
API assets/actions/local-link checks pass, and the four existing wiki-rewrite
checks pass. Final source hashes match and `git diff --check` passes.

Independent arithmetic, connected-call-path and validation reviews find no
remaining actionable issue in this change. Close this bounded throughput pass:
the remaining full-width coefficients, exact ratio bounds and finite witnesses
belong to existing shared owners, with no demonstrated smaller optimization to
retain here. Full batches remain linear and the 2,048-triangle row still costs
46 ms; this does not establish a universal 32-FPS simulation budget. Reopen on
a representative residual profile or a measured shared-arithmetic improvement.
Consumer upper-distance pruning remains separate. The polygon-normal defect
was subsequently resolved in
[FMS-Issue-030](2026-10-14-convex-sweep-normal-witness.md). This validation uses
the Windows local source stack; released-package validation remains a future release gate.

## Reproduction

From the FixedMathSharp repository root:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
$env:BuildInParallel = 'false'
$env:UseSharedCompilation = 'false'
$env:DOTNET_TieredCompilation = '0'
$env:COMPlus_gcConcurrent = '0'
(Get-Process -Id $PID).PriorityClass = 'BelowNormal'
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -m:1
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll projected-triangle-sweep --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/benchmarks/projected-triangle-sweep
```

For the sampled diagnostic, filter `*VerticalSubdivisionSweep*`, add `--profiler
EP`, and use one launch, two warmups and three measurements. The Gravitas control
is its unchanged `mixed-mesh-triangle-scaling` selection, filtered to the exact
`SweepCircleAgainst3DAll_DenseTriangleMeshTarget` and
`SweepCircleAgainst3DAll_FalsePositiveTriangleMeshTarget` method names, across all
three subdivision counts. It uses the same full timing arguments with unroll
factor one. Preserve expected launches and every raw GC row; a zero summary
alone does not establish zero allocation. No universal simulation-frame or
contact-count threshold is inferred from these synthetic workloads.
