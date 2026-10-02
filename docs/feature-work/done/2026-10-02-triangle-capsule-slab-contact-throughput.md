# Triangle/Capsule-Slab Rim Contact Throughput

This records the bounded refinement of the general triangle/capsule-slab rim
contact [benchmark signal](../benchmark-signal-hardening-backlog.md). This shape
is a flat-capped planar stadium prism, not a rounded three-dimensional capsule.
The exact complete support fan remains the classification and minimum-depth
authority. No observed simulation workload establishes a universal frame budget.

## Earlier Complete-Solver Evidence

The signal was measured on 2026-09-28 while repairing FMS-Issue-027, from working
tree revision `04c2f21`. The historical solver demonstrably misclassified the
separated rim fixture. Its incomplete feature set is not an equivalent-correctness
performance target; this does not establish that every positive historical
fixture was wrong.

The original captures used Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29,
Release, BenchmarkDotNet 0.15.8, concurrent workstation GC, two-core affinity and
a below-normal launcher. Two launches, five warmups and fifteen measurements,
with `UseLocalLsfStack=true` and `DOTNET_PROCESSOR_COUNT=2`; workloads were
serialized. Means +/- 99.9% confidence half-width, microseconds per contact:

| Frozen fixture | Historical solver | Complete solver, retained face certificates |
| --- | ---: | ---: |
| `CapFace` | 188.3 +/- 0.77 | 57.50 +/- 0.160 |
| `StraightSideSeam` | 229.9 +/- 0.26 | 93.65 +/- 0.223 |
| `RoundedEndOddCore` | 235.8 +/- 1.66 | 226.95 +/- 0.913 |
| `ObliqueRimOverlap` | 235.0 +/- 0.69 | 1,560.62 +/- 10.562 |
| `CertifiedRimGap` | 232.7 +/- 0.40 (wrong hit) | 60.23 +/- 0.212 (miss) |
| `UnmaterializedScalarFace` | 230.8 +/- 0.59 | 97.34 +/- 0.761 |

Earlier exact face certificates skip unnecessary rim traversal. A
core-perpendicular face reuses the contained cylinder certificate; a
core-parallel face proves a triangle disk plus contained normal-axis segment
supplies the attained lower bound. The latter reduced `RoundedEndOddCore` from
1,449.92 +/- 14.593 us to 226.95 +/- 0.913 us. It does not certify the oblique
rim fixture and is not a new optimization in this refinement.

Original captures are under `artifacts/fms027-baseline`,
`artifacts/fms027-final-benchmarks` (before the disk certificate) and
`artifacts/fms027-retained-benchmarks`. All twelve retained children exited zero
with zero raw allocation and GC counters. The unchanged Gravitas
`mesh-cylinder-contact` group also passed all twelve cylinder/circle-slab rows
and twenty-four child launches with zero counters. Its oblique contacts measured
1,026.97 / 1,024.35 us against the previous 1,179.01 / 1,160.16 us capture;
those controls are under `artifacts/fms027-cylinder-controls` in Gravitas.

## Retained Changes

`TriangleCylinderEdgeContacts.TryChart` rejects wholly inadmissible charts before
constructing their stationary quartic. Cap, triangle-cone and endpoint-hemisphere
projections are affine on `t in (0,1]`, and admission requires strict positivity.
If both endpoint values are nonpositive, their convex combination is nonpositive
throughout the chart. Existing cap/cone prechecks now include a zero first
endpoint; the core projection gains the same precheck. Mixed-sign charts still
run. Roots on analytic face, side and seam boundaries retain their earlier
canonical ownership. Zero-core cylinders bypass the core constraint.

The existing chart basis has exactly one nonzero axial component; sign changes
and chart swaps preserve that property. Its cap projection is a nonzero constant
or a nonzero multiple of `t`, so it cannot change sign on `(0,1]`. After the
strict chart precheck, the per-root cap sign query is redundant. Removing it
also removes its two stored constraint coefficients, reducing each chart and
winner-reconstruction parameter buffer from thirty-two to thirty slots.

`TriangleCylinderAnalyticFeatures.TryGetBest` transforms the retained local
direction to world space once after selection. Comparisons and face certificates
read only gap fields, and both core endpoints share the same world basis. This
removes repeated transformation of temporary winners without changing ordering.

Positive admitted edge roots reuse the selected analytic endpoint's nearest-even
depth as a certified upper bound, following the existing box/cylinder pattern.
An unclamped rounded raw depth `d` implies an exact analytic gap at most
`d + 1/2` raw. The shared `CompareSquaredGapToTwiceRaw` evaluates
`4N - (2d+1)^2 D` at the parameter root. A nonnegative sign proves that edge
cannot beat the earlier analytic feature, so value construction and mapping are
unnecessary. Here `N/D` already represents raw gap squared, including `RawScale`
in the denominator; `ValueShift` belongs to the separate mapped value polynomial.
The denominator is positive after the existing nonzero-K admission. Negative
and zero gaps retain their handling before this filter. Clamped depths bypass
the bound; the unsigned threshold supports an unclamped `long.MaxValue` depth.
Surviving roots retain exact ranking, and the analytic fallback reuses its depth.

No new comparator, solver, cache, buffer owner, public API or coverage exclusion
was introduced. Comments retain the domain, boundary and rounding invariants.

## Fresh Measurements

Baseline revision: `5fb4dc4`; retained source: this refinement. Fixtures and
measured calls are unchanged. The environment and launch/warmup/measurement
counts match the earlier capture above, with `BuildInParallel=false` and
`UseSharedCompilation=false` as well. Builds, tests and timing captures are
serialized.

Independent narrow experiments measured:

| Retained stage | `ObliqueRimOverlap`, us |
| --- | ---: |
| Fresh complete baseline | 1,674.863 +/- 22.574 |
| Core hemisphere chart rejection | 1,325.790 +/- 20.726 |
| Include zero cap/cone chart endpoints | 1,080.972 +/- 18.441 |
| Transform the final analytic direction once | 1,042.127 +/- 16.871 |
| Reuse the analytic half-raw upper bound | 841.680 +/- 13.292 |

The isolated final result is approximately 50% faster. Use the complete matched
group below for the final retained gain; the isolated stages were timed
separately and do not establish additive gains across other workloads.

The final matched six-fixture group measured:

| Frozen fixture | Fresh baseline, us | Retained, us |
| --- | ---: | ---: |
| `CapFace` | 48.284 +/- 0.688 | 44.998 +/- 0.270 |
| `StraightSideSeam` | 78.320 +/- 1.477 | 67.924 +/- 0.498 |
| `RoundedEndOddCore` | 176.371 +/- 2.457 | 159.419 +/- 1.328 |
| `ObliqueRimOverlap` | 1,674.863 +/- 22.574 | 804.258 +/- 12.327 |
| `CertifiedRimGap` | 51.610 +/- 0.672 | 47.499 +/- 0.232 |
| `UnmaterializedScalarFace` | 77.889 +/- 0.897 | 73.900 +/- 0.337 |

The target improves **52.0%**, or 2.08x throughput. The other rows are controls;
their smaller differences are not the target claim. Captures are under
`artifacts/triangle-slab-cost1-baseline` and `artifacts/triangle-slab-cost1-final-cap`,
with sibling logs. Narrow stage captures use the same prefix with `hemisphere`,
`strictcharts`, `deferred-world` and `upper` suffixes.

An earlier full group, before the cap-query/slot cleanup, measured
871.158 +/- 13.242 us (48.0%) under `artifacts/triangle-slab-cost1-final`.
Both full groups were captured separately; do not attribute their entire
difference to the final deletion, because control means also shifted.

All twelve baseline and twelve final FixedMathSharp children pass their
preflights and exit zero, with zero raw allocated bytes and GC collections.

The unchanged Gravitas `mesh-cylinder-contact` controls measured:

| Geometry / method | Fresh baseline, us | Retained, us |
| --- | ---: | ---: |
| Cap face / cylinder | 52.036 +/- 0.677 | 46.640 +/- 0.232 |
| Cap face / circle-slab | 20.522 +/- 0.303 | 18.119 +/- 0.075 |
| Cap intrusion / cylinder | 997.528 +/- 14.803 | 532.950 +/- 7.978 |
| Cap intrusion / circle-slab | 989.378 +/- 13.771 | 530.292 +/- 3.341 |
| Oblique rim / cylinder | 1,115.387 +/- 15.051 | 665.629 +/- 7.030 |
| Oblique rim / circle-slab | 1,126.852 +/- 18.206 | 658.169 +/- 2.638 |
| Rim gap / cylinder | 38.767 +/- 0.499 | 34.541 +/- 0.097 |
| Rim gap / circle-slab | 38.626 +/- 0.542 | 34.848 +/- 0.168 |
| Rim touch / cylinder | 805.872 +/- 14.866 | 592.756 +/- 11.352 |
| Rim touch / circle-slab | 793.632 +/- 10.695 | 582.745 +/- 2.046 |
| Side face / cylinder | 26.401 +/- 0.348 | 23.301 +/- 0.054 |
| Side face / circle-slab | 27.815 +/- 0.356 | 23.143 +/- 0.084 |

The oblique rows improve 40.3% / 41.6%; the rim-touch rows improve 26.4% / 26.6%.
No control mean increases. Both captures contain twenty-four successful child
launches, all with zero raw allocation and GC counters. They are under
`artifacts/triangle-slab-cost1-baseline` and `artifacts/triangle-slab-cost1-final-cap`
in Gravitas. This benefits the zero-core consumers of the shared triangle solver
without a separate downstream implementation.

The preceding full downstream capture is preserved under
`artifacts/triangle-slab-cost1-final`. Its oblique rows were 718.613 +/- 12.652 /
705.216 +/- 12.794 us. As in FixedMathSharp, the final capture also shifts the
ordinary controls; their difference is not an isolated measurement of the final
cap-query deletion.

The initial full coverage audit identified the redundant cap query above and
two genuinely reachable multiple-value paths.
The latter remain in the runtime. A tiny rotated regression has two admitted
stationary values below the analytic half-raw bound, exercising value-polynomial
reuse and comparison against an incumbent edge root. A contained half-raw ball
and an exact 4/5-raw support gap independently prove its true depth lies in
`(1/2,4/5]` raw and rounds to one raw unit. Independent normal bounds distinguish
the true minimum from a later maximum whose rounded depth is also one raw unit.
Its long edges preserve that support chart while respecting the public triangle
area threshold. It is not a solver-derived golden value.

## Validation

Fresh solution builds cover both target frameworks. All tests pass with
`UseLocalLsfStack=true`, using the existing generated-code filters and no new
coverage exclusions. All 49 Chronicler tests pass in each configuration:

| Repository / configuration | Core tests | Covered lines | Covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 4,123 | 52,671 / 52,671 | 12,136 / 12,136 | 3,935 / 3,935 |
| FixedMathSharp ReleaseLean | 4,102 | 52,764 / 52,764 | 12,136 / 12,136 | 3,931 / 3,931 |
| Gravitas Release | 4,363 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |
| Gravitas ReleaseLean | 4,304 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |

FixedMathSharp's final reports use `artifacts/triangle-slab-cost2-Release-report`
and its ReleaseLean counterpart. Gravitas Release uses that same prefix in its
repository; its final-source Lean report is
`artifacts/triangle-slab-cost1-ReleaseLean-report`. Lean was built after the cap
cleanup, and the final runtime-source checkpoint remains identical. Release was
revalidated after cleanup. The initial FixedMathSharp coverage misses remain in
the earlier `cost1` reports; the final `cost2` reports replace them.

Relative to the committed baseline, the Release report adds five reachable
runtime lines and four branches, with no additional methods. Existing full-width,
odd-core/nearest-even, exact touch/raw-neighbor, unmaterialized-anchor, warmed
zero-allocation and constrained-stack tests all pass. Independent arithmetic,
ownership, test and minimality reviews found no actionable remaining findings.

The fresh out-of-process Short smoke completes all 459 children with zero exit
codes and populated statistics in 25 reports. All 459 raw GC rows are retained;
the thirteen allocating enumerable/formatting/serialization controls report
their expected nonzero summaries, and no zero-summary/raw-counter warning occurs.
The short iteration-time advisories are diagnostic only; this smoke is not timing
evidence. The build, catalog and smoke logs use the
`artifacts/triangle-slab-cost2` prefix; smoke exports are under
`artifacts/triangle-slab-cost2-smoke`.

## Remaining Cost And Closure

The final EventPipe diagnostic samples 6,876 ms of benchmark activity. Inclusive
shares are approximately 62.6% in edge contact selection, 60.2% in chart traversal,
34.4% in parameter construction, 25.2% in stationary-polynomial construction,
18.0% in exact root-sign queries and 15.8% in root refinement. Analytic feature
selection accounts for 27.3%. These owners overlap; their shares must not be
added. The capture and summary are under
`artifacts/triangle-slab-cost2-post-validation-profile` and its sibling summary.

The remaining work predominantly constructs and proves the exact support fan.
The retained changes remove unnecessary charts, repeated transformations and
nonwinning value construction through existing owners. A broader algebraic
redesign needs a concrete shared proof reduction and its own correctness and
performance evidence. This bounded refinement is complete;
reopen for such a reduction, a representative simulation workload with excessive
contact cost, or a reproducible regression. This establishes no universal contact
count or frame-time acceptance threshold.

## Reproduction

From this repository root, set `UseLocalLsfStack=true` in the environment too,
so BenchmarkDotNet child builds inherit the coordinated source stack:

```powershell
$env:UseLocalLsfStack = 'true'
$env:BuildInParallel = 'false'
$env:UseSharedCompilation = 'false'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll triangle-capsule-slab-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --affinity 3 --keepFiles --exporters json --artifacts artifacts/benchmarks/triangle-slab-final
```

For the sampled diagnostic, restrict the same group to `*ObliqueRimOverlap`, add
`--profiler EP`, and use one launch, two warmups and three measurements. The
unchanged Gravitas `mesh-cylinder-contact` group is the zero-core cylinder and
circle-slab control; build its Release runner with the same local-stack settings
and use the same full timing arguments. Keep CPU affinity, process priority and
serialization consistent when comparing captures. Profile percentages are
inclusive shares of sampled benchmark activity, so overlapping owners must not
be added together.
