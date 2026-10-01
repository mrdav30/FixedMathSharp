# Benchmark Signal Hardening Backlog

## Purpose

This document tracks unresolved benchmark-derived concerns that do not belong to
an active implementation plan. Use it for measured throughput, allocation,
scaling, profiler, and benchmark-harness signals.

Correctness defects belong in [`issue-tracker.md`](issue-tracker.md). Work that
requires API design, multiple subsystems, or staged implementation belongs in a
focused feature-work plan.

## Intake Rules

- Add a signal only when a benchmark, allocation guard, profiler trace, or
  repeated validation run produces concrete evidence.
- Record the affected benchmark, exact command, environment, measured value,
  expected behavior, and smallest useful isolation step.
- Use repository-relative commands and paths. Do not record developer-specific
  drive letters, home directories, or machine-local dependency locations.
- Keep diagnostic instrumentation in tests or benchmark support unless the
  runtime needs a durable public diagnostic contract.
- Prefer the smallest root-cause fix that improves a representative workload.
- Promote a signal to a focused plan when it crosses subsystem boundaries or
  needs phased review.
- Remove a signal after verification supports either a fix or an explicit
  no-change decision. Preserve durable design evidence in the completed plan or
  release notes rather than retaining a closed backlog entry.

## Baseline Commands

Run commands from the repository root. Build the benchmark runner before
capturing evidence:

```powershell
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
```

Use the benchmark's alias and an explicit artifact directory for the signal:

```powershell
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll <alias> --exporters json --artifacts artifacts/benchmarks/<signal>-baseline
```

## Active Signals

### Complete nonparallel cylinder contacts are expensive

- **Status / priority:** Partially improved, high; six retained throughput
  passes on 2026-09-30 / 2026-10-01 reduce penetrating-rim cost to about 4.5 ms
  and multi-radical cost to about 14.4 ms, with interlaunch variation.
  Originally measured on 2026-09-26 while closing
  FMS-Issue-024. The complete exact relation is correct, but its general positive
  contacts have not met a high-volume physics budget. This is distinct from
  the explicitly accepted zero-radius dispatch trade-off.
- **Source:** `RigidFiniteShapeRelationBenchmarks` on the working tree based on
  `ad9c88b`. The [completed design](done/2026-09-25-cylinder-pair-contact-design.md)
  retains exact geometry, arithmetic/resource proofs, prior profiles, matched
  controls and validation evidence.
- **Environment:** Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29, Release,
  BenchmarkDotNet 0.15.8, concurrent workstation GC, two-core affinity and
  below-normal launcher; two launches, three warmups, twelve measurements.
  Source mode used `UseLocalLsfStack=true`, `BuildInParallel=false`,
  `UseSharedCompilation=false` and `DOTNET_PROCESSOR_COUNT=2`.
- **Historical measurement (2026-09-26):** Penetrating rims **11.566 +/- 0.110ms**; multi-radical contact
  **25.137 +/- 0.164ms**, mean and 99.9% confidence half-width. Separated/tangent
  rim workloads cost **5.409 / 5.141ms**. By contrast, ordinary parallel contact
  costs **19.448 +/- 0.098us** and its strict predicate **0.856 +/- 0.008us**.
  All 22 matched child launches exited zero. Artifacts:
  `artifacts/fms024-matched-final`. The old limited-direction rim answers were
  wrong and cannot serve as equivalent-correctness throughput targets.
- **Allocation boundary:** Direct warmed calling-thread guards pass at zero.
  One matched penetrating-rim child reported 12,336 process-wide bytes across
  64 calls, although the second child and summary reported zero. The completed
  plan records the bounded counter investigation; do not silently round this
  discrepancy away or attribute it to the solver without evidence.
- **Retained change (2026-09-30):** The shared exact normal owner sizes squared
  coefficients from active gradient limbs, keeping the previous worst-case
  capacity. One additional limb covers convolution/axis-summation carry; the
  existing generated-gradient proof supplies the other capacity bound. This
  removes padding from repeated nearest-even threshold queries without changing
  coefficient values or increasing stack use. A zero first radial derivative
  now rejects a simple rim before constructing the other admission queries.
  No public API, root-selection policy, dependency or allocation was added.
- **Fresh matched evidence:** Baseline is `f6d9a2a`; geometry and commands are
  unchanged between captures. Same environment/job as above, with `--affinity 3`.
  Values below are microseconds, mean +/- 99.9% confidence half-width.

  | Frozen workload | Before | Retained change |
  | --- | ---: | ---: |
  | Penetrating rims | 11,393.033 +/- 287.947 | 7,970.354 +/- 110.001 |
  | Multi-radical contact | 26,363.528 +/- 368.507 | 25,415.384 +/- 129.939 |
  | Ordinary parallel contact | 19.611 +/- 0.227 | 19.066 +/- 0.079 |
  | Ordinary cylinder strict | 0.895 +/- 0.010 | 0.893 +/- 0.010 |
  | Ordinary cylinder/capsule | 20.268 +/- 0.360 | 19.402 +/- 0.345 |
  | Ordinary cylinder/capsule strict | 0.421 +/- 0.005 | 0.412 +/- 0.005 |
  | Oblique interior capsule rim | 558.685 +/- 12.912 | 564.926 +/- 7.455 |
  | Intersecting capsule core | 436.615 +/- 6.318 | 422.260 +/- 3.808 |

  Artifacts: `artifacts/cylinder-cost-baseline`, `cylinder-cost-profile`,
  `cylinder-cost-pruning`, `cylinder-cost-final` (combined experiment), and
  `cylinder-cost-retained` (final source). Both matched captures completed all
  sixteen child launches with zero exits and collections. Allocation summaries
  report zero; one retained strict cylinder/capsule child reported 336
  process-wide bytes over 1,048,576 operations. Preserve that counter record;
  it does not identify an allocation in the contact solver. Small changes in
  unchanged controls limit interpretation of the modest multi-radical delta.
- **Second retained change (2026-09-30):** Exact sign queries use a root-local
  dyadic change of variable inside the existing normalized Horner evaluator.
  A certified cell-wide power-of-two bound keeps the virtual variable in
  `[0,1]`; coefficient shifts are evaluated directly without a polynomial copy.
  The original resultant bound still proves zero/nonzero termination. A fallback
  keeps the previous worst-case stack/refinement bound whenever local scaling
  would increase it. Negative normalization exponents and completely discarded
  coefficients are handled explicitly. Point-refinement hints retain their
  original scale, and borrowed numerator/denominator metadata remain unscaled.
  No new solver, cache, dependency or public API was introduced.
- **Second matched evidence:** Fresh baseline is committed `43de9db`; the same
  eight frozen workloads and environment/job above are used before and after.
  Values are microseconds, mean +/- 99.9% confidence half-width.

  | Frozen workload | Before local scaling | With local scaling |
  | --- | ---: | ---: |
  | Penetrating rims | 7,808.041 +/- 47.540 | 7,066.599 +/- 138.217 |
  | Multi-radical contact | 26,219.694 +/- 299.818 | 21,470.704 +/- 224.632 |
  | Ordinary parallel contact | 19.764 +/- 0.333 | 18.628 +/- 0.086 |
  | Ordinary cylinder strict | 0.906 +/- 0.013 | 0.889 +/- 0.006 |
  | Ordinary cylinder/capsule | 20.373 +/- 0.130 | 19.210 +/- 0.200 |
  | Ordinary cylinder/capsule strict | 0.426 +/- 0.001 | 0.418 +/- 0.002 |
  | Oblique interior capsule rim | 553.774 +/- 14.819 | 552.398 +/- 7.443 |
  | Intersecting capsule core | 435.926 +/- 5.307 | 422.410 +/- 6.086 |

  Artifacts: `artifacts/cylinder-cost2-baseline`, `cylinder-cost2-profile` and
  `cylinder-cost2-scaled`. Both captures complete all sixteen child launches
  with zero exits, collections and process-wide bytes. The multi-radical mean
  improves by about 18%; penetrating rims by about 9.5%. Unchanged controls also
  move modestly, so these are matched workload deltas rather than a universal
  speedup claim. Tiny-root regression fixtures independently reproduce the
  previous excessive refinement and verify exact signs/cell containment.
- **Third retained change (2026-09-30):** The existing cubic-pencil builder
  narrows its construction workspace using the smaller of the caller's proven
  capacity and the documented generic `32*b+128`-bit bound. The actual input
  height includes optional direction invariants. Inputs and retained outputs
  keep their original strides; shared polynomial addition already supports
  independent source/output strides. This removes padded arithmetic and clears
  for small invariants without another representation or arithmetic owner.
  Caller stack allocations and worst-case capacity remain unchanged. Independent
  BigInteger fixtures cover six-word scratch with sixteen-word outputs, exact
  derivatives, a direction wider than the values, dirty padding and zero input.
- **Third matched evidence:** Fresh baseline is committed `f5d4f43`, with the
  same eight frozen workloads and environment/job above. Values are microseconds,
  mean +/- 99.9% confidence half-width.

  | Frozen workload | Before compact workspace | With compact workspace |
  | --- | ---: | ---: |
  | Penetrating rims | 6,894.589 +/- 87.459 | 5,251.505 +/- 53.626 |
  | Multi-radical contact | 22,497.088 +/- 374.413 | 22,187.533 +/- 237.640 |
  | Ordinary parallel contact | 19.922 +/- 0.218 | 20.270 +/- 0.343 |
  | Ordinary cylinder strict | 0.919 +/- 0.012 | 0.893 +/- 0.006 |
  | Ordinary cylinder/capsule | 19.702 +/- 0.200 | 19.428 +/- 0.173 |
  | Ordinary cylinder/capsule strict | 0.420 +/- 0.004 | 0.415 +/- 0.003 |
  | Oblique interior capsule rim | 565.759 +/- 8.205 | 546.317 +/- 4.842 |
  | Intersecting capsule core | 445.729 +/- 5.331 | 439.319 +/- 5.463 |

  Artifacts: `artifacts/cylinder-cost3-baseline` and `cylinder-cost3-compact`.
  Both captures complete sixteen child launches with zero exits, collections
  and process-wide bytes. Penetrating rims improve by about 24%; the
  multi-radical confidence intervals overlap, so no gain is established there.
  Unchanged controls drift modestly; these are matched workload measurements.
- **Fourth retained change (2026-09-30):** The shared shifted-magnitude owner
  limits same-sign addition to the shifted source's active support, then reuses
  its existing carry helper. Lower limbs are unchanged; higher limbs need work
  only while carry remains. The clipped support calculation uses wide integer
  arithmetic and preserves destination-edge truncation and existing signs.
  Zero/opposite-sign paths, disjoint storage, scratch bounds and allocations are
  unchanged. Independent integer tests cover padded inputs, both signs,
  cross-word shifts, carry beyond the source, input preservation and edge shifts.
- **Fourth matched evidence:** Baseline is the third retained source, with the
  same eight workloads and job. Values are microseconds, mean +/- 99.9%
  confidence half-width.

  | Frozen workload | Before bounded addition | With bounded addition |
  | --- | ---: | ---: |
  | Penetrating rims | 5,307.459 +/- 99.581 | 5,217.296 +/- 103.923 |
  | Multi-radical contact | 22,446.355 +/- 459.394 | 21,817.759 +/- 374.135 |
  | Ordinary parallel contact | 20.104 +/- 0.225 | 20.077 +/- 0.267 |
  | Ordinary cylinder strict | 0.905 +/- 0.012 | 0.908 +/- 0.012 |
  | Ordinary cylinder/capsule | 19.882 +/- 0.348 | 19.809 +/- 0.345 |
  | Ordinary cylinder/capsule strict | 0.424 +/- 0.004 | 0.421 +/- 0.006 |
  | Oblique interior capsule rim | 578.789 +/- 10.426 | 562.700 +/- 14.469 |
  | Intersecting capsule core | 446.533 +/- 7.999 | 440.965 +/- 7.010 |

  Artifacts: `artifacts/cylinder-cost4-baseline` and `cylinder-cost4-carry`.
  The baseline has split timing modes and more interlaunch variation, so the
  pooled 2.80% multi-radical gain was checked in reverse capture order. That
  confirmation measures `22,512.525 +/- 440.588us` before bounded addition
  versus `21,899.239 +/- 431.526us` with it, a 2.72% gain. All four prototype
  launch means are below all four baseline launch means. Penetrating rims are
  `5,385.256 +/- 80.650us` versus `5,271.773 +/- 101.187us` in the reversal;
  no strong additional penetrating-rim gain is established. Report a modest
  reproducible multi-radical workload improvement, not a universal speedup.
  Artifacts: `artifacts/cylinder-cost4-baseline-confirm` and `-carry-confirm`.
  The two broad captures complete sixteen children each and the two narrow
  confirmations four each, all with zero exits, collections and process bytes.
- **Fifth retained change (2026-10-01):** Simple rim admission already proves
  the defining root's slope sign. Pass its negation through the existing
  retained-sign query as the lower endpoint sign, avoiding two exact defining
  polynomial evaluations whenever refinement is needed. A nonrational cell
  isolating that simple root has opposite endpoint signs; its descendants
  preserve that crossing. Rational discovery still exits through the existing
  path, and repeated roots and generic callers retain exact endpoint evaluation.
  The optional internal certificate adds no scratch or cache and leaves query
  coefficients, metadata, ranking, refinement bounds and fallbacks unchanged.
  Integer-oracle tests compare hinted and ordinary signs/cells for both slope
  orientations, tiny roots, cancellation, undersized caller storage and dyadic
  discovery followed by another query.
- **Fifth matched evidence:** Fresh baseline is committed `e548376`. The same
  eight frozen workloads and job above were captured before and after; a narrow
  reverse-order baseline checks the modest gain. Values are microseconds, mean
  +/- 99.9% confidence half-width.

  | Frozen workload | Fresh baseline | Crossing reuse | Reverse baseline |
  | --- | ---: | ---: | ---: |
  | Penetrating rims | 5,195.724 +/- 85.085 | 5,166.120 +/- 73.863 | 5,321.807 +/- 95.625 |
  | Multi-radical contact | 22,202.157 +/- 376.602 | 21,327.191 +/- 293.130 | 21,985.681 +/- 253.988 |

  Artifacts: `artifacts/cylinder-cost5-baseline`, `-crossing` and
  `-baseline-crossing-confirm`. Multi-radical cost improves by about 3% against
  the reverse baseline; both prototype launch means are below both reversal
  baseline means, and their pooled confidence intervals are disjoint.
  Penetrating rims show no consistent additional improvement. Across the
  captures, unchanged controls drift modestly, including an oblique capsule
  rim increase; this is a modest measured workload gain, not a universal
  speedup. All 36 child launches exit zero, with zero raw process bytes and
  collections. The reverse baseline also agrees with an earlier two-case
  baseline (`cylinder-cost5-baseline-confirm`, 21,833.825 +/- 342.645us).
- **Sixth retained change (2026-10-01):** The existing shared magnitude
  multiplier clips inputs to the retained product width and factors out their
  whole zero low limbs before convolution. Limbs at or above the output width
  cannot affect its low product; factoring `a+b` zero limbs leaves that cleared
  output prefix zero and computes the same truncated product in the suffix.
  The subtraction-based width guard avoids overflow when adding offsets. Both
  prefix scans are bounded by output width, and the original convolution,
  sparse carry and disjoint-output contracts remain intact. No scratch,
  representation, dependency or helper signature is added. Shared integer
  oracles cover full/truncated products, dirty output, sparse interior carry,
  zero/empty inputs in either position, same-input squares and source preservation.
- **Sixth matched evidence:** Baseline is the fifth retained source. Same eight
  workloads, environment and job; values are microseconds, mean +/- 99.9%
  confidence half-width.

  | Frozen workload | Before factoring | With factoring |
  | --- | ---: | ---: |
  | Penetrating rims | 5,166.120 +/- 73.863 | 4,613.120 +/- 98.457 |
  | Multi-radical contact | 21,327.191 +/- 293.130 | 15,402.844 +/- 613.027 |
  | Ordinary parallel contact | 20.123 +/- 0.269 | 19.443 +/- 0.253 |
  | Ordinary cylinder strict | 0.910 +/- 0.014 | 0.902 +/- 0.010 |
  | Ordinary cylinder/capsule | 20.083 +/- 0.368 | 17.850 +/- 0.268 |
  | Ordinary cylinder/capsule strict | 0.420 +/- 0.007 | 0.415 +/- 0.005 |
  | Oblique interior capsule rim | 579.012 +/- 12.377 | 555.532 +/- 7.609 |
  | Intersecting capsule core | 443.227 +/- 8.604 | 430.512 +/- 6.093 |

  Artifacts: `artifacts/cylinder-cost5-crossing` and `-prefix`. The pooled
  multi-radical gain is about 28%, with launch means of 14.718 / 16.149ms and
  several detected timing modes; report that variation. Even the slower launch
  remains substantially faster than either baseline launch. Penetrating rims
  improve by about 11%. All sixteen prototype children exit zero with zero raw
  process bytes and collections.
  Four existing nonzero-low-limb controls check the shared helper's overhead:

  | Existing workload | Before factoring | With factoring |
  | --- | ---: | ---: |
  | `RadialProjectionWorstCaseComparison` | 3.802 +/- 0.053 | 3.792 +/- 0.064 |
  | `TrianglePairTinyAxisFallback` | 10.699 +/- 0.127 | 10.580 +/- 0.177 |
  | `ArbitraryRawCylinderIntersectionInterval`, Scale=1 | 2.944 +/- 0.038 | 2.983 +/- 0.038 |
  | Same interval, Scale=100000 | 7.373 +/- 0.096 | 7.375 +/- 0.096 |

  No meaningful overhead regression is established in these controls. Artifacts:
  `artifacts/cylinder-cost6-controls-before-box`, `-before-axis`, `-prefix-box`
  and `-prefix-axis`; reuse these fixtures instead of adding benchmark scaffolding.
  A reverse-order two-case baseline without factoring measures penetrating rims
  at 5,101.971 +/- 76.136us and multi-radical contact at 20,956.847 +/- 339.380us
  (`cylinder-cost5-crossing-prefix-confirm`). The final factoring-source
  confirmation measures 4,497.877 +/- 56.579us and 14,388.334 +/- 117.786us
  (`cylinder-cost5-prefix-confirm`). Both captures support substantial gains;
  the multi-radical reduction ranges about 26.5--31.3% against that reverse
  baseline. The original broad prototype has greater interlaunch variation,
  so retain both reports rather than treating the final mean as a guarantee.
  All eight confirmation children exit zero with zero collections. One final
  penetrating-rim child reports 336 process-wide bytes across 128 operations,
  while its paired child, both multi-radical children and JSON summaries report
  zero. Preserve that discrepancy; it does not establish a solver allocation.
  Direct warmed calling-thread guards remain the solver's allocation gate.
- **Rejected experiments:** Ranking values before full radial/cap admission
  skipped only potential positive nonwinners, preserving negative separation
  candidates. It added comparisons for inadmissible roots and did not resolve
  the multi-radical cost; that selection-policy change was removed. The retained
  patch keeps the original exact ranking order and borrowed-root contracts.
  A third-pass experiment reused local dyadic scaling for byte-refinement
  endpoint certificates while preserving the absolute error quantum and exact
  fallback. It passed 153 focused exact tests but produced no measurable gain:
  penetrating rims were `5,229.851 +/- 61.943us` and multi-radical contact
  `22,448.588 +/- 268.844us`, compared with the compact-workspace column above.
  All sixteen child launches passed with zero allocations/collections. That
  runtime/test experiment was removed; do not repeat it as a new optimization.
  Artifact: `artifacts/cylinder-cost3-refinement`.
  A fifth-pass square-free root-order shortcut obtained slope signs from parity
  instead of exact derivative queries. Its full capture measured multi-radical
  cost at 22,124.165 +/- 413.197us versus 22,202.157 +/- 376.602us: no established
  gain, so the additional API/tests were removed (`cylinder-cost5-slope`).
  A dual-polynomial square shortcut replaced the second equal cross-product
  with whole-span doubling. An initial apparent gain did not survive reversal:
  21,801.680 +/- 389.358us with it versus 21,833.825 +/- 342.645us without it;
  penetrating rims became slower. That runtime/test experiment was removed
  (`cylinder-cost5-dual`, `-dual-confirm` and `-baseline-confirm`). Do not
  repeat these as newly demonstrated optimizations.
  Skipping the normalized Horner evaluator's initial zero multiplication and
  zero-coefficient assembly passed 212 focused tests, but its 14,932.604 +/-
  642.645us multi-radical result overlaps the factoring capture's range.
  The extra branches were removed (`cylinder-cost5-horner`); no further gain
  is established from that experiment.
- **Validation:** Fresh source-backed solution builds pass in both repositories
  for `Release` and `ReleaseLean`, targeting `netstandard2.1` and `net8.0`, with
  zero warnings/errors. All tests pass with no skips or new coverage exclusions;
  both Chronicler suites also pass all 49 tests. Exact covered/total counts from
  ReportGenerator are below. The core suites include the warmed zero-allocation,
  1 MiB worker stack / live 64 KiB caller-buffer, full-domain, deterministic tie
  and nearest-even rounding gates. New tests independently check full-limb
  summation carry, padded normal coefficients, input preservation, zero-radial
  lazy admission, relative precision for tiny/zero-lower cells, low-bit
  cancellation, negative normalization heights and upper-endpoint capacity.
  Compact construction fixtures also check padded outputs, exact derivatives,
  direction-dominant input height and canonical zero with dirty storage.
  Shared arithmetic fixtures check carry beyond active support and retained
  destination-edge behavior against independent integer results.
  Crossing fixtures additionally verify exported nonzero endpoints beside an
  excluded zero and another dyadic root, while shared multiplication fixtures
  verify full/truncated zero-prefix products and same-input squares.
  Independent correctness/resource and Ponytail reviews found no actionable
  issues in the retained changes.

  | Repository / configuration | Tests passed | Lines | Branches | Methods |
  | --- | ---: | ---: | ---: | ---: |
  | FixedMathSharp Release | 4,080 | 52,673 / 52,673 | 12,118 / 12,118 | 3,934 / 3,934 |
  | FixedMathSharp ReleaseLean | 4,059 | 52,766 / 52,766 | 12,118 / 12,118 | 3,930 / 3,930 |
  | Gravitas Release | 4,363 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |
  | Gravitas ReleaseLean | 4,304 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |

  Every reported method is fully covered. Each repository's
  `artifacts/cylinder-cost6-Release*-tests.log`,
  `-coverage` and `-report/Summary.json` retain the raw validation evidence.
  The benchmark catalog lists successfully. The first-pass broad smoke passed
  all 459 cases (`artifacts/cylinder-cost-smoke`). The fresh second-pass native
  capture and final ordinary confirmation both pass all 459 cases, with complete
  results and zero child/launcher exit codes (`artifacts/cylinder-cost2-native/broad`
  and `cylinder-cost2-smoke-confirm`). The intervening failure is recorded below.
  The third-pass ordinary smoke also completes all 459 child launches with
  complete statistics, zero child/launcher exits and no native fault
  (`artifacts/cylinder-cost3-smoke`).
  The final fourth-pass source-backed smoke also passes all 459 child launches,
  with 459 complete statistics across 25 reports, zero child/launcher exits and
  no native fault (`artifacts/cylinder-cost4-smoke`).
  The final sixth-pass source-backed smoke likewise passes all 459 child
  launches, with 459 complete statistics across 25 reports, zero child/launcher
  exits and no native fault (`artifacts/cylinder-cost6-smoke`). The fresh catalog
  and both retained-source EventPipe profile children also pass.
  These `all -j Short --iterationTime 10 --affinity 3` runs verify execution
  only; performance claims use the separate matched captures above.
- **Native smoke follow-up:** The initial second-pass broad run failed two
  children with `0xC0000005` in the shared finite-shape setup, at
  `GetShiftedMagnitudeWord` during unchanged value/derivative construction.
  Tangency and zero-core capsule selections were affected; the latter failed
  while setup checked a separated-rim fixture. There were no workload results
  for those children, and the launcher correctly returned failure. Preserve
  `artifacts/cylinder-cost2-smoke`; it is not a successful smoke capture.
  A twelve-child ordinary rerun, forty-eight-child process-scoped CDB rerun,
  full 459-child CDB capture and final ordinary 459-child confirmation all pass,
  with no AV dump. Artifacts:
  `artifacts/cylinder-cost2-smoke-repro` and `cylinder-cost2-native`.
  Safe-span/resource review found no corruption path, and the completed design
  records an unexplained pre-optimization AV on 2026-09-26. These reruns do not
  establish a JIT/harness cause or explain the new failures. Keep this native
  follow-up open; capture the faulting instruction, registers and span backing
  storage on recurrence. No runtime source or JIT/GC settings were changed
  during this investigation.
- **Impact:** A few difficult oblique contacts can dominate a fixed frame.
  At 32 FPS, the entire simulation has 31.25 ms per step: one latest penetrating
  fixture consumes about 14% of that time and one multi-radical fixture about
  46%, before other collision queries, solver and simulation work. These are
  illustrative single-thread costs, not a universal acceptance threshold.
  No observed game workload or contact-count budget establishes acceptability.
  Exact determinism, minimum-depth selection and rounding remain mandatory.
- **Next isolation step:** The sixth-pass retained-source profile places about
  58% of multi-radical cost in cap-root selection, 37% in derivative construction,
  34% in side features, 15% in exact root refinement and 15% in sign queries.
  Inclusive costs overlap and must not be summed. The fourth-pass profile had
  approximately 62 / 25 / 28 / 37 / 31% in those respective owners; shared
  magnitude multiplication falls from about 53% to 40% of sampled time.
  These are sampled shares, not separately measured phase-speedup claims.
  Penetrating rims retain about 54% cap selection, 31% sign queries, 29%
  derivative construction and 20% side features. Focus the next pass on exact
  derivative construction and side-family polynomial work, preserving width,
  ranking, certification and live stack bounds. Avoid retaining another large
  polynomial cache across sign queries or reviving the rejected whole-span
  square shortcut without new evidence. Artifacts:
  `artifacts/cylinder-cost6-after-profile` and `-after-profile-summary.json`;
  the earlier comparison is `cylinder-cost4-after-profile`.
  Existing profiles already removed repeated
  GCDs/chains, redundant normals and worst-case mapping refinement; do not
  repeat those experiments or add another approximate fallback.

In coordinated source mode, reproduce the two expensive positive contacts:

```powershell
$env:UseLocalLsfStack = 'true'
$env:BuildInParallel = 'false'
$env:UseSharedCompilation = 'false'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll rigid-finite-shape-relation --filter '*PenetratingRimsCylinderCylinder*' '*MultiRadicalCylinderCylinder*' --warmupCount 3 --iterationCount 12 --launchCount 2 --affinity 3 --keepFiles --exporters json --artifacts artifacts/benchmarks/cylinder-pair-cost
```

Serialize runs with other build/test/profile work. Inspect each child exit and
GC record as well as the aggregate report. Recheck shared cylinder/capsule and
parallel controls for any retained optimization.

### General rotated manifold and oblique primary box/cylinder contacts need throughput work

- **Status / priority:** Isolated, high; measured on 2026-09-27 while repairing
  FMS-Issue-025. Exact contact generation is substantially more expensive than
  Boolean classification. A high-volume physics workload must budget these
  contacts explicitly; no observed game workload establishes acceptability.
- **Scope:** Both general rotated manifold (`CylinderManifold`) and oblique
  primary contact (`CylinderObliquePrimary`) are explicit optimization targets,
  not incidental controls. Keep them under this shared solver signal, but
  report each independently; improving one does not close the other.
- **Source:** `OrientedBoxAnchorBenchmarks`, working tree based on `5059cb2`.
  `FixedOrientedBoxCylinderContactRegressionTests` retains the original
  cap-clipped miss and wrong minimum-depth reproducers. Edge/rim and vertex/rim
  suites additionally cover exact touch, raw neighbors, rational and algebraic
  depths, sign reversal, and the principal-direction degeneracy.
- **Environment:** Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29, Release,
  BenchmarkDotNet 0.15.8, concurrent workstation GC, two-core affinity and
  below-normal launcher. Two launches, three warmups, twelve measurements;
  `UseLocalLsfStack=true`, `BuildInParallel=false`, `UseSharedCompilation=false`
  and `DOTNET_PROCESSOR_COUNT=2`. Runs were serialized with build/test work.
- **Measurement:** Mean +/- 99.9% confidence half-width per contact, in
  microseconds. The historical solver did not implement the complete relation;
  its costs are not equivalent-correctness performance targets. That does not
  establish that either positive fixture's historical result was wrong; the
  confirmed wrong-hit cases are marked separately below.

  | Frozen fixture | Historical solver | Complete solver, retained pruning |
  | --- | ---: | ---: |
  | General rotated manifold (`CylinderManifold`) | 539.790 +/- 9.907 | 1,969.743 +/- 32.357 |
  | Oblique primary contact (`CylinderObliquePrimary`) | 581.907 +/- 9.562 | 3,037.938 +/- 62.071 |
  | `CylinderParallelManifold` | 491.034 +/- 8.360 | 279.434 +/- 6.596 |
  | `CylinderZeroRadiusPrimary` | 291.761 +/- 5.101 | 103.398 +/- 1.727 |
  | `CylinderCapClippedSeparatedPrimary` | 577.393 +/- 10.087 (wrong hit) | 281.916 +/- 10.168 (miss) |
  | `CylinderCapClippedSeparatedManifold` | 576.751 +/- 10.168 (wrong hit) | 285.832 +/- 8.114 (miss) |
  | `CylinderStrict` control | 0.320 +/- 0.006 | 0.322 +/- 0.006 |

- **Impact:** General rotated manifold increased by approximately 1.43 ms
  (3.65x historical cost), and oblique primary contact by approximately 2.46 ms
  (5.22x). Correctness justifies replacing the incomplete solver, not dismissing
  either throughput concern. These fixtures use different cylinder rotations,
  so their timings do not isolate the overhead of producing a manifold.
- **Retained optimization:** The first complete implementation cost 4.002 /
  5.822 ms on the two general contacts. Profiling placed roughly three quarters
  of their time in edge/rim work, particularly exact root admission/refinement.
  A proven cone-wide support lower bound skips noncompetitive quartics before
  construction; it reduced those costs to approximately 1.97 / 3.04 ms without
  changing contact semantics. The bound reuses the existing exact candidate
  comparator, not a second solver, approximate test, or cache. Do not repeat
  this experiment as new work.
- **Evidence / allocation:** `artifacts/fms025-baseline`, `fms025-after`,
  `fms025-profile`, `fms025-pruning`, and `fms025-final-benchmarks` contain the
  successive captures. All 14 final child launches exited zero and reported
  zero allocated bytes and GC collections; the warmed calling-thread guard
  also passes. The final separated-primary distribution is bimodal, so use its
  broad trend rather than its last digits. Fixtures and the command below
  remain available without those disposable artifacts.
- **Next isolation step:** Profile both `CylinderManifold` and
  `CylinderObliquePrimary` before more changes, separating necessary
  parameter-root admission from squared-value
  construction/mapping and final rounding. In particular, assess whether exact
  nonwinning-value rejection can precede value mapping. Preserve full-domain
  geometry, minimum-depth ranking, deterministic ties, nearest-even rounding,
  zero allocations, and the verified 1 MiB stack with 64 KiB caller headroom.

After the baseline build above, reproduce all matched controls:

```powershell
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll oriented-box-anchor --filter '*Cylinder*' --launchCount 2 --warmupCount 3 --iterationCount 12 --exporters json --artifacts artifacts/benchmarks/box-cylinder-cost
```

### General triangle/capsule-slab rim contacts need throughput work

- **Status / priority:** Isolated, high; measured on 2026-09-28 while repairing
  FMS-Issue-027. Ordinary cap and side contacts became faster, but a positive
  oblique edge/rim contact remains substantially more expensive. No observed
  game workload establishes an acceptable contact-count/frame budget.
- **Source:** `TriangleCapsuleSlabContactBenchmarks`, working tree based on
  `04c2f21`. `FixedTriangleCapsuleSlabFeatureTests` retains the separated-rim,
  exact interior-root, touch/raw-neighbor and paired-witness regressions.
  This is a flat-capped planar stadium prism, not a rounded 3D capsule.
- **Environment:** Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29, Release,
  BenchmarkDotNet 0.15.8, concurrent workstation GC, two-core affinity and
  below-normal launcher. Two launches, five warmups, fifteen measurements;
  `UseLocalLsfStack=true`, `DOTNET_PROCESSOR_COUNT=2`, serialized with all other
  build/test/benchmark work.
- **Measurement:** Mean +/- 99.9% confidence half-width, microseconds per
  contact. Inputs and measured calls are unchanged. The historical solver
  demonstrably misclassified the gap; its incomplete feature set is not an
  equivalent-correctness baseline. This does not establish that every positive
  fixture's historical result was wrong.

  | Frozen fixture | Historical solver | Complete solver, retained pruning |
  | --- | ---: | ---: |
  | `CapFace` | 188.3 +/- 0.77 | 57.50 +/- 0.160 |
  | `StraightSideSeam` | 229.9 +/- 0.26 | 93.65 +/- 0.223 |
  | `RoundedEndOddCore` | 235.8 +/- 1.66 | 226.95 +/- 0.913 |
  | `ObliqueRimOverlap` | 235.0 +/- 0.69 | 1,560.62 +/- 10.562 |
  | `CertifiedRimGap` | 232.7 +/- 0.40 (wrong hit) | 60.23 +/- 0.212 (miss) |
  | `UnmaterializedScalarFace` | 230.8 +/- 0.59 | 97.34 +/- 0.761 |

- **Impact:** The oblique positive fixture costs approximately 1.33 ms more
  than the incomplete solver, or 6.64x historical cost. Correctness justifies
  the complete feature set, not ignoring this throughput concern.
- **Retained optimization:** Exact face certificates skip unnecessary rim
  traversal without changing the winner or tie order. The core-perpendicular
  certificate reuses the contained cylinder; the core-parallel certificate
  proves a triangle disk plus contained normal-axis segment supplies the
  attained lower bound. The latter reduced `RoundedEndOddCore` from
  1,449.92 +/- 14.593 us to 226.95 +/- 0.913 us. It deliberately does not
  certify the oblique rim fixture. Do not repeat these experiments as new work.
- **Evidence / allocation:** `artifacts/fms027-baseline`,
  `fms027-final-benchmarks` (before the disk certificate), and
  `fms027-retained-benchmarks` contain the matched captures. All twelve final
  child launches exited zero and reported zero allocated bytes and GC
  collections. Fixtures and commands remain available without those artifacts.
- **Shared-owner controls:** Gravitas's unchanged `mesh-cylinder-contact`
  group also completed all twelve cylinder/circle-slab rows with the same
  launch/warmup/iteration counts and two-core affinity. All twenty-four child
  launches passed their classification/depth preflights with zero allocations
  or collections. Compared with its `grv082-final-focused-bench` capture, no
  row became slower; oblique contacts measured 1,026.97 / 1,024.35 us against
  1,179.01 / 1,160.16 us previously. The fresh reports are under
  `artifacts/fms027-cylinder-controls` in Gravitas.
- **Next isolation step:** Profile `ObliqueRimOverlap`, separating necessary
  endpoint-region/root admission from squared-value construction, mapping,
  ranking and final witness rounding. Assess exact nonwinning-root rejection
  using the existing shared contact owners before considering larger changes.
  Preserve complete classification, minimum depth, deterministic ties,
  nearest-even combined-coordinate rounding, full-domain anchors, bounded
  stack scratch and zero allocations. Recheck zero-core cylinder contacts.

After the baseline build above, reproduce the frozen group:

```powershell
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll triangle-capsule-slab-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --affinity 3 --exporters json --artifacts artifacts/benchmarks/triangle-capsule-slab-cost
```

## Signal Template

```markdown
### Signal title

- **Status:** Needs reproduction | Isolated | Planned
- **Priority:** Critical | High | Medium | Low
- **Source:** Benchmark, allocation guard, profiler trace, or validation command
- **Environment:** OS, CPU, SDK/runtime, configuration, and benchmark job
- **Measurement:** Baseline value, variance, allocation, and artifact path
- **Impact:** Why the signal matters to a supported workload
- **Next isolation step:** The smallest experiment that can confirm the cause
```
