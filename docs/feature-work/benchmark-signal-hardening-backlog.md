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

No active signals remain.

## Archived Signals

### Centered forward-scaled planar transforms repeat zero-offset wide arithmetic

- **Status:** Committed as `5447b34` on 2026-10-04; coordinated with Gravitas
  GRV-Benchmark-023. Local source-stack validation is complete; released-package
  validation remains a future release gate.
- **Change:** The existing `WideVector2dTransform.TryTransformScaledPoint` owner
  returns the origin when both the local point and unscaled local displacement
  are exactly zero. Both rotated numerators are then zero regardless of scale,
  and the final ratio is the origin's exact raw coordinate. This retains the
  forward-transform contract for zero, mirrored and extreme scales, all yaw
  values, and scalar-boundary origins. Nonzero inputs retain the fused wide
  arithmetic, one final half-even rounding per coordinate and atomic failure.
  Caller-owned transform/scale admission and physics snapshot publication are
  unchanged; no transform cache, public API or friendship was added.
- **Matched controls:** `TryTransformScaledPointCentered` changes from
  1196.941993 +/- 12.673961 ns to 16.279771 +/- 0.228879 ns, a 98.64% reduction.
  The nonzero `TryTransformScaledPointOffset` control changes from
  1536.411243 +/- 15.455714 ns to 1500.423388 +/- 18.672809 ns, with no observed
  regression. Error is half the 99.9% confidence interval. Both rows use the
  existing deterministic origin/angle arrays and unit scale, with 256 operations
  per invocation. All eight child launches report exactly zero raw allocated
  bytes and GC collections, as well as 0 B/op summaries.
- **Reproduce:** From this repository, use the following environment/build and
  compiled-DLL command. Repeat the measurement with the same arguments and
  `refinement4-transform-after` artifact path after applying the owner change.
  The source baseline is `6789c09`; captures use Windows 11, i7-9700K,
  SDK 10.0.302, runtime 8.0.29 and BenchmarkDotNet 0.15.8.

  ```powershell
  $env:UseLocalLsfStack = 'true'
  $env:DOTNET_PROCESSOR_COUNT = '2'
  (Get-Process -Id $PID).PriorityClass = 'BelowNormal'
  dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true
  dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll vector2d --filter '*TryTransformScaledPointCentered*' '*TryTransformScaledPointOffset*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 500 --affinity 3 --exporters json --artifacts ../Gravitas/artifacts/grv-benchmark-023/refinement4-transform-before
  ```

  Before/after logs and exported statistics are retained in the sibling
  Gravitas repository under `artifacts/grv-benchmark-023/refinement4-transform-{before,after}`.
- **Integrated limit:** The initial math-only Gravitas consumer comparison
  measures the complete 1024-pair circle step at 36.543975 +/- 0.468481 ms
  before and 34.223361 +/- 0.549934 ms after. The automatic-grounding control,
  which does not perform this centered transform work, instead changes from
  9.478350 +/- 0.121474 ms to 9.963933 +/- 0.202656 ms. Both report 0 B/op.
  A second matched pair measures 39.031101 +/- 0.605361 ms before and
  33.779813 +/- 0.583872 ms after, with the grounding control at
  9.926890 +/- 0.167635 / 9.767688 +/- 0.210000 ms (overlapping intervals).
  All summaries report 0 B/op. The initial after grounding capture has one
  raw child record with 24,624 allocated bytes over 53 operations, despite the
  final-launch zero summary. Both repeat captures have exact zero raw allocated
  bytes and collections in every child; the initial exception is retained.
  Both full-step pairs improve, but baseline variation
  makes a single precise percentage inappropriate. Preserve both captures;
  no simulation-budget threshold has been accepted. Repetition uses the same
  protocol and `refinement4-math-repeat-{before,after}` artifact directories.
- **Validation:** All 16 focused `ScaledCompositeTransformTests` pass in Release
  before the change and in Release/ReleaseLean afterward with
  `-p:UseLocalLsfStack=true`. New characterization covers both public overloads,
  full-domain origins/scales/yaw, and nonzero-displacement rotation and atomic
  overflow controls. Both benchmark captures exit successfully. Full local-stack
  solution builds and all 4140/4119 core tests plus 49 Chronicler tests per
  configuration pass without skips. The owning core/FluentAssertions raw
  captures cover 52678/52678 sequence points, 12142/12142 branches and
  3935/3935 methods in Release, and 52771/52771, 12142/12142 and 3931/3931 in
  ReleaseLean. Captures use `refinement4-FixedMathSharp-<configuration>-*` in the
  sibling Gravitas artifact directory. Rendered owning core/FluentAssertions
  reports confirm 100% lines, branches and fully covered methods with the same
  counts; independently filtered Chronicler reports cover 85/85 lines,
  12/12 branches and 18/18 methods in each configuration. The raw partial math
  dependency capture from the Chronicler suite is not merged over the owning
  core report. DocFX passes with warnings as errors, and API resources,
  repository actions and local links are verified in
  `refinement4-FixedMathSharp-docfx.log`. No coverage or allocation gate changed.
- **Compatibility:** Release FixedMathSharp first, then validate Gravitas
  against that released package. The current evidence uses sibling source.

### Unit-interval radial distance endpoint evaluation

- **Status:** Complete for review, 2026-10-02; coordinated with Gravitas
  GRV-Benchmark-023's automatic grounding experiment.
- **Change:** The existing bounded distance solver uses widened sums for the
  signs of `C`, `A + 2B + C`, `B`, and `A + B` on `[0, 1]`. These have the same
  signs as its general rational endpoint polynomial/derivative evaluations.
  Other intervals, discriminants, roots, rounding and caller-supplied distance
  mapping are unchanged. Circle, sphere, capsule-cap and rounded-cylinder-cap
  consumers share the existing owner; there is no new public API or friendship.
- **Matched controls:** At scales 1/100000, circle intervals change from
  3828.9 +/- 18.00 / 5455.9 +/- 27.68 ns to 3304.82 +/- 17.454 /
  4921.34 +/- 24.061 ns. Sphere intervals change from 3866.7 +/- 30.14 /
  5514.1 +/- 69.26 ns to 3302.94 +/- 26.690 / 4980.02 +/- 53.401 ns.
  Moving-away circle misses change from 539.1 +/- 3.72 / 554.3 +/- 17.01 ns
  to 87.88 +/- 1.661 / 89.89 +/- 0.490 ns; sphere misses change from
  576.6 +/- 7.16 / 569.5 +/- 5.26 ns to 120.68 +/- 1.388 / 122.01 +/- 0.208 ns.
  All eight rows report 0 B/op. Error is half the 99.9% confidence interval.
- **Integrated limit:** Gravitas's corrected-pose 1024-pair automatic probes
  improve 7.0% with this math change alone. Its math-only full-step confidence
  intervals overlap; do not infer an isolated full-step speedup.
- **Reproduce:** From this repository, set `UseLocalLsfStack=true` and
  `DOTNET_PROCESSOR_COUNT=2` in the environment, build the Release benchmark
  project with `-p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false`, then run
  `finite-axis-intersection --filter '*Circle2DDistanceInterval*' '*Sphere3DDistanceInterval*' '*MovingAwayDistanceInterval*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json`
  through the compiled DLL. Captures use Windows 11, i7-9700K, SDK 10.0.302,
  runtime 8.0.29 and BenchmarkDotNet 0.15.8. The source baseline is `00a38bd`;
  results are retained in the sibling Gravitas repository under
  `artifacts/grv-benchmark-023/refinement-math-before-verified` and
  `refinement-math-after`. Keep the raw moving-away multimodality warnings.
- **Compatibility:** Release FixedMathSharp first, then validate Gravitas
  against that released package. The current evidence uses sibling source.
- **Validation:** Local-stack Release/ReleaseLean solution builds cover both
  target frameworks with no warnings/errors. All 4138/4117 core tests and 49
  Chronicler tests per configuration pass with no failures/skips. Exact covered
  sequence points/branches/methods are 52675/12138/3935 in Release and
  52768/12138/3931 in Lean, each equal to its total. Gravitas also passes both
  suites with 100% reachable coverage. No exclusion or allocation gate changes
  were made; independent review finds no actionable issue. Logs, TRX and raw
  coverage are retained under the sibling Gravitas repository's
  `artifacts/grv-benchmark-023/refinement-<repository>-<configuration>-*`.
  Rendered reports use core/FluentAssertions from the core suite and Chronicler
  from its owning suite (85/85 lines, 12/12 branches, 18/18 fully covered methods).
  A merged partial dependency capture lost three covered lines in Lean's rendered
  aggregation; the complete raw and core-only rendered reports remain 100%.
  Raw captures and the failed merged-format control are preserved.
  All 463 Short smoke cases complete with successful child exits and populated
  statistics (one launch, three warmups, three 10-ms iterations); all 62
  finite-axis controls report zero allocation. Short timings are smoke evidence
  only. Both repositories' DocFX, API resources and local-link checks pass.

### General triangle/capsule-slab rim contacts need throughput work

- **Status:** Complete, 2026-10-02; bounded exact throughput refinements retained.
- **Result:** A fresh matched complete-solver baseline measures 1,674.863 +/-
  22.574 us before and 804.258 +/- 12.327 us afterward: 52.0% lower cost, or
  2.08x throughput. Affine admission rejects unnecessary charts, analytic
  directions transform once, and positive roots reuse the existing half-raw
  upper-bound query before value construction. A redundant cap query and its
  two parameter slots are removed. Exact classification, ranking and ties remain.
- **Shared controls:** Gravitas's oblique cylinder/circle-slab contacts improve
  40.3% / 41.6%, to 665.629 / 658.169 us. All thirty-six final target/control
  child launches report zero raw allocations and GC collections.
- **Validation:** Both repositories pass Release/ReleaseLean tests with 100%
  reachable line, branch and fully covered method coverage, both target
  frameworks build, and all 49 Chronicler tests pass per configuration. All work
  uses `UseLocalLsfStack=true`. The broad smoke completes 459 successful children
  and populated statistics, with no zero-summary/raw-counter warnings.
- **Limits / decision:** Exact parameter and root proofs remain the main cost;
  edge selection accounts for about 63% of sampled activity. The target is below
  one millisecond on the measured machine. This closure establishes no universal
  contact-count or frame-time acceptance threshold.
- **Reopen with:** A concrete shared root-proof reduction, representative
  simulation evidence of excessive contact cost, or a reproducible regression.
  The [completed record](done/2026-10-02-triangle-capsule-slab-contact-throughput.md)
  preserves historical and fresh captures, confidence intervals, commands,
  independent arithmetic oracles, controls and remaining sampled costs.

### General rotated manifold and oblique primary box/cylinder contacts need throughput work

- **Status:** Complete, 2026-10-01; bounded exact throughput refinements retained,
  with an explicit no-change decision for the remaining root-proof cost.
- **Result:** After the first refinement reached 1.118 / 1.643 ms, a fresh matched
  second pass measured 1.194 / 1.759 ms before and 1.137 / 1.394 ms afterward
  (4.8% / 20.7%). Private common-factor ranking and the shared three-term radical
  reduction preserve exact contacts and ties while reducing arithmetic and scratch.
- **Validation:** Both repositories pass all Release/ReleaseLean tests with 100%
  reachable line, branch and fully covered method coverage. Both target frameworks
  build, all 49 Chronicler tests pass per configuration, and all fourteen matched
  target/control children report zero raw allocation and GC counters. All work
  uses `UseLocalLsfStack=true`. The broad smoke completes 459 successful children
  and populated statistics; two non-contact raw-counter warnings and their
  zero-counter longer repeat are preserved in the refinement record.
- **Limits / decision:** Both targets still exceed one millisecond. Edge work
  accounts for about 60% of sampled time; retained-root and zero-term shortcuts
  produced no further gain and were discarded. This closure establishes no
  universal contact-count or frame-time acceptance threshold.
- **Reopen with:** A concrete shared root-proof reduction, representative
  simulation evidence of excessive contact cost, or a reproducible throughput
  regression. The [refinement record](done/2026-10-01-box-cylinder-contact-throughput.md)
  retains matched confidence intervals, commands, controls, arithmetic oracles,
  rejected experiments and remaining profiler costs.

### Finite-shape benchmark native failures and allocation-counter discrepancies

- **Status:** Complete, 2026-10-01 — reporting hardened; bounded investigation
  closed with an explicit no-change decision for runtime math.
- **Result:** The launcher exposes positive raw child counters hidden by a zero
  summary. A matched native interval attributes a fresh 336-byte increase to
  finalizer-thread array-pool cleanup; calling-thread contact guards stay zero
  and the real-allocation control remains exact.
- **Validation:** Release/ReleaseLean benchmark builds pass. Fresh CDB captures
  complete 487 children, including the full 459-case smoke and four longer
  penetrating-contact memory passes of 512 calls each. No AV recurred; those
  four memory passes report zero raw bytes. Generated binaries/PDBs are archived
  and hashed. All work uses `UseLocalLsfStack=true`.
- **Limits / decision:** The original AV and historical 12,336-byte count remain
  unattributed. The original faulting binaries were not retained; repeated
  captures and source inspection establish no actionable runtime defect. This
  closure does not claim a native fix or assign an unsupported JIT/GC cause.
- **Reopen with:** A fresh native fault dump plus matching binaries/PDBs, or a
  reproducible allocator/calling-thread counter failure attributable to the
  library. The [diagnostic record](done/2026-09-25-cylinder-pair-contact-design.md#benchmark-diagnostic-hardening-2026-10-01)
  preserves evidence, limitations and the recurrence capture procedure.

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
