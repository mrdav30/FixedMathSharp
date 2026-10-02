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

### General triangle/capsule-slab rim contacts need throughput work

- **Status / priority:** Isolated, high; measured on 2026-09-28 while repairing
  FMS-Issue-027. Ordinary cap and side contacts became faster, but a positive
  oblique edge/rim contact remains substantially more expensive. No observed
  game workload establishes an acceptable contact-count/frame budget.
- **Source:** `TriangleCapsuleSlabContactBenchmarks`, working tree based on
  `04c2f21`. `FixedTriangleCapsuleSlabFeatureTests` retains the separated-rim,
  exact interior-root, touch/raw-neighbor and paired-witness regressions. This
  is a flat-capped planar stadium prism, not a rounded 3D capsule.
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

  | Frozen fixture             |          Historical solver | Complete solver, retained pruning |
  | -------------------------- | -------------------------: | --------------------------------: |
  | `CapFace`                  |             188.3 +/- 0.77 |                   57.50 +/- 0.160 |
  | `StraightSideSeam`         |             229.9 +/- 0.26 |                   93.65 +/- 0.223 |
  | `RoundedEndOddCore`        |             235.8 +/- 1.66 |                  226.95 +/- 0.913 |
  | `ObliqueRimOverlap`        |             235.0 +/- 0.69 |               1,560.62 +/- 10.562 |
  | `CertifiedRimGap`          | 232.7 +/- 0.40 (wrong hit) |            60.23 +/- 0.212 (miss) |
  | `UnmaterializedScalarFace` |             230.8 +/- 0.59 |                   97.34 +/- 0.761 |

- **Impact:** The oblique positive fixture costs approximately 1.33 ms more than
  the incomplete solver, or 6.64x historical cost. Correctness justifies the
  complete feature set, not ignoring this throughput concern.
- **Retained optimization:** Exact face certificates skip unnecessary rim
  traversal without changing the winner or tie order. The core-perpendicular
  certificate reuses the contained cylinder; the core-parallel certificate
  proves a triangle disk plus contained normal-axis segment supplies the
  attained lower bound. The latter reduced `RoundedEndOddCore` from 1,449.92 +/-
  14.593 us to 226.95 +/- 0.913 us. It deliberately does not certify the oblique
  rim fixture. Do not repeat these experiments as new work.
- **Evidence / allocation:** `artifacts/fms027-baseline`,
  `fms027-final-benchmarks` (before the disk certificate), and
  `fms027-retained-benchmarks` contain the matched captures. All twelve final
  child launches exited zero and reported zero allocated bytes and GC
  collections. Fixtures and commands remain available without those artifacts.
- **Shared-owner controls:** Gravitas's unchanged `mesh-cylinder-contact` group
  also completed all twelve cylinder/circle-slab rows with the same
  launch/warmup/iteration counts and two-core affinity. All twenty-four child
  launches passed their classification/depth preflights with zero allocations or
  collections. Compared with its `grv082-final-focused-bench` capture, no row
  became slower; oblique contacts measured 1,026.97 / 1,024.35 us against
  1,179.01 / 1,160.16 us previously. The fresh reports are under
  `artifacts/fms027-cylinder-controls` in Gravitas.
- **Next isolation step:** Profile `ObliqueRimOverlap`, separating necessary
  endpoint-region/root admission from squared-value construction, mapping,
  ranking and final witness rounding. Assess exact nonwinning-root rejection
  using the existing shared contact owners before considering larger changes.
  Preserve complete classification, minimum depth, deterministic ties,
  nearest-even combined-coordinate rounding, full-domain anchors, bounded stack
  scratch and zero allocations. Recheck zero-core cylinder contacts.

After the baseline build above, reproduce the frozen group:

```powershell
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll triangle-capsule-slab-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --affinity 3 --exporters json --artifacts artifacts/benchmarks/triangle-capsule-slab-cost
```

## Archived Signals

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
