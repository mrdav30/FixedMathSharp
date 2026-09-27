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

- **Status / priority:** Isolated, high; measured on 2026-09-26 while closing
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
- **Measurement:** Penetrating rims **11.566 +/- 0.110ms**; multi-radical contact
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
- **Impact:** A few difficult oblique contacts can dominate a fixed frame.
  No observed game workload or contact-count budget establishes acceptability.
  Exact determinism, minimum-depth selection and rounding remain mandatory.
- **Next isolation step:** Profile the retained complete solver on these frozen
  cases, distinguishing feature construction, admissibility/selection and final
  rounding. Establish the intended contact-count/frame budget before proposing
  a larger exact-algorithm redesign. Existing profiles already removed repeated
  GCDs/chains, redundant normals and worst-case mapping refinement; do not
  repeat those experiments or add another approximate fallback.

After the baseline build above, reproduce the two expensive positive contacts:

```powershell
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll rigid-finite-shape-relation --filter '*PenetratingRimsCylinderCylinder*' '*MultiRadicalCylinderCylinder*' --warmupCount 3 --iterationCount 12 --launchCount 2 --keepFiles --exporters json --artifacts artifacts/benchmarks/cylinder-pair-cost
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
