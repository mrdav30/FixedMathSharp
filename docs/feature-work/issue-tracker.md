# Feature Work Issue Tracker

## Purpose

This document tracks unresolved correctness, determinism, API, documentation,
build, and tooling issues that fall outside an active implementation plan.

Measured performance and allocation concerns belong in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md).
Work that requires staged implementation belongs in a focused feature-work plan.

## Tracker Rules

- Record only unresolved, reproducible concerns.
- Include the affected area, evidence, user or runtime impact, priority, and
  smallest useful next action.
- Use repository-relative paths and portable commands. Do not record
  developer-specific drive letters, home directories, or machine-local
  dependency locations.
- Keep an issue here while it needs investigation or is deliberately deferred.
- Link an active implementation plan instead of duplicating its task details.
- Remove an entry after its resolution and verification are complete. Preserve
  durable design decisions in the completed plan, public documentation, or
  release notes rather than retaining a resolved-issue archive here.

## Active Issues

### FMS-Issue-018: Convex containment benchmark child fails during measurement

- **Status:** Needs reproduction with fault capture. Two separate unchanged-
  runtime captures fail on 2026-09-13 UTC before the transformed-endpoint
  optimization; no source or runtime cause is established.
- **Priority:** Medium
- **Affected area:** Out-of-process convex containment benchmark execution.
- **Evidence:** On FixedMathSharp `e8a2ab5`, Windows 11, .NET 8.0.29 x64 and
  BenchmarkDotNet 0.15.8, run:
  `dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll convex-point-containment --filter '*' --launchCount 3 --exporters json --keepFiles --artifacts artifacts/convex-containment-baseline`
  after a Release build. `Outside(VertexCount: 4, RotationDegrees: 30)`, launch
  2, PID 24476, emits six actual measurements then exits `-1073741819`
  (`0xC0000005`) without workload results or a managed stack. Windows Application
  event 1000 at 02:55:46 UTC confirms the process, unknown faulting module and
  fault offset `0x000000007ffffff8` (report
  `9e9bf172-7e18-4e20-b6a3-cbe0a87037b1`). Only 23/24 planned children execute.
  The coordinating Trailblazer checkout preserves the complete log, generated
  runner, actual child binaries and SHA-256 manifest under
  `artifacts/benchmark005/fms-vertex-baseline*`, plus the Windows event export.
- **Second capture:** `fms-vertex-baseline2.log` fails in the same case and
  launch position, PID 11308, after four actual measurements. This time BDN
  reports `NullReferenceException` at `GetWorldPoint` (ProjectionMath.cs:445),
  through `ContainsPoint` (WideConvex2dRelations.cs:516), `Outside()` (benchmark
  line 42) and generated `WorkloadActionUnroll` (line 893); child exit is `-1`.
  The exact case uses origin `(40, 50)`, query `(50, 50)`, four rectangle
  vertices and 30-degree rotation. Keep this second incomplete capture separate
  from the first native-only failure; neither is eligible for comparison.
- **Impact:** Both captures are incomplete and excluded from performance
  comparison, including their partially populated summaries. The existing
  FixedMathSharp launcher returns zero despite the failed child; inspect child
  exits and expected launch/sample counts, not just the parent exit code.
- **Bounded diagnostic follow-up:** One console-host execution of the frozen
  second child, benchmark ID 3, runs under ProcDump's first-chance
  `C0000005,*NullReferenceException*` filter with a 120-second collector limit.
  PID 6148 finishes with child exit zero, 15 raw actual iterations, zero
  measured allocation and no dump. Both frozen 51-file manifests remain
  unchanged. The log is `artifacts/benchmark005/fms-vertex-fault-probe/capture.log`.
  This changes the parent/hosting and debugger timing; it is not a replacement
  baseline, a reproduction of either fault or evidence of a fix.
- **Next action:** If it recurs, use the existing cross-stack native-fault
  capture procedure against the frozen child to retain a dump and exact fault
  state. Do not add null guards, retries or arithmetic workarounds without a
  cause. Neither a relationship to `GF-Issue-006` / `TRB-Issue-119` nor a fix is
  established. No new launcher/CI framework is part of this performance slice.

### FMS-Issue-019: Product-comparison allocation guard reported an unexplained burst

- **Status:** Needs reproduction after test-protocol alignment. No production
  allocator or source for the observed bytes is established.
- **Priority:** Low
- **Affected area:** `Fixed64ProductComparisonTests.CompareProducts_WarmedExecution_DoesNotAllocate`.
- **Evidence:** The first covered ReleaseLean run on 2026-09-13 UTC reports
  2,208 bytes (8,298,416 before / 8,300,624 after); 2,694 other core tests pass.
  `CompareProducts` source is unchanged by the containment optimization.
  The coordinating Trailblazer checkout retains the failed log/TRX/coverage in
  `artifacts/benchmark005/vertex-verification` and source plus post-run binary
  snapshots in `vertex-allocation-failure` beside it.
- **Confirmed protocol correction:** The original guard warmed a discard-result
  loop and measured a separate accumulating loop, using direct counters despite
  the existing repository guidance. Both phases now use the same operation via
  `FixedMathTestHelper.MeasureWarmedAllocations`: 64 calls to each overload,
  unchanged inputs, zero checksum and exactly zero bytes. No retry, tolerance,
  extra warmup or runtime change was added. The aligned seven-test class passes;
  temporarily allocating a 64-byte array per iteration fails at 5,632 bytes.
  That mutation is removed. This proves guard sensitivity, not the burst's cause.
- **Next action:** Preserve any recurrence with its configuration and coverage
  context before attributing it to production code. Successful aligned runs do
  not establish that flakiness is cured; no broader test-harness work is planned.

**Next issue ID:** `FMS-Issue-020`

## Issue Template

```markdown
### FMS-Issue-###: Concise title

- **Status:** Needs reproduction | Confirmed | Planned | Deferred
- **Priority:** Critical | High | Medium | Low
- **Affected area:** Public API, source owner, package, build, or documentation
- **Evidence:** Minimal reproduction, failing test, or portable command
- **Impact:** Observable correctness, determinism, usability, or tooling risk
- **Next action:** The smallest useful investigation or implementation step
```

When adding an issue, use the next ID and increment the field above. Never reuse
an ID that appears in a completed plan or repository history.
