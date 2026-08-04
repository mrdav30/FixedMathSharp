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

No active benchmark signals.

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
