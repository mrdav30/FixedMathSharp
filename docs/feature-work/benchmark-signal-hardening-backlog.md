# Benchmark Signal Hardening Backlog

## Purpose

This document captures benchmark-derived hardening signals that fall outside the
active feature plan. It is intentionally undated and long-lived: individual
entries carry their own discovery dates, evidence, status, and next isolation
step.

Use this backlog for measured performance, allocation, scaling, and benchmark
evidence concerns. Bugs or correctness risks that are not primarily benchmark
signals belong in [`issue-tracker.md`](issue-tracker.md). Broad feature or
architecture work should be promoted into its own dated plan and referenced from
this backlog.

## Intake Rules

- Add a signal only when it comes from a benchmark, allocation guardrail,
  profiler trace, or repeated validation run.
- Record the command, date, affected row or test, measured value, why it
  matters, and the smallest useful next isolation step.
- Keep benchmark-only instrumentation in tests or benchmark support unless the
  runtime needs a durable diagnostic API.
- Prefer a focused fix when the signal has a narrow cause.
- Promote to a dated feature-work plan when the signal spans multiple
  subsystems, requires API design, or needs staged implementation.
- Close entries only after a runtime/test/docs change lands or after a written
  no-change decision explains why the signal is expected.

## Baseline Commands

Build the benchmark project before capturing evidence:

```powershell
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
```

## Active Signals

| Signal                       | Status  | Priority | Tracking |
| ---------------------------- | ------- | -------- | -------- |
| Single-limb-denominator path | Holding | Low      | N/A      |

### Signal: Single-limb-denominator path

**Discovered:** 2026-07-19  
**Source:** repeated `Fixed64.TryGetSignedRawRatio(Signed576, Signed576, ...)`
benchmark runs  
**Status:** Observed; terminated run, no completed timing sample

- Benchmark any proposed single-limb-denominator path for the general
  `Fixed64.TryGetSignedRawRatio(Signed576, Signed576, ...)` contract before
  replacing its fixed-limb divider. Exact coordinate interpolation now owns a
  narrower `Signed320 / Signed192` path whose positive single-word denominator
  and representable quotient are construction invariants; it reduced the
  existing segment reconstruction benchmark from roughly 514/759 nanoseconds to
  about 121/118 nanoseconds at unit/100,000 scale. Do not generalize those
  invariants to arbitrary signed 576-bit callers without separate evidence.

## Closed Signals

| Signal | Status | Closed | Resolution |
| ------ | ------ | ------ | ---------- |
| none   |        |        |            |
