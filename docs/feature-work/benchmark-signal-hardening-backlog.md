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

| Signal | Status | Priority | Tracking |
| ------ | ------ | -------- | -------- |
| none   |        |          |          |

## Closed Signals

| Signal | Status | Closed | Resolution |
| ------ | ------ | ------ | ---------- |
| Single-limb-denominator path | Accepted | 2026-08-02 | Shared portable specialization retained; canonical affected rows are 65.11%-78.12% faster by mean, the control improves 1.27%, and every row remains 0 B/op. |

### Closed: Single-limb-denominator path

Canonical BenchmarkDotNet `DefaultJob` comparison from base `28aef44` to the
accepted candidate:

| Row | Baseline mean | Candidate mean | Delta | Allocated |
| --- | ---: | ---: | ---: | ---: |
| One-word numerator/denominator | 160.759 ns | 51.770 ns | -67.80% | 0 B -> 0 B |
| Two-word numerator / 32-bit denominator | 279.194 ns | 61.077 ns | -78.12% | 0 B -> 0 B |
| Two-word numerator / 64-bit denominator | 376.112 ns | 131.217 ns | -65.11% | 0 B -> 0 B |
| Unrepresentable quotient | 42.586 ns | 43.207 ns | +1.46% | 0 B -> 0 B |
| Multi-word denominator control | 73.190 ns | 72.257 ns | -1.27% | 0 B -> 0 B |

Reproduction commands, run from the matching baseline and candidate worktrees;
the baseline is detached at `28aef44` with the final benchmark fixture copied
unchanged before the build:

```powershell
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --nologo
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll wide-raw-ratio --exporters json --artifacts artifacts/benchmarks/2026-08-02-single-limb-raw-ratio-canonical-baseline
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll wide-raw-ratio --exporters json --artifacts artifacts/benchmarks/2026-08-02-single-limb-raw-ratio-canonical-candidate
```

Artifacts are preserved under the two command paths above. FixedMathSharp
passed Release 2,652/2,652, ReleaseLean 2,631/2,631, warning-free standard and
Lean package builds, and 100% coverage at 53,003/53,003 lines,
8,768/8,768 branches, and 3,411/3,411 methods. Gravitas passed Release
3,925/3,925 and ReleaseLean 3,870/3,870 through the existing local links after
version alignment and a fresh isolated restore; no Gravitas file changed. The
independent pre-closure review's canonical-evidence finding was resolved with
the matched DefaultJob artifacts, and follow-up review left no open Critical or
Important findings.
