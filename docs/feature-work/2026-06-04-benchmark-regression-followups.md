# Benchmark Regression Follow-Up Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before changing runtime code for any
> benchmark-discovered issue, use superpowers:test-driven-development for
> correctness fixes, and use superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Active

**Goal:** Convert the completed benchmark scaffold into repeatable performance
regression practice without adding noisy CI gates or unsupported optimization
claims.

**Architecture:** Keep `tests/FixedMathSharp.Benchmarks` as the evidence lab,
keep `docs/feature-work/issue-tracker.md` as the tooling/correctness issue
queue, and use full `Release` BenchmarkDotNet artifacts for performance claims.
Short in-process runs remain smoke checks only.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, .NET 8
BenchmarkDotNet runner, xUnit, MemoryPack, System.Text.Json, `Release`, and
`ReleaseLean`.

---

## Phase 1: Capture A Fresh Full Baseline

**Files:**

- Review: `tests/FixedMathSharp.Benchmarks/README.md`
- Review: `BenchmarkDotNet.Artifacts/results/`

- [ ] Run a full `Release` benchmark export after the Phase 1-6 scaffold
  changes are committed.
- [ ] Preserve the JSON artifacts under a dated local archive or agreed docs
  location.
- [ ] Record the SDK/runtime, OS, CPU, power mode, and command used.
- [ ] Do not compare against short in-process smoke numbers.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

## Phase 2: Compare And Prioritize Real Hot Spots

**Files:**

- Review: `BenchmarkDotNet.Artifacts/results/*-report-github.md`
- Review: `docs/feature-work/issue-tracker.md`

- [ ] Compare speed, allocation, GC counts, branch/data movement, and
  algorithmic complexity by subsystem.
- [ ] Separate intentional serialization payload allocation from accidental
  hot-path allocation.
- [ ] Promote confirmed defects or tooling failures to
  `docs/feature-work/issue-tracker.md`.
- [ ] Create focused optimization plans only for measured bottlenecks with a
  clear correctness test strategy.

## Phase 3: Decide CI Guardrails

**Files:**

- Review: `.github/workflows/build-and-test.yml`
- Review: `tests/FixedMathSharp.Benchmarks/README.md`

- [ ] Decide whether CI should remain compile-only for benchmarks or add a
  narrow smoke benchmark job.
- [ ] Avoid raw timing thresholds until runner variance is understood.
- [ ] If a smoke job is adopted, keep it alias-scoped and make it validate
  benchmark selection/execution rather than performance deltas.
- [ ] Account for the host/configuration-isolated intermediate output paths
  added for `Release`/`ReleaseLean` conditional package graphs.

## Related Issue Tracker Items

- Resolved on 2026-06-04: `FMS-Issue-005`, `FMS-Issue-006`, and
  `FMS-Issue-008` were closed by making coverage opt-in for local test runs and
  isolating intermediate/NuGet assets by host platform and configuration.
