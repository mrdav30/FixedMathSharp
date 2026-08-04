# Single-Limb Raw-Ratio Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure the arbitrary `Signed576` single-limb-denominator ratio path
and specialize the shared divider only if current evidence proves a material
performance gap.

**Architecture:** A dedicated BenchmarkDotNet fixture establishes the current
production baseline and retains a multi-limb control. If the approved evidence
gate opens, `TryGetSignedRawRatioCore(...)` handles a one-word denominator with
the existing portable `Divide128By64(...)` and existing signed materializer; all
other denominators retain the current fixed-limb divider.

**Tech Stack:** C# 11, .NET 8 / .NET Standard 2.1, Q32.32 `Fixed64`, internal
`Signed576` arithmetic, BenchmarkDotNet 0.15.8, xUnit v3, `BigInteger` test
oracle, Coverlet/ReportGenerator.

## Global Constraints

- Preserve every existing sign, zero, round-half-to-even, and representability
  contract.
- Keep one portable deterministic implementation for `netstandard2.1` and
  `net8.0`; do not introduce `UInt128` in this workstream.
- Keep the measured path at zero managed allocation.
- Do not change public APIs or the general multi-limb division algorithm.
- Accept production specialization only for at least 15% stable improvement in
  affected representable rows, with no multi-limb control regression beyond 5%
  or ordinary run noise.
- Preserve 100% reachable line, branch, and method coverage.
- Leave all changes unstaged and uncommitted for repository-owner review.

---

### Task 1: Establish The Current Production Baseline

**Files:**

- Create: `tests/FixedMathSharp.Benchmarks/WideRawRatioBenchmarks.cs`
- Modify: `docs/feature-work/done/2026-08-01-single-limb-raw-ratio-plan.md`

**Interfaces:**

- Consumes: `Fixed64.TryGetSignedRawRatio(Signed576, Signed576, out Fixed64)`.
- Produces: benchmark alias `wide-raw-ratio` and an immutable baseline artifact.

- [x] **Step 1: Add the focused benchmark fixture**

  Add a `[MemoryDiagnoser]` fixture whose five `[Benchmark]` methods return the
  boolean result of the production contract. Construct values once in fields;
  the measured methods must contain only the ratio call.

  Use these deterministic magnitudes:

  ```csharp
  private readonly Signed576 _oneWordNumerator = Positive(0UL, 123_456_789UL);
  private readonly Signed576 _oneWordDenominator = Positive(0UL, 97UL);

  private readonly Signed576 _twoWordNumeratorFor32Bit =
      Positive(1UL, 0x1234_5678_9ABC_DEF0UL);
  private readonly Signed576 _denominator32Bit = Positive(0UL, uint.MaxValue);

  private readonly Signed576 _twoWordNumeratorFor64Bit =
      Positive(0x1000_0000_0000_0000UL, 0x1234_5678_9ABC_DEF0UL);
  private readonly Signed576 _denominator64Bit =
      Positive(0UL, 0xF000_0000_0000_0001UL);

  private readonly Signed576 _unrepresentableNumerator = Positive(1UL, 0UL);
  private readonly Signed576 _unitDenominator = Positive(0UL, 1UL);

  private readonly Signed576 _multiWordNumerator =
      Positive(3UL, 0x1234_5678_9ABC_DEF0UL);
  private readonly Signed576 _multiWordDenominator =
      Positive(1UL, 0xFEDC_BA98_7654_3210UL);
  ```

  `Positive(high, low)` must return
  `new Signed576(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, high, low)`. Mark
  `MultiWordDenominatorControl` as the BenchmarkDotNet baseline.

- [x] **Step 2: Build and list the new benchmark**

  Run:

  ```powershell
  dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
  dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
  ```

  Require a `wide-raw-ratio` alias and a warning-free build.

- [x] **Step 3: Capture the out-of-process baseline**

  Run:

  ```powershell
  dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll wide-raw-ratio --filter "*" -j Short --artifacts artifacts/benchmarks/2026-08-01-single-limb-raw-ratio-baseline
  ```

  Record the exact mean, error, ratio, and allocation for all five rows in this
  plan. Do not change production code before this artifact completes.

- [x] **Step 4: Apply the evidence gate**

  If the affected representable rows are not materially disproportionate to the
  multi-limb control, skip Tasks 2 and 3, complete Task 4 as a measured
  no-change closure, and retain the benchmark. Otherwise document the measured
  bottleneck and proceed.

**Task 1 evidence (2026-08-01):**
`dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0`
completed with 0 warnings and 0 errors. The benchmark catalog lists
`wide-raw-ratio`. The required out-of-process Short job completed with artifacts
at `artifacts/benchmarks/2026-08-01-single-limb-raw-ratio-baseline/` on .NET
8.0.28 / BenchmarkDotNet 0.15.8 (Intel Core i7-9700K, Windows 11).

| Method                                 |      Mean |      Error |     Median | Ratio | Allocated |
| -------------------------------------- | --------: | ---------: | ---------: | ----: | --------: |
| OneWordNumeratorAndDenominator         | 163.20 ns |   9.078 ns | 163.093 ns |  2.30 |       0 B |
| TwoWordNumeratorAnd32BitDenominator    | 279.40 ns |  67.769 ns | 281.244 ns |  3.93 |       0 B |
| TwoWordNumeratorAnd64BitDenominator    | 382.32 ns | 142.754 ns | 382.544 ns |  5.38 |       0 B |
| UnrepresentableQuotient                |  43.28 ns |  12.788 ns |  43.139 ns |  0.61 |       0 B |
| MultiWordDenominatorControl (baseline) |  71.06 ns |  17.245 ns |  70.653 ns |  1.00 |       0 B |

**Gate:** Open. The three representable one-limb-denominator rows are 2.30x,
3.93x, and 5.38x the multi-word control with zero managed allocation. This
materially disproportionate baseline supports the documented shared-divider
single-limb bottleneck hypothesis; continue with Tasks 2 and 3. The Short job's
wide error intervals are baseline evidence only and do not satisfy the candidate
acceptance gate.

### Task 2: Pin The Complete Single-Limb Contract

**Files:**

- Modify:
  `tests/FixedMathSharp.Tests/Numerics/Wide/WideFiniteAxisArithmetic.Tests.cs`

**Interfaces:**

- Consumes: the existing `AssertRawRatio(...)`, `ToSigned576(...)`, and
  `BigInteger` oracle helpers.
- Produces: behavior characterization for the conditional shared-core branch.

- [x] **Step 1: Audit existing cases before adding tests**

  Retain existing coverage for zero, denominator zero, sign pairs, midpoint
  rounding, every magnitude word, and obvious range rejection. Add only missing
  single-limb boundaries; do not duplicate equivalent assertions.

- [x] **Step 2: Add one focused boundary matrix if gaps remain**

  Add `Signed576_SingleLimbDenominator_PreservesRoundingAndSignedRange` with
  direct assertions for these missing cases:

  ```csharp
  AssertRawRatio(1L, 4, 3);                           // below half
  AssertRawRatio(2L, 5, 3);                           // above half
  AssertRawRatio(2L, 5, 2);                           // even midpoint
  AssertRawRatio(4L, 7, 2);                           // odd midpoint
  AssertRawRatio(long.MaxValue, long.MaxValue, 1);
  AssertRawRatio(long.MinValue, -(BigInteger.One << 63), 1);
  AssertRawRatio(1L, (BigInteger.One << 64) + 7, ulong.MaxValue);

  Assert.False(Fixed64.TryGetSignedRawRatio(
      ToSigned576((BigInteger.One << 64) - 1),
      ToSigned576(2),
      out _)); // positive midpoint rounds beyond long.MaxValue
  Assert.False(Fixed64.TryGetSignedRawRatio(
      ToSigned576(-(((BigInteger.One << 63) * 3) + 2)),
      ToSigned576(3),
      out _)); // negative above-half rounds beyond long.MinValue
  ```

  Delete any line already proven by the existing test instead of repeating it.

- [x] **Step 3: Run the characterization suite before production changes**

  Run:

  ```powershell
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~WideFiniteAxisArithmetic"
  ```

  Expect all characterization tests to remain green. The benchmark from Task 1
  is the approved RED performance evidence for this behavior-preserving change.

**Task 2 evidence (2026-08-01):** Audited the existing Signed576 raw-ratio
coverage. It already covers zero, zero denominator, sign pairs, `5 / 3`, both
midpoint parity cases, `long.MinValue`, every magnitude word, and ordinary range
rejection. Added only the missing below-half, `long.MaxValue`,
`ulong.MaxValue`-denominator, and positive/negative rounding-overflow cases.
`dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~WideFiniteAxisArithmetic"`
passed: 47 passed, 0 failed, 0 skipped (net8.0; 256 ms).

### Task 3: Specialize The Shared Single-Limb Boundary

**Conditional:** Execute only if Task 1 opens the evidence gate.

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.WideRatio.cs`
- Modify: `docs/complexity-exceptions.md` only if the touched method crosses or
  changes a registered complexity threshold.

**Interfaces:**

- Consumes: `Divide128By64(...)`, `TryCreateRawRatioResult(...)`, active
  magnitude lengths, and the existing `roundToEven` policy flag.
- Produces: one shared single-word-denominator branch inside
  `TryGetSignedRawRatioCore(...)`.

- [x] **Step 1: Add the minimal shared-core branch**

  After active lengths and `quotientBit > 63` rejection, handle
  `denominatorLength == 1`:

  ```csharp
  if (denominatorLength == 1)
  {
      ulong denominator = denominatorMagnitude[0];
      ulong numeratorHigh = remainderLength == 2 ? remainder[1] : 0UL;
      ulong quotient = Divide128By64(
          numeratorHigh,
          remainder[0],
          denominator,
          out ulong singleWordRemainder);
      int midpointComparison = roundToEven
          ? singleWordRemainder.CompareTo(denominator - singleWordRemainder)
          : -1;
      return TryCreateRawRatioResult(
          quotient,
          midpointComparison,
          negative,
          out result);
  }
  ```

  The preceding bit-length gate proves a representable unsigned quotient has at
  most two numerator limbs and satisfies `Divide128By64`'s quotient-width
  invariant. Do not add an overload-specific path or another helper.

- [x] **Step 2: Run the focused correctness suite**

  Run the Task 2 command. Require zero failures.

- [x] **Step 3: Capture the candidate benchmark**

  Rebuild Release and run the exact Task 1 benchmark command with artifact path:

  ```text
  artifacts/benchmarks/2026-08-01-single-limb-raw-ratio-candidate
  ```

- [x] **Step 4: Compare and decide**

  Accept the branch only if the affected representable rows improve by at least
  15%, allocation stays at zero, and the multi-limb control stays within 5% or
  ordinary statistical noise. Otherwise revert only the production branch,
  retain the benchmark and characterization, and close with a no-change result.

**Task 3 evidence (2026-08-02):** Added the conditional branch only in
`TryGetSignedRawRatioCore(...)`, after the existing `quotientBit > 63`
rejection. It reuses `Divide128By64(...)` and `TryCreateRawRatioResult(...)`;
the general multi-limb divider is unchanged. The final focused Release command
passed 49 tests with 0 failures and 0 skips. The Release benchmark build
completed with 0 warnings and 0 errors. The exact out-of-process Short candidate
job completed under
`artifacts/benchmarks/2026-08-01-single-limb-raw-ratio-candidate/` on the same
.NET 8.0.28 / BenchmarkDotNet 0.15.8 / Intel Core i7-9700K / Windows 11
environment as Task 1. This Short job is diagnostic evidence only; it is not the
canonical performance claim.

| Method                              | Baseline median | Candidate median |   Delta | Allocated |
| ----------------------------------- | --------------: | ---------------: | ------: | --------: |
| OneWordNumeratorAndDenominator      |      163.093 ns |        54.314 ns | -66.70% |       0 B |
| TwoWordNumeratorAnd32BitDenominator |      281.244 ns |        64.888 ns | -76.93% |       0 B |
| TwoWordNumeratorAnd64BitDenominator |      382.544 ns |       131.618 ns | -65.59% |       0 B |
| UnrepresentableQuotient             |       43.139 ns |        42.282 ns |  -1.99% |       0 B |
| MultiWordDenominatorControl         |       70.653 ns |        71.567 ns |  +1.29% |       0 B |

**Review fix round 1 canonical evidence (2026-08-02):** The canonical comparison
uses BenchmarkDotNet `DefaultJob`, without `-j Short`, from a temporary detached
worktree at baseline `28aef445471177ffc499c4ae0c86dad3b217733b` and from the
final current worktree. The fixture SHA-256 matched in both worktrees:
`C121ABB6D5AED2FF9716134C8E599AE20696D9977D3DBB29ABB4D521AD082145`. Both runners
built in Release/net8.0 with 0 warnings and 0 errors, then ran the same five-row
`wide-raw-ratio --exporters json` alias. Canonical artifacts are preserved under
`artifacts/benchmarks/2026-08-02-single-limb-raw-ratio-canonical-baseline/` and
`artifacts/benchmarks/2026-08-02-single-limb-raw-ratio-canonical-candidate/`.

| Method                              | Baseline mean | Candidate mean | Mean delta | Baseline median | Candidate median | Median delta | Error (base -> candidate) | StdDev (base -> candidate) |  Allocated |
| ----------------------------------- | ------------: | -------------: | ---------: | --------------: | ---------------: | -----------: | ------------------------: | -------------------------: | ---------: |
| OneWordNumeratorAndDenominator      |    160.759 ns |      51.770 ns |    -67.80% |      160.561 ns |        51.676 ns |      -67.82% |         1.290 -> 0.331 ns |          1.077 -> 0.310 ns | 0 B -> 0 B |
| TwoWordNumeratorAnd32BitDenominator |    279.194 ns |      61.077 ns |    -78.12% |      277.527 ns |        60.916 ns |      -78.05% |         4.104 -> 0.530 ns |          3.839 -> 0.414 ns | 0 B -> 0 B |
| TwoWordNumeratorAnd64BitDenominator |    376.112 ns |     131.217 ns |    -65.11% |      374.211 ns |       131.199 ns |      -64.94% |         4.120 -> 0.284 ns |          3.854 -> 0.222 ns | 0 B -> 0 B |
| UnrepresentableQuotient             |     42.586 ns |      43.207 ns |     +1.46% |       42.568 ns |        43.174 ns |       +1.43% |         0.206 -> 0.138 ns |          0.172 -> 0.122 ns | 0 B -> 0 B |
| MultiWordDenominatorControl         |     73.190 ns |      72.257 ns |     -1.27% |       72.626 ns |        71.670 ns |       -1.32% |         1.438 -> 1.112 ns |          1.413 -> 1.041 ns | 0 B -> 0 B |

**Decision:** Accept and retain the specialization based on the canonical
default-job evidence. All three affected representable rows improve by more than
the required 15% on both mean and median and remain allocation-free. The
multi-limb control improves by 1.27% mean and 1.32% median, safely within the 5%
regression gate. The non-target unrepresentable row regresses by 1.46% mean and
1.43% median, with lower absolute error and standard deviation; it is not a
gated affected row. Review round 1 added shared zero-denominator rejection and
two focused caller-policy regressions; the focused Release suite now passes 49
tests. The touched shared core has cyclomatic complexity 11 by the register's
counting policy, so `docs/complexity-exceptions.md` now records it for the Task
4 full coverage refresh.

### Task 4: Release And Documentation Closure

**Files:**

- Modify: `docs/feature-work/benchmark-signal-hardening-backlog.md`
- Modify: `docs/feature-work/2026-08-01-single-limb-raw-ratio-design.md`
- Move: this plan to `docs/feature-work/done/`

**Interfaces:**

- Consumes: the accepted candidate or documented no-change result.
- Produces: an empty FixedMathSharp benchmark-signal backlog with reproducible
  evidence.

- [x] **Step 1: Run focused allocation and ordering checks**

  Confirm every benchmark row reports zero managed bytes and rerun the complete
  wide-arithmetic test class.

- [x] **Step 2: Run release gates sequentially**

  Run:

  ```powershell
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c ReleaseLean
  dotnet build src/FixedMathSharp/FixedMathSharp.csproj -c Release
  dotnet build src/FixedMathSharp/FixedMathSharp.csproj -c ReleaseLean
  ```

  Require zero failures and zero warnings for both target frameworks and package
  variants.

- [x] **Step 3: Re-run independent coverage**

  Collect coverage with `tests/FixedMathSharp.Tests/coverlet.runsettings`,
  generate a ReportGenerator summary, and require 100% reachable line, branch,
  and method coverage. Remove unreachable branches instead of adding hollow
  tests.

- [x] **Step 4: Validate the downstream boundary when applicable**

  If production code changes, run Gravitas `Release` and `ReleaseLean` directly
  from `tests/Gravitas.Tests/Gravitas.Tests.csproj` through the existing local
  project link. If the result is benchmark/docs-only, record why downstream
  execution is unnecessary.

**Task 4 Steps 1-4 evidence (2026-08-02):** The final candidate CSV contains
five benchmark rows and every row reports `0 B`. The complete focused
`WideFiniteAxisArithmetic` Release filter passed 49/49. The four required
release gates then ran sequentially: Release passed 2,652/2,652 tests,
ReleaseLean passed 2,631/2,631 tests, and both package builds produced `net8.0`
and `netstandard2.1` outputs with 0 warnings and 0 errors.

The first full main-suite coverage refresh exposed two structurally unreachable
short-circuit predicates in `WideArithmetic.Comparison.cs`: a zero squared-axis
branch despite every production caller supplying a nonzero squared axis, and a
nonpositive common denominator branch despite every producer supplying a
positive quaternion-norm denominator or product. Both zombie predicates were
removed instead of adding invalid-input coverage tests. The combined
normalized-depth and wide-arithmetic Release filter then passed 59/59, and all
four release gates above were rerun successfully from the final source. Fresh
coverage passed 2,652/2,652 tests and reports 53,003/53,003 lines, 8,768/8,768
branches, and 3,411/3,411 methods. `TryGetSignedRawRatioCore(...)` itself
reports 100% line and branch coverage. The CRAP scripts analyzed 3,407 methods,
found 0 below-threshold coverage gaps or uncovered methods, and reported 9
score-above-30 methods; all nine remain fully covered and registered
deterministic complexity exceptions.

The unmodified Gravitas local-link graph first failed Release before tests
because packaged `Chronicler.Core` 0.4.0 and the local 0.0.0 `Chronicler`
assembly had the same simple name. A no-file-change `-p:SemVer=0.4.0` retry
retained the existing local project links and passed Release 3,925/3,925.
ReleaseLean initially exposed a stale global NuGet cache entry for
`Chronicler.MemoryPackShim` 0.4.0. A fresh isolated `--no-cache --force` restore
fetched the current 6,656-byte net8.0 shim with the required `GenerateType` and
`SerializeLayout` constructors; ReleaseLean then passed 3,870/3,870 through the
same local links and fresh package path. Neither Gravitas files nor the
local-link scaffolding were modified.

- [x] **Step 5: Complete independent review**

  Request a read-only correctness/performance review of the complete diff,
  benchmark comparison, coverage evidence, and backlog closure. Resolve every
  Critical or Important finding before closure.

- [x] **Step 6: Close the living documents**

  Move the signal to Closed Signals with exact baseline/candidate values,
  allocation, test, coverage, package, and review evidence. Mark the design
  complete and archive this plan under `docs/feature-work/done/`.

**Task 4 Steps 5-6 closure evidence (2026-08-02):** The controller's read-only
pre-closure review covered the complete diff, correctness and performance
contracts, benchmark comparison, release/Lean/package gates, coverage/CRAP
evidence, downstream validation, and planned documentation closure. Its one
Important finding was that the accepted performance claim relied on a Short
diagnostic job. Fix round 1 resolved it with matched BenchmarkDotNet
`DefaultJob` baseline/candidate artifacts from base `28aef44` and the final
worktree; follow-up review verified both artifacts, the updated evidence, and
safe temporary-worktree cleanup, leaving no open Critical or Important findings.

The single-limb-denominator entry is now in Closed Signals with canonical
values, 0 B/op, FixedMathSharp Release 2,652/2,652, ReleaseLean 2,631/2,631,
53,003/53,003 lines, 8,768/8,768 branches, 3,411/3,411 coverage methods,
warning-free package builds, and Gravitas Release 3,925/3,925 plus ReleaseLean
3,870/3,870 evidence. The design is marked complete, Active Signals is empty,
and this plan is archived under `docs/feature-work/done/`.
