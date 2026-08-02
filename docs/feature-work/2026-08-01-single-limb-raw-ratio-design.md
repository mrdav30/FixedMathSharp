# Single-Limb Raw-Ratio Performance Design

**Date:** 2026-08-01  
**Status:** Complete — specialization accepted  
**Repository:** `FixedMathSharp`

## Problem

The benchmark backlog records an incomplete performance signal for
`Fixed64.TryGetSignedRawRatio(Signed576, Signed576, ...)` when the denominator
has one active 64-bit limb. The original investigation was terminated without
a completed timing sample and predates the overload's current `Signed192` and
`Signed320` narrowing paths.

The shared divider still routes a one-limb denominator through equal-length
multi-limb shift, compare, subtract, and midpoint work. That shape is a
plausible bottleneck, but production specialization is not justified until a
current isolated benchmark proves it.

## Goals

- Produce a stable current baseline for representative one-limb denominators.
- Compare those rows with a multi-limb denominator control.
- Specialize only if the measured cost is material.
- Preserve the complete arbitrary signed wide-ratio contract: denominator and
  numerator signs, zero handling, round-half-to-even, signed raw range, and
  honest rejection of unrepresentable results.
- Preserve deterministic behavior, zero allocation, `netstandard2.1`
  compatibility, and one portable implementation across target frameworks.
- Retain a focused benchmark guard and close the backlog with measured evidence.

## Non-Goals

- Do not change any public API or rounding contract.
- Do not assume the denominator is positive or that the quotient is
  representable.
- Do not replace the general multi-limb divider.
- Do not add `UInt128` or another target-specific implementation without a
  separate benchmark proving the portable `Divide128By64` helper is the
  remaining bottleneck.
- Do not optimize unrelated wide-arithmetic paths.

## Benchmark Design

Add one dedicated `WideRawRatioBenchmarks` fixture with deterministic values
covering:

1. A one-word numerator and one-word denominator.
2. A two-word numerator and 32-bit denominator.
3. A two-word numerator and 64-bit denominator.
4. A one-word denominator whose quotient is immediately unrepresentable.
5. A multi-word denominator control that remains on the general divider.

Every row calls the production `Signed576 / Signed576` contract and reports
managed allocation. Capture the baseline and candidate with the same Release
build, BenchmarkDotNet job, machine, filters, and artifact layout.

The specialization is accepted only when a stable out-of-process comparison
shows at least a provisional 15% improvement in the affected representable
rows, zero managed allocation, and no multi-limb control regression beyond 5%
or ordinary run noise. If the baseline is already competitive, retain the
benchmark and close the signal with a no-change decision.

## Conditional Production Design

If the evidence gate opens, specialize the shared
`TryGetSignedRawRatioCore(...)` boundary when the active denominator length is
one. The numerator can contain at most two active limbs when its quotient fits
in 64 bits; larger magnitudes are rejected before division. Reuse the existing
portable `Divide128By64(...)` helper, compare the remainder with
`denominator - remainder` for overflow-free half-even classification, and
delegate final rounding, signed-range validation, and materialization to the
existing `TryCreateRawRatioResult(...)` owner.

This location serves `Signed576`, `Signed704`, `Signed832`, and span-backed
callers without duplicating representation mechanics in an overload-specific
fast path. Multi-limb denominators continue through the current fixed-limb
divider unchanged.

## Correctness And Test Design

The optimization is behavior-preserving, so its RED evidence is the completed
benchmark showing the current path misses the approved performance gate rather
than an artificial unit-test failure. Existing wide-ratio tests remain the
contract baseline. Before production changes, extend the `BigInteger` oracle
matrix only where needed to pin:

- positive and negative numerator/denominator combinations;
- below-half, exact even/odd midpoint, and above-half rounding;
- one- and two-limb numerators;
- `long.MaxValue` and `long.MinValue` materialization boundaries;
- quotient overflow before rounding and overflow caused by rounding; and
- equivalence between single-limb and general denominator results.

No test-only production hook, path counter, reflection check, or benchmark
timing assertion belongs in the unit suite.

## Verification And Closure

- Re-run the exact baseline benchmark after each candidate change.
- Run focused wide-arithmetic tests, then complete `Release` and `ReleaseLean`
  suites.
- Preserve 100% reachable line, branch, and method coverage.
- Build standard and Lean packages for both target frameworks with zero
  warnings.
- Re-run downstream Gravitas gates only if the internal ABI or a consumed path
  changes materially.
- Request independent correctness and performance review before closure.
- Move the backlog entry to Closed Signals with the baseline, final evidence,
  and either the accepted specialization or explicit no-change decision.
- Leave all changes unstaged and uncommitted for repository-owner review.

## Final Outcome

The shared one-limb-denominator specialization was accepted. Canonical
BenchmarkDotNet `DefaultJob` artifacts compare base `28aef44` with the final
candidate under the same Release/net8.0 environment:

| Row | Baseline mean | Candidate mean | Mean delta | Baseline median | Candidate median | Median delta | Allocated |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| One-word numerator/denominator | 160.759 ns | 51.770 ns | -67.80% | 160.561 ns | 51.676 ns | -67.82% | 0 B -> 0 B |
| Two-word numerator / 32-bit denominator | 279.194 ns | 61.077 ns | -78.12% | 277.527 ns | 60.916 ns | -78.05% | 0 B -> 0 B |
| Two-word numerator / 64-bit denominator | 376.112 ns | 131.217 ns | -65.11% | 374.211 ns | 131.199 ns | -64.94% | 0 B -> 0 B |
| Unrepresentable quotient | 42.586 ns | 43.207 ns | +1.46% | 42.568 ns | 43.174 ns | +1.43% | 0 B -> 0 B |
| Multi-word denominator control | 73.190 ns | 72.257 ns | -1.27% | 72.626 ns | 71.670 ns | -1.32% | 0 B -> 0 B |

All three affected representable rows exceed the 15% improvement gate on mean
and median. The multi-word control stays within the 5% regression gate by
improving 1.27% by mean and 1.32% by median. Every row remains allocation-free,
the public/general-divider contracts are unchanged, and final Release,
ReleaseLean, coverage, package, downstream, and independent-review gates pass.
Canonical artifacts are preserved under
`artifacts/benchmarks/2026-08-02-single-limb-raw-ratio-canonical-baseline/` and
`artifacts/benchmarks/2026-08-02-single-limb-raw-ratio-canonical-candidate/`.
