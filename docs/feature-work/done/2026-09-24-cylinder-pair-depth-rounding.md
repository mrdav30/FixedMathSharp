# FMS-Issue-026: Bounded exact cylinder-pair depth rounding

## Resolution and scope

`GetRoundedCylinderCylinderDepth` retains four attempts to correct its scalar
estimate with the existing exact midpoint comparator, then uses the existing
exact binary search over the nonnegative `Fixed64` raw domain. Four is a work
budget, not an error tolerance: no estimate is accepted without exact midpoint
proof. The search terminates in at most 64 steps and still uses the same
nearest-even increment. The exact comparison against `long.MaxValue` preserves
the distinction between an exact maximum and a depth that must be clamped.

Candidate enumeration, ranking, normal generation, support anchors, and the
comparison arithmetic are unchanged. This is a result-preserving runtime
correction, not a repair for the incomplete cylinder-pair contact geometry in
[FMS-Issue-024](../issue-tracker.md). No floating-point runtime math, new
allocation, public API, or approximate fallback was added.

## Root cause

The approximation floors the candidate axis length before dividing rational
and disk-support terms. The original correction loop assumed the resulting
depth was only a few raw units from its exact value and moved one raw unit per
iteration with no bound.

That assumption is false for an admitted closest-core candidate.
`WideOrientedBox.RigidClosestCandidate` removes common powers of two from the
exact direction; the resulting integer components can be small. Flooring a
small nonsquare length introduces substantial relative error, even though each
later scalar conversion is correctly rounded.

For the FMS-Issue-024 separated fixture, cylinder A has center zero, axis +Y,
height 2, radius 1; B has center `(7/4,7/4,11/8)`, axis +X, height 2, radius 1.
Its selected core direction reduces to `(6,6,11)`, whose squared length is 193
and floor length is 13. The selected projection is
`(2*sqrt(157)-193/8)/sqrt(193)`. Its exact nearest-even result is raw
`289041002`, but the estimate is raw `308883531`: the old loop requires
`19842529` downward corrections. This is the selected projection, not a claim
that this axis proves the cylinders intersect.

For the penetrating control with second Z coordinate `5/4`, the reduced
direction is `(3,3,5)`. Its selected projection is
`(2*sqrt(34)-43/4)/sqrt(43)`, exactly rounding to raw `597275436`; the estimate
is raw `652766157`, requiring `55490721` old corrections.

## Reproduction and before/after evidence

The old rounding owner was inspected at source base `0c2a120`. A stack captured
while the first fixture was running showed
`GetMultiquadraticSign -> GetLinearRadicalSumSign ->
CompareCylinderPairDepthToTwiceRaw -> GetRoundedCylinderCylinderDepth`.
The unchanged public regression class exceeded a 90-second diagnostic timeout
without completing its first separated-rim case. A previous diagnostic was
stopped after roughly 4.5 CPU minutes. These were deliberate test-host aborts,
not newly observed runtime crashes.

The new independent-oracle regression was executed before the production edit.
Its `zNumerator=19, denominator=16` case also exceeded a 15-second diagnostic
timeout without completing. After the edit, the same case passed in about
80 ms including both argument orders; the other two new reduced-axis cases
each completed in under 5 ms in that focused run. The unchanged five-case
public geometric regression finished in 124 ms with its expected three
failures and two passes; the formerly stalled separated case took 78 ms.
These are Windows test-runner diagnostics, not steady-state benchmark claims.

Portable reproductions (add `-p:UseLocalLsfStack=true` for coordinated source
development):

```text
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter FullyQualifiedName~CenteredCylinderPairDepthRoundingTests
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter FullyQualifiedName~CenteredCylinderPairStrictRegressionTests
```

The second command intentionally remains red for FMS-Issue-024. No old
geometric assertions were removed, weakened, skipped, or excluded.

## Regression proof

`CenteredCylinderPairDepthRoundingTests` checks independent literal midpoint
inequalities with test-only `BigInteger` arithmetic:

- Three reduced nonsquare axes that force the bounded search; both operand
  orders preserve exact depth, opposite normals, and the unclamped flag.
- Parallel-core cases needing zero correction, one upward correction, and one
  downward correction, with independent squared-distance midpoint checks.
- Half-raw ties rounding in both nearest-even directions.

Existing `CenteredCylinderRigidFrameTests` cover exact maximum without
clamping, half a raw unit above maximum with clamping, deeper overflow,
full-domain cancellation, multiradical ranking, and warmed allocations.
The first focused Release run passed all 30 then-present cases.

Final Windows verification included every core test, with no excluded or
skipped geometric regressions:

| Configuration | Complete core suite | Instrumented covered lines | Instrumented covered branches | Chronicler suite |
| --- | --- | --- | --- | --- |
| Release | 3,123 passed; 6 known failures | 49,417 / 49,417 | 9,578 / 9,578 | 49 passed |
| ReleaseLean | 3,102 passed; 6 known failures | 49,510 / 49,510 | 9,578 / 9,578 | 49 passed |

The coverage totals include `FixedMathSharp` and `FixedMathSharp.FluentAssertions`
instrumented by the core suite; both assemblies have complete line, branch and
method coverage. The separate Chronicler suite was executed without coverage.

The six failures are the three FMS-Issue-023 capsule/cylinder cases and three
FMS-Issue-024 cylinder-pair cases. All eight new rounding cases passed in both
configurations. All four methods in the rounding owner have 100% line and
branch coverage; `GetRoundedCylinderCylinderDepth` has cyclomatic complexity
14. Core `netstandard2.1` builds succeeded in both configurations with zero
warnings and errors. Tests target `net8.0`; these source-mode results do not
establish released-package readiness or cross-platform verification.

Full coverage commands use the core test project with
`--collect:"XPlat Code Coverage" --settings tests/FixedMathSharp.Tests/coverlet.runsettings`.
The complete core runs took 22 seconds in Release and 23 seconds in ReleaseLean.
Independent review found no actionable change to exact rounding, clamping, or
geometry. Steady-state benchmark comparisons remain separate from these
diagnostic timings.

Ignored logs, stack captures, TRX files and diagnostic dumps under `artifacts`
are supplementary; the fixtures, numerical mechanism and commands above
preserve the reproducer without those files.
