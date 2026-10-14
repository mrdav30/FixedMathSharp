# FMS-Issue-029: scaled rotation decomposition

## Status

Completed 2026-10-14. Large rotated TRS matrices no longer fail solely because
scale amplifies quaternion rounding. All validation gates pass; changes remain
unstaged for owner review.

## Reproducer and cause

The issue was recorded against `819dbde` and reproduced before this fix on
`5860fea`, using Windows x64, .NET SDK 10.0.302 and runtime 8.0.29. Construct a
`FixedTransform` with zero position, uniform scale 2,048 and either rotation:

```csharp
FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)19, (Fixed64)31, (Fixed64)11)
FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.PiOver4)
```

`TryGetLocalToWorldMatrix` succeeds, but `Fixed4x4.Decompose` returns false.
Scale one succeeds. The normalized basis is orthogonal within one raw increment
and its quaternion reconstruction passes. The final scaled reconstruction differs
by 3,724 raw increments for the Euler rotation and 2,097 for the planar rotation,
exceeding the fixed absolute epsilon of 256. The rejection is caused by scale
amplifying quaternion rounding, rather than meaningful normalized shear.

The existing scale-1,000 arbitrary-axis regression also rejected valid normalized
TRS solely for this absolute residual. The new acceptance regression replaces
that old policy assertion. Ten of eleven initial acceptance rows fail before the
runtime change; the scale-one control passes (`artifacts/issue029-red.log`).

## Contract and smallest shared change

`Decompose` retains its existing affine, nonzero/representable magnitude,
normalized orthogonality, handedness and quaternion-reconstruction gates.
Reflections still canonicalize their one recoverable negative scale to X.

For an original coefficient `a`, extracted signed row scale `s`, reconstructed
normalized coefficient `r` and raw Q32.32 unit `q = 1 / 2^32`, let
`b = round(a / s)`. The existing normalized check establishes
`abs(r - b) <= Fixed64.Epsilon`. Nearest-even division contributes at most
`q/2`, and final multiplication contributes at most another `q/2`, so:

```text
abs(round(s * r) - a) <= abs(s) * (Fixed64.Epsilon + q/2) + q/2
```

The bound belongs to each row's scale. A large X scale cannot hide Y-row shear.
Reflection preserves the proof because nearest-even rounding is symmetric under
sign reversal. Translation is copied exactly, and the affine column is fixed.
No additional residual comparison is needed: the normalized gate and rounding
already prove the bound. A duplicate runtime check would add no rejection role.

The final gate instead requires representable scaled reconstruction. The existing
nine checked products in `FixedTransform.TryGetStrictLocalMatrix` move unchanged
to an internal `Fixed4x4.TryCreateTransform` owner, shared by decomposition and
transform materialization. This reuses `Fixed64.TryMultiplyAdd` with zero addends,
preserving product rounding while rejecting saturation. There are no public API,
serialization, dependency or GC-setting changes.

Every decomposition failure still returns zero translation, identity rotation
and unit scale. Matrix import returns null and false on failure. The strict
construction helper retains its existing zero-matrix failure output.

## Boundary evidence and callers

An affine matrix with raw basis rows below reaches the new final rejection gate:

```text
X = (0, long.MaxValue, 0)
Y = (-4265141137, 3, 505287218)
Z = (505287218, -6, 4265141136)
```

Its magnitudes and normalized checks pass, but quaternion reconstruction produces
`M12 = One + 1 raw`. Multiplication by the maximum X scale is unrepresentable.
Tests retain this case and its X-reflected counterpart, asserting neutral outputs
and null import. This prevents saturation from masquerading as success under
the revised error contract.

`FixedTransform.TryCreateFromLocalMatrix` and Gravitas's ordinary, hierarchy and
planar collider-scale capture paths benefit through the shared decomposition.
Planar embedding and physical positivity checks are unchanged. Downstream tests
initialize actual sphere/circle colliders of radius `1/2048` under rotated
scale-2,048 transforms and identity-child hierarchies, verifying one-unit physical
radii and successful registration. Gravitas runtime code needs no change.

`TrySetParentKeepingWorld` retains its separate absolute-epsilon comparison of
the complete recomposed world matrix. A newly admissible large-scale import can
still fail that stronger world-preservation contract atomically. No world-pose,
inverse or hierarchy policy is relaxed as part of this decomposition fix.

Focused regressions also cover nonuniform scale, billion-unit rows, fractional
rows, reflections, planar parity, nonsheared hierarchy composition, normalized
shear just beyond epsilon on each row, and warmed zero allocations. The coefficient
bound is checked independently with `BigInteger`, without saturating test
arithmetic or substituting the largest scale for an individual row.

## Validation

All development and validation use `UseLocalLsfStack=true`, serial source builds
and the existing coverage settings with no exclusions. The three runtime files
are frozen in `artifacts/issue029-final-source.json`. Focused captures pass 247
FixedMathSharp tests and 26 Gravitas tests; the changed decomposition, shared
construction and transform-forwarding methods have 100% line/branch coverage.

| Repository / configuration | Passed tests | Reachable lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 5,861 | 59,890 / 59,890 | 17,100 / 17,100 | 4,552 / 4,552 |
| FixedMathSharp ReleaseLean | 5,840 | 59,983 / 59,983 | 17,100 / 17,100 | 4,548 / 4,548 |
| Gravitas Release | 6,030 | 61,041 / 61,041 | 20,144 / 20,144 | 6,042 / 6,042 |
| Gravitas ReleaseLean | 5,964 | 61,041 / 61,041 | 20,144 / 20,144 | 6,042 / 6,042 |

Every count equals its total: 100% reachable line, branch and fully covered
method coverage in all four configurations. There are zero failed/skipped tests.
Both FixedMathSharp configurations also pass all 49 Chronicler tests. All four
solution builds, including netstandard2.1/net8.0 targets, have zero warnings and
errors. The runtime hashes match after the matrix. Evidence prefix:
`artifacts/issue029-` in each repository, plus the FixedMathSharp aggregate
`artifacts/issue029-validation-output.log`.

## Throughput and allocations

The existing `Matrix4x4Benchmarks.Decompose` fixture is unchanged: 256 matrices
per invocation, with deterministic ordinary rotations and nonuniform scales.
Matched full Release captures use five warmups, 15 measurement iterations of
500 ms, two launches and affinity mask 3. BenchmarkDotNet 0.15.8 reports:

| Capture | Mean per 256-matrix batch | Error | Mean per decomposition |
| --- | ---: | ---: | ---: |
| Before (`issue029-baseline`) | 310.332 us | 3.159 us | 1.212 us |
| After (`issue029-after`) | 305.118 us | 1.021 us | 1.192 us |

Error is half the 99.9% confidence interval. This is a correctness fix with no
observed throughput regression; the modest timing difference is not a new
performance guarantee. All four measured child launches exit zero, with exactly
zero raw allocated bytes and GC collections; exported summaries are also 0 B/op.
The focused mixed-scale allocation guard independently reports zero warmed
calling-thread bytes.

Capture commands, after the normal benchmark Release build with local sources:

```powershell
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix4x4 --filter '*Decompose' --warmupCount 5 --iterationCount 15 --iterationTime 500 --launchCount 2 --affinity 3 --exporters json --artifacts artifacts/issue029-baseline
```

Repeat with `artifacts/issue029-after` after applying the runtime change. Logs
are adjacent to the artifact directories.

## Final checks and review

The out-of-process Short smoke completes all 479 children with successful exits,
479 populated statistical records and 479 raw GC rows. The only 13 positive raw
allocation rows and exported summaries are the intentional enumerable,
formatting and serialization controls. There are no additional raw allocation
observations in this capture. The 10 ms smoke is validation, not throughput
measurement evidence. Captures: `artifacts/issue029-smoke` and adjacent log.

Both DocFX builds pass with warnings treated as errors. API local links, icons
and repository actions pass in both repositories, along with all four existing
wiki-link rewrite tests. Evidence: `artifacts/issue029-checks-output.log`,
`artifacts/issue029-checks-run.log`, `artifacts/issue029-wiki-checks.log` and each
repository's `artifacts/issue029-docfx.log`. Whitespace checks pass, and all three
runtime hashes still match after final checks.

Independent numerical, connected-code, documentation and evidence reviews find
no actionable issue. The Gravitas supported-range plan now records this upstream
prerequisite as resolved; its physical ceiling/admission work remains separate.
No benchmark test suite or new runtime abstraction is introduced. Validation is
on Windows with local sibling sources; Linux and exact released-package checks
remain release gates.

## Reproduction

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1 --filter 'FullyQualifiedName~MatrixScaleContractTests|FullyQualifiedName~FixedTransformTests'
```

From the Gravitas sibling root, run its existing collider-admission tests with
the same source-selection properties and filter
`FullyQualifiedName~ColliderExactScaleAdmissionTests`. Release and ReleaseLean
solution validation in both repositories remains the completion gate, followed
by exact released-package validation before releasing Gravitas.
