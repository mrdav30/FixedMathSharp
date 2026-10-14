# FMS-Issue-030: Polygon Sweep Normals And Witnesses

Status: Complete.

## Reproduction And Cause

The source baseline is FixedMathSharp `eae5775`, with unchanged Gravitas
`10cf5e8`. Use polygon origin `(7,-13)` and local offsets
`[(1.5,0),(2.5,0),(2.5,1.5),(1.5,2.5)]`. Sweep a circle from `(5,-13)`,
radius `1/2`, direction Right, maximum distance `8`. The first distance is
correctly `3`, but the old result reports normal Right and target offset
`(2.5,0)`. The geometric normal is Left and the target offset is `(1.5,0)`:
the impact center is `(8,-13)` and the contact point is `(8.5,-13)`.

Both circle/capsule and convex-polygon reducers oriented candidate normals using
the source position relative to the target's authored origin. Neither polygon
origin must lie inside its geometry. Rebasing an origin while subtracting the
same local shift from its offsets could therefore flip the normal, change
equal-distance candidate ranking and reconstruct the opposite support witness.
The convex mover's authored origin was an additional invalid sign reference.

## Retained Fix

Remove both origin-based reorientation blocks and their unused wide helper.
Positive-radius sweeps negate the finite-axis source-to-target normal to obtain
the polygon query's target-to-source normal. Preserve that geometric sign at
tangency: rounded normalized components can have a small motion dot even when
the exact geometric dot is zero.

Zero-radius sweeps require whole-shape entry axes. Rounded distance can move a
point beyond a face; at an apex, orienting an arbitrary edge normal against
motion can still select the wrong support vertex. Recover the entering support
plane from exact initial gaps and closing velocities after the existing
finite-axis kernel admits a hit. Compare gap/velocity ratios by wide cross
products, preserving stable first-axis ties. This adds no distance solver or
root iteration. Polygon/core recovery is linear in target edges; convex/convex
recovery retains the existing quadratic feature-pair complexity.

Keep the exact axis for support selection and normalize only the public normal.
Project source support onto the finite target feature at the exact rational
entry event, rounding once into the authored local coordinates. Reconstructing
from the rounded public distance instead introduces a one-raw witness error in
a tiny diagonal ray fixture. Doubled core coordinates preserve odd-raw half-axis
endpoints. Existing wide arithmetic, support features and raw-ratio rounding own
these operations; there are no new numeric types, allocations or dependencies.

The convex caller excludes initial contact before admission, so a strict entry
axis exists. Its old witness fallback and candidate-normal tie ranking are
removed. The centered-axis helper retains its initial/degenerate fallback.
Positive-radius witness translation and all public APIs remain unchanged.

The positive-radius split follows the existing travel contract. Normalized
public motion moves less than one spatial raw increment when distance rounds
by half an increment; the smallest positive radius is one increment. Admitted
authored chords use a positive rounded physical length, so their output error
likewise cannot move the core across a positive-radius contact. Whole-shape
initial overlap remains owned by the existing static contact/containment paths.
The separate zero-area precheck defect is tracked as
[FMS-Issue-031](../issue-tracker.md#fms-issue-031-zero-area-convex-polygons-report-false-containment-and-initial-overlap).

## Regression Evidence

The literal regression and its matrix fail before the fix. An initial focused
run records 19 failures and 13 passes in `artifacts/issue030-red.log`.
The intermediate adapter experiments expose two necessary distinctions:

- Preserving a motion sign test for positive radius fails the existing authored
  Pythagorean tangencies: `(3,4,5)`, `(5,12,13)` and `(7,24,25)`, selecting
  `(100,-100)` instead of the vertex `(0,0)`.
- Negating every normal fails a zero-radius diagonal crossing. For polygon
  `[(1,-100),(3,-100),(3,100),(1,100)]`, start zero, direction
  `(FromFraction(3,5),FromFraction(4,5))`, maximum distance `3`, entry rounds
  upward to raw `7158278826`. The reconstructed point lies beyond X one, but
  the entry normal must still be Left. The minimum positive-radius neighbor
  instead enters at raw `7158278824` and retains its geometric Left normal.

The focused regression set covers:

- The literal circle, positive-core capsule, reverse motion, both windings,
  quarter-turn frames and equivalent target-origin rebasing.
- Convex movers with independently rebased source and target origins, checking
  distance, normal, witness validity and the same resolved world witness.
- Exact vertex tangency in both travel directions, with circle/capsule parity
  and translated/rotated authored frames.
- Diagonal authored tangencies with independent analytic normal components and
  rebased target origins; zero-radius and minimum-radius rounded crossings.
- Scalar-limit entry with an unmaterializable world anchor, and restored full
  normal/witness comparison between clipped triangles and their convex polygon.
- Apex grazing across cyclic vertex orders, winding reversal and independently
  rebased frames, checking that the normal supports the actual apex.
- Exact oblique face support, tiny exact-event witnesses, finite edge clamps,
  repeated vertices, collapsed finite targets and odd-raw centered cores.
- Warmed zero-allocation checks for ray, zero-radius core, circle and convex
  queries; the callback consumes successful results.

Apex and tiny-witness counterexamples failed before exact recovery. Earlier
failed adapter experiments remain diagnostic evidence, not final validation.
Collapsed-target recovery tests use finite-feature admission explicitly: the
separate public containment defect in issue 031 is not asserted to be fixed.

## Validation

Validation uses `UseLocalLsfStack=true`, serial source builds and existing
coverage settings without new exclusions. Final evidence uses the prefix
`artifacts/issue030-exact-` in each repository; runtime source hashes are retained
in `artifacts/issue030-exact-final-source.json`.

| Repository / configuration | Passed core tests | Reachable lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 5,802 | 59,863 / 59,863 | 17,072 / 17,072 | 4,549 / 4,549 |
| FixedMathSharp ReleaseLean | 5,781 | 59,956 / 59,956 | 17,072 / 17,072 | 4,545 / 4,545 |
| Gravitas Release | 6,026 | 61,041 / 61,041 | 20,144 / 20,144 | 6,042 / 6,042 |
| Gravitas ReleaseLean | 5,960 | 61,041 / 61,041 | 20,144 / 20,144 | 6,042 / 6,042 |

Both FixedMathSharp configurations also pass all 49 Chronicler tests. There are
zero failed or skipped tests and all four solution builds report zero warnings
or errors. The final source checkpoint matches after the matrix and final checks.
The focused final capture passes all 128 selected tests, including every line and
branch of all nine new recovery methods and the warmed zero-allocation guard.

The Release out-of-process Short benchmark smoke completes all 479 processes
with exit code zero and all 479 statistical records. All 479 raw GC rows agree
with exported memory records: 13 intentional allocating controls (one enumerable,
four formatting and eight serialization cases), with no new allocating case or
zero-measurement warning. This smoke uses 10 ms iterations and is validation,
not a new throughput claim. Evidence is in `artifacts/issue030-exact-smoke` and
`artifacts/issue030-exact-smoke.log`.

Both DocFX builds pass with warnings treated as errors. API local links, assets
and repository actions pass for both repositories, as do the four existing wiki
link-rewrite tests and `git diff --check`. No Gravitas source changes are needed.

Independent connected-code and evidence reviews found no actionable issue,
including the rounded tangent, zero-radius crossing, exact-event projection
widths, retired helper callers and documented coverage counts. Released-package validation remains a future release
gate; this investigation runs on Windows with the local source stack.

## Reproduction Commands

From the FixedMathSharp repository root:

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1 --filter 'FullyQualifiedName~SweepContracts|FullyQualifiedName~CenteredCapsuleSweepRelations|FullyQualifiedName~AuthoredChordSweepRelations|FullyQualifiedName~FixedConvex2dRelationsTests|FullyQualifiedName~FixedTriangleProjectedCircleSweepTests'
```

For full validation, use the solution Release and ReleaseLean build/test
workflows and the core project's existing `coverlet.runsettings`. Retain
`-p:UseLocalLsfStack=true` through restore, build and test in both repositories.
No coverage exclusions, benchmark-launcher tests or new test projects are added.
