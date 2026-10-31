# FMS-Issue-031: Finite Degenerate Convex Relations

Status: Complete.

## Reproduction And Contract

Baseline FixedMathSharp `122c638`, Gravitas `10cf5e8`. With origin zero and
polygon offsets `[(0,0),(0,0),(2,0)]`, a ray starting at `(-2,0)`, direction
Right and maximum distance four incorrectly reports initial distance zero. The
finite entry is two. A convex mover with offsets `[(-0.5,0),(0,0),(0.5,0)]` at
`(-2,0)` likewise reports zero instead of 1.5. Coincident polygon offsets
incorrectly contain every point in the plane.

Accepted boundary-ordered offsets retain their finite closed convex hull.
Collinear offsets describe a segment; coincident offsets describe a point. The
minimum three-offset validation remains unchanged. These rules now apply
consistently to containment, convex/circle/capsule contacts, minimum
translation, closed upright-capsule intersection and initial sweep admission.

This deliberately corrects the older circle/capsule expectation that every
coincident-point target is rejected. A positive-radius capsule containing that
point has positive separating depth. Exact tangency and zero-depth contact
remain closed hits. Crossing zero-radius segments can have positive minimum
translation, while a point on a segment or collinear overlap has zero depth.
`IntersectsSweptUprightCapsuleStrict` retains its different, explicit polygon
interior contract: every zero-area target has empty interior and returns false.

## Retained Changes And Proofs

- Exact edge determinants still own containment. If all are zero, two existing
  wide coordinate projections close finite extent. For collinear vertices these
  bounds close the segment; for coincident vertices both coordinates must match.
- Static convex contacts retain both original edge-normal passes and their tie
  order. Only when both hulls have zero winding, world X/Y projections complete
  finite longitudinal and point gaps. Nonparallel segment normals already form
  complete SAT; additional valid projections cannot lower the true minimum
  translation below a supporting-facet minimum.
- Support selection extends the complete tied boundary run. If every vertex
  ties, exact tangential extrema retain the complete segment or point. The
  ordinary strict-polygon previous-to-best/best-to-next endpoint order remains.
  Capsule edge features reuse this owner, replacing duplicate adjacent-only
  logic.
- Static point-to-feature projection retains both source and target frames. The
  old identity-frame overload remains needed by unrotated closest-point queries;
  its callers were verified before retaining it.
- Circle/capsule point targets reach the existing core-perpendicular and nearest
  vertex axes. A cardinal fallback exists only when no geometric axis remains,
  as for a zero-length core coincident with a point. Existing exact depth-sign
  comparison handles positive radius, exact touch and zero radius.
- Closed planar core SAT retains both supporting-line directions of a closed
  collinear boundary, plus coordinate axes for finite extent. Existing core
  normals and endpoint-to-edge distance checks handle nonparallel crossings and
  positive radius. Work remains linear in target vertices, with no alternate
  distance solver or point-materialization dependency.

There are no public signature, numeric-width, dependency, serialization or
allocation changes. Public XML and geometry guidance describe the finite-hull
contract. The two old coincident-target rejection tests now assert actual
contact geometry/positive translation rather than blanket rejection.

## Regression Evidence

`artifacts/issue031-red.log` captures all 26 initial regression rows failing
against the committed implementation. `artifacts/issue031-closed-red2.log`
captures eight closed-predicate/point-contact cases failing before closed planar
SAT was corrected. Compile-only authoring errors are not treated as red
evidence.

The final focused capture `artifacts/issue031-focused5.log` passes 582 tests,
including all changed geometry methods and the retained initial-contact recovery
fallback at 100% line and branch coverage:

- Literal finite-entry reproducers; finite gaps, overlaps, endpoint touch and
  one-raw neighbors for points and segments, in both argument orders.
- Repeated/cyclic/winding-varied collinear spans and subdivided support edges.
- Unrotated, quarter-turn and oblique authored frames with retained witnesses.
- Crossing zero-radius segments, boundary touch after minimum exit and
  separation one raw increment beyond it.
- Circle/capsule point targets with positive/zero radius and core length;
  positive translation, exact tangency and one-raw separation.
- Scalar-domain-spanning offsets, unsaturated opposite extremes and valid local
  contact anchors whose conceptual world point cannot be materialized.
- Warmed zero-allocation checks covering containment, convex contact, closed
  intersection, circle/point contact and public ray entry.

The prior ten collapsed-target sweep rows now use public ray, convex and capsule
entry points directly, preserving their distance/normal/witness and
odd-half-axis assertions. They no longer need an internal admission/recovery
seam.

## Validation

The final matrix and checks use `UseLocalLsfStack=true`, serial source builds
and existing coverage settings with no new exclusions. Evidence prefix:
`artifacts/issue031-` in both repositories. The six changed runtime files are
frozen in `artifacts/issue031-final-source.json`.

The first full FMS captures pass every test and cover every line/method, but
miss one existing branch: zero-radius entry recovery's no-strict-gap fallback.
Public initial-contact checks now correctly bypass that path. It remains
necessary for the directly callable shared-core contract. Gravitas service paths
generally perform their own initial-contact prechecks. Four boundary-start
regressions cover unit-direction and physical authored-chord modes in
unrotated/quarter-turn frames. No runtime code is changed for this coverage
repair.

Fresh complete FMS runs use prefix `artifacts/issue031-final-`; the earlier FMS
captures are diagnostic evidence only, without merging. Gravitas validates the
same frozen runtime under `artifacts/issue031-`. Benchmark smoke and both
documentation builds also use `artifacts/issue031-`.

| Repository / configuration | Passed tests | Reachable lines |        Branches | Fully covered methods |
| -------------------------- | -----------: | --------------: | --------------: | --------------------: |
| FixedMathSharp Release     |        5,843 | 59,890 / 59,890 | 17,100 / 17,100 |         4,551 / 4,551 |
| FixedMathSharp ReleaseLean |        5,822 | 59,983 / 59,983 | 17,100 / 17,100 |         4,547 / 4,547 |
| Gravitas Release           |        6,026 | 61,041 / 61,041 | 20,144 / 20,144 |         6,042 / 6,042 |
| Gravitas ReleaseLean       |        5,960 | 61,041 / 61,041 | 20,144 / 20,144 |         6,042 / 6,042 |

All four configurations have zero failed/skipped tests and warning-free builds.
Both FixedMathSharp configurations also pass all 49 Chronicler tests. Every
covered line, branch and fully covered method count equals its total. All six
runtime hashes match after validation and final checks.

The Release out-of-process Short smoke passes all 479 benchmark processes and
retains 479 populated statistical records. All 479 raw GC rows are present; 13
exported allocating cases remain the intentional enumerable, formatting and
serialization controls. The launcher correctly reports one additional raw
336-byte / 864-operation process-counter discrepancy in the unchanged 3D
`Capsule3DIntersectionInterval` scale-100,000 case, whose summary rounds to
zero. This 10 ms smoke is validation rather than throughput evidence.

A three-launch repeat of the eight matching capsule-interval cases passes all 24
processes. One concurrent-GC child again reports 336 bytes (784 operations); all
exported summaries remain zero. The same control with `COMPlus_gcConcurrent=0`
passes 24 processes with all 24 raw GC rows exactly zero. These process-wide
observations remain unattributed. The GC-mode control does not establish their
cause or exclude asynchronous finalizer allocations. Earlier
[process-wide diagnostics](2026-09-25-cylinder-pair-contact-design.md#benchmark-diagnostic-hardening-2026-10-01)
matched a separate fresh 336-byte interval to actual finalizer-thread
allocations; that finding does not attribute this capture. Issue 019's
per-thread accounting artifact is a different mechanism. The changed 2D queries
independently pass their warmed zero-allocation guards. No persisted runtime,
benchmark or test-host GC configuration is changed in this fix. Evidence:
`artifacts/issue031-counter-control` and
`artifacts/issue031-counter-blocking-control`, with adjacent `.log` files.

Both DocFX builds pass with warnings treated as errors. API local links, icons
and repository actions pass in both repositories, as do the four existing wiki
link-rewrite tests and `git diff --check`. Aggregate check output is retained in
`artifacts/issue031-checks-output.log`; the native wiki-test output is saved
separately in `artifacts/issue031-wiki-checks.log`. No Gravitas source changes
are needed.

Independent connected-code review found no actionable issue and verified the
actual Gravitas callers, support ownership, retained forwarding overloads,
unchanged signatures and existing wide bounds. Windows local-source validation
is separate from Linux and exact released-package release gates.

## Reproduction

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1 --filter 'FullyQualifiedName~FixedConvex2dDegenerateRelationsTests'
```

Use the normal Release and ReleaseLean solution builds/tests plus existing core
`coverlet.runsettings` for full validation in both repositories. Retain explicit
`UseLocalLsfStack=true` through restore, build and test. No benchmark-launcher
unit tests or new test projects are added.
