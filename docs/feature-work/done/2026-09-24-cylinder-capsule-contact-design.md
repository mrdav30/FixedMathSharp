# Complete Cylinder/Capsule Contact Design

**Status:** Complete — verified 2026-09-25, with the accepted correctness trade-off and focused optimizations  
**Issue:** FMS-Issue-023 (resolved; original reproductions preserved below)  
**Scope:** FixedMathSharp's existing centered finite cylinder/capsule contact

## Intended Outcome

Make `FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact` fulfill its
existing minimum-translation contract for all admitted inputs. This includes
correct separation, exact touch, penetration, direction, rounded depth, and
overflow reporting. Preserve deterministic lockstep behavior, zero managed
allocation, both package configurations, and `netstandard2.1`/`net8.0` support.

This is a private geometry repair, not a new public collision API. Gravitas
continues to own collision-response policy; FixedMathSharp owns the geometric
answer. Full reachable line/branch/method coverage remains an acceptance gate.

## Why The Current Approach Cannot Be Extended With One More Axis

The current owner tests cylinder/capsule axes, their cross product, the center
difference, and a closest-core direction. These are useful candidate directions
but not a complete description of curved finite surfaces. The preserved tests
demonstrate both false contact and incorrect positive depth.

The repair must cover two situations:

- When the capsule core is outside the cylinder, depth is capsule radius minus
  the shortest core-to-cylinder distance.
- When the core intersects the cylinder, that distance is zero, but moving by
  the capsule radius is not generally enough. Depth also includes the distance
  needed to escape the underlying cylinder/core configuration.

An exact overlap predicate followed by the old contact output would fix only
some classification failures. An outside-only distance solver would still fail
the captured core-overlap upper-bound test. Neither closes FMS-Issue-023.

## Approach Selection

### Preserved Original Failure Evidence

The original defect was confirmed against `b7a6b02` and expanded against
`5ddf2c5`; the implementation began at `f8cc274`. Its public reproductions remain
in `FixedSegmentStrictClearanceRegressionTests` and
`CenteredCylinderCapsuleRimContactTests`, not only in disposable captures:

- Cylinder center zero, axis +Y, length 2, radius 10; capsule center
  `(20.75,1.75,0)`, axis +X, core length 20, radius 1. The nearest endpoint
  `(10.75,1.75,0)` is squared distance `9/8 > 1` from the rim, but the old
  contact returned true. At center `(20.75,2,0)` and radius `5/4`, it returned
  depth `1/4` for exact touch and still admitted a one-raw outward Y offset.
- Oblique cylinder radius 1 and length 2; capsule center `(4,5,0)`, core
  length 2, quaternion raw components `607400100*(5,0,-3,4)`. Its exact +Y
  axis is `(12,-9,20)/25`. Rim point `(1,1,0)` and normal `(3,4,0)/5` prove
  core distance 5. Radius `5-1 raw` must miss, radius 5 touches, and radius
  `5+1 raw` has depth 1 raw. The old method admitted all three with roughly
  `0.02118` excess depth.
- Intersecting core: unit-radius, length-2 cylinder; capsule center
  `(7/4,0,1/4)`, core length 10, radius `1/4`, quaternion raw components
  `1920767767*(0,0,-1,2)` giving axis `(4/5,3/5,0)`. Translation `3/5` along
  `(3,-4,0)/5` already separates the shapes, but the old depth raw
  `3096962338` (about `0.72107`) exceeded that upper bound. This excludes an
  outside-only distance repair.

The pre-implementation expanded suite had 12 issue-specific failures and the
three separate FMS-Issue-024 failures in each package configuration. Coverage
was already 100%; executed code was not proof of correct contact geometry.

### Selected Approach

| Approach | Trade-off | Decision |
| --- | --- | --- |
| Add selected rim directions or an exact Boolean gate | Small change, but incomplete depth/direction and containment handling | Reject as the final repair |
| Introduce a general iterative convex-contact system | Broader shape reuse, but brings new convergence, rounding, ownership, and performance questions | Outside this issue's scope |
| Complete the cylinder/capsule feature solver | More focused exact arithmetic, with explicit geometric completeness and bounded work | Recommended |

Reuse existing rigid-frame, fixed-limb, polynomial, and output machinery.
Introduce only the missing private operations actually consumed by this pair.
Do not build a public algebraic-number API or a generic symbolic-math framework.

### Implementation Record

- Starting revision: `f8cc274` on `develop`. The owner staged the reviewed
  repair before the focused follow-up; follow-up edits remain unstaged and
  no commit is made by this work.
- Reproduced the committed regression set before runtime changes: 12 failed,
  3 passed, no skips (`artifacts/fms023-implementation-red.log`).
- Freeze matched public-query benchmarks, including endpoint rims, oblique
  interior rims, overlapping cores, zero cylinder radius and zero capsule-core
  length. Historical incorrect results are cost baselines only.
- Implement and verify retained-root sign/equality mechanics alongside the
  complete feature solver; their shared interface must preserve exact feature
  ownership and cross-candidate ordering before output rounding.
- Integrate the complete solver, remove superseded pair-only machinery, then
  run focused/full standard and Lean tests, exact coverage, downstream Gravitas
  checks, matched performance captures and an independent review.
- Execution decision: the approved design is the authority; no additional
  approval checkpoint or automatic commits are introduced by workflow tooling.
- Implementation refinement: reflection symmetry reduces the capsule-interior
  ellipse family to at most one interior minimum. Compare that retained root
  directly with analytic candidates; no cross-quartic resultant subsystem is
  needed. The source records the derivative and degenerate-family proofs.
- First integrated gate: 76/76 focused geometry/root tests passed; the full
  Release suite passed 3,197 tests with only the three pre-existing FMS-Issue-024
  cylinder/cylinder failures. These are intermediate, not final acceptance.
- First matched implementation timing exposed unacceptable cost: ordinary
  parallel contact 42.21 us versus 28.06 us; oblique interior rim 138.46 ms and
  overlapping-core 346.62 ms. Full-domain cancellation and irreducible-wide
  contacts improved, but those wins do not excuse the new costs. Evidence is
  in `artifacts/fms023-expanded-baseline` and
  `artifacts/fms023-first-complete-benchmark`; common-factor reduction and
  proven analytic fast paths are being evaluated before acceptance.
- The resumed verification uses one build/test/benchmark process tree at a
  time, two-core affinity, `DOTNET_PROCESSOR_COUNT=2`, single-worker MSBuild
  and xUnit, and below-normal launcher priority. An interrupted benchmark
  capture is not accepted as evidence; the final comparison repeats both
  revisions under the same resource-limited settings.
- Exact common-factor reduction, retained root-cell reuse, and certified
  analytic search bounds remove avoidable work without approximation. The
  bounded curved-case capture (`artifacts/fms023-resume-curved-timing`;
  6 warmups, 15 measured iterations, affinity mask 3) reports 1.356 ms for
  the oblique interior rim and 1.103 ms for overlapping cores, both 0 B/op.
  These results are not a matched-environment speedup claim against the
  unrestricted historical baseline; corrected curved cases remain costlier.
- Final Release verification after cutover cleanup: 3,259 passed, three
  existing FMS-Issue-024 failures, no skips; 49,975/49,975 lines,
  9,984/9,984 branches, and 3,627/3,627 methods covered. Evidence:
  `artifacts/fms023-cutover-Release.log` and its coverage directory.
- Final ReleaseLean verification: 3,238 passed, the same three FMS-Issue-024
  failures, no skips; 50,068/50,068 lines, 9,984/9,984 branches, and
  3,623/3,623 methods covered (`artifacts/fms023-cutover-ReleaseLean.log`).
- Both complete solution builds succeed with zero warnings/errors, including
  `netstandard2.1` and `net8.0`. The separate Chronicler integration suite
  passes 49/49 in each configuration. Build logs are
  `artifacts/fms023-cutover-build-Release.log` and
  `artifacts/fms023-cutover-build-ReleaseLean.log`; integration logs are
  `artifacts/fms023-chronicler-Release.log` and
  `artifacts/fms023-chronicler-ReleaseLean.log`.
- Independent peer passes approved geometric completeness and root/candidate
  arithmetic. The root author did not provide its independent approval;
  a different reviewer checked that owner. The only actionable review item
  was aggregate stack-headroom documentation, now included in the complexity
  register, public XML and geometry guide. No runtime/JIT total-stack
  measurement or cross-platform execution is claimed.
- Downstream source-stack coverage: Gravitas Release passed 4,274 tests;
  ReleaseLean passed 4,215. Each retained exactly one known GRV-Issue-082
  failure (`CylinderContact_ShouldFindTriangleIntrusionAwayFromCenterClosestPoint`)
  and no skips. Coverage remains 100%: Release 44,687 lines / 13,270 branches /
  4,570 methods; Lean 44,685 lines / 13,270 branches / 4,569 methods.
  Logs and reports are in Gravitas's `artifacts/fms023-contact-release` and
  `artifacts/fms023-contact-lean` captures. No Gravitas source change was needed.
- Final cutover cleanup removes the now-redundant internal forwarder together
  with its emptied file. The existing internal entry now owns the complete
  feature solver directly; the shared magnitude helper lives beside the other
  arithmetic helpers. The final Standard/Lean builds and coverage above include
  this behavior-preserving cleanup.
- After that final cleanup, rebuild the Gravitas source graph and repeat its
  focused `FullyQualifiedName~CylinderCapsule` tests: 4/4 pass in Release and
  4/4 in ReleaseLean (`artifacts/fms023-cutover-contacts-Release.log` and
  `artifacts/fms023-cutover-contacts-ReleaseLean.log` in Gravitas).
- The API site builds successfully with `dotnet tool run docfx
  docs/api/docfx.json --warningsAsErrors`: zero warnings and errors
  (`artifacts/fms023-docfx.log`). Generated site output remains ignored.

### Initial Complete-Solver Matched Performance Capture

On 2026-09-25, the frozen eight-row fixture was repeated against runtime revision
`f8cc274` and the complete solver before the focused follow-up below. The
historical runtime was built from an ignored `git archive` copy, with only the
frozen benchmark source substituted;
the checkout and branch were not changed. Both benchmark source files have
SHA-256 `F335A1C4D6A2B678D01F5A57DC44280E859AF93BF80659CBFB101F22E33AB98A`.

Environment: BenchmarkDotNet 0.15.8, Windows 11, i7-9700K, .NET SDK 10.0.302,
.NET 8.0.29, Release, out-of-process, one launch per row, six warmups and
15 measured iterations. Both runs used two-core affinity mask 3 (displayed
as binary `11` by BenchmarkDotNet), `DOTNET_PROCESSOR_COUNT=2`, and the same
High Performance power plan. Launches and builds were serialized; the
launcher ran below normal priority. These are individual contact-query costs,
not physics-frame measurements or worst-case latency bounds.

All values below are microseconds. Error is the half-width of BenchmarkDotNet's
99.9% confidence interval; SD is standard deviation. Every row reports 0 B/op
before and after.

| Fixture | Before mean ± error (SD) | After mean ± error (SD) | Interpretation |
| --- | ---: | ---: | --- |
| Ordinary | 29.245 ± 0.710 (0.664) | 21.809 ± 0.355 (0.332) | 25.4% lower cost |
| Irreducible wide | 1,480.531 ± 27.013 (25.268) | 740.444 ± 13.331 (12.470) | 50.0% lower fixture cost |
| Full-domain cancellation | 190.965 ± 3.928 (3.674) | 80.206 ± 1.387 (1.297) | 58.0% lower cost |
| Zero cylinder radius | 28.616 ± 0.612 (0.573) | 34.539 ± 0.587 (0.549) | 20.7% higher cost, +5.923 us |
| Endpoint rim | 45.625 ± 1.013 (0.948) | 91.402 ± 1.672 (1.564) | Old contact depth was incorrect |
| Interior oblique rim | 85.851 ± 1.676 (1.568) | 1,357.427 ± 37.201 (34.798) | Old contact depth was incorrect |
| Intersecting core | 97.222 ± 1.864 (1.744) | 1,096.508 ± 28.520 (26.678) | Old contact depth was incorrect |
| Zero capsule-core length | 45.103 ± 0.891 (0.833) | 73.687 ± 1.348 (1.261) | Old rim-contact depth was incorrect |

The ordinary baseline has a multimodal-distribution warning; its mean/error
are retained rather than selecting a favorable subset. The common-case
improvement is larger than the observed spread. The final curved-case costs
also agree with the earlier resource-limited diagnostic capture. Historical
incorrect-contact rows remain cost-only comparisons, not equivalent-answer
speed comparisons.

The zero-radius row crosses the 5% review threshold and must not be hidden.
Source inspection identifies the complete finite-segment feature evaluation
(both endpoint families and dual edge directions) that replaces the incomplete
selected-direction set. Its 5.923 us increase and the roughly 1.1–1.36 ms
curved-case costs are explicit acceptance trade-offs. No claim is made that
these costs cannot be reduced further. The complete solver remains the only
authority; no old-solver fallback, approximation budget, or tolerance was added.

Captures: `artifacts/fms023-controlled-old-valid` and
`artifacts/fms023-controlled-new-final`, with sibling `.log` files. Each completed
all eight rows with exit code zero. The earlier interrupted capture and the
duplicate-project-discovery failures contain no valid final comparison and
are excluded. BenchmarkDotNet searches the solution tree for matching projects;
after capturing the archived baseline, its benchmark `.csproj` was renamed to
`.csproj.baseline` to keep it out of the live runner's project discovery.

To reproduce, build each runtime with the same frozen fixture and run from its
benchmark project directory, selecting the eight rows ending in
`CylinderCapsule`:

```powershell
dotnet bin/Release/net8.0/FixedMathSharp.Benchmarks.dll rigid-finite-shape-relation --filter '*CylinderCapsule' --affinity 3 --warmupCount 6 --iterationCount 15 --exporters json
```

Apply the environment/serialization limits above to both revisions. Keep
archived benchmark projects outside the active solution's discovery tree or
disable their `.csproj` extension between runs. Do not mix these restricted
captures with unrestricted historical timings to calculate an improvement.

### Focused Follow-Up: Reuse Exact Work Before Adding Machinery

The owner accepted the correctness/performance trade-off and staged the repair,
then requested one focused optimization pass before committing: oblique interior
rims and intersecting cores first, followed by a brief endpoint-rim and
zero-core-length check. The staged **correct** implementation above is the
baseline for this pass, not the historical solver's incorrect contacts. The
benchmark source and its SHA-256 remain unchanged.

An EventPipe diagnostic identified repeated normal-polynomial construction as
roughly 58% of intersecting-core and 33% of oblique-rim sampled inclusive time.
Exact sign-at-root queries were the second useful oblique-case target. These
are diagnostic attribution samples, not independent end-to-end speed claims
(`artifacts/fms023-focused-profile`). The retained changes are:

- Build each normal component's exact numerator/denominator once, reusing the
  existing depth threshold comparison for each rounding probe.
- Reuse an already-proved exact touch instead of searching for zero depth again.
  Normal and support-anchor construction still run.
- Bound ellipse depth using the admitted major-axis candidate: a winning
  positive ellipse gap is smaller than the cylinder radius. A bound exceeding
  `Fixed64.MaxValue` still requires an exact depth comparison before clamping.
- For a linear query whose interval sign is unresolved, evaluate the defining
  polynomial at the query's exact rational zero. This proves equality or orders
  a crossing root without repeated refinement. Noncrossing unresolved cases
  retain the existing exact fallback; no tolerance or approximate root is used.

The complete eight-row out-of-process capture repeats the same environment,
two-core limits, six warmups and 15 measured iterations used above. Values are
microseconds; error/SD have the same meaning as in the preceding table.

| Fixture | Correct baseline mean | Optimized mean ± error (SD) | Change |
| --- | ---: | ---: | --- |
| Oblique interior rim | 1,357.427 | 610.871 ± 12.257 (11.465) | 55.0% lower cost |
| Intersecting core | 1,096.508 | 465.117 ± 8.328 (7.790) | 57.6% lower cost |
| Endpoint rim | 91.402 | 71.568 ± 1.609 (1.505) | 21.7% lower cost |
| Zero capsule-core length | 73.687 | 59.709 ± 1.066 (0.998) | 19.0% lower cost |
| Ordinary | 21.809 | 21.629 ± 0.414 (0.387) | Within measured spread |
| Irreducible wide | 740.444 | 742.392 ± 24.119 (22.561) | Within measured spread |
| Full-domain cancellation | 80.206 | 79.896 ± 1.679 (1.571) | Within measured spread |
| Zero cylinder radius | 34.539 | 34.724 ± 0.723 (0.676) | Within measured spread |

Every row remains **0 B/op**. All eight executions completed successfully in
`artifacts/fms023-focused-final` (with sibling `.log`); the zero-core-length row
reports two detected low outliers, retained in its 15 measurements. These are
individual contact-query improvements, not a claim about whole physics frames
or all possible curved geometries. The earlier profiled/short diagnostic runs
are not substituted for this full capture.

An independent reviewer checked the unstaged follow-up separately from the
staged repair, including feature admission, homogeneous width bounds, exact
ties, clamping and nearest-even rounding. No actionable correctness or
over-engineering findings remained. The new root tests use independently known
non-dyadic roots, all multiplicities and leading signs, adjacent rational
thresholds and full-width coefficients; they passed before and after the
shortcut. Public contact tests additionally distinguish exact maximum depth
from its one-raw neighbors.

Final FixedMathSharp verification after the follow-up:

| Configuration | Passed / known failures / skipped | Covered lines | Covered branches | Covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 3,269 / 3 / 0 | 50,011 / 50,011 | 10,006 / 10,006 | 3,629 / 3,629 |
| ReleaseLean | 3,248 / 3 / 0 | 50,104 / 50,104 | 10,006 / 10,006 | 3,625 / 3,625 |

The three failures remain exactly the enabled `FMS-Issue-024` cylinder-pair
regressions; no new failure or coverage exclusion was introduced. The two
solution builds include both runtime targets and finish with zero warnings or
errors; Chronicler passes 49/49 in each configuration. Evidence is in
`artifacts/fms023-focused-final-coverage-Release[Lean]`,
`artifacts/fms023-focused-build-Release[Lean].log`, and
`artifacts/fms023-focused-chronicler-Release[Lean].log`.

The downstream Gravitas source graph was rebuilt and its full suites repeated
serially with the same two-core limit. Release passes 4,274 tests and Lean
passes 4,215, each retaining exactly the one enabled `GRV-Issue-082`
`CylinderContact_ShouldFindTriangleIntrusionAwayFromCenterClosestPoint`
failure and no skips. Both retain 100% coverage: Release 44,687 lines /
13,270 branches / 4,570 methods; Lean 44,685 lines / 13,270 branches /
4,569 methods. Captures are in Gravitas's
`artifacts/fms023-focused-coverage-Release[Lean]` and sibling logs. These are
Windows source-stack checks, not Linux or released-package certification.

The API site also rebuilds with warnings treated as errors: zero warnings and
errors (`artifacts/fms023-focused-docfx.log`). No generated site files are
committed. Independent review and the completed verification above close this
issue; the separate known contact defects remain in their owning trackers.

## One Geometric Authority

Keep one pair solver behind the current public validation boundary:

```text
Validated authored geometry
  -> exact rational frame and feature preparation
  -> complete feature candidates and exact comparison
  -> closed-contact decision and deterministic winner
  -> final normal/depth rounding and existing support anchors
```

An exact separating projection may return false early: one separating plane
is sufficient proof. A positive projection or one admitted candidate cannot
return true with a final depth until omitted feature families are proved unable
to improve it. Early positive returns therefore need a completeness certificate,
not a heuristic classification.

The conceptual objective is a signed support gap. Let:

- `A` be the cylinder's exact authored half-axis vector in world space;
- `B` be the capsule core's exact authored half-axis vector;
- `u` be the cylinder's nonzero exact axis direction;
- `d` point from cylinder center to capsule center;
- `R` and `r` be cylinder and capsule radii;
- `n` be a conceptual unit normal.

Then minimize over all `n`:

```text
gap(n) = |A dot n| + |B dot n|
       + R * sqrt(1 - (u dot n)^2 / (u dot u))
       - d dot n

signed contact depth = r + minimum gap(n)
```

This notation describes the geometric objective, not floating-point runtime
code. The implementation retains rational authored axes and clears denominators;
it must not renormalize endpoints or materialize half-raw lengths prematurely.
Negative signed depth means separated, zero means closed touch, and positive
means penetration. Classification precedes public rounding: a tiny positive
depth may legitimately round to zero without becoming a miss.

## Complete Feature Selection

Partition the possible normal directions at cylinder/capsule axial support
changes. Each family includes its boundary conditions and degenerate cases.

| Contact-feature family | Required treatment |
| --- | --- |
| Cylinder cap normals | The two poles, including flat cap support |
| Cylinder-side directions | Analytic circle candidates, restricted to the applicable capsule-end support |
| Directions perpendicular to both axes | Cross-axis boundaries when nonzero |
| Capsule endpoint against a cylinder rim | Endpoint/rim stationary candidates with exact cap and endpoint ownership |
| Capsule core interior against a rim | Projected cap-disk ellipse candidates, restricted to the valid support region |

For oblique axes, the projected disk is generally an ellipse, not a circle.
This is the missing family that requires quartic stationary-root handling.
Parallel/perpendicular axes, zero cylinder radius, zero capsule radius, and
zero capsule-core length reduce to lower-dimensional cases; they are explicit
parts of the solver, not exception fallbacks. Cylinder length remains positive
under the existing validation contract.

The completeness proof must show that every minimum occurs in one of these
families, that feature restrictions reject roots introduced by squaring, and
that family boundaries and flat families have deterministic ownership. The
feature decomposition is the selected design; its implementation-specific
equations and width bounds must pass review before production cutover.

## Narrow Exact-Root Capability

Existing polynomial helpers count roots, test signs over intervals, or return
rounded ray distances. Contact selection must retain which exact root a
candidate represents until classification, comparison, and rounding are done.

The additional private capability needs to:

1. Identify admissible stationary roots, including repeated roots and reduced
   polynomial degrees, without losing their feature interval.
2. Evaluate exact signs at an identified root, including equality. Reuse
   bounded remainder/root-count arithmetic where its preconditions hold.
3. Compare candidate signed depths, including candidates belonging to different
   roots or feature families. A sign query at one root does not automatically
   solve this cross-candidate comparison; its construction needs its own proof.
4. Resolve final scalar midpoint comparisons and exact ties before narrowing.

Do not refine numerical approximations until they merely appear different:
genuine equal minima and exact half-raw outputs must terminate through equality
logic. Do not return an approximate contact, false miss, or old solver result
after an arbitrary iteration limit.

Coefficient growth, intermediate arithmetic, maximum live workspace, and
operation bounds must be derived for each new operation. Existing Sturm
workspace bounds do not automatically apply to new sign-at-root or
cross-candidate comparisons. A bound that fails review is a design defect to
resolve, not permission to silently truncate or allocate unbounded storage.

Use a fixed canonical feature order for exactly equal minima, then a stable
root order within that feature. Specify that order and the representative for
continuous ties in the implementation plan. Compare conceptual depths first;
two different depths rounding to the same `Fixed64` are not an exact tie.

## Output And Ownership Boundaries

- Keep the public signature, validation, normal orientation, and failure outputs.
- Retain nearest-even depth rounding and distinguish an exact maximum depth
  from a conceptual depth exceeding `Fixed64.MaxValue`.
- Materialize the normal only after selecting the true minimum. Its components
  are representable fixed-point outputs, not classification inputs.
- Preserve the current independent support-anchor convention. Anchors are
  constructed from the returned normal in their authored rigid frames; they
  are not a new promise of paired exact algebraic closest points. In particular,
  normal rounding can choose an endpoint of an otherwise tied capsule core.
- Keep new root/candidate state private and query-local. No serialized state,
  cache, global scratch buffer, new package, or engine dependency.
- Do not alter strict posture predicates or other contact pairs merely to route
  them through the more expensive contact constructor.
- After cutover, remove cylinder/capsule-only dead reducers and diagnostics.
  Preserve arithmetic and candidate helpers still used by other shape pairs.
  No compatibility wrapper or second selectable public solver remains.

Candidate preparation/selection belongs with the existing convex contact owner.
Reusable polynomial mechanics may be extracted from the finite-axis owner only
as far as actual callers require. Keep cohesive logic together; exact filenames
and helper boundaries are implementation details, not a mandate for many tiny
partial files.

## Performance Contract

Correctness is mandatory; speed must be measured rather than inferred from the
algorithm's sophistication. Preserve zero managed allocation and bounded work.

Protect ordinary analytic cases with proved fast paths. Build expensive curved
feature/root state only when cheaper geometry cannot certify the result. Any
candidate pruning requires an exact bound showing it cannot change the winner.
Do not compute the full strict-overlap solver and the full contact solver by
default when the contact calculation already supplies classification.

The pre-change measurements and commands are recorded above. Before
runtime edits, extend/freeze the matched benchmark fixtures to cover endpoint
and interior rims, oblique tangency/miss/penetration, core overlap, degeneracies,
and full-domain cancellation. An old wrong answer is not an equivalent-result
performance baseline; label those rows as correctness-sensitive costs.

The existing irreducible-wide row asserts an internal route. If that route is
removed, change the fixture to measure a stable public behavior and capture a
fresh old-implementation baseline before comparing the replacement.

Provisional review threshold: investigate a reproducible regression above 5%
and outside measurement noise on already-correct matched ordinary cases.
This is a review trigger, not permission to retain an incorrect solver or an
automatic rejection of necessary correctness work. Report difficult-case costs
and allocations separately; any material unavoidable trade-off returns for
review before acceptance. There is no promised speedup or arbitrary absolute
latency target without representative evidence.

## Verification And Acceptance

Use independent geometric expectations, not the new solver to calculate both
actual and expected results. Preserve the current public RED tests and extend
them only with distinct behavioral coverage:

- Endpoint/interior rims, cap/side boundaries, intersecting cores, and enclosure.
- Exact touch and both signs of a one-raw change; half-raw lengths and depths.
- Arbitrary admitted local axes and independent rigid frames; full-range origin
  differences, small axes, and honest output clamping.
- Unique-minimum rigid-transform consistency, axis/quaternion sign equivalence,
  exact ties, repeated roots, flat families, and all admitted degeneracies.
- Exact depth/normal controls and existing support-anchor semantics, without
  substituting anchor-coincidence assertions for that contract.
- Warmed zero-allocation checks and bounded pathological inputs.

Test-only `BigInteger` rational/polynomial oracles may supplement hand-derived
fixtures. Keep runtime code on portable fixed-width arithmetic. Review any
previously passing expectation that changes against an independent geometric
proof; do not simply adopt new output values.

Acceptance requires:

1. All FMS-Issue-023 reproductions pass and no unrelated regression is introduced.
2. Full Release/ReleaseLean suites and 100% reachable line/branch/method coverage;
   no new exclusions, skipped tests, or hollow private-route assertions.
3. Both runtime targets build; existing exact 2D/3D consumers remain correct if
   shared arithmetic changes. No speculative 2D contact API is introduced.
4. Full out-of-process matched benchmarks, including allocation and spread, with
   unexplained regressions resolved or explicitly reviewed.
5. Affected Gravitas contact consumers and source-stack tests are verified in
   both configurations, including full coverage when its consumed internals
   change. This is not released-package certification or a release action.
6. Independent geometry/width/rounding review and an over-engineering review;
   relevant XML/docs/complexity records updated and obsolete code removed.

Known FMS-Issue-024/025 and GRV-Issue-082 defects remain separate work. Record any
still-enabled failures explicitly; do not claim an entirely green release gate
while they remain, even if FMS-Issue-023 itself is resolved.

## Review Decision

The owner accepted the complete solver's measured correctness/performance
trade-off, then requested the focused pre-commit follow-up recorded above.
FMS-Issue-023 is resolved and this plan is complete. Retain the additional
optimizations: they remove repeated exact work and materially reduce both
targeted curved cases without weakening the contact contract. The zero-radius cost remains an
accepted correctness trade-off, not a claimed optimization win. No broader
algebra framework or speculative cache is warranted by this pass.

The broader suites are not entirely green because FMS-Issue-024 and
GRV-Issue-082 remain enabled and failing. Those separate repairs are not folded
into this issue. The owner's staged repair remains intact; only this follow-up's
edits are unstaged. No commit or release action was taken.
