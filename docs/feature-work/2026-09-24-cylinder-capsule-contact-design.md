# Complete Cylinder/Capsule Contact Design

**Status:** Proposed — awaiting design review; not implemented  
**Issue:** [FMS-Issue-023](issue-tracker.md#fms-issue-023-finite-cylindercapsule-contact-misses-rim-separation)  
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

| Approach | Trade-off | Decision |
| --- | --- | --- |
| Add selected rim directions or an exact Boolean gate | Small change, but incomplete depth/direction and containment handling | Reject as the final repair |
| Introduce a general iterative convex-contact system | Broader shape reuse, but brings new convergence, rounding, ownership, and performance questions | Outside this issue's scope |
| Complete the cylinder/capsule feature solver | More focused exact arithmetic, with explicit geometric completeness and bounded work | Recommended |

Reuse existing rigid-frame, fixed-limb, polynomial, and output machinery.
Introduce only the missing private operations actually consumed by this pair.
Do not build a public algebraic-number API or a generic symbolic-math framework.

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

The pre-change measurements and commands remain in the issue tracker. Before
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

Approve or revise this private, complete pair-solver design before producing the
implementation plan. The plan must make feature equations, exact comparison,
termination/width proofs, benchmark fixture parity, and cutover acceptance
concrete. The current issue remains open; diagnosis and this design are not an
implementation or a no-go verdict. Leave all changes unstaged and uncommitted
for review.
