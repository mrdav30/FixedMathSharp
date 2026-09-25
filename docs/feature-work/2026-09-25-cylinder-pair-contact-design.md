# FMS-Issue-024: Complete finite-cylinder pair contacts

## Status and intent

Design for review, 2026-09-25. The owner approved the complete-contact direction
and explicitly requested reuse, restructuring where useful, and removal of
superseded code. No runtime repair has been implemented by this document.

Repair `FixedSegment.TryGetCenteredFiniteCylindersContact` without changing its
public signature. Closed separation/tangency, minimum translation depth,
normal selection, rounding and clamping must agree with the exact authored
finite solids. Preserve the separate inexpensive strict-overlap path.

This is a focused geometry repair, not a general collision framework or a new
public circle-distance API. The implementation plan follows review of this
design. Mathematical completeness and resource proofs below are prerequisites
to retaining a new runtime solver, not claims already established.

## Evidence and ownership

At FixedMathSharp `6368582`, the issue's five enabled regression cases produce
three failures and two passes in Release. Another 75 strict-classification,
rigid-frame and rounding controls pass. The contact reducer tests the two
cylinder axes, their cross product and a closest-centerline direction. Exact
arithmetic on these directions does not make that finite set complete.

- Separated rims: height 2, radius 1, first center zero/axis +Y, second center
  `(7/4,7/4,11/8)`/axis +X. Contact incorrectly returns true.
- Exact rim touch: radii `5/4`, second Z coordinate 2. The unique common point
  is `(3/4,1,1)`, but reported depth is `0.09497214644216001`. A one-raw outward
  Z displacement is incorrectly admitted as well.
- Incorrect positive depth: radii 1 and second Z coordinate `5/4`. The old
  depth is raw `597275436`, about `0.139064`. Direction `(1,1,1)` has normalized
  support overlap `(2*sqrt(2)-11/4)/sqrt(3)`, about `0.045280`. Thus the old
  answer is not minimum even though the penetrating control passes. This is an
  exact witness, not a numerical optimizer: the old value exceeds `1/8`, while
  `sqrt(2)<99/70` and `sqrt(3)>5/3` bound the witness below `33/700<1/20`.

The last value is deliberately pinned by an existing selected-axis rounding
test and `ReducedCoreAxisCylinderCylinder` benchmark. Preserve the meaningful
rounding regression at its arithmetic boundary if that helper remains used;
do not retain the wrong public-contact expectation or retain dead code just
to exercise it. The separate [FMS-Issue-026 repair](done/2026-09-24-cylinder-pair-depth-rounding.md)
fixed bounded rounding work, not geometric completeness.

FixedMathSharp owns geometry, exact arithmetic and contact materialization.
Gravitas owns pair ordering, manifolds, materials and response. Its ordinary
cylinder-pair path already calls the public query. Its mixed cylinder versus
embedded circle-slab path independently repeats the incomplete directions:
repairing the upstream query alone will not repair that consumer. The concrete
mixed reproduction is recorded as `GRV-Issue-084`; its cylinder migration and
acceptance tests accompany this work without moving physics policy upstream.

## Chosen design and alternatives

Complete the existing feature-based contact authority. Reuse exact rational
frame construction, projection arithmetic, retained-root operations, ellipse
geometry and output conversion where their contracts actually match. Delete
superseded implementations once all real callers migrate.

A strict Boolean guard would remove some false contacts but leave wrong
positive depths; its current local-+Y, positive-radius contract also does not
cover the public query's arbitrary admitted local axes and zero radii.
Additional sampled directions likewise cannot establish complete minima.
An approximate iterative contact solver would require a different boundary
and rounding contract. None of these alternatives satisfies this repair.

### Exact geometric contract

Let `A` and `B` be the exact half-axis displacement vectors, `d` the second
center minus the first, and `Pa`, `Pb` the perpendicular projection matrices
for their axes. For unit direction `n`, the support gap is

```text
g(n) = |A dot n| + |B dot n|
     + ra * sqrt(n^T Pa n) + rb * sqrt(n^T Pb n) - d dot n.
```

Minimize over the whole unit sphere. A negative minimum rejects; zero admits
closed contact with zero depth; a positive minimum is the minimum separating
translation magnitude. Do not impose an extra center-projection boundary when
deriving candidates. Preserve the existing first-to-second normal convention
and a documented deterministic representative when the minimum is not unique.

Every minimum must be covered by the following features and their boundaries:

| Feature | Authority |
| --- | --- |
| Cap-normal directions | Exact support evaluation parallel to either axis |
| Side against side | Directions perpendicular to both axes; parallel axes handled explicitly |
| Side against rim | Stationary support minima on either axis-perpendicular plane, including boundary minima |
| Rim against rim | All admissible stationary common-normal configurations for all four cap pairs |

On a side-normal plane, one radial support is a constant radius and the other
is a projected ellipse. This is the reuse opportunity from
[FMS-Issue-023](done/2026-09-24-cylinder-capsule-contact-design.md). Reuse requires
proving the projected parameterization and finite-feature admission, not just
calling a method because its name contains "ellipse".

The rim/rim derivation must retain the original unsquared equations and verify
both radial support signs and cap-sign inequalities. Enumerate every admissible
solution; do not assume the largest positive root is the winner. Account for
parallel/coincident families, repeated roots, zero residuals, zero denominators,
zero radii and roots on feature boundaries. Preserve rational authored axes;
do not round a convenient orthonormal re-framing into new authoritative input.

### Reuse and deletion boundaries

| Existing code | Required treatment |
| --- | --- |
| `WideArithmetic`, signed fixed-width values and magnitude spans | Reuse integer arithmetic, radical signs and normalization; no copied limb implementation or runtime `BigInteger` |
| `WideOrientedBox.GetRotatedLocalAxisNumerators`, `RigidAxis3` | Preserve exact authored frame/axis construction and full-domain center differences |
| `CylinderCapsuleEllipse` geometry and coefficient preparation | Extract the shared projected-ellipse operation when both contact owners consume it; move the implementation, not a forwarding facade |
| `FiniteAxisPolynomialRoot` and `PolynomialRoot*` operations | Share coefficient/sign/root mechanics where proven; its current degree-at-most-four, largest-positive-root contract must not be silently broadened |
| `CylinderPairDepth` and radical comparisons | Retain useful exact rational-direction evaluation and rounding, removing obsolete ranking/materialization branches after caller migration |
| Existing support-anchor construction | Materialize only the selected result; retain origin-relative anchors for full-domain coordinates |

Only introduce a new internal owner when it owns a real shared operation or a
cohesive new rim/rim responsibility. No per-shape copies of polynomial kernels,
generic shape registry, solver interface hierarchy, callback-based hot-path
dispatch, compatibility wrappers, new dependency, or public API is required.
Shared geometry stays under the geometry owner; representation-only arithmetic
may move to `Numerics/Wide` only after removing geometry-specific dependencies.

Restructuring must preserve the optimized #023 paths. Do not force quartic
callers through a larger general-purpose workspace or slow equality algorithm.
Review net additions, removals, duplicate algorithms and caller ownership, not
just individual file lengths. A smaller diff that leaves two competing contact
implementations is not completion.

### Exact selection and output

Candidates remain unrounded through admission and global comparison. Compare
analytic candidates with algebraic candidates and distinct algebraic candidates
with each other. Interval refinement alone is insufficient on equal values:
the design must include a terminating exact equality/sign decision before any
new root family is accepted. Stable feature order resolves exact ties; swapped
input checks distinguish unique minima from ambiguous coincident cases.

Round the selected depth once to nearest-even and distinguish exact maximum
depth from conceptual overflow via `DepthIsClamped`. Round the selected normal
using exact threshold comparisons. Anchors remain independent support points
for that rounded normal, consistent with #023, rather than a new promise of
paired algebraic closest-point witnesses. False returns initialize contact to
default; invalid public inputs retain their current exception contract.

Before implementing the extended root path, derive its actual degree,
coefficient field, candidate-count bound, intermediate bit widths, isolation
bound and exact comparison method from this representation. The current
quartic Hermite fallback uses degree-specific storage and determinant work;
increasing array sizes is not a proof or an acceptable performance design.
Document peak simultaneously-live scratch storage, including nested calls, and
validate difficult cases on a bounded worker stack. Preserve the documented
minimum of a 1 MiB thread stack with adequate caller headroom; this is not a
claim that every caller fits within exactly 1 MiB. Do not raise that requirement
or add hidden allocation without explicit review.

The external [circle-distance derivation, section 7](https://www.geometrictools.com/Documentation/DistanceToCircle3.pdf)
shows why generic interacting rims need more care: its polynomial can have
degree eight. It solves nearest distance between rim curves, not signed
minimum translation of finite solids. It neither proves the degree for this
repository's exact representation nor supplies the missing validity/ranking
contract. No reference algorithm is adopted wholesale by this design.

## Verification and performance acceptance

First correct fixture validation and capture the unchanged geometric workloads
against the current revision, before runtime edits. Include ordinary parallel
and skew contacts, side/rim, separated/tangent/penetrating rim/rim cases,
intersecting axes, zero radii and full-domain arithmetic. Keep strict-only rows
separate. Historical incorrect answers are cost observations, not equivalent-
result speed comparisons. Freeze fixture inputs and measurement commands.

Use the existing test and benchmark projects. No new regression project, CI
performance gate, headless sample or diagnostic framework is needed.

Acceptance requires:

- Independent analytic or test-only `BigInteger` certificates for feature
  admission and minimum depth, not production helpers as the sole oracle.
- Exact touch, both one-raw neighbors, positive sub-raw penetration, midpoint
  ties, clamping boundaries, near-parallel axes, coincident cases and both
  zero-radius degeneracies.
- Unique-minimum swap symmetry, deterministic tie selection, exact rigid-frame
  cases and full-domain translation controls. Arbitrarily rounded rotations
  must not be mistaken for exact geometric isometries in test expectations.
- Actual ordinary and mixed Gravitas consumption, including the independently
  duplicated cylinder/circle-slab path from `GRV-Issue-084`.
- Revalidation of cylinder/capsule results and its frozen benchmark rows after
  shared-helper restructuring; no hidden regression in already-correct work.
- Both target-framework builds and Release/ReleaseLean test matrices; 100%
  reachable line, branch and method coverage in FixedMathSharp and Gravitas.
  Report unrelated existing failures explicitly; do not skip them or claim a
  green full suite merely because this issue's tests pass.
- Zero warmed allocations and matched out-of-process Release benchmarks.
  Investigate repeatable regressions above 5%, reporting absolute cost and
  error as well as percentages. This is a review trigger, not permission to
  retain an incorrect fast answer or a promise that exact rim/rim is cheap.

Serialize heavy validation: at most two cores, one build/test/benchmark tree
at a time, below-normal launcher priority. Read-only reviewers do not start
execution workloads. Keep portable reproduction commands and conclusions here;
ignored artifacts are supporting evidence, not the only retained context.

## Neighboring solver findings and scope limits

The circle-distance reference concerns free 3D rim curves. The existing
`FixedBoundCircle` instead classifies filled 2D disks using wide radius/distance
comparisons; its 20 Release tests passed during this focused audit. No separate
FixedMathSharp circle/circle defect or public rim-distance API was established.

The audit found and reproduced a separate Gravitas scalar-saturation defect
in 2D circle/circle contact (`GRV-Issue-083`); the existing zero-axis capsule
contact kernel correctly rejects its counterexample. It also confirmed that
mixed capsule/circle-slab contact still bypasses the now-complete #023 query
(`GRV-Issue-085`). Both stay separate queued repairs using existing geometry,
not new solvers. `GRV-Issue-084` tracks the duplicated mixed cylinder consumer
and belongs in this repair's downstream acceptance.

Already tracked: FMS-Issue-025 (box/cylinder) and GRV-Issue-082
(triangle/cylinder). Do not duplicate them or mark them resolved from unrelated
strict-predicate tests. Other finite-shape contacts are not certified by this
bounded audit. When further reproducible defects appear, record exact input,
expected geometric proof, observed output, owning revision and minimal next
action in the owning tracker; do not quietly expand #024 into all solvers.

## Completion boundary

Close #024 only after the complete contact authority, required shared-helper
cleanup, ordinary/mixed consumer validation and measured acceptance above are
finished. Move this document to `done` with concise evidence; remove the active
issue only then. No staging, commits or upstream releases are authorized by
this design.
