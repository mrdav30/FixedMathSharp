# FMS-Issue-024: Complete finite-cylinder pair contacts

## Status and intent

Design approved for execution, 2026-09-25. The owner explicitly requested
reuse, restructuring where useful, removal of superseded code, and independent
correctness/Ponytail reviews. Progress and remaining gates are recorded below;
design approval is not a claim that the general solver is complete.

Repair `FixedSegment.TryGetCenteredFiniteCylindersContact` without changing its
public signature. Closed separation/tangency, minimum translation depth,
normal selection, rounding and clamping must agree with the exact authored
finite solids. Preserve the separate inexpensive strict-overlap path.

This is a focused geometry repair, not a general collision framework or a new
public circle-distance API. Execution follows the ordered checklist below.
Mathematical completeness and resource proofs below are prerequisites
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

### Rim/rim construction refined during execution

The following is derived mathematics, not implemented runtime code or a
completed scratch-space proof. A direct polynomial in squared depth is simpler
than the initially investigated coordinate octic followed by a value resultant;
it avoids maintaining two independent algebraic representations per candidate.

Handle zero radii and parallel axes first. For nonparallel positive-radius
rims and one cap-sign pair, set `c=sA*A+sB*B-d`. Choose exact rational axis
directions `a,b`, nonzero rational `u` perpendicular to `a`, and define:

```text
alpha=a.a, beta=u.u, delta=b.b, w=a cross u
p=x*u+y*w, z=(x,y,1)
C=diag(beta,alpha*beta,-ra^2), z^T C z=0
Z=c.c+ra^2+2*c.p, J=b.(c+p), K=delta*Z-J^2
T(p,S)=delta*(Z+rb^2-S)^2-4*rb^2*K
```

`T` is a conic in `(x,y)`. Its quadratic block is independent of `S`, its
linear terms are affine in `S`, and its constant term is quadratic in `S`.
For its symmetric matrix `T(S)`, form the cubic pencil
`f(lambda,S)=det(T(S)-lambda*C)`. Every coefficient has degree at most two in
`S`; the leading cubic coefficient is a nonzero constant. At a stationary
distance the conics are tangent, giving a repeated root in `lambda`.
Consequently the cubic discriminant `V(S)` contains every stationary squared
distance and has degree at most eight. Its leading coefficient is
`16*alpha^2*beta^8*delta^4*rb^4*(alpha*delta-(a.b)^2)^2`, strictly positive
in this nonparallel domain. Parallel/coaxial circles are excluded deliberately:
their pencil can have an identically-zero discriminant from persistent complex
intersections at infinity, not a continuum of real contact depths.

For cubic coefficients `A,B,C1,D`, recover its repeated root using
`lambda=(9*A*D-B*C1)/(2*(B^2-3*A*C1))`, or `-B/(3*A)` for a triple root.
At an isolated real `S` root:

- Rank two of `T-lambda*C`: its kernel supplies the first-circle point.
  Differentiating the determinant proves that this point lies on `C`.
- Rank one: intersect the kernel line with `C`, retaining zero, one or two
  real points. This requires one quadratic extension over the retained `S`.
- Rank zero: the conics coincide; use the constant-distance-family reduction
  below rather than an arbitrary point.

With `H=Z+rb^2-S != 0`, reconstruct
`q=-2*rb^2*Pb(c+p)/H` and `v=c+p+q`. The conic equation proves
`q.q=rb^2` and `v.v=S`; tangency supplies the common-normal condition. For
`v!=0`, orientations `n=sigma*v/sqrt(S)` must satisfy positive
`sigma*(p.v)`, `sigma*(q.v)` and nonnegative cap projections
`sA*sigma*(A.v)`, `sB*sigma*(B.v)`. The complete signed gap is
`sigma*sqrt(S)`, not an unsigned nearest-circle distance. A negative admitted
gap proves separation. Radial equality belongs to a cap-normal boundary.

Keep these exceptional cases explicit:

- `H=0` implies `K=0`: intersect the first circle with `-c+tau*b`, then the
  second circle with its common-normal constraint. A continuous family with
  zero second-plane tangent projection belongs to the second side-normal plane.
- `v=0`: nonparallel rim tangents supply their cross-product normal. Parallel
  tangents give a planar cone defined by the radial and cap inequalities;
  any nontrivial cone has an active boundary represented by an earlier cap or
  side-plane stratum. Do not normalize a zero residual.
- Constant-distance families can be nonparallel. For example
  `a=(-3,0,4), b=(0,0,1), ra=rb=5, c=(0,3,0)` has a stationary arc at
  distance 3. In the nonorthogonal family the radii agree, `c` is perpendicular
  to both axes, and `|c|=r*sin(angle)`. The support-admissible arc terminates at
  cap-normal directions with the same gap and cap signs. Orthogonal continuous
  families belong to a side-normal plane. These are repeated roots of the
  nonzero value polynomial, not an excuse to discard it.

The representation has a concrete coefficient ceiling. In the exact inverse
first-cylinder frame, quaternion denominators `D1,D2<2^65` permit common raw
scale `Hscale=2*2^32*D1*D2<2^163`. Axis/perpendicular norms are bounded by
`|a|,|u|<2^33`, `|b|<2^163`, `|w|<2^66`, scaled `|c|<2^229` and scaled
radii `<2^226`. Matrix row weights `(33,66,229)` bound the coefficient of
`S^j` in `Tij` by `2^(790+weight_i+weight_j-458*j)`. Applying the cubic
discriminant bounds every integer coefficient of `V` below 7,400 bits, or
116 words. Physical raw squared depth is `S/Hscale^2`. This exact rational
frame must never be replaced with a rounded quaternion/vector transform.

Exact selection can now operate on value-root identities. A square-free GCD
identifies the shared-root set; selected cells must also identify the same
root ordinal before declaring equality. Unequal roots of coprime degree-eight
polynomials with heights `Bf,Bg` have separation greater than
`2^(-8*Bf-8*Bg-89)`, from the nonzero integer resultant and Mahler bounds.
Factor-height and within-factor separation bounds must accompany GCD reduction.
Side-plane and analytic candidates still need compatible complete-depth
representations; ranking their unshifted distances is not sufficient.

Remaining implementation gate: prove all-root isolation, sign queries,
repeated-value recovery and peak simultaneously-live scratch. A compact
graded subresultant chain is much smaller than uniformly padded coefficients,
but the current quartic machinery lacks arbitrary-limb exact division/content
GCD and uses five large temporary arrays in dyadic evaluation. Copying it or
increasing its degree constants does not establish the 1 MiB stack contract.
Reuse limb mechanics and shared evaluation operations; preserve the optimized
quartic path, and do not introduce an unproved workspace or hidden allocation.

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

### Execution checklist

- [x] Re-read the production query, retained-root contracts, matching tests and
  benchmark fixtures at `038d2b8`. Keep the existing develop checkout and leave
  changes unstaged/uncommitted as requested.
- [x] Strengthen the penetrating-rim regression with an independent separating
  translation bound. Release result before runtime edits: four failures and
  one pass in `CenteredCylinderPairStrictRegressionTests`; the new depth check
  fails for the intended geometric reason.
- [x] Remove the wrong selected-axis depth from the benchmark's public-contact
  success condition. Preserve the geometry under the clearer
  `PenetratingRimsCylinderCylinder` name; add separated/tangent rim and
  zero-radius workloads in the existing benchmark class.
- [x] Capture the frozen out-of-process Release baseline before runtime edits.
- [x] Reproduce and repair zero-radius cylinder pairs by using the existing
  complete cylinder/capsule query with capsule radius zero. Validate exact
  oblique minimum depth, swapped normal/anchors, full-domain translation,
  side one-raw boundaries, both-zero half-raw endpoints, default false output,
  and warmed zero allocations through the public cylinder-pair API.
- [ ] Finish the generic rim/rim elimination and exceptional-family proofs,
  including exact touch and identically-zero eliminants. Prove all-root
  isolation/comparison and scratch bounds before extending retained-root code.
- [ ] Share projected-ellipse preparation for both side-normal planes; compare
  complete gaps including each plane's constant radius, not unshifted gaps.
- [ ] Integrate complete global candidate selection, remove obsolete paths and
  migrate Gravitas's mixed cylinder consumer (`GRV-Issue-084`).
- [ ] Run the complete standard/Lean build, test and coverage matrix and matched
  benchmarks. Obtain independent correctness and Ponytail reviews and resolve
  their actionable findings before closing this issue.

### First implementation increment: zero-radius pairs

The public query now dispatches a zero-radius cylinder as its exact finite
segment to the existing cylinder/capsule authority. Reversing the inputs swaps
anchor ownership and negates the normal; it preserves depth, clamping and
default false output. Both-zero pairs use the same existing degenerate path.
No copied solver, public API, dependency or managed allocation was introduced.
The positive-radius candidate representation no longer checks for zero radii
that cannot reach its sole constructor. The shared cylinder/capsule traversal
also computes its minor cross axis only where it is consumed.

`CenteredCylinderPairSegmentContactTests` added eleven cases. Before runtime
edits, four failed and seven passed. The oblique fixture reported about
158.523848 instead of the independently certified minimum 135, and coincident
half-raw segments reported positive depth instead of zero. After dispatch,
these regressions and the existing cylinder/capsule, rigid-frame and rounding
controls all passed: 177 focused cases, none skipped.

Two old rigid-frame tests had asserted a false contact for a segment outside a
finite cap rim. Their corrected expectation is independently certified in the
test: the authored segment has `y=0,z=-k*x` with `0<k<11/10`; the cylinder
slab requires `x>=-1/2`, hence `z<=11/20`. Its radial squared separation is at
least `1/4+(39/20)^2=1621/400>4`. Dirty-stack and zero-allocation checks remain;
the obsolete approximate-vector assertion helper was removed.

Independent correctness and Ponytail reviews found no remaining actionable
issue in this increment. The correctness review prompted the public stack
warning: oblique zero-radius pairs inherit the existing cylinder/capsule stack
requirement. These reviews do not certify the unimplemented positive-radius
solver or close #024.

#### Verification boundary

Final source builds of `FixedMathSharp.slnx` succeeded in Release and
ReleaseLean, including `netstandard2.1` and `net8.0`, with zero warnings/errors.
Both Chronicler extension suites passed all 49 cases. The complete core runs
remain red only at the four enabled positive-radius regressions:

| Configuration | Passed | Failed | Skipped | Covered lines | Covered branches | Covered methods |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Release | 3,279 | 4 | 0 | 50,022 / 50,022 | 10,008 / 10,008 | 3,629 / 3,629 |
| ReleaseLean | 3,258 | 4 | 0 | 50,115 / 50,115 | 10,008 / 10,008 | 3,625 / 3,625 |

Coverage is 100% reachable line/branch/method in both configurations. This is
not a green-suite claim: separated rims, exact rim tangency, its one-raw outward
neighbor and the strengthened penetrating-depth check remain unresolved.

Portable core validation, after setting `UseLocalLsfStack=true`,
`BuildInParallel=false` and `DOTNET_PROCESSOR_COUNT=2` in the process
environment, uses:

```text
dotnet build FixedMathSharp.slnx -c Release -p:UseSharedCompilation=false -m:1
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --no-build --collect:"XPlat Code Coverage" --settings tests/FixedMathSharp.Tests/coverlet.runsettings --results-directory artifacts/fms024-coverage -- RunConfiguration.MaxCpuCount=1 xUnit.MaxParallelThreads=1
dotnet test tests/FixedMathSharp.Chronicler.Tests/FixedMathSharp.Chronicler.Tests.csproj -c Release --no-build -- RunConfiguration.MaxCpuCount=1 xUnit.MaxParallelThreads=1
```

Repeat with `ReleaseLean` and a distinct coverage directory. Core test exit 1
is expected until the four remaining regressions are repaired, not ignored by
the validation gate. No tests are skipped or newly excluded from coverage.

Gravitas `97fee61` was also source-built and checked with filter
`FullyQualifiedName~Cylinder|FullyQualifiedName~MixedNarrowPhase|FullyQualifiedName~Capsule`.
Release passed 519 cases and ReleaseLean passed 518; both had one failure and
no skips. The sole failure was the already-tracked cylinder/triangle contact
reproducer, `GRV-Issue-082`, outside this change. No Gravitas source or tracker
was modified. These focused consumer runs do not replace its eventual complete
test/coverage gate or the pending `GRV-Issue-084` migration.
The FixedMathSharp DocFX API build also succeeded with `--warningsAsErrors`,
zero warnings and zero errors. Generated output remains ignored.

#### Frozen performance evidence

Windows 11, Intel i7-9700K, .NET SDK 10.0.302, .NET runtime 8.0.29,
BenchmarkDotNet 0.15.8; two-core affinity, serialized workloads, below-normal
launcher. Both captures used two launches, three warmups and twelve measured
iterations. Every row reported zero managed allocation and both launchers
exited zero. Values below are mean microseconds with BenchmarkDotNet's 99.9%
confidence-interval half-width, not end-to-end simulation costs.

| Unchanged geometry | Before dispatch | After dispatch |
| --- | ---: | ---: |
| Penetrating rims | 357.780 +/- 5.341 | 358.924 +/- 5.253 |
| Separated rims | 363.220 +/- 5.715 | 360.420 +/- 7.346 |
| Tangent rims | 360.632 +/- 5.478 | 360.201 +/- 6.600 |
| Zero-radius parallel segment | 29.439 +/- 0.387 | 31.170 +/- 0.444 |
| Ordinary strict predicate | 0.899 +/- 0.012 | 0.905 +/- 0.013 |
| Ordinary contact | 25.825 +/- 0.487 | 25.940 +/- 0.380 |
| Multi-radical contact | 498.860 +/- 5.877 | 501.238 +/- 8.918 |
| Full-domain cancellation | 353.676 +/- 3.270 | 357.171 +/- 4.975 |

The first three rows still return incorrect classification/depth. Their times
are retained workload baselines, not equivalent-result comparisons or evidence
that the general repair is complete. Several distributions are bimodal; the
sub-percent control differences do not establish a speed change.

The ordinary zero-radius regression exceeded the 5% investigation threshold.
A second matched capture confirmed about 31.3 us after dispatch. Moving the
unused minor-axis calculation into its consuming block gave no measurable win:

| Shared-kernel control | Before minor-axis cleanup | After cleanup |
| --- | ---: | ---: |
| Ordinary cylinder/capsule | 21.31 +/- 0.265 | 21.30 +/- 0.274 |
| Zero-radius cylinder pair | 31.31 +/- 0.437 | 31.26 +/- 0.398 |
| Oblique interior rim | 600.88 +/- 10.344 | 597.95 +/- 9.237 |
| Intersecting capsule core | 461.19 +/- 6.871 | 463.19 +/- 7.055 |

Retain the reuse repair for correct contacts across the whole segment family,
with the explicit ordinary-case cost of about 1.8 us / 6% above the old path.
Do not describe this as a performance improvement or add a second segment
solver merely to recover that small absolute cost. The one-line cleanup is
retained because the computation has no consumer on those paths.

Reproduce from a Release benchmark build with `UseLocalLsfStack=true` and
`BuildInParallel=false` in the process environment. Apply the two-core affinity
and launcher priority above, then run:

```text
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll rigid-finite-shape-relation --filter '*CylinderCylinder*' --warmupCount 3 --iterationCount 12 --launchCount 2 --exporters json --artifacts artifacts/fms024-baseline
```

Use a distinct artifact directory per revision. For the shared-kernel control
capture, replace the filter with `'*ZeroRadiusCylinderCylinder*'
'*OrdinaryCylinderCapsule' '*InteriorObliqueRimCylinderCapsule*'
'*CoreInsideCylinderCapsule*'`. Baseline source was `038d2b8` plus the fixture
corrections, before runtime edits. Ignored logs under `artifacts/fms024-*`
support these results but are not required to reconstruct the workloads.

Close #024 only after the complete contact authority, required shared-helper
cleanup, ordinary/mixed consumer validation and measured acceptance above are
finished. Move this document to `done` with concise evidence; remove the active
issue only then. No staging, commits or upstream releases are authorized by
this design.
