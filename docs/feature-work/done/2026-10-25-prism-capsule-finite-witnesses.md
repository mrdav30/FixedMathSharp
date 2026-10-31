# FMS-Issue-036: Finite sphere/capsule prism witnesses

## Scope and source

Starting revisions: FixedMathSharp `e8c57d8`, Gravitas `f76b099`. Development
and all coordinated builds, tests and benchmark children use
`UseLocalLsfStack=true`. Changes remain uncommitted for owner review.

Correct finite witness ownership in the existing sphere/capsule prism owner.
Retain native scalar yaw, authored anchor origins/rotations, exact closed
admission and ranking, deterministic ties, full-domain coordinates and final
nearest-even rounding. No public API, persistent numeric type, allocation,
dependency, serialization layout or Gravitas production policy is added.

## Reproducers and physical meaning

- A Right-axis capsule centered at `(0,0.5,0)`, core length `10`, radius `0.25`,
  against a prism centered at `(2.5,0,0)`, footprint `1 x 2`, half-height `1`,
  had depth `0.75`/normal Down but witnesses `(0,0.25,0)` / `(2,1,0)`. The
  core's arbitrary midpoint is outside the finite opposing cap. Paired supports
  exist at every `X` in `[2,3]`; choose the one nearest the core midpoint. Its Y
  support proves the complete `0.75` exit.
- A centered radius-`0.75` sphere in the prism `X=+/-1`, `Y=+/-2`, `Z=+/-3` had
  depth `1.75`/normal Left but witnesses `(-0.75,0,0)` / `(-1,0,0)`. The target
  belongs at `(1,0,0)`. A centered unit ball in the prism supplies the matching
  lower bound; the X support attains it.
- Scalar-limit capsules could receive coincident finite points despite a clamped
  positive depth. Keep distinct opposing supports before clamping the published
  depth; the complete anchor difference can exceed the scalar domain.

The seven initial math regression rows fail on the committed implementation. The
two downstream host regressions also fail after real response: the long capsule
receives zero Z torque, while the sphere's rough opposing prism receives
positive planar yaw instead of negative yaw. Failure captures are
`artifacts/issue036-baseline-red.log` and
`artifacts/issue036-host-baseline-red-retained.log`.

Review exposed a related missing finite-rim direction. A sphere centered at
`(-1,-1,1)`, radius `2`, against cube `[0,2]^3` has nearest prism point
`(0,0,1)`, normal `(1,1,0)/sqrt(2)` and depth `2-sqrt(2)`. The old projection
inventory lacks this horizontal-rim direction; its selected plane can have no
finite witness. Radius `1.25` at the same center is strictly separated. This
needs a complete finite-edge correction rather than a nearest-boundary fallback
or an exception. A `3-4-5` variant supplies exact touching and both one-raw
radius neighbors.

## Implementation and invariants

Project the exposed finite capsule core onto the selected opposing prism plane.
For core point `C`, selected axis `n` and plane `n.P=m`, retain the rational
point `P=C+(m-n.C)*n/(n.n)`. Radial support cancels under that projection; no
square root is needed to select the core parameter.

Clip that projected segment against the native-yaw prism with the existing
`WideTriangleRelations.ClipScaledHalfPlane`. Real footprint edge halfspaces and
vertical bounds describe the complete prism; collapsed footprints also need
their finite X/Z endpoint bounds. Preserve the core midpoint when admitted,
otherwise choose its nearest closed interval endpoint. Exact core endpoints keep
the existing odd-half-length residual; interior points reuse the existing core
materializer. Normalize and scale the exact direction before rounding its local
radial displacement. Scaling an already rounded unit normal instead amplifies
its error with the radius: the scalar-limit diagonal test independently proves
the correct support coordinate `6,521,908,912,666,391,105` raw.

Only the final target materialization uses the published quaternion anchor
frame. Native sine/cosine coordinates are not silently replaced with that
quaternion's different rational map. Retain rational interpolation and origin
subtraction before the inverse frame map, then split-round the complete local
coordinate. The existing split-round helper moves to `ContactQuadratic.Ratios`,
with actual consumers in both prism owners and scratch sized to their existing
bank widths. Matched-source local radial rounding and the later frame
combination retain a bounded lattice error independent of radius; they do not
promise a single nearest-even rounding of the complete world sum. Keep the
core's exact endpoint residue separate from that radial displacement.

An empty support projection invokes the finite-edge owner. Prism face and
core/real-edge cross axes supply the complete facet inventory of the prism/core
Minkowski polytope. If the core intersects the prism, their minimum plane has a
finite support match. For an outside core, a nearest point in a face interior
would likewise certify an existing face normal. Thus an unmatched selected plane
requires an outside-core closest feature on a real prism edge or vertex. Honor
every already-computed face separation result directly.

Stream all bottom, top and vertical real prism edges. Exact finite segment
distance uses the four endpoint feet, an admitted interior line pair and the
core-midpoint foot. Point/parallel degeneracies remain finite. Rank squared
distance ratios exactly, then prefer the closest core parameter to its midpoint;
earlier edges own remaining ties. Admit only `distance <= radius`.

Materialize normal, depth `radius-distance`, source radial support and target
point from those retained ratios through final rounding. This preserves exact
`3-4-5` tangency instead of reconstructing support from a rounded unit normal.
Outside-core depth is bounded by the radius and therefore cannot clamp.

Collapsed footprints additionally retain a final perpendicular bound. If all
footprint vertices coincide and the core is parallel to the vertical prism line,
the original edge/cross normals vanish. Appending Right for zero-area footprints
supplies the missing radial minimum without replacing earlier equal candidates.
Quarter- and Max-radius sphere/vertical-core regressions prove depth exactly
equal to the radius, including an unclamped Max equality, closed touching and
one raw unit beyond it. A separated oblique-core control exercises early
rejection at that bound. These four regressions failed before the final
addition; the capture is `artifacts/issue036-point-footprint-red.log`.

Existing widths suffice: the projection clip fits Signed832; final native/frame
interpolation fits 32-word borrowed banks. Edge endpoints share a denominator
below 98 bits, finite valid parameters bound point numerators below 821 bits,
and final quadratic coefficients remain below 1720 bits. Source comments give
the intermediate bounds. Scratch is bounded and query-local.

Retire the old whole-boundary closest-point matcher, its independent planar-tie
flags and the private penetration wrapper. They cannot certify opposing finite
supports. The new edge partial owns one independent reason to change; it does
not introduce a general geometry framework or duplicate the arithmetic owners.

## Regression coverage

Public regressions cover mirrored/reversed long cores, both polygon windings,
scalar-limit supports, native yaw and scalar-face translations, zero-core sphere
parity under an unrelated quaternion, repeated footprint vertices, exact rim
touching and radius neighbors, and warmed zero-allocation paths. An exact `2:1`
quaternion ratio exercises an odd five-raw endpoint on `2X+Z=5` raw, preserving
its half-raw feature until final world rounding.

A 125-position oblique capsule grid compares the equivalent prism and oriented
box: classification, depth, finite paired anchors and whole-shape clearance.
Small final anchor/direction rounding bounds are explicit; they do not replace
exact feature admission. The finite-vertex fixture has independent support
normal `(1,1,2)`, perpendicular to its core, and exact distance `sqrt(6)`.
Independent integer half-cell bounds supply strict irrational witness oracles;
the public scalar `Sqrt` is an approximation, not a nearest-even oracle for
every non-perfect square.

Gravitas uses real `Simulate`/`LateSimulate` response, subsequent rotation,
independent separation bounds, events and three authoritative replay hashes
including solver caches. Frictionless capsule impact proves the real off-center
normal lever; rough sphere/prism response distinguishes the opposing face by
friction torque. No host response implementation is changed.

## Validation

The retained source passes the complete four-mode matrix below. An earlier
matrix also passed before the collapsed-footprint finding; only the collector
and its regression file changed afterward, and every configuration was rerun.
The final captures use `issue036-retained-*` prefixes in each owning repository.

| Configuration              | Core tests | Chronicler tests | Covered lines | Covered branches | Fully covered methods |
| -------------------------- | ---------: | ---------------: | ------------: | ---------------: | --------------------: |
| FixedMathSharp Release     |      6,850 |               49 | 66,003/66,003 |    20,976/20,976 |           5,113/5,113 |
| FixedMathSharp ReleaseLean |      6,829 |               49 | 66,096/66,096 |    20,976/20,976 |           5,109/5,109 |
| Gravitas Release           |      7,455 |                — | 50,698/50,698 |    18,878/18,878 |           5,370/5,370 |
| Gravitas ReleaseLean       |      7,386 |                — | 50,698/50,698 |    18,878/18,878 |           5,370/5,370 |

All solution builds report zero warnings/errors; tests have no failures/skips.
Counts are exact non-generated raw Cobertura class-line, branch and method-node
counts, not rounded percentages. FMS raw and ReportGenerator totals agree.
Gravitas's ReportGenerator presentation adds a synthetic collision-namespace
container alongside its child classes, reporting 66,205/25,446/6,646, all
covered. Its unfiltered standard XML also includes two covered lines and one
method from the generated PhysicsLayer MemoryPack formatter. Neither reporting
detail hides any uncovered raw point. Independent artifact review checked each
configuration's sole XML, every condition/method and the test/build logs.

An ignored resource probe executes five public query paths on a thread
configured for a 128 KiB stack. After warmup, each 32-call sample reports zero
calling-thread bytes: scalar-limit exact diagonal scaling, crossing-core cap
matching, parallel-core `3-4-5` rim contact, and a scalar-scale rim with an
unrelated zero-core quaternion, plus a radius-zero point on a vertical
point-footprint prism. Capture:
`artifacts/issue036-retained-resource-probe.log`. The maintained warmed
allocation guard also passes. The requested thread-stack size and these
exercised paths are a resource diagnostic, not a proof of every platform's
effective stack limit.

The final source/test checkpoint has 1,933 actual-byte hashes across the five
coordinated repositories; the frozen benchmark copy matches all entries after
line-ending normalization.

The retained focused captures pass 272 math prism cases and 552 mixed narrow
phase/response cases. Changed owners have full raw line/branch coverage in the
focused capture, including exact endpoints and finite interior edge feet.
Capture prefixes are `artifacts/issue036-*` in each owning repository. Keep raw
coverage counts, source checkpoints and child payload bindings with the final
captures.

Portable focused reproduction, from the respective repository roots:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~FixedConvexPrism'
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~MixedNarrowPhaseTests|FullyQualifiedName~MixedResponseTests'
```

## Benchmark diagnostics

The final Release dispatcher capture passes all fourteen prepared `TryCollide`
cases, retaining the original eleven and adding crossing-core, opposing-face and
horizontal-rim cases. Every child exits zero with complete statistics; all
fourteen raw GC rows report zero collections and zero managed bytes. The child
PE/PDB bindings verify authored source for FixedMathSharp (332 documents),
Gravitas (402), its benchmarks (79), Chronicler (29), SwiftCollections (80) and
GridForge (121). Capture prefixes are
`artifacts/issue036-retained-dispatcher-*`.

Environment: Windows 11, Intel i7-9700K, .NET SDK 10.0.302, .NET 8.0.29,
Release, affinity mask 3 (rendered `11` by BenchmarkDotNet), one launch, three
warmups and eight measured iterations. BenchmarkDotNet retained seven samples
after removing an outlier in ConeOblique, CylinderSeparated, CylinderSideTouch,
CapsuleContained and SphereHorizontalRim; the other nine retain eight. No
samples are manually discarded.

| Prepared dispatcher query   | Historical baseline means (two captures) | Final mean | Retained samples |
| --------------------------- | ---------------------------------------: | ---------: | ---------------: |
| SphereContained             |                       315.491–332.716 µs |  311.19 µs |                8 |
| CapsuleContained            |                       317.888–326.452 µs |  309.91 µs |                7 |
| CapsuleCrossingOffcenter    |                       243.792–247.846 µs |  238.14 µs |                8 |
| SphereContainedOpposingFace |                       323.807–331.677 µs |  306.22 µs |                8 |
| SphereHorizontalRim         |                    No committed baseline |  266.48 µs |                7 |

The historical captures use committed runtime sources and the same prepared
fixtures, but publish the defective witnesses. They are retained as timing
context, not an equivalent-output performance reference. These sequential short
diagnostics do not establish a speedup, a universal throughput threshold or
complete host-step capacity. The horizontal rim has no historical baseline.
Original baseline prefixes are `artifacts/issue036-benchmark-baseline` and
`artifacts/issue036-benchmark-baseline-2`; intermediate candidate captures are
exploratory and do not describe the retained source.

The frozen benchmark builds inherit both the outer checkout's and snapshot's
global analyzer configuration. Their eight dispatcher and two math-build
duplicate-key warnings are capture-layout diagnostics; the four ordinary
solution builds above have zero warnings/errors.

The broad frozen Release math smoke passes all 479 children with complete
statistics across 26 reports and an independently checked 332-document
FixedMathSharp PE/PDB source binding. All 479 raw GC rows are retained: 466
report zero bytes; the thirteen positive rows are the enumerable-construction,
four formatting and eight serialization controls that intentionally allocate.
Their case/PID/operation/byte records are in
`artifacts/issue036-retained-smoke-allocation-observations.json`. The preceding
complete smoke likewise has only those thirteen allocating controls. This 10 ms
iteration smoke emits expected minimum-iteration-time warnings and is not a
timing regression gate. Final capture prefixes are
`artifacts/issue036-retained-smoke*`.

DocFX reports zero warnings/errors, and all 28 checked local Markdown targets
and fragments resolve, including the preserved historical issue-036 anchor.
Independent source, geometric proof, raw coverage and dispatcher artifact
reviews have no outstanding findings. Source/test hashes remain unchanged
through the final captures.

Released-package validation remains a release gate after FixedMathSharp is
published. Local Windows execution does not establish all-platform execution.
