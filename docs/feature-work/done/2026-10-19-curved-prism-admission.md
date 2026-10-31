# FMS-Issue-033: Closed cylinder/cone admission against finite prisms

## Problem and scope

The finite projection directions in `WideConvexPrismRelations` can all overlap
while a cylinder or cone is strictly separated from a prism along a curved
support direction. The two tracker fixtures have an independent whole-shape
separating-plane margin of approximately 1/1024 unit. Both returned contacts on
revision `5674103`; downstream mixed AABB/polygon host steps also published and
resolved those false contacts.

Resolve classification in the existing prism owner. The public prism methods
promise canonical contact anchors and a representable depth; they do not promise
a global minimum translation. Preserve the current exact projection-depth and
witness reducer for admitted contacts. A retained projection overlap is a
sufficient separating displacement before final rounding, rather than a promise
of the first exit or physical minimum correction. More sampled directions cannot
establish complete intersection admission.

## Complete admission

Retain the native scalar-yaw vertex numerators and exact rational rotated shape
axis, including its actual squared norm. Do not materialize world vertices or
construct a rounded quaternion that maps a canonical axis to the authored axis.

A center inside the closed prism proves intersection. For a world-upright axis,
closed footprint containment and the already accepted world-Up projection also
prove a shared finite-axis point. Reuse the planar containment owner for these
positive certificates before constructing the full admission context. Otherwise
any intersection must reach the prism boundary. Test its real edges with the
existing closed, axially clipped quadratic criterion. For each face plane, a
nonempty primitive section has a rational intrinsic point: a finite-axis/plane
intersection or a cap-disk projection (the base disk for a cone). These points
can provide cheap positive certificates before the edge pass. If no prism edge
intersects the solid, its connected plane section cannot cross a face boundary;
any intrinsic section point decides containment in that face.

For axis A, plane normal n and cap center C, let U=A.A, b=n.A, W=U*n-b*A,
J=U*(n.n)-b*b, and delta=k-n.C. The disk projection is C+delta*W/J, admitted
exactly when delta*delta*U <= radius*radius*J. Parallel cases use the axis
section. For a cylinder the plane-distance residual is an absolute affine
function along its finite axis; without an axial zero its minimum occurs at a
cap. For a cone, residual divided by affine section radius is monotone away from
an axial zero, so the base decides feasibility.

Keep touching closed, retain zero-radius segments and finite collapsed prism
footprints, and use fixed-width arithmetic with proven complete intermediates.
Place admission in the existing materialization helper, after the native
projection rejection pass and before normal, depth and witness construction.
Reuse its exact rotated axis; the projection loop and successful witness reducer
retain their existing algorithms. The boundary pass remains O(n^2), matching the
existing projection inventory, and uses fixed stack storage.

## Validation and performance

All commands use `UseLocalLsfStack=true`. Initial red: two public FixedMathSharp
regressions and four complete Gravitas host-step rows (cone/cylinder times
AABB/polygon) fail against the old implementation.

The final core file contains 63 focused cases. It covers exact touches and
one-raw gaps, independently witnessed nearby overlaps, winding and redundant
vertices, containment and independently proven clearing displacements, collapsed
footprints, arbitrary axes with their actual norm, scalar yaw, large origins and
warmed zero allocation. The original separated reproducers also run under a
common translation near the coordinate limits, retaining their independent
relative support gaps through the complete negative-admission path. Mirrored
oblique-axis cases independently witness cap-face intersections while clearing
the opposite prism cap. Wide coefficient tests retain a common factor of `2^896`
through axial clipping, a double root and a positive gap.

The four downstream rows execute both `Simulate` and `LateSimulate`. Each
retains one broad-phase candidate, rejects the direct narrow-phase query,
creates no active pair or notifications, and leaves poses and velocities
unchanged.

Complete solution builds and tests pass for the frozen source in both
configurations, with no skipped tests or new coverage exclusions:

| Repository/configuration   |               Passing tests | Covered/coverable lines | Covered/total branches | Fully covered/total methods |
| -------------------------- | --------------------------: | ----------------------: | ---------------------: | --------------------------: |
| FixedMathSharp Release     | 6,687 core + 49 integration |           64,919/64,919 |          20,400/20,400 |                 5,036/5,036 |
| FixedMathSharp ReleaseLean | 6,666 core + 49 integration |           65,012/65,012 |          20,400/20,400 |                 5,032/5,032 |
| Gravitas Release           |                       7,445 |           66,205/66,205 |          25,446/25,446 |                 6,646/6,646 |
| Gravitas ReleaseLean       |                       7,376 |           66,205/66,205 |          25,446/25,446 |                 6,646/6,646 |

For each repository, build its solution with `-c Release` and `-c ReleaseLean`,
`-p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1`. Collect core
coverage with its existing `coverlet.runsettings` and
`--collect:"XPlat Code Coverage"`; run the separate FixedMathSharp.Chronicler
test project in each configuration. Generated-source exclusions remain
unchanged. The final checkpoint covers 976 runtime source/project files across
the coordinated stack.

Six deterministic Gravitas `CurvedPrismContactBenchmarks` methods measure the
actual mixed dispatcher: each primitive has separated, face-overlap and oblique
overlap cases. A frozen source-stack copy keeps the pre-fix implementation
available for matched Release BenchmarkDotNet runs. Separation setup checks an
independent support-plane oracle while allowing the old false result, so the
same harness measures both versions.

Matched out-of-process Release captures use BenchmarkDotNet 0.15.8 on Windows
11, Intel i7-9700K, .NET 8.0.29 X64, processor mask 3, five warmups and ten
measured iterations. Run order is baseline/fixed/fixed/baseline (`A3/E1/E2/A4`).
The table averages the two process means for each implementation; times are
microseconds per complete dispatcher query.

| Fixture           | Pre-fix us | Fixed us |  Change |
| ----------------- | ---------: | -------: | ------: |
| ConeSeparated     |     248.13 |   293.13 | +18.14% |
| ConeFace          |     271.29 |   265.32 |  -2.20% |
| ConeOblique       |     534.73 |   537.38 |  +0.50% |
| CylinderSeparated |     415.46 |   450.14 |  +8.35% |
| CylinderFace      |     264.17 |   261.72 |  -0.93% |
| CylinderOblique   |     578.68 |   592.68 |  +2.42% |

The four ordinary overlap controls retain identical raw normals, depths, all
public anchor components and materialized points. Their paired means range from
2.20% faster to 2.42% slower; this small mixed result is not an equivalence
proof. The corrected separated paths add about 45 us for the cone and 35 us for
the cylinder. The old result was a false contact, so its lower cost is not
equivalent correct behavior. These fixtures do not establish full-step or
long-session capacity, or a universal frame budget.

All 24 children exit successfully with complete statistics and zero raw managed
allocation counters, in addition to the warmed calling-thread regression guard.
Retained child DLL/PDB identities and authored-document checksums bind the
actual FixedMathSharp, Gravitas and benchmark payloads to their frozen source
copies. A simple DLL comparison initially rejected the baseline because the
standalone probe built identity 0.0.0 while local-stack benchmark references
select 7.1.0; matching portable-PDB content identities and source checksums
resolve that comparison without accepting stale binaries.

A direct gate in the projection worker measured approximately 9.3% higher
oblique-cylinder cost in the earlier balanced `A1/B1/B2/A2` captures. Its CPU
profile attributed less than 1% of that positive query to admission itself; that
profile does not establish the cause of the remaining timing difference. The
selected materialization-helper placement reduces the measured ordinary cost,
reuses the existing rational axis and avoids building discarded witnesses.
Post-materialization and alternative certificate placements were also measured;
their adverse captures remain available rather than being used as acceptance
results.

Reproduce the focused measurement after building the Gravitas benchmark project
in Release with `-p:UseLocalLsfStack=true -m:1` and exporting the same property
to child builds:

```sh
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll curved-prism-contact -j Short --warmupCount 5 --iterationCount 10 --affinity 3 --keepFiles --exporters json
```

Ignored evidence: each repository's `artifacts/issue033-final-*` build, test,
coverage and report captures; Gravitas `artifacts/issue033-{A3,E1,E2,A4}`
exports and companion logs/binding records; the earlier `A1/B1/B2/A2`,
`C0/D0/E0` and `issue033-profile-B` diagnostics; FixedMathSharp
`artifacts/issue033-{baseline,candidate}-final-probe.log`. Runtime source hashes
still match the 976-file checkpoint after the paired measurements.

The final broad out-of-process Short smoke completes all 479 FixedMathSharp
benchmark cases, with 479 successful child exits and complete statistics. Its
retained child DLL/PDB and authored-source checksums also match the fixed
source. This uses
`all -j Short --iterationTime 10 --affinity 3 --keepFiles --exporters json`; the
shortened iterations are a harness smoke check, not latency evidence. Both
repositories' DocFX builds pass with zero warnings/errors, API-local links pass,
and all four existing wiki rewrite checks pass. Independent code, geometry, test
and final evidence reviews have no outstanding findings. Final whitespace checks
and the unchanged 976-file runtime checkpoint pass. Evidence is retained in
FixedMathSharp's `artifacts/issue033-smoke*`, `artifacts/issue033-checks.log`
and `artifacts/issue033-wiki-checks.log`, plus each repository's
`artifacts/issue033-docfx.log`.

## Separate contact-depth finding

The bottom-edge classification test exposed a pre-existing depth/normal defect,
recorded as FMS-Issue-034 in the issue tracker. A height-2, radius-1 upright
cone at zero touches the full prism `[0.5,1.5] x [0,2] x [-1,1]` only at
`(0.5,0,0)`. The exact plane `2*x+y=1` supports both solids. Both the frozen
pre-fix source and this admission fix return depth `0.5` and normal Right at
that true touch. The one-positive-raw X neighbor changes from a false contact to
a miss with the admission fix; touching and overlapping neighbors retain their
existing output.

This work completes intersection admission, not the complete contact normal and
minimum-correction solver. The public XML and geometry guide state that the
canonical projection bound can remain positive at tangency. No test pins the
incorrect correction as desired physical behavior. FMS-Issue-034 retains the
full-volume reproducer and the remaining native-owner work.
