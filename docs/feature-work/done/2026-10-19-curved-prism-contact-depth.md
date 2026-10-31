# FMS-Issue-034: Complete curved-prism contact selection

## Scope and acceptance

Replace the finite canonical projection bound for cone/cylinder contacts with
minimum whole-prism supporting translation and matched finite witnesses.
Preserve the public API, native scalar yaw, actual authored axis norm, closed
Issue 033 admission and existing capsule behavior. Gravitas consumes these
contacts for mixed-dimensional positional correction.

Starting revisions: FixedMathSharp `b813236`, Gravitas `db5c3c2`, Chronicler
`5397348`, SwiftCollections `5e0c4e0`, GridForge `09f810e`. All development,
benchmark children and validation use `UseLocalLsfStack=true`. The correctness
repair and initial bounded refinement were committed in `5f2bf65`; the
incident-face cone refinement was committed in `43500eb`; exact-zero minimum
certificates and support-scan reductions were committed in `90722e9`, with
Gravitas touching benchmark controls in `8bdef3a`. Both the pre-refinement
repair and retained refinement pass coordinated Release/Lean correctness and
complete coverage, benchmark smoke and documentation checks. The evidence below
distinguishes those snapshots. Issue 034's correctness repair is resolved on
2026-10-20. The remaining penetrating oblique cost is an
[active benchmark signal](../benchmark-signal-hardening-backlog.md#penetrating-oblique-conecylinder-prism-contact-throughput),
not a production-throughput acceptance claim.

The final bounded source review found no further small experiment with a
complete correctness proof and convincing reach on these penetrating fixtures.
The measured bounds and reach checks below did not justify their added
arithmetic. Revisit exact candidate/root work when a concrete reduction or a
representative simulation profile justifies another investigation.

## Reproducer and geometric reason

For the height-2/radius-1 cone at zero, local Up and identity rotation, use the
prism `[0.5,1.5] x [0,2] x [-1,1]`. The plane `2*x+y=1` supports both complete
solids. Their only common point is `(0.5,0,0)`, so depth is zero and the unique
normal is `(2,1,0)/sqrt(5)`. The old result was depth `0.5`, normal Right and an
endpoint witness. The true point is inside a cone generator. Correcting only the
depth or adding this fixture's gradient cannot fix the general contact contract.

The frozen starting source fails 12 of the initial 14 public regression rows;
the two separation controls pass. Ignored evidence:
`artifacts/issue034-baseline-red.log`. The fixture and its independent plane
oracle survive in the maintained tests.

## Retained implementation

- Keep exact geometry in the primitive's authored quaternion frame. Retain the
  native scalar-yaw prism and the actual squared norm of its accepted local
  axis. No approximate alignment quaternion is introduced.
- Rank prism face, real-edge and vertex support against cylinder caps, side and
  rims, or cone base disk/rim, apex and generators. Every candidate uses the
  entire prism support. An independently solved boundary triangle cannot prove
  whole-solid clearance.
- Reuse `ConvexContactCandidate`, quadratic arithmetic, stationary edge/rim
  quartics and retained value roots. An open vertex/rim patch has zero
  meridional curvature and cannot own a positive interior minimum; fan
  boundaries and stationary real-edge arcs complete the positive inventory.
  Negative separation still requires the complete Issue 033 admission gate.
- Retain the exact winning feature and root through finite witness matching.
  Clip side/generator segments to the complete prism. For a cap, try its
  projected center, finite supporting vertices and real-edge interior feet.
  Round physical coordinates once, with complete-coordinate nearest-even parity
  when an additive anchor term is required.
- Preserve each anchor's own origin. Both use the primitive quaternion frame,
  keeping scalar yaw authoritative. Public XML and the geometry wiki explain
  this and the depth/normal rounding bounds.
- Keep query-owned bounded scratch. A complete 32-vertex cache serves ordinary
  footprints; larger footprints stream identical exact vertices and ordering. No
  managed cache, dependency, public API or numeric framework was added.

The geometric coordinate proof bounds original prism vertices below 165 bits,
axis-square below 65 bits and rational candidate directions below 504 bits.
Typed projections remain below 674 bits. Forty-word candidate/root fields and
112-word analytic witness banks cover the complete products; detailed bounds are
beside the operations. Split anchors preserve local coordinates outside one
scalar without constructing an overflowing world point.

Superseded curved canonical reducers, disk-only branches, feature fields and
unused support overloads were removed after tracing actual callers. Capsule and
parallel-cylinder consumers retain their live paths. Seven new cohesive owners
add 1,763 physical lines; the complete runtime diff adds 1,862 and removes 409,
net **1,453** including headers, comments and public XML (1,418 excluding that
XML change). Independent review found no further justified deletion.

## Behavioral and resource checks

The focused prism suite has 189 rows. New cases include:

- Unique generator touching and both one-raw neighbors; a known positive exit;
  collapsed footprints, winding changes and transformed frames.
- Both containment directions, finite caps and real cap-edge feet, side,
  generator, apex and rim features. Native yaw `1/3` uses an independent raw
  affine integer oracle before one nearest-even division.
- Accepted axes with squared norms on both sides of one, odd dimensions,
  radius-zero finite segments and full-domain translations/anchors.
- Rim stationary roots, repeated and nearly coincident extrema, a zero
  stationary numerator/denominator, tiny vertical tangency and whole-coordinate
  half ties. Equivalent boxes provide independent minimum-depth/normal checks;
  each retained pair separately satisfies finite-surface and tangential checks.
- Complete-shape touching at the reported exit and strict separation beyond the
  documented rounding margin; anchor distance alone is not the clearance oracle.
- Cached/streamed equivalent footprints; warmed zero-byte assertions using the
  existing allocation helper; repeated results on a 1 MiB worker stack with a
  live dirty 64 KiB caller buffer.

Four downstream Gravitas rows exercise the actual `Simulate`/`LateSimulate` host
step for touching/penetration and both dynamic participant roles. Touching has
zero positional correction. Penetration honors deliberate response slop; an
authored outward impulse clears the full supporting plane on the next step.
Events, active pairs, positions and velocities replay exactly in fresh contexts.
Gravitas production code is unchanged.

## Pre-refinement performance evidence

These captures describe the correctness repair before the ongoing throughput
refinement. They remain comparison evidence and do not establish results for the
current refinements.

The operation is one warmed `CollisionDetectionMixed.TryCollide` query on the
existing two-collider fixtures, with four footprint vertices. This is dispatcher
cost, not a complete simulation step or a capacity estimate. Separation and face
fixtures retain their classification; face depth is exactly `1/4` with Down
normal. The oblique depths happen to match the old raw numbers, but their
physical outputs are intentionally corrected: both old normals are reversed and
the witnesses do not match the reported depth. The final cone uses Left and the
cylinder uses `(-4290580198,-194076362,0)` raw; the old cylinder has the
opposite sign and an additional one-raw normalization difference. These are not
physically equivalent output controls.

Measurements use Release/net8.0, BenchmarkDotNet 0.15.8, one out-of-process
launch, five warmups, ten measured iterations and affinity mask 3, without
competing builds/tests/profilers. Host: Windows 11 `10.0.26300.9711`, i7-9700K
3.60 GHz, .NET 8.0.29 X64 RyuJIT, SDK 10.0.302, concurrent workstation GC. Both
source and dependency stacks were frozen. The pre-refinement runs used balanced
order baseline A3, candidate B2, candidate B3, baseline A4, with identical
fixtures and job settings. Each table entry averages its two independent run
means.

The first complete fan cost 6.105/7.677 ms for cone/cylinder oblique contacts
and regressed the ordinary controls too. EventPipe attributed much of the cost
to analytic ranking and stationary rim selection. The retained refinements:

- Run complete admission before positive fan traversal, retain a proven
  contained-ball pole certificate and reduce common geometry content.
- Reuse exact cached vertices and typed rational dots; reject real-edge and
  generator directions outside their actual supporting feature before ranking.
- Skip dominated rim charts using exact hemisphere and projected-disk bounds;
  reduce the rim basis and reuse value construction across admitted roots.

Balanced internal controls favored the eight signed reciprocal charts with the
stronger Euclidean bound: cone/cylinder means 2.870/2.758 ms versus 2.971/2.829
ms for the simpler bound without basis reduction. Horizontal charts, four
quadrants and extra per-normal GCD reduction were measured and retired; they
added cost or offered no useful gain. Adverse captures are preserved.

| Dispatcher query  | Starting mean (us) | Retained mean (us) | Ratio |
| ----------------- | -----------------: | -----------------: | ----: |
| ConeSeparated     |             298.12 |              88.85 | 0.30x |
| ConeFace          |             271.80 |             192.09 | 0.71x |
| ConeOblique       |             531.37 |            2849.16 | 5.36x |
| CylinderSeparated |             455.13 |             139.63 | 0.31x |
| CylinderFace      |             266.97 |             185.89 | 0.70x |
| CylinderOblique   |             581.14 |            2733.96 | 4.70x |

All 24 child executions succeed, with six complete statistics per run. Every raw
GC row has zero Gen0/1/2 collections and zero allocated bytes; exported
MemoryDiagnoser values are also zero. Miss/face mean reductions are about 69-70%
/ 29-30%. The oblique costs rise by 2.318 ms for cone and 2.153 ms for cylinder
versus the old bound. Correct minimum normals and matched finite witnesses are
the intended behavioral difference, not output equivalence.

PE CodeView/PDB identity and every authored PDB document checksum bind each
actual benchmark child to its frozen source. Counts per payload are FMS 324
baseline / 331 retained, Gravitas 402, Chronicler 29, SwiftCollections 80 and
GridForge 121. Logs show local-stack child build arguments or inherited
local-stack selection; the compiled source checks establish the loaded local
payload rather than relying on the property alone.

A separate identical-source dispatcher probe runs all six fixtures against both
stacks. It preserves the adverse baseline results and records raw depths,
normals and physical witness points. The final cone depth passes a BigInteger
nearest-even oracle for `1/4 + (sqrt(1-Ax²)+Ax)/2`, using the exact rational
quaternion axis `Ax=2*(x*y-z*w)/(x²+y²+z²+w²)` from the collider's retained
rotation. Its normal is Left. The final cylinder depth and normal match the
complete equivalent-box owner. The old normal/witness mismatches are reproduced
in `artifacts/issue034-dispatcher-baseline.jsonl`; corrected outputs are in
`artifacts/issue034-dispatcher-final.jsonl`. The ordinary miss/face controls
improved during refinement; oblique queries remain more expensive than the old
incomplete solver. This is an unresolved measured tradeoff, not an unrecorded
follow-up or a claim of zero overhead. Those costs prompted the requested
throughput refinement recorded below, with matched measurements and coordinated
validation on its retained source.

Evidence resides under ignored `artifacts/issue034-*`; snapshots are under
`artifacts/issue034-perf`. Baseline A1/A2, first fan B1, typed EventPipe,
balanced S1/T1/T2/S2 controls, horizontal H1, quadrants J1 and normal-content K1
are retained. The pre-refinement evidence uses
`artifacts/issue034-final-{A3,B2,B3,A4}` and matching `-payload.json` records;
additional dependency bindings are in
`artifacts/issue034-final-upstream-payload.json`. Numeric comparison is in
`artifacts/issue034-performance-summary.json`.

## Throughput refinement

A profile of the staged correctness repair attributes 47.5%/53.6% of sampled
cone/cylinder activity to rim selection and 40.5%/35.1% to the analytic fan.
These are inclusive, nonadditive percentages. Ignored evidence:
`artifacts/issue034-refine-profile-summary.json`.

The retained changes select the supporting extrusion level once for rational and
quadratic normals, reject required edge endpoints on the wrong level while
allowing exact zero ties, and reuse `Half=T*Axis` for axial projection. Affine
support constraints nonnegative at both chart endpoints skip root-sign work. Two
shared Signed832 multiplication overloads return immediately for zero operands.
The net runtime change is 15 physical lines across three existing files, with no
new owner, persistent state or API. Eight test lines add four BigInteger
zero-operand assertions.

Successive diagnostic pilot means are recorded below; these are not balanced
performance acceptance results. Capture names are relative to ignored
`artifacts/`.

| Capture                     | Cone oblique (ms) | Cylinder oblique (ms) |
| --------------------------- | ----------------: | --------------------: |
| `issue034-support-level-R2` |             2.544 |                 2.505 |
| `issue034-support-zero-R3`  |             2.475 |                 2.374 |
| `issue034-chart-clip-R4`    |             2.497 |                 2.426 |
| `issue034-axis-reuse-R5`    |             2.473 |                 2.293 |
| `issue034-level-skip-R6`    |             2.329 |                 2.349 |

Chart-interval clipping added 26 runtime lines and was retired. The axis-reuse
cylinder child recorded 336 process-wide bytes over 256 operations, exported as
1 B/op; that capture does not attribute the delta. Both level-skip children
recorded zero raw allocated bytes.

The support-level pilot passed 189 focused rows; each subsequent pilot
passed 282. The latest focused report has complete raw line/branch coverage for
the changed methods, including the wrong-level precheck; `TryBuildCandidate` has
cyclomatic complexity 24.

The final balanced comparison uses the exact staged correctness repair as A and
the retained level-skip refinement as B, in order A1/B1/B2/A2. Each operation is
one warmed `CollisionDetectionMixed.TryCollide` query on two prepared colliders.
Release/net8.0 jobs retain one process launch, five warmups, ten measured
iterations and affinity mask 3. Each entry averages the two run means for its
variant; the baseline already supplies the corrected minimum contacts.

| Dispatcher query  | Accurate repair (us) | Refined (us) | Change |
| ----------------- | -------------------: | -----------: | -----: |
| ConeSeparated     |                90.64 |        81.70 |  -9.9% |
| ConeFace          |               191.56 |       124.54 | -35.0% |
| ConeOblique       |             2,769.20 |     2,421.03 | -12.6% |
| CylinderSeparated |               137.74 |       125.11 |  -9.2% |
| CylinderFace      |               186.27 |       120.27 | -35.4% |
| CylinderOblique   |             2,635.03 |     2,368.59 | -10.1% |

All 24 children exit successfully with complete statistics. All 24 raw GC rows
record zero Gen0/1/2 collections and zero allocated bytes. Captures use
`artifacts/issue034-optimized-{A1,B1,B2,A2}` with matching logs and
`-payload.json` records. Each payload record binds 331 authored FixedMathSharp
documents and 402 Gravitas documents to its frozen child; the summary is
`artifacts/issue034-optimized-performance-summary.json`. The requested ten
measurements retain eight to ten observations after BenchmarkDotNet's outlier
exclusions. All four transitive payloads also bind 29 Chronicler, 80
SwiftCollections and 121 GridForge documents; records are in
`artifacts/issue034-optimized-upstream-payload.json`.

The actual dispatcher-output probe is byte-identical to the accurate repair for
all six fixtures, including depths, normals and both physical witnesses
(`artifacts/issue034-dispatcher-level-skip.jsonl`). The retained-source profile
attributes about 51%/55% of sampled native contact activity to rim-chart
processing and 37%/32% to analytic selection, for cone/cylinder respectively.
These inclusive fractions overlap; different operation counts prevent using
absolute sample totals as before/after timings. The captures are
`artifacts/issue034-optimized-profile` and its `-summary.json`. No further small
deletion emerged from the independent connected review. This is a bounded
refinement, not proof of optimality or high-volume capacity: ten queries at
these mean costs would consume roughly 24 ms serially before other simulation
work in a 31.25 ms/32 FPS step. That is a budget illustration, not a measured
host-step capacity. The remaining oblique cost stays with Issue 034.

### Refined-source validation

The ordinary-workspace matrix passes on the retained source, with zero failures
or skips and zero build warnings/errors. The table uses exact ReportGenerator
counts, not rounded percentages:

| Repository/configuration   | Core tests |           Lines |        Branches | Fully covered methods |
| -------------------------- | ---------: | --------------: | --------------: | --------------------: |
| FixedMathSharp Release     |      6,767 | 65,733 / 65,733 | 20,782 / 20,782 |         5,101 / 5,101 |
| FixedMathSharp ReleaseLean |      6,746 | 65,826 / 65,826 | 20,782 / 20,782 |         5,097 / 5,097 |
| Gravitas Release           |      7,449 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |
| Gravitas ReleaseLean       |      7,380 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |

Both FMS configurations also pass all 49 Chronicler tests. The full core suites
include the warmed allocation guards, 1 MiB worker with live 64 KiB caller
headroom, whole-shape clearance and actual Gravitas `Simulate`/`LateSimulate`
response regressions. Independent raw coverage inspection finds no uncovered
methods, executable lines or partial branches. FMS's raw Cobertura totals match
the table. Gravitas's raw XML roots separately report 50,700/50,700 lines,
18,878/18,878 branches and 5,371/5,371 fully covered method nodes for Release;
Lean reports 50,698/50,698, 18,878/18,878 and 5,370/5,370 respectively. The
canonical ReportGenerator representation has different totals, but both
representations are complete; each capture contains one input XML, with no
stale-capture merge. All 1,931 files in
`artifacts/issue034-optimized-final-source.json` match the ordinary workspace.
Evidence uses `artifacts/issue034-optimized-{Release,ReleaseLean}-*` in each
owning repository and `artifacts/issue034-optimized-validation-sequence.log`.
Broad smoke passes all 479 child executions and all 479 statistics across 26
reports, with the child DLL/PDB binding all 331 authored FMS documents to the
retained source (`artifacts/issue034-optimized-smoke-payload.json`). DocFX
completes with zero warnings/errors. All 26 local Markdown targets exist. The
smoke, catalog and documentation captures use the
`artifacts/issue034-optimized-{smoke,catalog,docfx}` prefixes. Independent
source, width, tie-order, raw coverage, paired timing and payload reviews are
clean.

The fresh broad smoke retains four process-counter warnings, each 336 bytes:
circle point containment (PID 30312, 115,968 operations), capsule 2D interval at
scale 1 (15324, 1,104), failed reparent (18252, 5,845), and ordinary strict
cylinder/capsule relation (30732, 4,656). These individual observations remain
unattributed in `artifacts/issue034-optimized-smoke.log`; the R5 pilot's
separate 336-byte observation also remains preserved. The matched 24
curved-prism children all have zero raw counters, and the warmed guards pass,
without attributing or erasing those process-wide observations. The archived
allocation diagnostic linked below retains the investigation and recurrence
criteria. No counter suppression or tolerance was introduced.

## Exact rim normal-cone refinement

The committed accurate implementation (`5f2bf65`, Gravitas `0560128`) is this
experiment's baseline. A proper prism edge exposes the positive span of its two
inward incident face normals. Previously the rim solver searched eight signed
reciprocal charts in the complete edge-perpendicular plane, then rejected roots
outside that support cone. The retained refinement certifies the two planes
against the complete prism once and searches their single affine chart,
`n(t) = (1-t)*first + t*last`, with the existing stationary quartic and exact
value/witness machinery. Earlier analytic features already own its endpoints.

The certificate uses actual bottom/top/vertical topology and exact support
values, so winding does not change the result. Zero, ambiguous and parallel
incident planes retain the complete general-chart fallback. No convexity guess,
rounded normal or approximate root enters admission. Primitive plane directions
stay below 165 bits and their affine difference below 166; the existing
forty-word polynomial fields and witness bounds remain sufficient.

A changed parameterization can encounter equal positive minima in a different
order. Only retained or tied candidates recover their historical signed chart
and root order through exact linear sign queries. This keeps canonical normals
and paired witnesses stable without solving the old charts again. At zero gap,
stationarity places the origin on the strictly convex projected ellipse; its
unique supporting normal permits the existing immediate return.

This changes two existing runtime owners, with no new file, persistent state,
cache, dependency or public API. The general fallback remains necessary for
collapsed and repeated-vertex footprints. Independent review checked the
complete support domain, sign polarity, fixed-width bounds, reciprocal order and
zero-gap uniqueness proof.

### Measured normal-cone result

Final timings compare frozen `cone-baseline` with `cone-checked` in ABBA order,
using the unchanged six-case actual Gravitas dispatcher benchmark. The jobs use
Release/net8.0, one launch, five warmups, ten requested measurement iterations,
affinity mask 3 and the same local five-repository stack. BenchmarkDotNet
retains nine samples after outlier analysis for A1's two separated controls;
other cases retain ten. Each table entry equally weights the two independent
launch means.

| Actual dispatcher query | Committed baseline | Normal-cone refinement |  Change |
| ----------------------- | -----------------: | ---------------------: | ------: |
| Cone separated          |          83.782 us |              82.618 us |  -1.39% |
| Cone face               |         123.927 us |             125.360 us |  +1.16% |
| Cone oblique            |       2,407.369 us |           2,042.862 us | -15.14% |
| Cylinder separated      |         129.071 us |             128.703 us |  -0.28% |
| Cylinder face           |         125.658 us |             121.208 us |  -3.54% |
| Cylinder oblique        |       2,356.575 us |           2,109.566 us | -10.48% |

Both oblique candidate means beat both baseline means. The cone face adverse
point estimate is 1.43 us, compared with a 4.98 us baseline launch spread and
2.57--3.14 us candidate within-run standard deviations. Retain this variation;
it establishes neither a confirmed regression nor a general zero-overhead claim.
Small separated/face changes are controls, not attributed optimization wins. All
24 children exit successfully, have statistics and report zero raw GC/allocation
counters. The real dispatcher reproduces all six baseline records exactly,
including classification, raw depth, normal and both physical anchors.

PE/PDB and authored-source checks bind each A/B child payload to its frozen
source. Repeated launches use identical assembly/PDB hashes within each variant.
A complete candidate child also binds 331 FMS, 402 Gravitas, 29 Chronicler, 80
SwiftCollections and 121 GridForge authored documents. Runtime sources match the
ordinary workspace allowing only checked CRLF differences. Captures use
`artifacts/issue034-cone-{A1,B1,B2,A2}*`,
`issue034-cone-performance-summary.json`, `issue034-cone-stack-payload.json` and
`issue034-dispatcher-cone-checked.jsonl`. A1's Windows PowerShell 5 wrapper
failed only after all benchmarks completed because its runtime lacks the
metadata reader type. Its raw capture is preserved; PowerShell 7 successfully
rebound that retained payload before B1/B2/A2. No child timing or counter was
replaced. Both the wrapper failure and recovery are retained in the measurement
sequence logs.

The pilot sampling profile precedes the final zero-only simplification and is
diagnostic, while ABBA measures the final runtime. Inclusive rim traversal falls
from about 52% to 37% of sampled cone activity and 57% to 52% for cylinder;
parameter construction falls from 16.9% to 7.2% and 15.6% to 13.7%,
respectively. Analytic candidates remain about 44%/34% of sampled activity. The
new complete cone certificate itself accounts for about 5.6% of cylinder
activity. These overlapping inclusive fractions are neither additive time
savings nor polynomial counts. The remaining roughly 2 ms oblique cost is
distributed across exact candidate construction, algebraic sign/rank comparisons
and finite witnesses. This refinement does not establish universal frame
capacity or an optimal algorithm. Pilot evidence uses
`artifacts/issue034-cone-pilot-profile*`.

The runtime diff adds 102 and removes six lines, net **96**, in two existing
owners. Sixteen new maintained regression rows use full-volume exact rim touches
and one-raw neighbors, alternate authored frames, independently derived
equal-root minima, cap ties, analytic/rim transitions and equivalent
repeated/degenerate footprints. They separately check finite witnesses and
complete-shape clearance. The committed-runtime control passes all 29 feature
rows present at that checkpoint. The final focused candidate passes 298 rows
with complete raw line/branch/method coverage of both changed files; its
additional repeated-tip case exercises a surviving partially admitted support
wall with exact `d^2 = 3375/16`. Earlier failed coverage captures remain
retained; no exclusion, tolerance change or test-only production seam closes the
gap.

### Coordinated normal-cone validation

The final ordinary-workspace matrix uses the unchanged canonical configurations
and local source dependencies. Every row passes without failures or skips, and
all four solution builds have zero warnings/errors. Counts below use the
canonical ReportGenerator representation:

| Repository/configuration   | Core tests |           Lines |        Branches | Fully covered methods |
| -------------------------- | ---------: | --------------: | --------------: | --------------------: |
| FixedMathSharp Release     |      6,783 | 65,794 / 65,794 | 20,860 / 20,860 |         5,104 / 5,104 |
| FixedMathSharp ReleaseLean |      6,762 | 65,887 / 65,887 | 20,860 / 20,860 |         5,100 / 5,100 |
| Gravitas Release           |      7,449 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |
| Gravitas ReleaseLean       |      7,380 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |

Both FMS configurations also pass all 49 Chronicler rows. Each coverage
directory contains exactly one input Cobertura XML; no executable line, branch
condition or method is partial or unhit. FMS raw totals match the table.
Gravitas raw XML roots separately report 50,700/50,700 lines, 18,878/18,878
branches and 5,371/5,371 fully covered method nodes for Release; Lean reports
50,698/50,698, 18,878/18,878 and 5,370/5,370. Both representations are complete.
The final 1,931-file actual-byte checkpoint matches the ordinary workspace. The
existing actual host-step/replay, full-domain, one-MiB worker-stack and warmed
allocation regressions remain green.

Evidence is `artifacts/issue034-cone-{Release,ReleaseLean}-*` in each owning
repository, `issue034-cone-validation-sequence.log`,
`issue034-cone-validation-summary.json` and `issue034-cone-final-source.json`.

The final bounded broad smoke passes all 479 child executions and 479 statistics
across 26 reports. Its child PE/PDB binds all 331 authored FMS documents to the
retained runtime. DocFX completes with zero warnings/errors; all 23 local
Markdown targets exist. Captures use
`artifacts/issue034-cone-{smoke,catalog,docfx}*` and
`issue034-cone-smoke-payload.json`. This short smoke is harness/runtime
validation rather than throughput acceptance evidence. Independent runtime,
width, tie-order, exact-output, payload, ABBA and four-mode raw coverage reviews
are clean.

The broad smoke preserves four 336-byte process-counter observations: triangle
3D containment (PID 31784, 10,240 operations), strict sphere/box relation
(12212, 4,336), rigid-frame anchor reexpression (8428, 864), and axis zero-core
2D circle contact (26300, 2,112). These callers do not enter the changed
curved-prism owners. The individual observations remain unattributed in the raw
log; all 24 matched curved-prism children report zero raw counters and the
warmed allocation guards pass. The
[archived allocation diagnostic](../benchmark-signal-hardening-backlog.md#capsule-interval-process-wide-allocation-observations)
retains the earlier evidence and recurrence criteria. No counter suppression,
tolerance or runtime GC change is introduced.

## Exact-zero certificates and support-scan follow-through (2026-10-20)

The next refinement starts from committed `43500eb`, with Gravitas `0560128`. It
retains two small reductions, using the existing collectors and arithmetic:

- After complete Issue 033 admission, every support gap is nonnegative. An exact
  initial zero therefore certifies the global minimum before analytic traversal.
  A zero found later ends the remaining real-edge, cylinder-vertex or
  cone-generator loops. Strict candidate ranking already retains the first equal
  result, so these exits preserve its feature, normal and both witnesses.
  Rounded zero is deliberately insufficient for this certificate.
- Incident wall normals are perpendicular to native Up. Bottom and top prism
  vertices therefore have identical wall projections. One bottom-level scan
  records the minimum and maximum signs relative to the incident edge point;
  their sum identifies the unique supporting orientation. Mixed signs and an
  entirely coplanar footprint retain the existing general-chart fallback. This
  replaces two complete support queries without changing their result.

Thirteen added public regression rows cover initial zero with penetrating and
separated neighbors, an exterior footprint whose vertical gap remains zero, the
canonical first edge on a constant cylinder-side patch, and a smooth
cylinder-side contact at a strict prism vertex. The last fixture has
`A=(4,3,0)/5`, `P=(3,-4,12)` and radius `13`: `A.P=0`, `|P|=13`, and the plane
`3X-4Y+12Z=169` proves the unique touch. An inward raw multiple of `P` places a
radius-13-raw ball inside the cylinder centered at the prism vertex; the
matching support plane proves the complete minimum, rather than just
anchor-distance agreement.

The Gravitas `curved-prism-contact` group retains its six original cases and
adds exact cone-apex, cone-generator and horizontal cylinder-side touching
controls. Setup checks classification, zero unclamped depth, normal and both
finite physical witnesses. The generator control uses `2X+Y=1/2`, whose unique
common point is `(1/4,0,0)`. The cylinder uses an exactly representable cyclic
axis permutation. Earlier fixture attempts changed by one quaternion raw unit
during host-transform matrix reconstruction and produced one/two raw units of
penetration; those failed captures are preserved and excluded from timing
evidence. The direct FMS oblique fixture retains its authored frame and its
exact-neighbor assertions.

The investigation also discarded several correct but unhelpful experiments:

- Early required-edge scan failure and a signed endpoint-offset bound did not
  establish a convincing net gain in their pilots. Their captures are
  `artifacts/issue034-{support-scan,signed-bound}-*`.
- A cylinder-only nonpositive-projection rejection improved cylinder oblique
  time by 5.20% in a balanced comparison, but cone oblique time worsened by
  3.97%; its added candidate flags were removed. The complete six-case ABBA
  evidence remains in `artifacts/issue034-radial-abba-*`.
- A tighter chart radius bound was mathematically valid and stayed within the
  existing forty-word fields. Its combined eight-case ABBA pilot showed only
  0.57%/1.44% lower cone/cylinder oblique means, with a 2.80% adverse cone-face
  estimate. No confirmed benefit specific to its extra arithmetic was
  established, so that arithmetic was removed. Preserve
  `artifacts/issue034-certificate2-*` as a discarded pilot, not retained-source
  performance evidence.
- A whole-chart apex certificate was rejected by reach analysis on the ordinary
  cone fixture. No measured gain is claimed for that unretained gate.

### Retained-source performance

The final balanced ABBA comparison uses the committed `43500eb` runtime and the
retained source, with identical nine-case benchmark source on both sides.
Release out-of-process jobs use five warmups, ten measured iterations, one
launch per case and affinity `3`. All children inherit `UseLocalLsfStack=true`.
Each table mean weights the two launch means equally; times are microseconds.
All ten measurement iterations ran; BenchmarkDotNet retained eight to ten
observations per case after its outlier filtering.

| Dispatcher case              |    Before |     After |  Change |
| ---------------------------- | --------: | --------: | ------: |
| Cone separated               |    82.086 |    82.944 |  +1.04% |
| Cone face penetration        |   125.880 |   122.459 |  -2.72% |
| Cone oblique penetration     | 2,044.753 | 1,959.352 |  -4.18% |
| Cylinder separated           |   129.457 |   126.341 |  -2.41% |
| Cylinder face penetration    |   129.077 |   126.141 |  -2.27% |
| Cylinder oblique penetration | 2,074.251 | 2,071.732 |  -0.12% |
| Cone apex touch              |   542.536 |   116.185 | -78.58% |
| Cone generator touch         |   659.718 |   510.822 | -22.57% |
| Cylinder side touch          |   472.473 |   142.354 | -69.87% |

The touching reductions are consistent across both retained launches. Ordinary
oblique cylinder time is effectively unchanged; the cone's lower mean remains a
modest fixture-specific result. Keep the adverse separated-cone estimate:
baseline launches are 81.697/82.476 us and retained launches are 79.981/85.906
us. This does not establish a general zero-overhead claim or a universal
simulation budget. Penetrating oblique contacts still cost about two
milliseconds each.

All 36 children exit zero, supply complete statistics and report raw
`GC: 0 0 0 0 <operations>` counters. Before timing, the actual dispatcher
returns identical classifications, raw depths, normals and both physical
witnesses for all nine fixtures. Independent cone support and complete
oriented-box/cylinder oracles also pass. Child PE/portable-PDB identities bind
authored source hashes for FMS (331 documents), Gravitas (402), the benchmark
fixtures (79), Chronicler (29), SwiftCollections (80) and GridForge (121).

Captures are `artifacts/issue034-minimum-{A1,B1,B2,A2}*`,
`issue034-minimum-performance-summary.json`, the two
`issue034-dispatcher-minimum-{base,checked}.jsonl` files and the per-launch
`*-payload.json` records. The frozen runtime variants are `minimum-base` and
`minimum-checked`; earlier pilots do not substitute for this retained source.

The retained-source EventPipe diagnostic still splits oblique work between
analytic collection and rim charts. Inclusive shares of benchmark activity are
42.9%/34.9% for `KeepAnalytic` and 38.9%/44.6% for `KeepChart` (cone/cylinder).
Nested work includes `TryBuildCandidate` at 18.2%/25.0%, `BuildParameter` at
12.6%/13.1%, and exact candidate comparison at 9.6%/9.7%. These overlapping
sampled shares are diagnostic, not additive costs or acceptance timings. The
two-case capture is `artifacts/issue034-minimum-final-profile*`. Further gains
need enough eliminated candidate/root work to outweigh the proof arithmetic; the
discarded bounds do not justify another retained helper or representation.

### Retained-source validation

The ordinary-workspace matrix passes with `UseLocalLsfStack=true`, no failed or
skipped tests, and zero warnings/errors in all four solution builds. Exact
ReportGenerator counts are:

| Repository/configuration   | Core tests |           Lines |        Branches | Fully covered methods |
| -------------------------- | ---------: | --------------: | --------------: | --------------------: |
| FixedMathSharp Release     |      6,796 | 65,798 / 65,798 | 20,870 / 20,870 |         5,104 / 5,104 |
| FixedMathSharp ReleaseLean |      6,775 | 65,891 / 65,891 | 20,870 / 20,870 |         5,100 / 5,100 |
| Gravitas Release           |      7,449 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |
| Gravitas ReleaseLean       |      7,380 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |

Both FixedMathSharp modes also pass all 49 Chronicler rows. All 1,931
runtime/test source checkpoint hashes match after the complete matrix. The
captures use `artifacts/issue034-minimum-{Release,ReleaseLean}-*` in each owning
repository; `issue034-minimum-validation-sequence.log` records the serial run.
Independent inspection also finds no unhit raw line nodes, partial branch lines
or partially covered methods. FMS raw Cobertura counts match the table.
Gravitas's separate literal Cobertura/OpenCover counts are 50,700/50,700 lines,
18,878/18,878 branches and 5,371/5,371 method nodes in Release, and
50,698/50,698 lines, 18,878/18,878 branches and 5,370/5,370 method nodes in
Lean. These raw representations are kept separate from ReportGenerator totals.

The final broad smoke passes all 479 child executions with statistics across 26
reports (`all -j Short --iterationTime 10`). Its child PE/PDB binds all 331
authored FMS documents to the retained source. These bounded smoke times do not
replace the nine-case ABBA evidence. Captures are
`artifacts/issue034-minimum-smoke*`.

The smoke retains six 336-byte process-counter observations on unchanged paths:
frustum/ray intersection (7,168 operations, process 18144), ranged random
`Fixed64` (288, process 25932), vertical triangle subdivision sweep (2, process
9396), penetrating cylinder rims (1, process 32188), scalar-face capsule/slab
projection (432, process 4248), and static vector subtraction (944, process
31808). The individual deltas remain unattributed; no calling-thread allocation
or workload-dependent growth is established. The
[archived allocation diagnostic](../benchmark-signal-hardening-backlog.md#capsule-interval-process-wide-allocation-observations)
retains matched finalizer evidence and recurrence criteria. The 36 matched
curved-prism timing children all have zero raw counters; these smoke
observations do not change that result. No counter suppression or tolerance was
introduced.

Independent source, geometric proof, retirement, payload, timing, profile and
raw coverage reviews found no actionable issue in the retained refinement. DocFX
completes with zero warnings/errors, and all 25 local Markdown targets in the
changed documentation exist. Both repositories' diffs pass whitespace checks.

## Pre-refinement validation

The pre-refinement ordinary-workspace matrix passed without failures or skips.
The following are exact ReportGenerator counts rather than rounded percentages:

| Repository/configuration   | Core tests |           Lines |        Branches | Fully covered methods |
| -------------------------- | ---------: | --------------: | --------------: | --------------------: |
| FixedMathSharp Release     |      6,767 | 65,725 / 65,725 | 20,764 / 20,764 |         5,101 / 5,101 |
| FixedMathSharp ReleaseLean |      6,746 | 65,818 / 65,818 | 20,764 / 20,764 |         5,097 / 5,097 |
| Gravitas Release           |      7,449 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |
| Gravitas ReleaseLean       |      7,380 | 66,205 / 66,205 | 25,446 / 25,446 |         6,646 / 6,646 |

Both FixedMathSharp modes also pass all 49 Chronicler rows, with zero skips. All
four solution builds have zero warnings/errors. The runtime/test checkpoint
matched all 1,931 ordinary-workspace files after that validation. Frozen
benchmark source content was separately compared allowing only checked CRLF
differences; DLL/PDB checks bind the benchmark payload to the frozen source
bytes. The bounded broad smoke passes all 479 child executions and all 479
statistics across 26 reports (`all -j Short --iterationTime 10`). This is a
harness/runtime smoke check, not performance acceptance evidence. Its child PDB
binds all 331 authored FMS documents to the retained source. DocFX also
completes with zero warnings/errors. All 26 local Markdown targets exist.
Independent source, retirement, raw coverage, payload, ABBA, dispatcher-output
and diagnostic reviews reported no actionable finding for the pre-refinement
repair. The retained-source refinement and its separate validation are recorded
above. The broad smoke also preserves five raw process-counter warnings, each
336 bytes despite a rounded zero memory summary: OBB point containment (5,120
operations), four-vertex/30-degree convex containment (2,064), checked inverse
add/subtract (816), two-word/32-bit division (79,520) and axis-circle contact
(2,016). The processes are 29888/24620/30772/20048/16912 respectively, retained
in `artifacts/issue034-smoke.log`. These are unchanged workload paths and are
separate from the 24 matched curved-prism children, whose raw counters are all
zero. This capture does not attribute these five individual deltas. The
[archived allocation diagnostic](../benchmark-signal-hardening-backlog.md#capsule-interval-process-wide-allocation-observations)
retains the earlier matched finalizer evidence and recurrence criteria; neither
calling-thread allocation nor workload-dependent growth is established here. No
counter suppression, tolerance or runtime change is introduced.

The pilot focused coverage was complete, but is not substituted for final source
verification. Required builds must be warning-free in the ordinary workspace;
nested frozen benchmark checkouts can emit duplicate parent `.globalconfig`
settings warnings without changing the runtime payload.

Portable reproduction uses the normal local-stack workflow. Set
`UseLocalLsfStack=true` in the environment as well as the CLI for generated
benchmark children, build the benchmark runner, then run:

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~FixedConvexPrism'
# From the sibling Gravitas checkout:
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll curved-prism-contact -j Short --warmupCount 5 --iterationCount 10 --affinity 3 --keepFiles --exporters json
```

Tracing the old oblique normal mismatch also exposed the same pre-existing
inside-projection orientation defect in the preserved sphere/capsule reducer.
The public capsule and sphere reproducer fails identically on the starting and
retained stacks; its incorrect returned exit still overlaps while the
independently proved exit separates. It is recorded as **FMS-Issue-035**, with a
self-contained reproducer and next action in the issue tracker. It is outside
this cone/cylinder repair and no capsule behavior was changed after the final
checkpoint. Ignored diagnostic captures are
`artifacts/issue034-capsule-{baseline,final}.jsonl`.

Package-mode validation awaits the FixedMathSharp release. Local repeated replay
and Windows tests do not establish all-platform execution evidence.
