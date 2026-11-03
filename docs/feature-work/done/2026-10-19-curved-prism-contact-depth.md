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
2026-10-20. The remaining penetrating oblique cost was rechecked on 2026-11-02
and reopened for deeper exact-work reduction at the owner's request in the
[experimental benchmark signal](../benchmark-signal-hardening-backlog.md#penetrating-oblique-conecylinder-prism-contact-throughput),
not a production-throughput acceptance claim.

Earlier bounded reviews did not find another small change with a complete proof
and convincing reach on these penetrating fixtures. Their rejected experiments
remain below. The
[2026-11-03 exact workload reduction](#exact-workload-reduction-2026-11-03)
records the later substantive improvement, current validation and unresolved
experimental target.

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

## Pre-release throughput recheck (2026-11-02)

**Initial pass (superseded):** The first pass closed this investigation with no
runtime change. The existing solver's cost is reproduced; it is not a newly
established regression or an unresolved correctness defect. The guarded
shared-polynomial square pilot below does not establish a repeatable benefit on
the target queries. It and its five characterization test rows were removed. The
runtime, maintained fixtures, public API, internal ABI and allocation policy
remain unchanged.

The owner reopened the investigation on 2026-11-02 because the remaining cost is
too high. The measurements below describe the rejected first pilot; they do not
resolve the throughput concern. Ordinary penetrating oblique queries still cost
roughly 1.8 ms each here. No universal simulation/contact budget has been
accepted. Reopen for a regression on unchanged fixtures, a concrete reduction of
executed exact candidate/root work, or a representative moving-contact/full-step
profile with an explicit workload budget. Do not infer full-step capacity from
these queries.

### Current workload and attribution

Starting revisions are FixedMathSharp `4a2d433`, Gravitas `7990c17`, Chronicler
`c45f9ab`, SwiftCollections `13a3c13` and GridForge `50d5540`. GridForge
includes pre-existing local development edits; this work does not alter them or
claim a released-package dependency graph. The maintained Gravitas
`CurvedPrismContactBenchmarks` supplies nine cone/cylinder controls: separated,
face penetration and oblique penetration for each primitive, cone apex and
generator touches, and a cylinder side touch. The five sphere/capsule fixtures
in the same class also participate in the untimed contact/allocation checks.
These exercise the actual mixed-contact dispatcher, not a Boolean overlap
surrogate or a full host step.

Host: Windows 11 `10.0.26300.9818`, Intel i7-9700K, SDK `10.0.302`, .NET
`10.0.10`, BenchmarkDotNet `0.15.8`, Release out of process, affinity mask `3`,
`DOTNET_PROCESSOR_COUNT=2`. Environment and generated build arguments both
select `UseLocalLsfStack=true`, `DisableTransitiveProjectReferences=true` and
serial builds. The old .NET 8 measurements above remain historical; they are not
an equivalent-environment performance comparison with this recheck.

Fresh EventPipe samples place analytic collection at 48.4%/33.4% and rim charts
at 33.8%/52.0% of benchmark activity for cone/cylinder oblique queries. Nested
parameter construction is 6.9%/10.7%; the shared edge-polynomial builder is
6.2%/9.9%. These inclusive sampled shares overlap and must not be added. The
executed path still spends its time on exact candidate and root work.

### Rejected square pilot

`WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials` performs both `(i,j)`
and `(j,i)` products for self-squares. The pilot visits only `j>=i` and adds
off-diagonal products with shift one, using the existing signed accumulator. It
adds five net runtime lines, no helper, state, allocation, API or caller change.
Its gate requires all of:

- accumulation is disabled and the coefficient count `n` exceeds one;
- both input magnitude spans and both sign spans have identical addresses and
  extents;
- `2*b + ceil(log2(n)) <= 64*w`, where `b` is the maximum input coefficient bit
  length and `w` the result coefficient's word stride, using existing helpers.

The last condition bounds every partial absolute coefficient sum by `n*2^(2*b)`.
A width guard is necessary: with one-word input/output coefficients,
`[3200000000,0,1600000000,0,-3200000000]` squares correctly in the original
order, but combining the degree-four off-diagonal terms first produces
`-20480000000000000000` before diagonal cancellation. That intermediate exceeds
`ulong.MaxValue`, even though the final coefficient `-17920000000000000000`
fits. Insufficient-width inputs therefore keep the original row order.

BigInteger characterization uses `[2^70-1,-3,0,2^65+1]` with independent input
and output strides, shared and nonuniformly changed sign spans, clearing and
accumulation, plus the cancellation counterexample. All 20 arithmetic rows pass
on baseline and pilot. Independent review confirms the guard and exact fallback;
correctness alone does not justify retaining its extra scan and inner-loop
decisions. No throughput improvement is claimed for this pilot.

### Balanced measurements

A is the unchanged runtime and B the guarded square pilot. All four launches use
identical nine-case fixture source, five warmups, ten measured iterations and
one child per case. Every launch completes all 90 measured iterations and all
nine children exit zero with populated statistics; outlier filtering retains
eight to ten observations per row. Values below are microseconds. The last
column compares the equally weighted two-launch means; positive is slower.

| Dispatcher case   |        A1 |        B1 |        B2 |        A2 | Pilot change |
| ----------------- | --------: | --------: | --------: | --------: | -----------: |
| ConeSeparated     |    75.443 |    76.004 |    80.947 |    77.689 |       +2.49% |
| ConeFace          |   106.309 |   106.509 |   107.306 |   108.666 |       -0.54% |
| ConeOblique       | 1,770.388 | 1,796.079 | 1,794.746 | 1,814.229 |       +0.17% |
| CylinderSeparated |   118.404 |   120.153 |   118.894 |   121.745 |       -0.46% |
| CylinderFace      |   105.789 |   110.520 |   109.997 |   106.746 |       +3.76% |
| CylinderOblique   | 1,730.681 | 1,758.133 | 1,831.900 | 1,869.061 |       -0.27% |
| ConeApexTouch     |    97.415 |    98.131 |    99.897 |    99.531 |       +0.55% |
| ConeSideTouch     |   438.671 |   441.362 |   439.286 |   442.889 |       -0.10% |
| CylinderSideTouch |   119.544 |   120.828 |   121.225 |   123.993 |       -0.61% |

Keep the control variation and adverse estimates. The pilot does not establish a
penetrating-oblique win and does not warrant adding a general arithmetic branch
before release. This is a rejected experiment, not an accepted performance
regression or a claim of zero overhead.

### Contact identity, allocation and evidence limits

All 14 dispatcher fixtures retain identical classifications, raw depths,
clamping flags, normals and both materialized physical witnesses. In particular:

- Cone oblique: depth `3130426887`, normal `(-4294967296,0,0)`, primitive point
  `(5459507705,1173094659,6622560427)`, prism point
  `(8589934592,1173094659,6622560426)`.
- Cylinder oblique: depth `5416131436`, normal `(-4290580198,-194076362,0)`,
  primitive point `(3179335467,4050228975,7962578931)`, prism point
  `(8589934593,4294967296,7962578931)`.

These are raw Q32.32 values and both depths are unclamped. The maintained
geometric regressions above remain the correctness oracles; baseline identity
alone would not prove an arbitrary contact correct.

All pilot children have zero raw allocation and GC counters. The A1 baseline
cylinder-oblique child reports 24,624 process-wide bytes over 512 operations (48
B/op rounded). The clean A2 baseline cylinder-separated child also reports
24,624 process-wide bytes, over 8,192 operations (3 B/op rounded). Preserve both
observations. A separate warmed, non-inlined direct-dispatch probe measures
exactly zero calling-thread bytes over 128 queries for every one of the 14
fixtures on both baseline and pilot, after three warm passes. This does not
attribute the baseline process-wide event or establish every process/thread
allocation interval is zero.

Child PE/portable-PDB binding checks confirm both pilot launches' changed
arithmetic and current contact owner (410 authored math documents), unchanged
consumer (417 documents) and unchanged benchmark fixture source (88 documents).
BenchmarkDotNet reuses its generated job directory: A1's child payload was
overwritten before its identity audit. Its baseline source lifecycle and build
log were recorded; its unmodified parent binary matches the separately checked
baseline profile, but it has no separately preserved child identity proof.

An initial A2 return was rejected: restoring a source copy also restored its
older timestamp, so the incremental build reused the candidate binary. The PDB
source audit caught this. That nine-child capture is preserved as `oblique-A2`
and excluded from the table. Updating the source timestamp and rebuilding
explicitly with `-t:Rebuild` precedes the replacement `oblique-A2-clean`, which
supplies A2 in the table and passes its source audit. The pilot and final
baseline child binaries/PDBs are saved before any further build. This limit
precludes claiming all 36 accepted children have separately frozen payloads;
neither a stale capture nor incomplete binding evidence justifies retaining the
pilot.

At the end of the initial square pilot, no runtime or maintained test change
survived; that pass claimed no new full Release/Lean coverage matrix, broad
benchmark smoke or complete simulation replay. Historical validation above
applies only to its recorded revisions. Final checks cover source restoration,
the changed documentation, local links and whitespace. Independent review covers
the rejected pilot and this disposition.

### Reproduction

From the sibling Gravitas repository root:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DisableTransitiveProjectReferences = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
$env:BuildInParallel = 'false'
$env:UseSharedCompilation = 'false'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net10.0 -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -p:UseSharedCompilation=false -m:1
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll curved-prism-contact --filter '*Cone*' '*Cylinder*' --warmupCount 5 --iterationCount 10 --launchCount 1 --affinity 3 --keepFiles --exporters json --artifacts ../FixedMathSharp/artifacts/oblique-A1
```

For diagnostic attribution, replace the timing options with
`--profiler EP --warmupCount 2 --iterationCount 3 --launchCount 1 --affinity 3 --keepFiles`
and use `artifacts/oblique-profile`. For a new pilot, preserve the exact source
and child DLL/PDBs before the next run overwrites the generated job directory;
keep fixture source identical and repeat in A1/B1/B2/A2 order with distinct
artifact directories. Characterization uses the existing
`FiniteAxisPolynomialArithmeticTests` owner; the mathematical gate, loop change
and discriminating inputs above describe the discarded pilot without relying on
temporary source copies.

Supplemental captures are `artifacts/oblique-{A1,B1,B2,A2-clean}*`,
`oblique-profile*`, `oblique-{baseline,candidate}-contacts.jsonl`,
`oblique-{baseline,candidate}-allocations.jsonl` and `oblique-summary.json`. The
revisions, fixtures, commands, pilot definition, measured values and decision in
this record are the durable evidence if those captures are removed.

## Exact candidate/root reduction (reopened 2026-11-02)

The owner requested deeper reduction of exact work and preservation of this
unresolved cost under Experimental Signals. Retain the rejected square pilot as
evidence, not as a reason to close the throughput problem. The provisional
investigation target is at least 2x lower cost on both penetrating fixtures,
with unchanged exact outputs, zero warmed calling-thread allocation and no
confirmed regression across the nine controls. It does not define a universal
simulation-step budget.

Attribute candidate construction/admission, analytic ranking, chart/root
isolation, root rejection, value mapping and final witnesses before changing the
inventory. Investigate mathematical redundancy, repeated proof work and
stationary extrema that cannot own a minimum. Reuse the existing owners; keep
disposable counters and timing probes out of the maintained runtime. Preserve
source and child payload identity before each variant is overwritten.

### Executed work and retained changes

Disposable counters on the unchanged dispatcher fixtures identify analytic
ranking and repeated root-sign proofs as useful targets. Cone performs 132
rational attempts (48 built, 58 rank comparisons), 34 generator attempts (10
built), and 12 rim charts (five dominated, seven quartics, 14 roots). Cylinder
performs 172 rational attempts (70 built/ranked) and 24 charts (six axial
rejects, six dominated, 12 quartics, 19 roots). Six cone and seven cylinder
roots pass the unsquared stationary gate; five and six respectively are strict
maxima. Every admitted root is rejected by the existing analytic upper bound:
neither fixture executes value-polynomial construction or root value mapping.
Optimizing that unexecuted machinery would not help this signal. The fixtures
make 55/57 root-sign calls, with 14/19 retained refinement calls. Of 25/31
linear queries, 25/28 have matching nonzero cell-endpoint signs. These are work
counts, not instrumented timing claims.

Three changes remain in the existing production owners:

- `CurvedPrismRimContacts.KeepChart` rejects strict stationary maxima only for
  certified ordinary support fans with square-free stationary polynomial
  `W=L²-K²Q`. At an admitted root `L=-K sqrt(Q)`, the sign of the gap's second
  derivative is `-sign(W') sign(K)`. Simple roots alternate `W`'s sign, starting
  from its first nonzero coefficient; excluded zero endpoints keep the same
  positive-side sign. `K=0` at a simple root is impossible, since it would also
  force `L=0` and a repeated root of `W`. Existing analytic candidates own fan
  endpoints. Uncertified, constrained and repeated-root charts retain the
  complete original admission path. Root ordinals and canonical ties are kept.
- `GetSignAtFiniteValueRootCore` uses the actual open dyadic cell of a
  nonsingleton root view. A linear query with matching endpoint signs has that
  exact sign throughout; if one endpoint is zero, the other determines the
  interior sign. Opposite signs retain exact refinement and shared-root
  equality. This avoids requiring a narrower cell solely to establish a sign
  already certified.
- `GetConvexContactCandidateThreeTermSign` first encloses each square root using
  the leading 63 or 64 radicand bits after an even discarded shift. If the
  integer floor square root of that prefix is `r`, the complete root lies
  between `r*2^shift` and `(r+1)*2^shift`, including discarded nonzero bits and
  exact prefix squares. Signed coefficients reverse endpoints as required. Two
  carry words cover the sum of all three complete shifted terms. A strict
  positive lower bound or negative upper bound proves the exact rank;
  cancellation, overlap and equality retain the original radical elimination.
  Pure-radical comparisons skip the enclosure and retain their existing exact
  reduction, avoiding unsuccessful bound work on zero-rational ties. Existing
  quadratic-bound accumulation is reused, not duplicated.

No public API, serialized state, floating-point runtime math, cache or
dependency pin changes. Runtime growth is confined to these three files. Tests
cover open cell endpoint zeros and interior equality, wide radical cancellation
and ties, a stationary chart whose constant coefficient is zero, winding
reversal, whole-shape exit, and compact/noncompact retained cells. Two older
tests now use a query threshold inside the initial cell so their original
refinement and precision assertions continue to test necessary work. The capsule
value test retains its rounding and scalar-limit assertions and adds a pair
centered at 1/3 that forces deep isolation independently of derivative-query
refinement.

Screening also tried common-content normalization of edge directions; it added
work without a useful gain over the root changes and was removed. A 96-bit
ranking enclosure was superseded by the cheaper conservative enclosure above.
The disposable warmed ABBA screen suggested approximately 18–20% lower cost;
only the controlled BenchmarkDotNet captures below can support acceptance.

### Control-driven correction

The first retained-work pilot completes a fourteen-case dispatcher ABBA and six
adjacent math controls. Its cone/cylinder oblique means fall by 18.02%/19.87%,
and the adjacent box/cylinder control improves 14.02%. Retain these as pilot
results rather than final acceptance. The contained-capsule balanced estimate is
3.01% slower; a focused BAAB comparison reduces that estimate to 0.76% with
overlapping launch ranges, so a repeatable capsule regression is not
established. Cylinder side-touch similarly changes from +0.91% to -0.43%. Cone
generator side-touch is different: its +1.34% main estimate repeats as +2.59% in
the focused comparison, so the original enclosure policy is rejected.

Disposable branch counts isolate seven cone side-touch enclosure attempts; all
have exactly zero rational term, and all fall through to the original exact
comparison. Neither of the other two new optimizations executes in this fixture.
The final guard skips the optional enclosure for pure-radical comparisons,
retaining their existing exact reduction and equality behavior. Each penetrating
target also has one such unsuccessful attempt removed; its remaining 58/65
enclosure attempts certify 58/65 signs in this diagnostic. These are operation
counts, not timing evidence. Instrumentation is confined to an isolated
disposable source copy; none remains in maintained runtime code.

Independent review caught that equal-rational wide ranking tests would now
bypass the enclosure. Those remain as pure-radical controls, supplemented by
`1+sqrt(T²+1)-sqrt((T+1)²+1)>0` and `1+sqrt(T²)-sqrt((T+1)²)=0`, with
`T=2^160+17`, both signs and comparison orders. The strict inequality follows by
squaring and using `sqrt(T²+1)>T`; the conservative enclosure remains ambiguous,
so these exercise wide fallback and exact equality with a nonzero rational term.

The first broad smoke attempt executes no cases: with globally disabled
transitive project references, BenchmarkDotNet's generated project references
only the benchmark project, so generated harness signatures returning math types
cannot resolve the core assembly. The zero-execution capture is excluded. A
disposable `CustomAfterMicrosoftCommonTargets` import adds an explicit core
`ProjectReference` only to `BenchmarkDotNet.Autogenerated`. This preserves local
source selection and the required transitive-reference setting; no maintained
runner or project is changed. The corrected pre-guard smoke completes all 479
out-of-process Short cases. Final guarded-source checks are recorded below.

### Final guarded correctness and coordinated validation

Serial local-stack builds and complete test suites pass for the guarded source
in both repositories, in Release and ReleaseLean, with no build warnings or
errors. FixedMathSharp passes 7,273 Release and 7,252 Lean core tests, plus all
49 companion tests in each configuration. Gravitas passes 8,348 Release and
8,277 Lean tests, including existing simulation/replay scenarios. All four
stationary-endpoint rows and the nonzero-rational wide fallback cases are now
included in the full math suites.

Every raw OpenCover sequence point, branch point and method is covered in each
full capture, without new exclusions or coverage unions:

| Repository/configuration   | Sequence points | Branch points |     Methods |
| -------------------------- | --------------: | ------------: | ----------: |
| FixedMathSharp Release     |   68,400/68,400 | 22,598/22,598 | 5,324/5,324 |
| FixedMathSharp ReleaseLean |   68,493/68,493 | 22,598/22,598 | 5,320/5,320 |
| Gravitas Release           |   53,559/53,559 | 21,772/21,772 | 5,602/5,602 |
| Gravitas ReleaseLean       |   53,557/53,557 | 21,772/21,772 | 5,601/5,601 |

The initial standard pilot capture missed the zero-coefficient branch; its
focused geometry capture closed that gap. Rounded Cobertura percentages alone
would have concealed it. The final full captures include both outcomes. Both
configurations' Cobertura reports also cover every reported source line, branch
and method; normalized source-line/method counts need not equal OpenCover's
sequence-point/method-file counts. The updated complexity register matches raw
method metadata: 92 for `KeepChart`, 24 for three-term ranking and 46 for root
signs.

The scoped semantic formatter verification passes with workspace-load warnings
and no required source changes; whitespace and documentation formatting are
checked separately. Independent review checks the three numerical gates,
connected root-cell consumers, whole-shape tests, both wide sign polarities,
exact equality and the rejection of the measured control regression.

These are local-source compatibility checks. Validation against the exact
released math package remains a future release gate; no package publication,
source pin, public API or serialized state is changed.

### Guarded balanced measurements and residual work

**Disposition:** Retain the three guarded optimizations for owner review, leave
all changes unstaged/uncommitted, and keep the throughput signal open under
Experimental Signals. A roughly 1.42 ms query is still expensive; the
provisional 2x investigation target is not met. No universal query/frame budget
or complete throughput resolution is claimed.

A is the unchanged runtime; B is the final guarded source. The balanced window
uses the already preserved A2 baseline, then fresh B3/B4/A3 captures after the
final matrix. The baseline source and settings are unchanged; the gap before B3
contains serialized validation and diagnostic work, with no competing work
during acceptance timing. All fourteen maintained dispatcher fixtures run with
five warmups, ten measured iterations, a 500 ms iteration target and one launch
each. All 56 children exit zero, all 560 actual measurements are present, and
every statistical summary retains at least eight observations. Values are
microseconds; the last column compares equally weighted baseline/candidate
launch means.

| Case                        |   A2 (us) |   B3 (us) |   B4 (us) |   A3 (us) | Cost change |
| --------------------------- | --------: | --------: | --------: | --------: | ----------: |
| ConeSeparated               |    72.942 |    72.557 |    71.598 |    73.550 |      -1.60% |
| ConeFace                    |   104.551 |   106.559 |   106.237 |   104.360 |      +1.86% |
| ConeOblique                 | 1,732.713 | 1,412.896 | 1,433.875 | 1,746.727 |     -18.18% |
| CylinderSeparated           |   114.063 |   113.758 |   116.350 |   113.587 |      +1.08% |
| CylinderFace                |   102.851 |   105.436 |   106.116 |   105.599 |      +1.49% |
| CylinderOblique             | 1,711.200 | 1,420.314 | 1,426.039 | 1,713.774 |     -16.89% |
| ConeApexTouch               |    94.575 |    94.943 |    94.840 |    94.923 |      +0.15% |
| ConeSideTouch               |   427.802 |   428.712 |   430.235 |   427.777 |      +0.39% |
| CylinderSideTouch           |   118.179 |   117.849 |   121.311 |   121.221 |      -0.10% |
| SphereContained             |   110.526 |   113.342 |   110.699 |   111.969 |      +0.69% |
| CapsuleContained            |   113.827 |   113.596 |   121.310 |   115.615 |      +2.38% |
| CapsuleCrossingOffcenter    |   118.535 |   113.250 |   115.683 |   116.448 |      -2.57% |
| SphereContainedOpposingFace |   112.828 |   116.124 |   110.653 |   110.423 |      +1.58% |
| SphereHorizontalRim         |   157.043 |   154.751 |   158.951 |   156.021 |      +0.20% |

Cone changes from 1,739.720 to 1,423.386 us (**18.18% lower cost**); cylinder
from 1,712.487 to 1,423.176 us (**16.89% lower cost**). The corrected side-touch
estimate is +0.39%, with 428.712/430.235 us candidate launches against
427.802/427.777 us baselines, rather than the repeated +2.59% pilot penalty.
Retain the small positive control estimates in the table. A further focused BAAB
comparison of the largest remaining face/containment concerns gives:

| Case             | A1 (us) | B1 (us) | B2 (us) | A2 (us) | Cost change |
| ---------------- | ------: | ------: | ------: | ------: | ----------: |
| ConeFace         | 104.167 | 106.209 | 102.952 | 104.107 |      +0.43% |
| CapsuleContained | 118.176 | 114.313 | 113.890 | 114.726 |      -2.02% |

The focused face candidate ranges from 102.952 to 106.209 us around the two
104.1 us baselines; the contained-capsule estimate reverses sign. These controls
do not establish a consistent regression or equivalence. The counters show one
existing three-term entry for each face query but no enclosure trial, affine
certificate or stationary-maximum trial; the capsule enters none of those new
paths. This distinguishes the variation from the seven unsuccessful enclosure
attempts that caused the rejected side-touch policy. It does not attribute every
small timing difference to a specific runtime or machine mechanism. No claim of
zero control overhead follows from these limited launch samples.

All fourteen guarded contact snapshots preserve classifications, clamping flags,
raw depths, normals and both finite witnesses. The warmed, non-inlined probe
records zero calling-thread bytes over 128 queries after three warm passes for
every fixture on both variants. All 56 main capture children and all eight final
focused children have exactly zero raw allocation and collection counters. The
historical square-pilot baseline process events and the 336-byte raw event in
the earlier adjacent generic-radical baseline remain recorded separately; these
zero observations do not retrospectively attribute those events.

The source trees preserve the five-sibling layout under
`artifacts/oblique-deep-perf/{baseline,candidate}`. The three baseline owner
files match HEAD `4a2d433`; the guarded files match the final working source.
Starting consumer/dependency revisions remain Gravitas `7990c17`, Chronicler
`c45f9ab`, SwiftCollections `13a3c13` and GridForge `50d5540`; pre-existing
GridForge changes are held identical and untouched. Each accepted child is bound
to its portable PDB, and authored-document hashes are checked before its
generated directory can be reused: 410 math documents, 417 consumer documents,
88 fixture documents, and the loaded sibling/diagnostics sources. PDB paths
inside these nested snapshots are mapped through their recorded
`/_/artifacts/...` source prefix. Separate source snapshots and forced fresh
candidate timestamps avoid the stale-restore hazard found in the rejected square
pilot. The pre-guard source and captures are preserved separately from the
guarded source.

Final EventPipe attribution is supplemental, not timing acceptance. Use only
stacks containing the specific benchmark method, excluding fixture setup.
Analytic collection retains about 40.9%/31.3% of cone/cylinder benchmark
activity; rim charts retain about 30.2%/44.4%. These inclusive sampled shares
overlap and must not be added. All seven cone and twelve cylinder quartics are
still built and isolated. A useful next bounded experiment is an exact
whole-chart lower bound against the incumbent before quartic
construction/isolation, with retained fan/end-point ownership; also count
repeated analytic support directions before considering canonical deduplication.
Neither unexecuted value mapping nor another general arithmetic
micro-optimization is the demonstrated next target.

Final full-solution semantic formatter verification passes (workspace-load
warnings only). The final guarded broad smoke runs all 479 out-of-process Short
cases: all 479 children exit zero, all 1,437 actual measurements and 479 raw GC
rows are present, and every report has populated statistics. Positive smoke
allocations belong to the existing enumerable construction, formatting and
serialization workloads; the geometry/query evidence above retains its own raw
allocation checks. Short smoke values and profiler values are not performance
acceptance. The generated-reference workaround described above applies only to
the disposable smoke build.

#### Guarded reproduction and capture inventory

The host/runtime/environment are the .NET 10 Windows configuration documented in
the recheck above. From each frozen Gravitas root, use the same local-stack
environment and build flags, then run the complete alias:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DisableTransitiveProjectReferences = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
$env:BuildInParallel = 'false'
$env:UseSharedCompilation = 'false'

dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net10.0 -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -p:UseSharedCompilation=false -m:1
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll curved-prism-contact --warmupCount 5 --iterationCount 10 --launchCount 1 --iterationTime 500 --affinity 3 --keepFiles --exporters json --artifacts ../../../oblique-deep-gated-B3
```

Use distinct capture names and preserve each child payload/source binding before
another run overwrites it. The final focused controls add
`--filter '*ConeFace' '*CapsuleContained'` and run B/A/A/B. Final profiling adds
`--filter '*ConeOblique' '*CylinderOblique' --profiler EP`, two warmups, three
iterations and a 250 ms iteration target. The corrected broad smoke uses the
standard math alias `all -j Short --iterationTime 10`, with an explicit core
project reference in the generated project through the disposable MSBuild import
already described. All source-mode builds retain both required global
properties.

Supplemental capture families are `artifacts/oblique-deep-{A1,B1,B2,A2}*`,
`oblique-deep-controls-*`, `oblique-deep-focus-*`,
`oblique-deep-gated-{B3,B4,A3}*`, `oblique-deep-gatedfocus-*`,
`oblique-deep-gated-profile*`, `oblique-deep-gated-smoke*`, and
`oblique-deep-gated-{FixedMathSharp,Gravitas}-{Release,ReleaseLean}-*`. Source
identities, commands, numerical gates, rejected policy, current values,
validation counts and remaining experiment in this record are the durable
evidence if ignored captures and disposable probes are removed.

## Post-commit squeeze investigation (2026-11-02)

**Disposition at this checkpoint:** Keep committed `e30266d` unchanged. No new
runtime or maintained test change survives this bounded follow-up. The later
canonical-inventory investigation below retains a separate small reduction;
neither investigation resolves the roughly 1.4 ms penetrating-query cost.

The baseline is the committed implementation, not the earlier `4a2d433`
pre-optimization runtime. Source trees are isolated under
`artifacts/oblique-next-perf/{baseline,candidate,diagnostic,dedup}`, with the
same five-sibling layout, source mode, runtime, affinity and fourteen maintained
fixtures as the preceding investigation. The root worktree starts clean. All
prototype code and counters stay in those ignored copies; no stage, commit,
dependency pin or sibling source change is made.

### Chart rejection reach

Two sound certificates were counted before adding production branches:

- A quadratic gap lower bound drops the nonnegative cross term from
  `(P+sqrt(Q))²`, with an available unclamped incumbent upper `U/2` where
  `U=2*roundedDepthRaw+1`. For endpoint normals `n0,n1`, offset projections
  `p_i`, axial projections `a_i`, metric entries `m_ij`, and `q=Axis²`, its
  Bernstein entries are `B_ij=4*q*p_i*p_j +4*R²*(q*m_ij-a_i*a_j) -U²*q*S²*m_ij`.
  Nonnegative offset endpoints and a nonnegative quadratic certify domination.
  All three nonnegative Bernstein entries are sufficient; with both diagonal
  entries nonnegative, exact copositivity also allows a negative middle entry
  when `B00*B11>=B01²`. Equality is safe because earlier analytic ties win.
- The unsquared stationary expression is `K*sqrt(Q)+L`. An affine `K` with
  matching weak endpoint signs, not identically zero, is strictly one-signed
  inside the chart. Matching weak quadratic signs for `L` then exclude physical
  stationary roots. At the included endpoint one, either the expression stays
  nonzero or the existing `K==0` gate rejects it. This proof covers `(0,1]`,
  repeated squared-polynomial roots and constrained callers; it does not assume
  a face certificate absent from some domains.

The existing bound rejects five cone and six cylinder charts, leaving seven and
twelve quartics. The new quadratic bound rejects five cone charts, four already
rejected and **one additional**, and four cylinder charts, all already rejected.
The stronger copositivity test adds no reach. The unsquared sign certificate
rejects **zero** of the remaining charts on either target. These are exact
BigInteger diagnostic counts, not instrumented timing claims. The limited reach
does not justify another bound or derivative gate in production.

### Rejected factored-polynomial prototype

The curved caller already knows `Q=F*Q0`, with positive `F=R²*q`. Thus `L=F*L0`
and `W=F*(F*L0²-K²*Q0)`. A disposable shared-builder prototype forms `W/F`
directly, then restores full `Q,L` before the unchanged `J,N,D` work. Positive
normalization preserves roots, multiplicities, signs and ordinals; removing the
odd part of `F` can also reduce defining coefficient height beyond existing
power-of-two normalization. No division or GCD is introduced to recover this
already known factor.

The screen preserves all fourteen recorded depths, normals and paired witnesses,
but suggests only -1.99% cone cost and +0.02% cylinder cost. A full Release
out-of-process ABBA, with five warmups, ten measured iterations, a 500 ms
target, one launch per query and affinity three, gives the following
microseconds:

| Query           |   A1 (us) |   B1 (us) |   B2 (us) |   A2 (us) | Cost change |
| --------------- | --------: | --------: | --------: | --------: | ----------: |
| ConeOblique     | 1,501.449 | 1,463.386 | 1,465.705 | 1,470.022 |      -1.43% |
| CylinderOblique | 1,411.336 | 1,467.340 | 1,452.013 | 1,439.514 |      +2.40% |

All eight children exit zero, all eighty actual measurements are present,
statistical summaries retain at least eight observations, and every raw
allocation/collection counter is zero. Child PE/PDB and authored-source bindings
are checked for both variants before generated outputs are reused. The small
cone estimate does not justify the **+2.40% cylinder** estimate or the extra
builder branch/products/API argument. The factorization is rejected. Its direct
coefficient characterization and broad test matrix are not pursued after this
failed throughput gate; fixture agreement is not a general correctness proof.

Reproduce from each frozen Gravitas root with the preceding local-stack flags:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll curved-prism-contact --filter '*ConeOblique' '*CylinderOblique' --warmupCount 5 --iterationCount 10 --launchCount 1 --iterationTime 500 --affinity 3 --keepFiles --exporters json --artifacts ../../../oblique-next-A1
```

Repeat in baseline/candidate/candidate/baseline order with distinct artifact
names. These fresh absolute values are a matched follow-up, not an equivalent
comparison with the preceding captures at another time/harness selection.

### Exact duplicate-direction screen

Counting only successfully admitted oriented rational directions finds ten
repeated rays among 48 cone builds and ten among 70 cylinder builds. All ten are
exact vector duplicates in these fixtures, so normalization or cross-product
arithmetic is unnecessary. A bounded eight-entry FIFO retains six duplicates in
each penetrating query; sixteen entries retain the same six. The touching cone
has 46 repeated rays among 58 builds, of which 36 are exact duplicates; eight
entries retain 23, while sixteen retain 36.

A temporary ordinary-query cache therefore uses eight exact vectors, records
only successful admissions, treats opposite directions separately, and leaves
the constrained-domain collector untouched. A hit still performs the normal
orientation-loop sign flips. Canonical support vertex/primitive feature and all
physical ratios depend on the oriented ray, not the generating required vertex;
once admitted, an identical candidate cannot improve a monotonically decreasing
incumbent or replace an earlier tie. Full component equality avoids the roughly
1,008-bit products a general ray-parallel comparison could require. A second
screen adds low-word rejection before full equality, without treating a partial
match as equality.

All fourteen screened depths/normals/witnesses match in both versions. The
unfiltered screen estimates -0.55%/-1.87% cone/cylinder cost; the filtered
version estimates -1.80%/-2.02%, with overlapping cone launch ranges. Calling
these repeatable gains is not justified. The touching estimates are
approximately -13.86% and -12.44%, respectively, and remain screening signals.
Cheap control paths in this short direct probe are not fully tier-warmed and are
not acceptance evidence. The extra lookup cost and approximately 1,728-byte
stack cache are not retained for this small uncertain penetrating benefit. A
touching-specific investigation would need its own full controls and complete
correctness, allocation and stack-budget checks before any cache is accepted.

No additional full solution matrix or broad smoke is claimed in this follow-up,
because maintained runtime/tests remain byte-identical to `e30266d`. The
preceding validated committed change remains the retained implementation.
Independent review checks the mathematical certificates, factorization,
ordinary-query deduplication proof, width limits and rejection criteria.
Documentation/link and working-tree scope checks cover the only retained edits.

The next substantial investigation should target a cheaper canonical analytic
inventory with explicit tie/witness equivalence, or a stronger whole-chart
certificate whose useful reach is established first. Do not re-add the rejected
bounds, factorized builder or query cache solely because their mathematics is
valid. Supplemental counts, prototype sources and capture families are
`artifacts/oblique-next-*`; the formulas, scope, commands, measured values,
disposition and remaining target above survive deletion of those ignored files.

## Canonical analytic inventory reduction (2026-11-02)

The next investigation starts from clean `c6bd41c` (runtime `e30266d`). It
retains only a structural repetition check in `CurvedPrismContact.KeepAnalytic`;
there is no candidate cache, normalization pass, extra wide product or new
public API. The duplicate zero-axis return in `KeepAxis` is removed:
`WriteDirection` accepts zero and `KeepDirection` already rejects it before
candidate work. This also removes an early check whose only covered zero source
was the now-pruned axis/edge cross. Both the ordinary query and
constrained-domain caller share this collector.

### Exact equivalence and ordering

For each nonzero real edge `E`, the collector already visits both orientations
of `GetBasis(E)` with that edge's same required start vertex. The later
`Cross(Axis,E)` is perpendicular to `E`. If `E.X != 0`, the two earlier basis
lines are `(-E.Y,E.X,0)` and `(-E.Z,0,E.X)`: a perpendicular vector with zero Z
or zero Y lies on one of them. If `E.X == 0`, the lines are `(0,-E.Z,E.Y)` and
the X axis: zero X, or both zero Y and Z, identifies those lines. The predicate
uses only existing exact zero tests; a zero cross was already a no-op.

These homogeneous repetitions have the same required-vertex admission, global
canonical support vertex and primitive feature. A failed earlier direction
cannot become admissible later. An admitted one cannot improve the monotonically
decreasing incumbent, and the strict comparison retains its earlier exact tie.
Domain windows and certificates are immutable during selection. Consequently the
removed attempts cannot change the winning feature, exact normal ratio, window
or any witness term. Final witness materialization stays with the same owner. In
particular, zero Z alone is insufficient when `E.X == 0`: an oblique side cross
can be the unique minimum and must remain in the inventory.

Twenty-one maintained public characterization cases pass before and after the
change. Coordinate-axis cylinders inside a cube have equal depth-two exits and
must retain the first positive authored axis and both complete pole anchors;
subdivided edges and reversed winding preserve that choice. An arbitrary-axis
cylinder with axis proportional to `(3,4,0)` against a finite prism edge along Z
has the unique cross normal `(4,-3,0)/5`. Radius five gives the exact paired
radial witness `(4,-3,0)` and prism point `(0,1/2,0)`. Cases at separation,
touching, scalar offsets of one raw unit and `1/64` along `(4,-3,0)`, reversed
winding and origins near the representable limits check exact depth, every
anchor field, boundary touch and whole-shape clearance beyond the exit. These
assert production behavior, without exposing the private predicate or
reproducing the collector in tests.

### Rejected wider inventory scan

A second prototype omits a later unrestricted side-face direction when its
native footprint edge delta exactly equals or negates an earlier edge delta.
Wide 65-bit differences avoid saturated `Vector2d` subtraction. The native-yaw
and primitive-frame maps are linear, so this also preserves the earlier global
support choice. It is mathematically valid, but adds a scan for each native
edge.

A symmetric baseline/cross-only/combined/combined/cross-only/baseline pilot,
before retirement of the duplicate zero-axis guard, uses the same out-of-process
Release settings below. The two target means estimate cross-only cost changes of
**-2.99% cone / -3.15% cylinder**, versus **-4.24% / -5.81%** combined. The
small incremental target benefit is uncertain. A separate balanced direct screen
uses convex parabolic footprints with 8, 32 and 64 vertices, identity or oblique
primitive frames, both primitive kinds, no repeated/opposite edge deltas, three
warm batches and three measured batches of eight queries. At 32/64 vertices the
combined scan consistently costs about **2-3% more than cross-only**, while
cross-only improves over baseline. Small unrotated cases have tier-warmup
variation and are inconclusive. This screen is not a BenchmarkDotNet acceptance
result. All twelve physical results match. The extra scan is rejected; only the
constant-time structural check survives.

The combined prototype also matches a 1,320-case byte-for-byte differential
snapshot against the committed implementation. The disposable public-query probe
spans cones/cylinders; six axes; identity, oblique and rational-quaternion
frames; square, trapezoid, triangle, subdivided, collinear and duplicate-vertex
footprints; three relative centers; both windings; yaw zero/one-third; zero and
positive radius; translated near-limit origins; and 24 full-raw-width footprint
cases. Snapshots recursively include every contact/anchor instance field,
including private exact residuals. No exception occurs. The final cross-only
comparison and full maintained validation below establish the retained variant.

### Matched acceptance measurements

**Disposition:** Retain the structural repetition check and redundant guard
retirement: eight net runtime lines, one private predicate, no new runtime file,
cache, allocation or public surface. Cylinder and side-touch wins repeat across
the main and focused windows. Cone's gain is window-dependent and remains
inconclusive as a general improvement; the throughput signal stays experimental.

The final Release out-of-process window uses all fourteen maintained Gravitas
fixtures, baseline/candidate/candidate/baseline order, five warmups, ten actual
iterations, a 500 ms iteration target, one launch, affinity three and
`DOTNET_PROCESSOR_COUNT=2`. The host is Windows 11 x64, i7-9700K, SDK 10.0.302,
runtime 10.0.10 and BenchmarkDotNet 0.15.8. Frozen standard source graphs use
`UseLocalLsfStack=true`, `DisableTransitiveProjectReferences=true`,
`UseSharedCompilation=false` and serial builds. No competing build, test or
profiler runs during these acceptance timings. Values are microseconds/query;
change is the candidate mean over the baseline mean, using both launches.

| Query                       |   A1 (us) |   B1 (us) |   B2 (us) |   A2 (us) | Cost change |
| --------------------------- | --------: | --------: | --------: | --------: | ----------: |
| CapsuleContained            |   124.210 |   120.913 |   122.065 |   129.707 |      -4.31% |
| CapsuleCrossingOffcenter    |   120.836 |   136.837 |   124.413 |   121.233 |      +7.92% |
| ConeApexTouch               |   100.761 |   101.120 |   102.442 |   102.442 |      +0.18% |
| ConeFace                    |   111.629 |   111.273 |   114.175 |   113.221 |      +0.27% |
| ConeOblique                 | 1,503.221 | 1,478.614 | 1,503.882 | 1,474.606 |      +0.16% |
| ConeSeparated               |    78.801 |    77.146 |    78.149 |    76.519 |      -0.02% |
| ConeSideTouch               |   454.618 |   430.108 |   428.400 |   462.662 |      -6.41% |
| CylinderFace                |   112.802 |   113.594 |   113.098 |   112.157 |      +0.77% |
| CylinderOblique             | 1,495.755 | 1,467.796 | 1,451.029 | 1,496.928 |      -2.47% |
| CylinderSeparated           |   122.349 |   120.023 |   121.192 |   122.285 |      -1.40% |
| CylinderSideTouch           |   128.855 |   128.141 |   126.660 |   126.715 |      -0.30% |
| SphereContained             |   122.667 |   118.057 |   118.633 |   125.087 |      -4.47% |
| SphereContainedOpposingFace |   118.167 |   120.591 |   118.276 |   129.564 |      -3.58% |
| SphereHorizontalRim         |   168.901 |   167.158 |   168.376 |   168.981 |      -0.70% |

Cone's **+0.16%** main-window estimate establishes no improvement. The capsule
crossing control's **+7.92%** is driven especially by the 136.837 us B1 launch;
it requires investigation rather than dismissal based on untouched source. A
fresh focused **candidate/baseline/baseline/candidate** window uses the same
settings and five selected fixtures. Columns retain their variant labels;
chronological order is B1, A1, A2, B2.

| Query                    |   A1 (us) |   B1 (us) |   B2 (us) |   A2 (us) | Cost change |
| ------------------------ | --------: | --------: | --------: | --------: | ----------: |
| CapsuleCrossingOffcenter |   122.796 |   125.222 |   124.348 |   125.789 |      +0.40% |
| ConeOblique              | 1,496.515 | 1,459.582 | 1,450.064 | 1,481.129 |      -2.28% |
| ConeSideTouch            |   459.556 |   433.013 |   426.229 |   455.518 |      -6.10% |
| CylinderFace             |   112.009 |   111.167 |   113.300 |   111.769 |      +0.31% |
| CylinderOblique          | 1,527.151 | 1,463.168 | 1,452.654 | 1,503.688 |      -3.79% |

Cylinder improves **2.47% / 3.80%** and side-touch improves **6.41% / 6.10%** in
the two matched windows. Both candidate cone launches in the focused window are
below both baseline launches (**-2.28%**), but the flat main window prevents a
claim of a general repeatable cone win. The capsule crossing estimate becomes
**+0.40%**: baseline launch means range 122.796-125.789 us and candidate means
124.348-125.222 us. Cylinder-face is **+0.31%**, with candidate launches on both
sides of the baseline range. These controls do not confirm a regression and do
not establish zero overhead. No adverse run is removed or pooled away.

All **76 children exit zero**, all **760 actual measurements** are present, each
statistical summary retains at least eight observations, and every raw
allocation/collection counter is zero. PE/PDB identities and authored-source
checksums bind each child to its frozen seven-library source graph before
outputs are reused. These are fresh matched comparisons against `e30266d`; their
absolute values must not be combined with the earlier 1.423 ms captures from a
different window. The present target means remain about **1.45-1.49 ms**, well
short of the provisional 2x reduction.

### Final correctness and validation

The retained variant repeats the full 1,320-case differential with every exact
contact/anchor field identical and no exceptions. Both fourteen-case mixed
snapshots match, the probes' copied math/consumer DLL hashes match the selected
frozen bins, and both variants record zero warmed calling-thread bytes over 128
queries per fixture after three warm batches.

Full live-source solution builds and collected test runs pass with explicit
local-source flags, `UseSharedCompilation=false`, `-m:1` and eight advertised
processors. Counts below come directly from OpenCover points/method entries; all
raw unvisited lists are empty. The FixedMathSharp collection includes core and
FluentAssertions; companion tests run separately.

| Repository / configuration |                Tests | Sequence points |   Branch points |       Methods |
| -------------------------- | -------------------: | --------------: | --------------: | ------------: |
| FixedMathSharp Release     | 7,294 + 49 companion | 68,404 / 68,404 | 22,606 / 22,606 | 5,325 / 5,325 |
| FixedMathSharp ReleaseLean | 7,273 + 49 companion | 68,497 / 68,497 | 22,606 / 22,606 | 5,321 / 5,321 |
| Gravitas Release           |                8,348 | 53,559 / 53,559 | 21,772 / 21,772 | 5,602 / 5,602 |
| Gravitas ReleaseLean       |                8,277 | 53,557 / 53,557 | 21,772 / 21,772 | 5,601 / 5,601 |

The raw metadata reports `KeepAnalytic` complexity 28 and the new predicate
complexity 8, with every branch covered. The complexity register retains the
collector's ordering/admission rationale. Independent review covers actual
ordinary/domain callers, the physical test oracles, exact anchor equality,
scratch lifetime, duplication and retirement of the zero guard. No speculative
helper or rejected face-scan code remains. Scoped C# formatting and error-level
format verification pass; formatting makes no semantic change.

The rebuilt benchmark host lists successfully. The final broad out-of-process
Short smoke (`all -j Short --iterationTime 10`) passes all **479 cases**, with
479 zero child exits and all **1,437 actual measurements**. Thirteen allocation
rows belong to the existing enumerable, string-formatting and serialization
workloads; all remaining rows have zero raw allocation/collection counters. The
disposable generated-project import supplies the explicit core reference needed
by typed return signatures under globally disabled transitive project
references; no maintained harness or CI change is made. The smoke's
three-library PE/PDB/source bindings are checked, and its timing is not
acceptance evidence. Captures are `artifacts/oblique-inventory-smoke*`.

Preliminary captures are explicitly excluded: `oblique-inventory-final-*`
**matrix** files reused the committed binary because copying source preserved an
older timestamp; raw helper absence exposed this and the run was stopped. The
`verified-*` matrix supersedes that binary but precedes removal of the redundant
zero guard and is also not final acceptance. Only the `accepted-*` matrix, final
bound performance children and focused children above validate the retained
variant. The stale-build incident is local experiment provenance, not a library
defect.

Reproduce the main performance window from each frozen Gravitas root:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll curved-prism-contact --warmupCount 5 --iterationCount 10 --launchCount 1 --iterationTime 500 --affinity 3 --keepFiles --exporters json --artifacts <distinct-output-directory>
```

Use baseline/candidate/candidate/baseline order. For the reversed focused
window, add filters `*ConeOblique`, `*CylinderOblique`, `*ConeSideTouch`,
`*CapsuleCrossingOffcenter`, `*CylinderFace` and run
candidate/baseline/baseline/ candidate. Baseline is `c6bd41c` with runtime
`e30266d`; candidate is the tracked collector diff. Sibling bases are Gravitas
`7990c176`, Chronicler `c45f9ab5`, SwiftCollections `13a3c13e` and GridForge
`50d55404`; pre-existing GridForge working changes are held identical between
timing variants and left untouched.

Capture families are
`artifacts/oblique-inventory-{pilot,large,accepted,final,focus}-*` and source
trees are `artifacts/oblique-inventory-perf/{baseline,cross,candidate}`. The
combined prototype remains disposable; final runtime is the cross tree. The
proof, fixture recipe, commands, measured values and raw validation totals above
survive deletion of those ignored artifacts. No release-package or
cross-platform validation is claimed. Remaining work is the expensive analytic
fan and seven cone/twelve cylinder stationary quartics; establish useful reach
before adding another pruning rule or certificate.

## Local-Y cross-construction follow-up (2026-11-02)

**Disposition at this checkpoint:** Keep committed `87f18d2` unchanged. This
bounded experiment retains no runtime or test change; the later exact-work
reduction below continues the open experimental signal.

The previous reduction forms `Cross(Axis,E)` before recognizing an earlier basis
line. A disposable prototype precomputes whether the primitive-local axis has
zero X/Z components and bypasses that construction for those queries. For
`A=(0,Ay,0)`, the cross is `(Ay*Ez,0,-Ay*Ex)`: if `Ex!=0`, this is `-Ay` times
the earlier second basis; otherwise it is on the earlier X-axis line or zero.
Both orientations and the same required vertex were already evaluated. This
includes negative Y axes and constrained domains. Other axes retain the
committed general predicate, and the end-of-edge zero-gap return stays outside
the bypass. Independent review confirms the equivalence and scratch behavior.

The prototype matches all **1,320 complete contact/anchor snapshots** and all
**fourteen mixed fixtures** against the committed implementation. Its probe core
DLL hash matches the selected frozen prototype. A narrow Release out-of-process
ABBA uses the same host, local-source settings, affinity three, two advertised
processors, five warmups, ten actual iterations, one launch and 500 ms iteration
target as the preceding acceptance window. Baseline is the committed inventory
reduction, not the older pre-inventory runtime. Values are microseconds/query.

| Query           |   A1 (us) |   B1 (us) |   B2 (us) |   A2 (us) | Cost change |
| --------------- | --------: | --------: | --------: | --------: | ----------: |
| ConeOblique     | 1,453.853 | 1,481.337 | 1,459.182 | 1,459.709 |      +0.93% |
| ConeSideTouch   |   428.700 |   427.341 |   433.655 |   429.154 |      +0.37% |
| CylinderOblique | 1,480.671 | 1,471.278 | 1,484.514 | 1,465.429 |      +0.33% |

All twelve children exit zero, all 120 actual measurements are present,
statistical summaries retain at least eight observations, and all raw
allocation/collection counters are zero. Child PE/PDB and authored-source
checksums bind the seven-library source graphs. No competing build, test or
profiler runs during these timings. The roughly **+0.93% cone / +0.33% cylinder
/ +0.37% side-touch** cost estimates are small and inconsistent across launches;
they establish no incremental win or conclusive regression. The additional
boolean and per-edge branch are rejected despite their valid equivalence proof.

Captures are `artifacts/oblique-up-screen-*`, with the disposable candidate
under `artifacts/oblique-up-perf/candidate`. Reproduce the preceding main
command with filters `*ConeOblique`, `*CylinderOblique`, `*ConeSideTouch` in
baseline/candidate/ candidate/baseline order. No new full solution matrix,
broader controls or smoke is claimed: maintained runtime/tests remain
byte-identical to `87f18d2`, and the prototype failed the initial throughput
gate. Documentation formatting, links and scope checks cover the retained
record. Further substantial progress needs a measured reduction in the remaining
analytic/root work; this result does not justify another fast-path condition.

## Exact workload reduction (2026-11-03)

The owner continued the investigation after the local-Y construction shortcut
failed its gate. Baseline is committed `87f18d2`, including the earlier guarded
and canonical-inventory changes. The retained candidate keeps the public API,
exact minimum, first ties and complete anchor representation unchanged. This is
still an experimental throughput signal: the provisional twofold reduction on
both penetrating fixtures remains unmet despite the repeatable improvement.

### Executed work and retained changes

Disposable counters on the actual mixed queries found seven cone and twelve
cylinder stationary charts, with fourteen/nineteen roots. Only one chart per
query reaches a signed gap query; none reaches squared-value construction or
mapping. Building J/N/D eagerly and resolving every stationary root therefore
spent work on candidates that could not improve the incumbent.

The retained changes act on those measured costs:

- `CylinderEdgeContactPolynomial` builds W/Q/K/L first and completes J/N/D only
  after a root survives maximum, cap, support and unsquared-sign admission.
  Future J/N/D slots temporarily retain P, M, scale and K squared. Completion
  consumes them before overwrite, once per chart. The existing eager `Build`
  wrapper remains for box and circular-rim callers, with the same final fields
  and common-power normalization. Materialization completes before reading J.
- Certified ordinary charts use W=L squared minus K squared times Q to exclude
  entire sets of maxima. At K's positive zero beta, W(beta)=L(beta) squared is
  nonnegative. Negative W(0), positive K(0), negative K slope and two Descartes
  sign variations exhaust the positive roots as two maxima bracketing beta, or
  the earlier analytic K-zero direction. This check precedes isolation and
  includes multiplicity. A second check after isolation uses square-freeness,
  K(0)>0>K(1), nonzero W(0), and one root or two roots starting from W(0)<0.
  Existing per-root admission remains for other charts. Constrained and
  uncertified domains do not use either new proof.
- The target cone/cylinder isolate six/nine quartics instead of seven/twelve and
  enter only three/six root iterations instead of fourteen/nineteen. Both
  complete gap polynomials once instead of seven/twelve times. Analytic
  endpoints own feature switches and equal minima; no witnesses are fabricated
  for rejected maxima.
- A cone-generator family can be excluded using one certified point of the
  complete prism. Orient native Up so d=Up dot A>0, let m=minPrism(Up), and
  p=(m/d)A. Require lambda=T*d-m>0 and verify p against both sides of every
  prism plane, including coordinate extents needed by degenerate footprints.
  Every generator normal has A dot n/|n|=R*sqrt(q)/sqrt(4*T*T*q+R*R), so its gap
  is at least lambda*R*sqrt(q)/(d*S*sqrt(4*T*T*q+R*R)). Compare its exact square
  with the incumbent. Equality belongs to the earlier feature. The membership
  products fit 576/832 bits; squared numerator/denominator bounds are 913/723
  bits, retained in sixteen-word fields. This certificate excludes the whole
  generator family on `ConeOblique`, while required generator winners retain the
  full traversal.
- Certified edge charts already own incident normals. Reduce the real edge and
  construct the generic reciprocal basis only for fallback charts. Derive side
  normals from exact 65-bit native footprint differences and transform the
  native perpendicular directly. The old cross product differs by a positive
  scalar; existing reduction returns the same primitive integer endpoints.
- Project analytic directions with bounded fixed-width arithmetic for
  axis-squared times value minus axis-dot-value times axis. Under these caller
  bounds all results fit 576 bits and complete products fit 832. Reuse the same
  difference-of-products owner in the generator collector; wider existing
  triangle/capsule projection callers retain their established general owner.
- Replace single-bit GCD halving/restoration with trailing-zero counts and bulk
  shifts, and finish immediately when the ordered odd operand is integer one.
  Recognize exact power-of-two divisors across all three words and shift their
  magnitudes before restoring sign. Generic exact division remains unchanged.
  The old queries execute 52 three-word GCD calls, about 7,516/7,518 single-bit
  halves, and 66 exact divisions with about 6,933/6,935 long-division steps.
  Their coordinate divisor is 2^34. The admitted magnitude remains below 2^191;
  `Signed192.One` is Q32.32 one, not the integer-one termination test.

The implementation adds 193 net runtime lines across eight existing files, with
no new runtime type/file, cache, allocation or public API. It retires the
generator's duplicate scaled-difference helpers and the old private GCD halving
helper. Independent review checks actual callers, scratch ownership, complete
widths, exact ties and removal opportunities.

### Experiments and decisions

All pilot comparisons use frozen sibling graphs, Release out-of-process
BenchmarkDotNet, affinity three, two advertised processors, five warmups, ten
actual iterations, one launch and a 500-ms target. Balanced means compare only
within their own windows; absolute values from different windows are not pooled.

The balanced eight-pass structural pilot has sixteen successful children and 160
actual samples, with zero raw allocation/collection counters. Whole-family
generator exclusion alone reduces cone cost 17.15% and is flat on cylinder
(+0.52%). Chart topology alone reduces cone/cylinder cost 4.56%/9.04%. Combined
with deferred coefficients and cheaper chart preparation, the candidate reduces
them 30.80%/17.85%. All tested variants match 1,320 full contact/anchor
snapshots and fourteen mixed fixtures.

A fresh profile of that candidate attributes approximately 11.4%/13.7% to the
older chart bound, 7.1%/14.0% to analytic projection and 12.6%/5.3% to geometry
construction. Attribution is inclusive sampled activity, not additive timing.
The following balanced ablations determine the retained combination:

| Follow-up against its immediate candidate                  | Cone cost change | Cylinder cost change | Decision |
| ---------------------------------------------------------- | ---------------: | -------------------: | -------- |
| Remove ordinary certified-chart bound                      |           +4.06% |               +0.73% | Reject   |
| Compact exact projection                                   |           -3.27% |               -4.44% | Keep     |
| Remove bound and compact projection                        |           +2.27% |               -0.15% | Reject   |
| Bulk three-word content reduction after compact projection |           -4.77% |               -1.93% | Keep     |

The first ablation window has sixteen successful children/160 actual samples;
the numeric window has eight/eighty. All raw allocation/collection counters are
zero, with at least eight retained statistical observations per case. PE/PDB
identities and authored-source checksums bind the actual generated payloads.

Additional disposable bounds were not retained. A squared level-crossing quartic
has no useful reach. A branch-aware version with dyadic subdivision rejects
three cone/four cylinder charts but leaves four/eight with actual
below-threshold samples, including analytic endpoint ties; stronger subdivision
cannot exclude those whole charts. A concavity certificate and K-sign cell
shortcut overlap the simpler whole-maximum proof or still require refinement. No
helper/API from those prototypes remains in the runtime.

### Final fourteen-fixture comparison

A fresh full-query ABBA against `87f18d2` completes **56 successful children and
560 actual measurements**, with zero raw allocation/collection counters and at
least eight retained observations per case. These are microseconds per complete
mixed query on the measured Windows 11/i7-9700K/.NET 10.0.10 host, SDK 10.0.302.
Both candidate target launches beat both baseline launches.

| Query                       |        A1 |        B1 |        B2 |        A2 | Balanced cost change |
| --------------------------- | --------: | --------: | --------: | --------: | -------------------: |
| ConeOblique                 | 1,405.490 |   949.997 |   924.603 | 1,394.566 |              -33.05% |
| CylinderOblique             | 1,464.112 | 1,068.994 | 1,042.154 | 1,378.501 |              -25.73% |
| ConeSeparated               |    74.329 |    61.179 |    62.130 |    74.453 |              -17.12% |
| CylinderSeparated           |   118.007 |    67.217 |    64.480 |   116.372 |              -43.81% |
| ConeFace                    |   107.557 |    94.785 |    94.482 |   105.463 |              -11.15% |
| CylinderFace                |   109.401 |    93.256 |    94.648 |   106.668 |              -13.04% |
| ConeApexTouch               |    98.371 |    84.169 |    83.688 |    96.383 |              -13.81% |
| ConeSideTouch               |   434.535 |   388.266 |   386.408 |   409.242 |               -8.19% |
| CylinderSideTouch           |   129.980 |    91.958 |    91.934 |   119.463 |              -26.28% |
| SphereContained             |   115.416 |   117.422 |   117.383 |   112.467 |               +3.04% |
| CapsuleContained            |   127.976 |   117.348 |   115.421 |   116.649 |               -4.85% |
| CapsuleCrossingOffcenter    |   120.947 |   116.706 |   120.882 |   117.212 |               -0.24% |
| SphereContainedOpposingFace |   116.416 |   114.265 |   112.367 |   114.477 |               -1.85% |
| SphereHorizontalRim         |   165.548 |   157.404 |   165.234 |   159.402 |               -0.71% |

Balanced penetrating means are **1,400.028 to 937.300 us cone (-33.05%)** and
**1,421.307 to 1,055.574 us cylinder (-25.73%)**. Absolute pilot means from
other windows are not combined with these. A fresh candidate/baseline/baseline/
candidate five-case window repeats the target gains at **-34.39% cone** and
**-21.62% cylinder** (1,423.876 to 934.216 us; 1,427.690 to 1,118.996 us). Both
candidate launches again beat both baseline launches.

`SphereContained` is +3.04% in the main window and -2.36% in the focused window;
the initial slowdown does not repeat. `CapsuleContained` is -1.32% in that
focused window. `SphereContainedOpposingFace` is +0.89%, with overlapping launch
ranges. These noisy controls establish no repeatable penalty or zero overhead.

Six shared-caller controls use another Release ABBA with the same job settings.
A fresh reversed three-case window checks the capsule signal and adjacent eager
polynomial callers:

| Shared control                    | Main cost change | Reversed focused change | Interpretation                               |
| --------------------------------- | ---------------: | ----------------------: | -------------------------------------------- |
| Box/capsule primary               |           +3.55% |                  -0.62% | Initial slowdown does not repeat             |
| Oblique box/cylinder primary      |           +0.86% |                  +2.55% | Small positive means; launch ranges overlap  |
| Ordinary cylinder/capsule         |           -1.61% |            Not repeated | No observed penalty                          |
| Penetrating cylinder rims         |           -1.09% |            Not repeated | No observed penalty                          |
| Multi-radical cylinder pair       |           -3.78% |            Not repeated | Both candidate launches below both baselines |
| Oblique triangle/capsule-slab rim |           +0.30% |                  -2.23% | Initial flat result; focused gain            |

The box/cylinder control remains uncertain. Its main baseline/candidate launch
ranges are 1,016.698-1,036.910 / 1,030.638-1,040.697 us; focused ranges are
1,003.694-1,027.879 / 1,026.097-1,057.254 us. Each window has a candidate launch
inside the baseline range. The positive means are retained, not called zero
overhead or a confirmed regression. If the cost repeats in a stronger matched
capture, isolate the eager shared builder before adding a workaround.

All **112 final, focused and shared-control children** exit zero and supply all
**1,120 actual measurements**, with zero raw allocation/collection counters and
at least eight retained observations per case. Actual child PE/PDB identities
and authored-source checksums bind every source graph. The candidate matches all
**1,320 complete contact/anchor snapshots** and fourteen mixed fixtures; all
fourteen fixtures report zero calling-thread bytes over 128 warmed queries.

### Validation

Independent polynomial tests derive K/L by differentiation and W/J/N/D by
BigInteger algebra, including cross-limb staged P/M/scale and joint N/D
normalization. They check eager and deferred owners and that completion leaves
W/Q/K/L unchanged. Numeric oracles cover dense full-word power divisors, signed
and swapped GCD inputs, asymmetric carries, integer-one false positives,
non-power fallbacks, maximum admitted magnitudes and warmed zero allocation.

The containing tall-prism cone cases prove depth four with complete anchor
equality, touching after the full exit and separation one raw unit beyond it. A
cone above-apex case needs its generator minimum: its native-Up axis point
belongs to the prism but lambda is negative, so membership alone cannot justify
squaring the bound. A rational supporting plane and contained ball give exact
depth sqrt(5)/64 and normal (1,2,0)/sqrt(5), with matching witness points and
exact touching/separation. A separate oblique cylinder has a zero cubic
stationary coefficient; an independent complete-support inequality proves unique
Up/depth 2836, followed by exact touching and one-raw separation. Both windings
and reversed-axis stationary winners remain covered.

The final standard/Lean matrix passes:

| Repository/configuration   | Core tests | Companion tests | Raw sequence points | Raw branch points | Raw methods |
| -------------------------- | ---------: | --------------: | ------------------: | ----------------: | ----------: |
| FixedMathSharp Release     |      7,338 |              49 |       68,505/68,505 |     22,688/22,688 | 5,332/5,332 |
| FixedMathSharp ReleaseLean |      7,317 |              49 |       68,598/68,598 |     22,688/22,688 | 5,328/5,328 |
| Gravitas Release           |      8,348 |  Not applicable |       53,559/53,559 |     21,772/21,772 | 5,602/5,602 |
| Gravitas ReleaseLean       |      8,277 |  Not applicable |       53,557/53,557 |     21,772/21,772 | 5,601/5,601 |

The initial complete matrix exposed exactly two unvisited math branches despite
rounded 100% reports: nonpositive generator lambda and a zero stationary
coefficient. The physical cases above close both without changing runtime logic.
The final math suites are rerun in both configurations; consumer and companion
suites remain valid because runtime/source dependencies are unchanged. Both
runtime target frameworks build. No exclusion, assertion weakening,
allocation-counter or package/configuration change is used. The live runtime
remains byte-identical to the frozen timed source.

The final out-of-process Short smoke completes **479 successful children**, all
**1,437 actual iterations**, and 479 populated summaries. Raw positive counters
occur only in the same thirteen enumerable/formatting/serialization workloads as
the committed smoke control; all other cases, including geometry, retain zero
raw allocation/collection counters. Generated math/benchmark/diagnostics PE/PDB
and authored-source bindings pass. Short timings are smoke evidence only. Scoped
C# formatting, documentation formatting, relative-link/anchor checks and
`git diff --check` pass. Source and test changes remain unstaged/uncommitted.

### Interrupted pilot and bounded native-fault investigation

An early deferred-coefficient/chart-preparation pilot stops at its second
candidate cylinder launch. The child throws a managed `NullReferenceException`
after warmup, at `Signed832.IsZero`, through
`CurvedPrismContactGeometry.ProjectionMinimum`, `TryBuildCandidate`,
`CurvedPrismContact.KeepDirection`, `KeepAnalytic` and `TryGetContact`. No
actual samples exist for that child. The entire interrupted pilot is excluded
from accepted comparisons; preceding successful measurements are not
cherry-picked.

That intermediate candidate contains only deferred coefficients and certified
edge preparation over `87f18d2`, before generator/topology/direct-normal,
projection and numeric refinements. The three fault-stack source owners
`Signed832`, `CurvedPrismContactGeometry` and `CurvedPrismContact` are then
byte-identical to the committed baseline. This does not establish a pre-existing
cause: earlier calls, generated layout and runtime state can still differ.
Static inspection finds no span overlap, ref escape or invalid lifetime in
staging, and no verified unsafe write.

The failed generated seven-library DLL/PDB payload is archived and bound to its
source checksums; its math DLL SHA-256 is
`4A1309AC9A785B584458FB71B9E92896AE93CEB16893456C4EFF21AA4CFAE615`. The child
recipe is
`dotnet Gravitas.Benchmarks-Job-MRMQXX-1.dll --benchmarkName Gravitas.Benchmarks.CurvedPrismContactBenchmarks.CylinderOblique 1`,
using its original five-warmup/ten-actual 500-ms job. It supports a console host
without anonymous benchmark pipes.

A deliberately caught null dereference calibrates a current-runtime CDB
first-chance AV collector before replay. The collector uses `sxe ... av` to
write a full dump, exception record, registers, native stack and faulting
instructions before `gn`. Use `cdb -G -o -cf <commands> dotnet <child> ...`:
`-G` avoids the final debugger breakpoint reading EOF. The first replay without
`-G` completes ten actual iterations but the debugger exits with its own read
fault; it is not a failed application workload or throughput evidence. Two
further debugger replays, three normal intermediate-candidate replays and three
normal committed-baseline replays all complete ten actual measurements, with no
AV/NRE. No native dump of the original fault was captured.

For recurrence, create `artifacts/curved-fault`, run CDB from the repository
root, and save these commands to `artifacts/curved-fault/capture.cdb`:

```text
sxi ibp
sxe -c ".echo CURVED_FIRST_CHANCE_AV; .dump /ma /u artifacts/curved-fault/first-chance.dmp; .exr -1; r; k; u @rip-20 L40; gn" av
g
```

Launch the preserved child with
`cdb -G -o -cf artifacts/curved-fault/capture.cdb dotnet <preserved-child.dll> --benchmarkName Gravitas.Benchmarks.CurvedPrismContactBenchmarks.CylinderOblique 1`.
First verify the collector using a caught field read through a null reference in
a method marked `NoInlining | NoOptimization`; throwing a managed exception
explicitly does not calibrate a native AV collector. Preserve the faulting child
payload and PDBs before any generated rebuild reuses its output directory.

The cause remains **unattributed and unreproduced**, not fixed. No null guard,
retry, runtime optimization switch or speculative arithmetic workaround is
retained. Reopen fault RCA with a fresh first-chance capture and matching child
DLLs/PDBs; stable final launches establish their own execution results only.

### Reproduction and durable evidence

Keep the same sibling state for baseline and candidate: Gravitas `7990c176`,
Chronicler `c45f9ab5`, SwiftCollections `13a3c13e`, GridForge `50d55404`,
including the unchanged pre-existing GridForge working changes. Set
`UseLocalLsfStack=true`, `DisableTransitiveProjectReferences=true`,
`UseSharedCompilation=false` and `BuildInParallel=false` in the environment and
on serial restore/build commands. Build the owning benchmark project with
`-c Release -f net10.0 -m:1`. Use separate source graphs and fresh timestamps
when copying a candidate; preserved file times can otherwise leave stale
incremental binaries. Confirm generated PE/PDB bindings and source checksums.

For the full mixed comparison, run from each Gravitas checkout:

```powershell
$env:DOTNET_PROCESSOR_COUNT='2'
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll curved-prism-contact --warmupCount 5 --iterationCount 10 --launchCount 1 --iterationTime 500 --affinity 3 --keepFiles --exporters json --artifacts <capture>
```

Run baseline/candidate/candidate/baseline; add filters `*ConeOblique` and
`*CylinderOblique` only for the narrow pilots. All acceptance timings exclude
competing builds, tests and profilers. Inspect every child exit, expected actual
sample count, retained statistics and raw GC counters, independently of parent
success and rounded summary allocation.

Capture families are
`artifacts/oblique-work-{pilot*,profile,final,focus,controls,sharedfocus,covered,accepted,smoke}-*`,
with source variants under `artifacts/oblique-work-perf`. The committed baseline
is `artifacts/oblique-inventory-perf/cross`. The failed intermediate payload and
replays are under `artifacts/oblique-work-pilot-D2-payload` and
`artifacts/oblique-work-failure-replay`. These ignored files preserve raw
evidence; the source revisions, fixture recipes, proofs, commands, dispositions
and numerical results here remain sufficient to understand/reopen the work after
their deletion. No release-package, cross-platform or simulation-frame
throughput claim follows from these local query measurements.

## Analytic construction and quartic preparation (2026-11-03)

This continuation uses the retained implementation in the preceding section as
its incremental baseline. The final cumulative comparison still uses committed
`87f18d2`; means from different windows are never pooled. The owner requested a
repeatable twofold gain on both queries while retaining exact ties, complete
witnesses and downstream Gravitas compatibility. The signal stays experimental
until both targets earn that comparison.

### Work removed and exactness contracts

A fresh sampled profile attributes roughly 38%/42% to analytic selection and
41%/41% to rim selection for cone/cylinder, including their callees. Root
isolation itself is only about 2% for either query. These inclusive samples are
not additive wall-clock costs. The retained work addresses construction,
admission and materialization before adding another root solver.

- Pass each fixed-width rational normal directly into candidate admission.
  Reject unsupported directions before writing generic magnitude fields. For a
  required edge endpoint, compute its exact projection once, scan the canonical
  extrusion level in ascending order, and reject at the first smaller value.
  Successful scans retain the same first equal vertex, including zero-Up ties;
  rejected candidates avoid the remaining scan and two redundant wide dots.
- Ordinary rational candidates use 24-word fields; domains keep their original
  forty-word representation. Normals below 504 bits, vertices below 165 bits,
  half-axis/radius below 163 bits, raw scale below 99 bits and q below 65 bits
  give T below 672, q*T below 737, and the largest complete squared field below
  1,475 bits. All 1,536 bits are retained. Compare the compact view before
  admission, then clear and pad every accepted slot into the existing forty-word
  selection. Exact equality keeps the earlier fields and witness metadata. The
  existing radial builder uses the supplied stride; its q/d inputs may still
  alias output slots 9/10. No new candidate type, cache or public API is needed.
- Reduce each nonzero ordinary analytic edge by its positive component GCD once.
  Crosses/bases scale by its inverse and perpendicular projections by its
  inverse square. Rays, signs, required vertices, family order and exact ranks
  therefore stay identical while subsequent products shrink. Real edges fit 192
  bits. The existing rim fallback uses the same relocated reducer; domain
  analytic edges retain their original representation for open-window mediants.
- Certify native wall orientation with complete 65-bit raw footprint differences
  and determinants below 131 bits, instead of transformed vertex dot products.
  The omitted transform factor is strictly positive for validated rotations.
  During segment clipping, a side with an exact authored-yaw X or Z component of
  zero duplicates an earlier coordinate constraint. Both orientations of the
  complete coordinate support were already applied, so skipping that duplicate
  cannot change the retained bound ratio. Zero edges are included in this test.
- For an ordinary non-pole cylinder side, H.n=0 and M=n.n>0. The translated
  segment's point, displacement and denominator share one positive M. Retain
  `point=-H*M+minimum*n`, `delta=2H*M`, `denominator=M` directly. All clipping
  ratios and final anchors are identical. Poles, cones and enabled windows keep
  the general representation. A fixed ordinary segment with zero displacement
  retains the original parameter zero directly; Debug keeps membership asserts.
- The final ordinary cylinder vertex family needs only positive projected-vertex
  rays. In the perpendicular plane the gap is R minus the prism minimum
  projection. An open vertex minimum uses the nearest projected point's positive
  ray; its negative ray is an open maximum. Boundary/flat cases belong to the
  earlier real-edge directions. The same supporting vertex and finite side
  witness are recovered independently of the seed. Domain traversal keeps both
  orientations and existing canonical order.
- Recognize the exact metric `M=m*(1-2t+2t*t)`. Build the stationary derivative
  with unit content, then restore physical M in its staged slots before gap
  completion. K/L/J lose m and N/D lose m squared together. Original Q, chart
  vectors, parameter roots, multiplicity and ordinals stay unchanged. Domain rim
  witnesses are unique stationary point ratios, not analytic interval mediants;
  different root isolation cells cannot change their exact predicates/rounding.
- After the existing complete chart bound misses, ordinary certified vertical
  edges may use a concavity certificate. Let d=A.E, K=q*(E.E), and let alpha be
  the existing smaller nonnegative normalized endpoint projection. The projected
  ellipse has semiaxes R and r=R*abs(d)/sqrt(K), with 0<r<R. Its maximum angular
  second derivative is `(R*R-r*r)/r`. If alpha is at least that value, the
  entire support gap is concave, so earlier physical endpoints/switches own
  minima and ties. For alpha squared=a squared/M, compare exactly
  `a*a*K*d*d >= R*R*(K-d*d)*(K-d*d)*M`. Complete products are below 1,480 bits;
  four 24-word slices reuse dead bound scratch. No additional bank/helper is
  introduced. Two additional target charts per query are certified before their
  stationary builder. Finite domains and other edge families retain the solver.

The independent public concavity fixture uses authored axis `(3k,4k,0)` raw,
k=858993459, length/radius 2, and footprint X=[-3/2,10], Z=[-1,10],
half-height 10. Choose center.x=-axis.x for the cylinder and +axis.x for the
cone. A chord and support-subadditivity proof gives the unique minimum 3 along
+Z. The old bound is only 13/5; the new cleared comparison is 400>=324. Both
windings retain full anchors: primitive local `-axis +2Z`, prism
`(0,-axis.y,-1)` for cone and `(-2axis.x,-axis.y,-1)` for cylinder. Translation
by -3Z touches; one further raw unit separates. All four rows pass against both
the pre-certificate and retained implementations. A separate nearly
perpendicular-axis fixture proves an earlier +X/-X exact tie, its complete
anchors and one-raw clearance. This prevents an apparently redundant offset
family from silently changing canonical ownership.

### Completed experiment windows

All rows use frozen sibling sources, Release out-of-process BenchmarkDotNet,
affinity 3, two advertised processors, five warmups, ten actual iterations, one
launch and 500-ms iterations. Every complete window has all expected successful
children, ten raw actual samples per child, at least eight statistical retained
observations per case, zero raw allocation/collection counters and PE/PDB/source
bindings for actual generated child payloads. Each variant matches 1,320 full
contact/anchor snapshots and fourteen mixed fixtures before timing.

| Window and variant                                  | Cone mean, us | Cylinder mean, us | Decision                              |
| --------------------------------------------------- | ------------: | ----------------: | ------------------------------------- |
| Pilot1 retained baseline                            |       932.609 |          1110.294 | Incremental reference                 |
| Rational upper bound plus smaller bound fields      |       920.236 |          1121.895 | Reject: cylinder slows 1.05%          |
| Compact candidates/direct normal alone              |       931.590 |          1069.260 | Retest in combined workload           |
| Both pilot1 changes                                 |       909.379 |          1076.860 | Reject weaker bound component         |
| Pilot3 retained baseline                            |       939.980 |          1101.279 | Incremental reference                 |
| Construction/witness changes, forty-word candidates |       813.665 |           975.746 | Keep measured bundle                  |
| Same changes, compact candidates                    |       799.256 |           926.624 | Keep: -14.97% / -15.86%               |
| Forty-word bundle plus three per-query squares      |       817.423 |           961.044 | Reject: fails to beat compact variant |
| Pilot4 compact bundle baseline                      |       806.560 |           913.583 | Incremental reference                 |
| Required-support scan and compact/domain cleanup    |       769.816 |           880.623 | Keep: -4.56% / -3.61%                 |
| Cleanup plus primitive analytic edges               |       733.278 |           850.414 | Keep: -9.09% / -6.91%                 |
| Pilot5 primitive-edge baseline                      |       709.491 |           833.747 | Incremental reference                 |
| Add vertical-edge concavity                         |       668.343 |           800.833 | Keep: -5.80% / -3.95%                 |

Pilot1/3/4/5 complete 16/16/12/8 children and 160/160/120/80 actual
measurements: 52 children and 520 actual measurements total. These window means
are not pooled. In pilot3, both compact launches beat both retained launches; in
pilot4, each successive variant beats both launches of its predecessor; in
pilot5, both candidate launches beat both baseline launches on both queries. The
per-query square cache adds 152 bytes to geometry and has mixed results; no
field or consumer change from that experiment remains. The all-chart curvature
version is not maintained; observed reach supports the narrower vertical family.
Deleting the projected-offset candidate family is rejected on an exact canonical
normal/anchor tie counterexample, despite ordinary fixture equality.

### Excluded native-fault window and bounded replay

The intervening pilot2 is excluded in full. Its second core launch's cone child
finishes five warmups, then reports fatal `0xC0000005` before its first actual
sample. The stack names
`System.Reflection.MethodBaseInvoker.InterpretedInvoke_Method`,
`InvokeDirectByRefWithFewArgs`, `MethodBase.Invoke`, and the generated
`AfterAssemblyLoadingAttached`/`Main`. It supplies no original instruction,
faulting address or geometry frame. The cylinder child completes, but those
samples and the preceding partial window are not accepted throughput evidence.

This core prototype contains direct-normal/compact construction, metric content
reduction, native orientation and the zero-displacement shortcut. It does not
contain the later positive vertex family, coordinate-plane clipping,
side-translation cancellation, required-support scan, primitive analytic edges,
query-square fields or vertical concavity. Passing B1 and faulting B2 bind the
same math DLL SHA256:
`B170539D7C95433F62335054A3A2CEA2505177695F0D2B609277D7187ED6A3FE`. The actual
generated child tree, seven PE/PDB pairs, source bindings and logs are preserved
under `artifacts/oblique-next-failure` and pilot2 payload directories. They aid
replay; the conclusions here do not depend on their future retention.

On the same Windows/.NET 10.0.10 host, CDB's current first-chance collector is
recalibrated with a caught-null control: an actual AV is captured and the
control then exits successfully. Five debugger replays of the preserved core
child and five of the retained baseline all complete ten actual measurements,
exit 0 and capture no first-chance AV. Static independent audits find no
verified span overrun, scratch alias, escaping lifetime, unsafe operation or
capacity defect. Console/debugger replays differ from the original pipe-hosted
launch; these bounded negatives establish no cause or fix. The fault remains
unattributed.

The relevant reproduction shape is source mode with UseLocalLsfStack=true,
DisableTransitiveProjectReferences=true, UseSharedCompilation=false and
BuildInParallel=false, then the generated Release cone child from the documented
five-warmup/ten-actual/500-ms/affinity 3 job. Preserve the original child and
source checksums before rebuilding. On recurrence capture first-chance AV
instruction, registers, native/managed stacks and a dump before continuation;
compare matched baseline/candidate launches and exclude incomplete windows. Do
not catch, retry and pool failed measurements or label later successful runs a
fix. The earlier managed-fault investigation remains separately recorded above.

### Final validation and cumulative comparison

The final source passes the complete local source-stack matrix on Windows x64,
SDK 10.0.302/runtime 10.0.10. Solution rebuilds cover both library target
frameworks; the test projects execute net10.0. All raw OpenCover points, rather
than rounded report percentages alone, are visited:

| Repository/configuration   | Core tests | Companion tests | Sequence points | Branch points |   Methods |
| -------------------------- | ---------: | --------------: | --------------: | ------------: | --------: |
| FixedMathSharp Release     |       7348 |              49 |     68577/68577 |   22746/22746 | 5333/5333 |
| FixedMathSharp ReleaseLean |       7327 |              49 |     68670/68670 |   22746/22746 | 5329/5329 |
| Gravitas Release           |       8348 |               — |     53559/53559 |   21772/21772 | 5602/5602 |
| Gravitas ReleaseLean       |       8277 |               — |     53557/53557 |   21772/21772 | 5601/5601 |

Independent review covers all eleven existing runtime files and five test files,
including actual shared/eager callers, retirement, complete intermediate widths,
scratch lifetimes, first ties and physical witnesses. The cumulative runtime
change is 318 net lines relative to `87f18d2`, with no new runtime file/type,
cache, public API or allocation. The two redundant constant/quadratic
metric-sign conditions are retired: valid chart vectors are nonzero, so their
squared lengths are strictly positive. A Debug assertion records that
precondition.

The final frozen graph matches all 738 authored source/test/project files in the
validated math checkout; its 430 source/project files include every runtime
owner. The unchanged dirty GridForge source matches all 138 checked files in the
baseline graph. Gravitas, Chronicler and SwiftCollections remain unmodified. The
benchmark source graph uses the same sibling revisions as the preceding section.
No released-package or other native platform/architecture lane is claimed.

A fresh full-query ABBA against `87f18d2` completes 56 successful children and
560 actual samples. Raw allocation/collection counters are zero for all
children; all expected exports, retained observations and generated payload
bindings pass. These are microseconds per complete mixed query on the measured
i7-9700K host.

| Query                       |       A1 |      B1 |      B2 |       A2 | Balanced cost change |
| --------------------------- | -------: | ------: | ------: | -------: | -------------------: |
| ConeSeparated               |   73.221 |  60.194 |  58.376 |   74.983 |              -20.00% |
| ConeFace                    |  109.506 |  70.936 |  71.633 |  108.598 |              -34.63% |
| ConeOblique                 | 1412.586 | 671.817 | 688.205 | 1425.227 |              -52.07% |
| CylinderSeparated           |  118.764 |  64.702 |  61.884 |  120.225 |              -47.03% |
| CylinderFace                |  108.008 |  72.599 |  71.789 |  107.944 |              -33.14% |
| CylinderOblique             | 1400.221 | 806.764 | 809.023 | 1397.028 |              -42.24% |
| ConeApexTouch               |   97.774 |  60.553 |  60.628 |   98.324 |              -38.20% |
| ConeSideTouch               |  414.230 | 318.709 | 319.983 |  408.203 |              -22.34% |
| CylinderSideTouch           |  123.921 |  67.273 |  69.161 |  121.388 |              -44.38% |
| SphereContained             |  117.060 | 115.130 | 115.758 |  112.720 |               +0.48% |
| CapsuleContained            |  121.300 | 117.868 | 118.692 |  118.383 |               -1.30% |
| CapsuleCrossingOffcenter    |  115.314 | 120.878 | 118.170 |  115.081 |               +3.76% |
| SphereContainedOpposingFace |  115.484 | 117.749 | 115.789 |  113.716 |               +1.89% |
| SphereHorizontalRim         |  159.792 | 162.686 | 158.266 |  159.890 |               +0.40% |

Cone averages 1418.906 to 680.011 us, a 52.07% cost reduction and 2.09x
throughput. Cylinder averages 1398.625 to 807.894 us, a 42.24% reduction and
1.73x throughput. Both candidate target launches beat both baselines. Cone's
slower candidate is also below half the faster baseline. Cylinder remains above
the provisional half-cost target, so the signal remains open under Experimental
Signals.

A separate direct ABBA against the previously retained 193-line implementation
completes 8 children/80 actual samples with the same zero raw counters and
binding checks. Cone averages 917.674 to 677.149 us (-26.21%); cylinder 1087.918
to 806.944 us (-25.83%). Both candidate launches beat both baselines for each
query. This measures the incremental gain without pooling earlier experiment
windows.

All 1320 final full contact/anchor snapshots and 14 mixed fixtures match the
original expected fields. Each of 14 fixtures reports zero calling-thread bytes
for 128 warmed queries. The complete out-of-process Short smoke run passes all
479 children,1437 actual iterations and 479 populated statistic exports. Its13
positive allocation cases are exactly the previous enumerable-construction,
formatting and serialization cases; all other 466 cases have zero raw
allocation/collection counters. All contact kernels retain zero allocation.

The main capsule-crossing control is +3.76%; sphere opposing-face is +1.89%. A
separate reversed-order shared window completes 12 children/120 actual samples:
box/capsule 98.262 to 99.509 us (+1.27%), box/cylinder 981.891 to 1002.416 us
(+2.09%), triangle/capsule 659.889 to 657.906 us (-0.30%). The older
box/cylinder uncertainty predates this continuation. Those initial signals
prompted the reversed mixed and direct shared comparisons below.

The reversed five-query BAAB completes 20 children/200 actual samples, all with
zero raw counters and complete payload/statistic checks. Target gains repeat at
-52.41% cone and -42.56% cylinder. The earlier capsule-crossing signal does not
repeat (+0.10%); sphere contained is +0.29%, opposing-face is -0.81%. These
control windows do not establish a repeatable penalty for those mixed queries.

A direct shared BAAB against the retained implementation completes 12 children
and 120 actual samples with zero raw counters. Box/capsule is +0.98%, with
interleaved launch ranges; box/cylinder is -0.74%, also with overlapping ranges.
This continuation does not establish additional box/cylinder overhead beyond the
older uncertainty. Triangle/capsule is +2.86%, with both candidate means higher.
A bounded opposite-order single-case ABBA repeats a +1.77% difference (630.910
to 642.066 us). This is relevant shared code: the triangle benchmark definitely
calls BuildRadialCandidate through its first admitted Up seam direction at 40
words. It cannot be dismissed as a disconnected path.

A final three-version ABC/CBA triangle check changes that result: retained
636.920, current 632.360 (-0.72%) and proposed scratch-reuse 632.128 us
(-0.75%). Current and retained launch ranges overlap. Across these windows the
small triangle percentage is not stable; neither no-overhead nor an assigned
regression cause is established. Preserve the uncertainty and reopen it for a
consistent matched signal plus attribution, instead of changing arithmetic on a
timing hypothesis.

The proposed scratch overload borrows a dead BuildPlaneCandidate temporary and
restores fixed forty-word scratch for legacy callers. It matches 1320
full-anchor snapshots,14 mixed fixtures and zero warmed allocations, but the
triangle check finds no independent gain. A separate target ABBA gives cone
652.096 to 657.863 us (+0.88%) and cylinder 772.784 to 769.765 us (-0.39%,
overlapping launch ranges). Reject it: neither the overload nor its caller
change remains in maintained source. The six triangle and eight target children
complete 140 actual samples with zero raw counters; they are a separate rejected
experiment, not pooled final performance evidence.

The main, direct, reversed and shared final/control windows total 112 successful
children and 1120 actual measurements, all with zero raw allocation/collection
counters. The rejected scratch experiment adds 14 children/140 actual samples;
the earlier four complete pilots add 52/520. The broad Short smoke has its
separate 479/1437 accounting and 13 intentional allocating cases. All
comparisons use their own paired means and source-bound payloads.

Scoped source whitespace verification, pinned Markdown formatting/checks and
`git diff --check` pass. Full semantic `dotnet format` exits 2 with workspace
loading/import diagnostics: it labels required Xunit imports unnecessary and
reports the Witnesses import order already present in HEAD. Removing the Xunit
imports was independently disproved by compiler errors, so those imports remain.
No unrelated formatter edits survive. This limitation does not change the clean
Release/Lean builds or the complete test/coverage results above.

Final independent code and evidence review finds no actionable correctness,
width, lifetime, tie, witness, duplication or evidence issue. The shared-control
uncertainty, semantic-formatter limitation and unattributed native fault remain
explicit. Changes remain unstaged and uncommitted for owner review.
