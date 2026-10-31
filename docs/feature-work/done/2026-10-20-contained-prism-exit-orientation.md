# FMS-Issue-035: Contained sphere/capsule prism exit orientation

## Scope and source

Correct the signed exit normal and associated support orientation in the
existing sphere/capsule prism projection reducer. Starting revisions are
FixedMathSharp `5a07adc` and Gravitas `11e7e85`. All development, benchmark
children and coordinated validation use `UseLocalLsfStack=true`.

The change retains the public APIs, projection inventory, exact depth ranking,
closed admission, arithmetic widths, rounding and clamping. Cylinder/cone prism,
oriented-box/hull capsule and pure 2D contact owners are separate paths.

## Reproducer and proof

Use a prism centered at zero with footprint `(-2,-2), (2,-2), (2,2), (-2,2)` and
half-thickness `1`. A sphere centered at `(0,0.5,0)` with radius `0.25`, or a
Right-axis capsule with that center, core length `1` and radius `0.25`, has Y
extent `[0.25,0.75]`.

The first complete exit is an upward translation of `0.75`: the lower shape
support touches the upper prism face at `Y=1`, then one raw unit farther
separates the complete shapes. Therefore the normal is Down. The old query
returns depth `0.75` with normal Up and witnesses `(0,0.75,0)` / `(0,1,0)`;
moving opposite that normal leaves the shape inside the prism. The repaired
witnesses are `(0,0.25,0)` / `(0,1,0)`, with difference `0.75 * Down`.

For any candidate axis, let the capsule's symmetric support extent be `h` and
the prism interval relative to the capsule center be `[m,M]`. The existing exact
depths are `positiveDepth = h-m` and `negativeDepth = h+M`:

- Positive depth belongs to normal `+axis`, capsule maximum and prism minimum.
- Negative depth belongs to normal `-axis`, capsule minimum and prism maximum.

The two exterior branches already obey this association. The two straddling
branches retained the correct depth and prism endpoint but reversed the normal.
Correct their two booleans and derive the prism endpoint directly from that same
orientation, removing the redundant independently assigned endpoint flag. The
final materializer then selects the correct capsule radial support and, when the
normal has an axial component, the facing finite end cap.

Keep all four branches and their comparisons. A `[0,0]` interval belongs to the
earlier nonnegative-minimum branch; an equal-depth straddling interval belongs
to the negative-depth branch. Earlier axes continue to own equal depths. Neither
a signed rational term alone nor a rounded depth replaces exact radial admission
and ranking. No helper, numeric representation or public API is added.

## Regressions and downstream behavior

The pre-fix focused math capture has 21 failures and 12 passing controls. The
four new downstream rows also fail on their forwarded normals after executing
the actual host response. These are failure captures, not unexplained test
skips.

The retained math suite adds 24 rows covering mirrored XYZ exits for spheres and
positive-core capsules, zero-core sphere parity, facing axial end caps, centered
ties, collapsed footprint intervals, native yaw, exact quaternion permutations,
anchors beyond the scalar domain, finite far-origin supports and warmed
zero-byte guards. Cardinal support extents independently prove the selected
minimum and the touching/one-raw separated neighbors; witness agreement alone is
insufficient.

Older tests explicitly expecting the wrong mirrored normals now assert the
physical exit and paired witnesses. The symmetric scalar-limit test retains
exact depth clamping and its first candidate's tie policy with the corrected
support sign. The both-world-points-unrepresentable test uses supports at
`MaxValue+2` and `MaxValue+1`, retaining its original anchoring purpose without
depending on the old wrong normal to make both points unrepresentable.

Gravitas adds four host-step rows: sphere/capsule with either the dynamic 3D
shape or the dynamic 2D prism. The former checks an upward exit; the latter uses
a horizontal contained fixture and checks a leftward prism exit within its 2D
mobility. Each runs `Simulate`/`LateSimulate` for penetration response, an
independently authored exact touching pose, and an outward impulse followed by
whole-shape clearance. Events and three authoritative hashes including solver
caches replay identically in fresh contexts.

The 3D response updates `SolidBody.Position3d` before the collider's next
refresh; post-response assertions read that authoritative pose. Direct later
geometry queries follow explicit positioning and a full step. The 2D response
refreshes its collider immediately. No Gravitas production behavior is changed.

Two older Gravitas narrow-phase facts also expected inward normals. Their
replacement expectations prove the outward whole-sphere minimum, exact paired
supports, independently authored touching poses and one-raw separation.
`SetPosition` publishes collider geometry immediately for those direct queries.

Two maintained dispatcher benchmarks, `SphereContained` and `CapsuleContained`,
join the existing nine controls. Setup requires depth `0.75`, normal Down,
unclamped output and exact finite witness points. These guards intentionally
reject the incorrect historical baseline; no before/after latency improvement is
claimed for this correctness repair.

## Separate finite-witness finding

Review also found a pre-existing finite support-matching defect for a long core
crossing a small off-center prism, and for a large-radius support that the
closest-polygon-boundary query matches to the same rather than opposing face.
Committed and corrected-source public probes reproduce both. A fully contained
radius-`0.75` sphere in the centered `X=+/-1`, `Y=+/-2`, `Z=+/-3` prism also
selects the same rather than opposing X face: its correct minimum is `1.75`, but
the witness difference is `0.25*Right` after orientation correction. This is
tracked as
[FMS-Issue-036](../issue-tracker.md#fms-issue-036-spherecapsule-prism-witnesses-can-miss-the-opposing-finite-support),
with a self-contained reproducer and complete-depth proof. It needs finite core
clipping/opposing feature ownership, rather than another normal sign change.
This orientation repair does not claim to complete that separate witness work.

Supplemental evidence is `artifacts/issue035-witness-{committed,fixed}.jsonl`.
The committed control is built from an isolated `git archive` of `5a07adc`; it
does not substitute the changed ordinary-workspace source.

## Validation

The retained focused math run passes all 242 prism rows. Raw coverage of
`TryKeepAxis` and `GetMatchedPrismOffset` is 100% line/branch; their complexity
is 18/12. All four coordinated configurations and benchmark smoke runs pass.

| Repository/configuration   | Core tests passed | Chronicler tests passed | Covered lines | Covered branches | Fully covered methods |
| -------------------------- | ----------------: | ----------------------: | ------------: | ---------------: | --------------------: |
| FixedMathSharp Release     |             6,820 |                      49 | 65,794/65,794 |    20,870/20,870 |           5,104/5,104 |
| FixedMathSharp ReleaseLean |             6,799 |                      49 | 65,887/65,887 |    20,870/20,870 |           5,100/5,100 |
| Gravitas Release           |             7,453 |                       — | 50,698/50,698 |    18,878/18,878 |           5,370/5,370 |
| Gravitas ReleaseLean       |             7,384 |                       — | 50,698/50,698 |    18,878/18,878 |           5,370/5,370 |

These are exact raw Cobertura class-line, branch and method-node counts, rather
than rounded percentages, after the existing generated-file filters. Gravitas
Release's unfiltered XML additionally includes two covered lines and one covered
method in the generated PhysicsLayer MemoryPack formatter. The FixedMathSharp
ReportGenerator totals agree with its raw counts. Gravitas's ReportGenerator
presentation reports 66,205/25,446/6,646, all covered: it creates a synthetic
`Gravitas.CollisionHandling` namespace container alongside its child classes,
counting that content twice. Raw XML has no unhit lines, partial branch
conditions or partially covered methods. Captures use `artifacts/issue035-*` in
FixedMathSharp and `artifacts/issue035-retained-*` in Gravitas. The first
coordinated run retained the two old Gravitas inward-normal failures; the fresh
Gravitas captures follow their corrected and strengthened physical expectations.
The 1,931-file source checkpoint differs between those phases only in that one
Gravitas test file; the passed FixedMathSharp runtime/tests are unchanged.

All solution builds report zero warnings/errors, and all tests pass without
skips. Independent review checked the connected source, physical oracles, raw
coverage and source checkpoints. The warmed contained-contact allocation guard
passes with zero calling-thread bytes.

The frozen-source Gravitas dispatcher run completes all 11 children with zero
exits, complete statistics and zero raw allocated bytes. Its short-run means for
`SphereContained` / `CapsuleContained` are 319.81 / 314.48 microseconds per
query (five measured iterations, three warmups, one launch). These are query
diagnostics, not full-step capacity or before/after performance acceptance.
Host: Windows 11 x64, Intel i7-9700K, .NET 8.0.29, SDK 10.0.302, BenchmarkDotNet
0.15.8; serial execution, processor count two and affinity `0x3`. Capture:
`artifacts/issue035-dispatcher-smoke`, with log and payload bindings beside it.
The child assemblies bind through PE/PDB IDs and authored source checksums to
FixedMathSharp (331 documents), Gravitas (402), benchmarks (79), Chronicler
(29), SwiftCollections (80) and GridForge (121).

The frozen 1,931 source/test files match the final ordinary-workspace checkpoint
after line-ending normalization. Snapshot builds below the owning checkout
inherit its `.globalconfig` as well as the copied one, producing duplicate
analyzer-key warnings; ordinary solution builds above are clean.

The broad FixedMathSharp `all -j Short --iterationTime 10` smoke completes
479/479 children with zero exits and complete statistics across 26 reports. It
verifies execution, not performance acceptance. Its child PE/PDB and 331
authored math documents bind to the frozen source. Captures are
`artifacts/issue035-smoke`, `issue035-smoke.log` and
`issue035-smoke-payload.json`. All 479 raw GC rows are retained: 458 zero, 13
expected enumerable/formatting/ serialization controls and eight 336-byte
process-wide observations:

| Unchanged benchmark                                                                   | Child PID | Operations |
| ------------------------------------------------------------------------------------- | --------: | ---------: |
| `BoundsBenchmarks.Triangle3dArea`                                                     |     32160 |      6,400 |
| `BoundsBenchmarks.BoxVolumeExpansionCost`                                             |     16456 |     69,120 |
| `Fixed64ArithmeticBenchmarks.TryMultiplyDivideTwoFactorOrdinary`                      |     25172 |        506 |
| `Fixed64ArithmeticBenchmarks.MultiplyDivideOperatorChainThreeFactorOrdinary`          |     27080 |        336 |
| `Fixed64ArithmeticBenchmarks.MultiplyDivideOperatorChainThreeFactorRescuedSaturation` |     31576 |        592 |
| `ProjectedTriangleSweepBenchmarks.VerticalSubdivisionSweep`, subdivisions 16          |     16376 |          1 |
| `RigidFiniteShapeRelationBenchmarks.MultiRadicalCylinderCylinder`                     |     32100 |          1 |
| `Vector4dBenchmarks.Dot`                                                              |     32188 |        336 |

These events are consistent in size with the
[closed process-counter investigation](2026-10-14-capsule-interval-allocation-diagnostic.md),
but no fresh native stack capture attributes these particular events. Preserve
that limit, the launcher's two zero-summary warnings and all positive counters
in `artifacts/issue035-smoke-allocation-observations.json`; neither suppress the
signal nor infer workload allocation from it. The changed contained-contact
controls have zero raw benchmark bytes and pass their warmed allocation guard.

DocFX completes with zero warnings/errors; changed local Markdown targets and
fragments pass. No tracked generated API files are modified.

Portable focused reproduction, from each owning repository root:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~FixedConvexPrism'
# From the sibling Gravitas root:
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~Step_WithContainedRoundPrismContact_ShouldResolveTheNearestExitAndReplay'
```

Released-package validation remains a release gate after FixedMathSharp is
published. Local Windows execution does not establish all-platform execution.
