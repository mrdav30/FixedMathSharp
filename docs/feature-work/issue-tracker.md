# Feature Work Issue Tracker

## Purpose

This document tracks unresolved correctness, determinism, API, documentation,
build, and tooling issues that fall outside an active implementation plan.

Measured performance and allocation concerns belong in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md).
Work that requires staged implementation belongs in a focused feature-work plan.

## Tracker Rules

- Record only unresolved, reproducible concerns.
- Include the affected area, evidence, user or runtime impact, priority, and
  smallest useful next action.
- Use repository-relative paths and portable commands. Do not record
  developer-specific drive letters, home directories, or machine-local
  dependency locations.
- Preserve the reproducer, source revision, observed failure and investigation
  conclusions here. Ignored artifacts are disposable supporting evidence;
  reproducing or continuing an issue must not require their survival.
- Keep an issue here while it needs investigation or is deliberately deferred.
- Link an active implementation plan instead of duplicating its task details.
- Remove an entry after its resolution and verification are complete. Preserve
  durable design decisions in the completed plan, public documentation, or
  release notes rather than retaining a resolved-issue archive here.

## Active Issues

### FMS-Issue-023: Finite cylinder/capsule contact misses rim separation

- **Status:** Confirmed on 2026-09-24 against `b7a6b02`, while evaluating
  exact posture clearance for Gravitas `GRV-Issue-081`. Reproduced and expanded
  against `5ddf2c5`; implementation remains open.
- **Proposed design:** [Complete cylinder/capsule contact](2026-09-24-cylinder-capsule-contact-design.md)
  captures the private solver replacement, reuse boundaries, and acceptance
  gates. Awaiting design review; no runtime cutover has occurred.
- **Priority:** High
- **Affected area:** `FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact` and
  `WideConvexPrismRelations.RigidFiniteShapePairs.cs`. The current finite set
  of tested directions does not completely classify cylinder-rim separation.
- **Evidence:** A cylinder at the origin, axis +Y, length 2, radius 10;
  a capsule at `(20.75, 1.75, 0)`, axis +X, length 20, radius 1. The nearest
  capsule endpoint is `(10.75, 1.75, 0)` and its squared distance to the rim
  `(10, 1, 0)` is `9/8 > 1`, yet contact returns true. At capsule center
  `(20.75, 2, 0)` and radius `5/4`, the exact rim distance is `5/4` (tangent),
  but returned depth is `0.25`; raising Y by one raw unit still returns true.
  `FixedSegmentStrictClearanceRegressionTests` preserves all three failures
  and penetrating controls. Reproduce with
  `dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter FullyQualifiedName~FixedSegmentStrictClearanceRegressionTests`.
- **Expanded contact evidence:** `CenteredCylinderCapsuleRimContactTests`
  covers endpoint, interior, and oblique interior rim features through the
  public contact API. A cylinder of radius 10 and length 2, and a capsule core
  through `(43/4, 2, 0)`, have closest rim point `(10, 1, 0)` and core distance
  `5/4`. Capsule radii `5/4` and `6/4` must give depths 0 and `1/4`, not the
  observed `1/4` and `1/2`. Both a +X endpoint and a +Z interior core exhibit
  the error.
  For a genuinely oblique case, use a cylinder of radius 1 and length 2,
  capsule center `(4, 5, 0)`, length 2, and quaternion raw components
  `607400100 * (5, 0, -3, 4)`. The exact rigid-frame +Y axis is
  `(12, -9, 20)/25`. The rim point `(1, 1, 0)` and normal `(3, 4, 0)/5`
  prove core distance 5, with an interior core witness. Radius `5 - 1 raw`
  must miss; radii 5 and `5 + 1 raw` must produce depths 0 and 1 raw. The
  existing method admits all three and reports about `0.02118` excess depth.
  Independent +/-1-raw vertical translations also preserve the tangency
  classification reproducer. Run the same test command with
  `--filter FullyQualifiedName~CenteredCylinderCapsuleRimContactTests`.
- **Core-overlap evidence:** The same new fixture also prevents an
  outside-only distance repair from masquerading as a complete fix. For the
  unit-radius, length-2 cylinder, use capsule center `(7/4, 0, 1/4)`, length
  10, radius `1/4`, and quaternion raw components
  `1920767767 * (0, 0, -1, 2)`, giving exact axis `(4/5, 3/5, 0)`. Its core
  intersects the cylinder. Translation `3/5` along `(3, -4, 0)/5` already
  separates the shapes, but the old method reports depth raw `3096962338`
  (about `0.72107`), exceeding that proved upper bound.
- **Impact:** False collision and incorrect solver depth; exact posture
  clearance cannot reuse this contact classifier merely by changing `<=` to
  `<`. This defect predates the new strict predicates.
- **Repair boundary:** The public method promises minimum translation, not
  merely closed-overlap truth. Its finite candidate list and closest-core
  direction are incomplete; adding the strict Boolean predicate cannot repair
  normal or depth. For disjoint cores, capsule depth is radius minus exact
  segment-to-cylinder distance. Intersecting cores require radius plus the
  inside boundary distance of the cylinder swept along the capsule core.
  On normals perpendicular to an oblique capsule axis, that boundary includes
  projected cap ellipses; their stationary normals require quartic root
  selection. Existing rounded-cylinder Sturm machinery is reusable, but its
  current result is root existence or a rounded ray distance, not retained
  root identity with exact sign/comparison and final contact rounding.
- **Next action:** Design a focused private contact-kernel replacement with
  complete feature selection for both core domains, arbitrary admitted rigid
  frames/local axes, exact tangency, and deterministic ties. Reuse fixed-limb
  arithmetic and Sturm machinery rather than introduce a generic algebra
  framework. Preserve the public API and existing representable-anchor
  contract; an algebraic public anchor/serialization redesign is not required.
  Select the feature and depth before rounding normal and anchor outputs.
  Do not round endpoints, add an epsilon, or label sampled directions exact.
  Finish with full Release/ReleaseLean line/branch/method coverage and matched
  benchmarks, then validate affected Gravitas consumers.
- **Pre-change performance evidence:** Default out-of-process BenchmarkDotNet
  0.15.8, .NET 8.0.29, SDK 10.0.302, Windows 11, i7-9700K, source at `5ddf2c5`:
  `OrdinaryCylinderCapsule` mean 27.76 us (error 0.198 us),
  `FullDomainCancellationCylinderCapsule` 185.61 us (error 1.777 us),
  `IrreducibleWideCylinderCapsule` 1436.41 us (error 11.950 us); all 0 B/op.
  These are baseline costs, not correctness or improvement claims. Build the
  Release benchmark project and run the `rigid-finite-shape-relation` alias
  with `--filter '*OrdinaryCylinderCapsule' '*FullDomainCancellationCylinderCapsule' '*IrreducibleWideCylinderCapsule' --exporters json`.
  The last row also asserts the existing internal wide-candidate route; if a
  replacement changes that diagnostic, recapture matched benchmark fixtures
  before implementation rather than silently compare different workloads.
- **Diagnostic verification:** With production unchanged at `5ddf2c5`, the
  complete core suite plus the expanded regressions reports Release 3,124
  passed / 15 failed, and ReleaseLean 3,103 passed / 15 failed, with no skips.
  Twelve failures expose FMS-Issue-023 (nine newly captured); three are the
  existing FMS-Issue-024 regressions. Core and FluentAssertions retain 100%
  reachable line/branch/method coverage in both configurations: Release
  49,417 lines / 9,578 branches / 3,568 methods; Lean
  49,510 lines / 9,578 branches / 3,564 methods. Coverage is execution evidence,
  not a claim that the contact behavior is fixed or that the suite is green.

### FMS-Issue-024: Finite cylinder-pair contact misses intersecting cap-rim constraints

- **Status:** Confirmed on 2026-09-24 while completing Gravitas `GRV-Issue-081`.
- **Affected area:** `FixedSegment.TryGetCenteredFiniteCylindersContact` and its
  finite separating-axis candidates. This predates the strict-classification work.
- **Evidence:** Cylinder A has center zero, axis +Y, height 2 and radius 1;
  cylinder B has center `(7/4, 7/4, 11/8)`, axis +X, height 2 and radius 1.
  A requires `x^2+z^2 <= 1, y <= 1`; B requires `x >= 3/4` and
  `(y-7/4)^2+(z-11/8)^2 <= 1`. Their possible Z intervals cannot intersect:
  `(11/8)^2 > (sqrt(7)/2)^2`. Nevertheless, contact reports true.
  With radii `5/4` and second Z coordinate 2, the only common point is
  `(3/4,1,1)`, but contact returns depth `0.09497214644216001` instead of zero;
  a one-raw outward Z offset is also incorrectly admitted.
- **Verification boundary:** `CenteredCylinderPairStrictRegressionTests`
  retains the public counterexamples and penetrating controls. The separate
  [FMS-Issue-026 rounding runtime repair](done/2026-09-24-cylinder-pair-depth-rounding.md)
  lets all five cases complete: three geometric failures and two passing
  penetration controls. It does not change candidate selection or fix these
  contact classification/depth failures. Boolean classification should not pay
  for contact materialization.
- **Required fix:** A complete finite cap/rim authority, not another sampled
  direction or a depth epsilon. Preserve touching/penetrating distinctions and
  truthful contact materialization in the owning repair.

### FMS-Issue-025: Box-cylinder contact can miss cap-clipped radial separation

- **Status:** Confirmed on 2026-09-24 while completing Gravitas `GRV-Issue-081`.
- **Affected area:** `FixedOrientedBox.TryGetCenteredCylinderContact`.
- **Evidence:** Cylinder center zero, +Y, height 2, radius 1. Box center
  `(1.26,-1.65,-0.92)`, half-extents `(0.83,0.75,0.52)`, rotation proportional to
  quaternion `(-1,-9,7,0)`. For exact authored fractions the cap-clipped box's
  minimum squared radial distance is `2530093/1922000 > 1`; admitted fixed
  representations remain well separated. The legacy public contact query
  nevertheless returns true with depth raw `160035987`.
- **Verification:** Executable legacy RED was observed before
  `CenteredCylinderPolytopeStrictOverlapTests` switched to the complete strict
  predicate. Keep the ordinary contact repair distinct from the new posture
  classification; a strict predicate passing does not certify solver contacts.

**Next issue ID:** `FMS-Issue-027`

## Issue Template

```markdown
### FMS-Issue-###: Concise title

- **Status:** Needs reproduction | Confirmed | Planned | Deferred
- **Priority:** Critical | High | Medium | Low
- **Affected area:** Public API, source owner, package, build, or documentation
- **Evidence:** Minimal reproduction, failing test, or portable command
- **Impact:** Observable correctness, determinism, usability, or tooling risk
- **Next action:** The smallest useful investigation or implementation step
```

When adding an issue, use the next ID and increment the field above. Never reuse
an ID that appears in a completed plan or repository history.
