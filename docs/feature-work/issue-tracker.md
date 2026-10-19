# Feature Work Issue Tracker

## Purpose

This document tracks active correctness, determinism, API, documentation,
build, and tooling issues that fall outside an active implementation plan, and
indexes their verified resolutions or documented investigation closures.

Measured performance and allocation concerns belong in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md).
Work that requires staged implementation belongs in a focused feature-work plan.

## Tracker Rules

- Add unresolved concerns with concrete evidence to `Active Issues`. Include the
  affected area, reproducer, source revision, impact, priority and next action.
- Use repository-relative paths and portable commands. Do not record
  developer-specific drive letters, home directories or dependency locations.
- Keep active entries self-contained. Ignored artifacts are supplemental;
  reproducing or continuing an issue must not require their survival.
- Link an active implementation plan instead of duplicating its task details.
- After verification, replace the active entry with a compact `Resolved Issues`
  row: the same ID, resolution/closure date, brief outcome and an evidence link.
  Keep detailed root causes, decisions and validation in an existing completion
  record, regression or fix commit rather than duplicating them in this index.
- Distinguish verified fixes, mitigations and investigations closed without a
  reproduced defect. A closed investigation does not establish a runtime fix or
  an attributed cause; preserve its limits and recurrence guidance in the record.
- Resolved means verified in source or closed by documented decision, not
  released. Add release/version claims only when separately confirmed.
- Preserve every assigned ID through closure; never delete or reuse it. Retain
  prior evidence when new findings change an issue's status. Unknown historical
  IDs stay reserved rather than acquiring an inferred cause.

## Active Issues

### FMS-Issue-034: Canonical curved-prism depth can correct an exactly touching cone

- **Status:** Correctness repair and bounded throughput refinement validated on
  2026-10-19: both repositories pass Release/Lean tests with complete coverage;
  all 479 benchmark smoke children and the documentation build pass. The issue
  remains active for the measured oblique cost. The defect was pre-existing
  on `5674103`, with identical touching outputs before/after Issue 033.
- **Priority:** High.
- **Implementation:** [Complete curved-prism contact selection](2026-10-19-curved-prism-contact-depth.md)
  records the complete support-fan repair, finite witness matching, verified
  retained-source Release/Lean coverage in both repositories and balanced
  performance evidence. The issue remains active for throughput follow-through.
- **Affected area:** `FixedConvexPrismRelations.TryGetCenteredConeContact(...)`
  and the canonical projection depth/normal reducer in
  `WideConvexPrismRelations.TryGetContact(...)`. Gravitas forwards this depth
  into mixed-contact positional correction.
- **Evidence:** Cone center zero, identity rotation, local axis `Vector3d.Up`,
  height `2`, radius `1`. Prism origin `(0.5,1,0)`, zero yaw, half-thickness `1`,
  offsets `(0,-1), (1,-1), (1,1), (0,1)`. This is a full-volume prism with
  `X in [0.5,1.5], Y in [0,2], Z in [-1,1]`. Both versions return `true`,
  `Depth = 0.5` (raw `2147483648`) and `Normal = Vector3d.Right`.

  The exact plane `2*x+y=1` proves touching: the cone's apex and base-disk
  maximum are both `1`, and the prism minimum is `1`. Their only common point
  is `(0.5,0,0)`. Any positive prism translation along Right separates the
  shapes, so `0.5` is only a sufficient selected-projection separation bound,
  not the first exit or physical minimum correction. The same result occurs
  with collapsed offsets `(0,-1), (0,1), (0,1)`.

  Translating prism X by one positive raw unit exposes the separate Issue 033
  classification defect on the old implementation; the completed admission
  gate correctly rejects that neighbor. It preserves the existing depth and
  normal for the admitted touching and overlapping neighbors.
- **Impact:** An exactly touching or slightly overlapping pair can receive an
  excessive correction along a noncontact support direction. Closed intersection
  correctness does not establish correct physical contact depth or normals.
  The starting public XML documented the canonical bound; the repaired XML and
  geometry guidance describe exact minimum selection and final rounding limits.
- **Next action:** Review the validated small refinement and remaining exact
  rim/analytic selection cost before closure. Balanced timing against the
  accurate pre-refinement repair reduces
  cone/cylinder miss time by 9.9%/9.2%, face time by 35.0%/35.4% and oblique
  time by 12.6%/10.1%; the oblique means are 2.421/2.369 ms. Keep Issue 034
  active for this remaining cost; no universal simulation budget is claimed.
  The linked plan preserves the
  older incomplete-bound comparison, exact-source captures, independent
  geometric oracles and raw coverage counts.

### FMS-Issue-035: Contained sphere/capsule prism contacts reverse the selected exit

- **Status:** Confirmed pre-existing, 2026-10-19. Reproduced on committed
  `b813236` and the retained Issue 034 source with identical outputs.
- **Priority:** High.
- **Affected area:** `FixedConvexPrismRelations.TryGetSphereContact(...)`,
  `TryGetCenteredCapsuleContact(...)` and the inside-projection branches of
  `WideConvexPrismRelations.TryKeepAxis(...)`. Gravitas forwards these contacts
  into mixed sphere/capsule positional response. Issue 034's new cone/cylinder
  owner bypasses this preserved capsule-only reducer.
- **Evidence:** Identity frames, prism origin zero, yaw zero, half-thickness
  `1`, footprint `(-2,-2), (2,-2), (2,2), (-2,2)`. Sphere center `(0,0.5,0)`,
  radius `0.25`; alternatively a capsule at that center with local axis Right,
  core length `1`, radius `0.25`. Both public queries return `true`, depth
  `0.75` (raw `3221225472`), normal Up and witnesses `(0,0.75,0)` /
  `(0,1,0)`. Their difference is `-0.25*Up`, inconsistent with the published
  `depth*normal`. A portable capsule reproducer is:

  ```csharp
  Vector2d[] offsets = { new(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
  var center = new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero);
  FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
      center, FixedQuaternion.Identity, Vector3d.Right, Fixed64.One,
      Fixed64.Quarter, Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One,
      out FixedContactAnchors contact);
  // The returned exit direction still leaves the complete capsule inside.
  FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
      center - contact.Normal * (contact.Depth + Fixed64.FromRaw(1)),
      FixedQuaternion.Identity, Vector3d.Right, Fixed64.One, Fixed64.Quarter,
      Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.One, out _); // true
  ```

  The exact minimum is `0.75` with normal Down. Both shapes have Y extent
  `[0.25,0.75]`; moving Up by `0.75` touches the complete prism's upper face,
  and another positive raw unit separates. Moving Down by the returned depth
  leaves Y extent `[-0.5,0]` inside the prism. The sphere overload reproduces
  the same failure. The inside-projection branch pairs `negativeDepth` with
  the positive axis and conversely, then materializes the mismatched support.
- **Impact:** A contained mixed sphere/capsule pair can receive correction in
  the wrong direction and a physically inconsistent witness pair. Complete
  execution coverage does not establish this geometric invariant.
- **Next action:** Add public regressions for both sphere and positive-core
  capsule before correcting the shared orientation/endpoint associations.
  Check finite paired witnesses, complete-shape exit, signed interval cases,
  closed tangency, transformed frames and downstream host-step response. Keep
  the existing canonical capsule scope distinct from Issue 034's complete
  cone/cylinder minimum solver.

**Next issue ID:** `FMS-Issue-036`

## Resolved Issues

Historical rows come from committed tracker entries and linked completion
records. Dates reflect recorded resolution/closure, which can precede the commit
date. Evidence describes verification at closure; it does not claim a fresh
rerun or package release.

| Issue | Resolved/closed | Outcome | Evidence |
| --- | --- | --- | --- |
| FMS-Issue-033 | 2026-10-19 | Completed closed cylinder/cone-prism admission; the separate contact depth/normal defect remains active as FMS-Issue-034. | [Record](done/2026-10-19-curved-prism-admission.md) |
| FMS-Issue-031 | 2026-10-14 | Preserved finite point/segment polygon extents and rejected false containment and initial sweep overlap. | [Record](done/2026-10-14-degenerate-convex-relations.md) |
| FMS-Issue-030 | 2026-10-14 | Preserved convex sweep normals and exact witnesses independently of polygon-origin placement. | [Record](done/2026-10-14-convex-sweep-normal-witness.md) |
| FMS-Issue-029 | 2026-10-14 | Made strict scaled-rotation decomposition scale-aware while retaining shear, perspective and representability rejection. | [Record](done/2026-10-14-scaled-rotation-decomposition.md) |
| FMS-Issue-028 | 2026-10-11 | Rematched finite hull/capsule and box/capsule support witnesses to remove tangential disagreement. | [7ca6c70](https://github.com/mrdav30/FixedMathSharp/commit/7ca6c707173eabe50582c6d660a77600e5002f5c) |
| FMS-Issue-027 | 2026-09-28 | Completed triangle/capsule-slab finite cap/rim/edge contacts, retaining positive-core and zero-core geometry. | [3f7a606](https://github.com/mrdav30/FixedMathSharp/commit/3f7a606c000b032d22e42022051ea1a44b0cd9ea) |
| FMS-Issue-026 | 2026-09-24 | Bounded exact cylinder-pair depth rounding with a midpoint-proven binary-search fallback; geometric completeness was separate issue 024. | [Record](done/2026-09-24-cylinder-pair-depth-rounding.md) |
| FMS-Issue-025 | 2026-09-27 | Completed exact box/cylinder contacts, including cap-clipped radial separation. | [fd1cb8f](https://github.com/mrdav30/FixedMathSharp/commit/fd1cb8ff014c64af2863f6c0b5a6d4fa80c9a2ce) |
| FMS-Issue-024 | 2026-09-26 | Completed finite-cylinder pair cap/rim contact classification, depth ranking and witnesses. | [Record](done/2026-09-25-cylinder-pair-contact-design.md) |
| FMS-Issue-023 | 2026-09-25 | Completed cylinder/capsule cap-rim classification, minimum depth and finite witness selection. | [Record](done/2026-09-24-cylinder-capsule-contact-design.md) |
| FMS-Issue-022 | 2026-09-21 | Rematched capsule-side witnesses to opposing polygon features, eliminating tangential anchor offsets. | [Record](done/2026-09-21-capsule-side-contact-witness.md) |
| FMS-Issue-021 | 2026-09-21 | Closed an unreproduced capsule benchmark crash investigation; no runtime fix or native-cause claim. | [Record](done/2026-09-21-capsule-benchmark-crash.md) |
| FMS-Issue-020 | 2026-09-21 | Retained exact capsule/convex projection axes through closed admission before final contact rounding. | [Record](done/2026-09-21-exact-capsule-convex-contacts.md) |
| FMS-Issue-019 | 2026-09-21 | Mitigated .NET 8 background-GC allocation-counter artifacts in the test host while preserving strict zero-byte guards. | [Record](done/2026-09-21-allocation-counter-accounting.md) |
| FMS-Issue-018 | 2026-09-21 | Closed unreproduced containment benchmark crashes and fixed launcher failure reporting; original native/NRE cause remains unattributed. | [Record](done/2026-09-11-convex-point-containment-performance.md#2026-09-21-fms-issue-018-closure) |
| FMS-Issue-017 | 2026-07-21 | Completed exact finite-cone segment intervals, point containment and conservative bounds. | [aa2ad5a](https://github.com/mrdav30/FixedMathSharp/commit/aa2ad5a0c9efa366045ba9592b6d276ef3aafd8a) |
| FMS-Issue-016 | 2026-07-20 | Hardened area/box centers, sizes and extents, atomic failures and explicit domain-clipped factories. | [a5a406c](https://github.com/mrdav30/FixedMathSharp/commit/a5a406cccf4b966fbd2ba4d6f37c344e0808587d) |
| FMS-Issue-015 | 2026-07-20 | Made sphere construction/merge containment-conservative across the full domain, with explicit overflow rejection. | [390b0d2](https://github.com/mrdav30/FixedMathSharp/commit/390b0d217e9e73c1725caba8aca5e414f5274cbc) |
| FMS-Issue-014 | 2026-07-19 | Added exact finite-axis capsule/cylinder intervals with wide clipping before final rounding; committed 2026-07-20. | [8effdb1](https://github.com/mrdav30/FixedMathSharp/commit/8effdb12b4147034ea2290f08afa9d708bd5f450) |
| FMS-Issue-013 | 2026-07-18 | Completed full-domain radial predicates, bounded ray intervals and sphere/slab cross-section queries. | [Record](done/2026-07-18-full-domain-radial-query-plan.md) |
| FMS-Issue-012 | 2026-06-30 | Closed broad in-process benchmark crash investigation; adopted out-of-process smoke checks without claiming a runtime fix or proven native cause. | [9ce101e](https://github.com/mrdav30/FixedMathSharp/commit/9ce101e16b2f01181300341359a7a6516eb3460d) |
| FMS-Issue-011 | 2026-06-08 | Guarded floating-point conversion range/rounding and retired obsolete floating-point APIs. | [e7ca0bc](https://github.com/mrdav30/FixedMathSharp/commit/e7ca0bc767bfa426bd747bb671c15235c10613bd) |
| FMS-Issue-010 | 2026-06-06 | Removed ambiguous long arithmetic overloads and bounded explicit integer conversion. | [5b62303](https://github.com/mrdav30/FixedMathSharp/commit/5b623032a4ff70d8c4a9fd560e0e38d807cebc29) |
| FMS-Issue-009 | 2026-06-05 | Aligned matrix point transforms, quaternion conversion and composition with the row-vector affine convention. | [7877e01](https://github.com/mrdav30/FixedMathSharp/commit/7877e01df5804b25d9deaf7bec2337d9923365ea) |
| FMS-Issue-008 | 2026-06-04 | Isolated restore/intermediate outputs by host and configuration, retaining the benchmark-generated-project exception. | [Record](2026-06-04-benchmark-regression-followups.md) |
| FMS-Issue-007 | 2026-06-03 | Retired FixedThrowHelper in favor of direct guarded throws that defer message construction. | [Record](done/2026-06-03-benchmark-hot-path-followups.md) |
| FMS-Issue-006 | 2026-06-04 | Resolved Lean MemoryPack shim warnings caused by stale conditional assets, using matching restore and output isolation. | [Record](2026-06-04-benchmark-regression-followups.md) |
| FMS-Issue-005 | 2026-06-04 | Mitigated Coverlet restoration failures by making local coverage opt-in; no upstream-engine fix claimed. | [Record](2026-06-04-benchmark-regression-followups.md) |
| FMS-Issue-004 | 2026-06-03 | Reused static Pow10Lookup backing storage behind the existing span API. | [Record](done/2026-06-03-benchmark-hot-path-followups.md) |
| FMS-Issue-003 | 2026-06-03 | Removed eager diagnostic-message allocations from valid scalar/trigonometry paths. | [Record](done/2026-06-03-benchmark-hot-path-followups.md) |
| FMS-Issue-002 | 2026-06-03 | Corrected Vector3d.Direction pitch/yaw documentation and signed-direction regressions. | [9c288c8](https://github.com/mrdav30/FixedMathSharp/commit/9c288c8245be6e8b7d8d80afbb45261762f64460) |
| FMS-Issue-001 | 2026-06-03 | Fixed backward-direction quaternion construction with deterministic antiparallel handling. | [c02069e](https://github.com/mrdav30/FixedMathSharp/commit/c02069eae77ba2eca190f520978f59c9166bb9b6) |

**Reserved historical ID:** `FMS-Issue-032`. [Commit 9145a7a](https://github.com/mrdav30/FixedMathSharp/commit/9145a7a20fcaae500eaf8c568ebfc2351d6bfacc)
advances the counter from 032 to 033, but no committed numbered entry or verified
identity survives. Keep this slot reserved; do not infer a fix or assign it to an
unrelated closed diagnostic.

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
