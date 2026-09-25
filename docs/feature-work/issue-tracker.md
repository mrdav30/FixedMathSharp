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
