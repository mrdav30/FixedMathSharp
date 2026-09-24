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
  exact posture clearance for Gravitas `GRV-Issue-081`.
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
- **Impact:** False collision and incorrect solver depth; exact posture
  clearance cannot reuse this contact classifier merely by changing `<=` to
  `<`. This defect predates the new strict predicates.
- **Next action:** Establish complete finite-rim classification, including
  arbitrary rigid frames, before replacing the old contact authority. Evaluate
  reuse of the existing rounded-cylinder polynomial machinery with exact
  centered-axis inputs; do not round endpoints, add an epsilon, or label a
  finite sampled-axis check exact. Keep independent analytic boundary tests
  and benchmark the contact path before and after any owning repair.

**Next issue ID:** `FMS-Issue-024`

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
