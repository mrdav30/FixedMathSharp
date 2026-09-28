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

### FMS-Issue-027: Triangle/capsule-slab contact admits a separated oblique rim

- **Status:** Confirmed, 2026-09-27 during GRV-Issue-082 verification, based on
  `fd1cb8f` plus the triangle/cylinder repair. The positive-core capsule-slab
  owner is unchanged by that repair.
- **Priority:** High
- **Affected area:** `FixedTriangle.TryGetCenteredCapsuleSlabContact` and
  `WideOrientedBox.TriangleCapsuleSlab.cs`. This is a planar capsule extruded
  through a flat Y slab, not a rounded 3D capsule.
- **Evidence:** Use triangle `(99/16,9/2,5/4)`, `(67/16,6,-5/4)`,
  `(131/16,37/4,0)`, with identity triangle frame and zero origin. Query a slab
  at zero, zero yaw, local axis `Vector2d.Right`, core length
  `Fixed64.MinIncrement`, radius `5`, and half-thickness `5`. The public query
  returns true. Along unit direction `(3/5,4/5,0)`, the cylinder-limit separating
  gap is `5/16`; adding this one-raw core decreases it by only
  `3/(10*2^32)`. The remaining gap is strictly positive.
  A temporary enabled diagnostic reproduced expected-false/actual-true in
  `artifacts/grv082-checkpoints/capsule-slab-parity-red.trx` before being removed
  from the completed cylinder change. The reproduction does not depend on
  that disposable artifact:

  ```csharp
  var triangle = new FixedTriangle(
      new Vector3d(Fixed64.FromFraction(99, 16), Fixed64.FromFraction(9, 2), Fixed64.FromFraction(5, 4)),
      new Vector3d(Fixed64.FromFraction(67, 16), (Fixed64)6, Fixed64.FromFraction(-5, 4)),
      new Vector3d(Fixed64.FromFraction(131, 16), Fixed64.FromFraction(37, 4), Fixed64.Zero));
  bool hit = triangle.TryGetCenteredCapsuleSlabContact(
      Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Fixed64.Zero,
      Vector2d.Right, Fixed64.MinIncrement, (Fixed64)5, (Fixed64)5, out _);
  // Expected false; currently true for the positive-core slab.
  ```

- **Impact:** Phantom contacts, and potentially incorrect minimum depth or
  witnesses, for mixed mesh/capsule-slab consumers. Zero-core slabs are handled
  by the complete triangle/cylinder owner; increasing that core above zero
  still reaches the incomplete candidate-axis path.
- **Next action:** Add the enabled positive-core regression, derive the full
  stadium-prism support-feature set, and reuse the existing cylinder contact
  arithmetic where valid. Preserve exact rim separation, yaw/local-axis
  ownership, full-width dimensions, witness pairing, and zero-core parity.
  Do not substitute a 3D capsule or treat a finite set of plausible axes as a
  completeness proof. Measure ordinary and oblique paths and restore full
  reachable coverage in both package configurations.

**Next issue ID:** `FMS-Issue-028`

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
