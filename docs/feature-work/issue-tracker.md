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

### FMS-Issue-033: Finite curved-prism axes can report overlap across a separating plane

- **Status:** Confirmed, 2026-10-17; pre-existing on source revision `039f4c4`.
- **Priority:** High.
- **Affected area:** `FixedConvexPrismRelations.TryGetCenteredConeContact(...)`
  and `TryGetCenteredCylinderContact(...)`, owned by
  `WideConvexPrismRelations.TryGetContact(...)`.
- **Evidence:** The finite candidate-axis set omits a curved separating
  direction. Both public methods return `true` for the following bounded inputs,
  despite a strict whole-primitive separating plane. Reproduced against the
  authenticated committed binary and the current source graph; this is not
  introduced by static-surface contact reduction.

  Cone: radius `0.5`, height `1`, identity rotation, center raw components
  `(-565576967, -557169159, -565576967)`. The target prism has origin `(2,1,2)`,
  zero yaw, half-thickness `1` and offsets `(-2,-2), (2,-2), (2,2), (-2,2)`.
  The axis with raw components `(2716375826,1920767767,2716375826)` has target
  minimum projection `0`; the cone maximum is `-4194303` raw units. Independently
  obtain the cone maximum as
  `dot(center,n) + max(h/2*n.Y, r*sqrt(n.X*n.X+n.Z*n.Z)-h/2*n.Y)`.

  Cylinder: radius `1`, height `2`, rotation
  `FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22,(Fixed64)17,(Fixed64)9)`.
  Set `n = new Vector3d(Fixed64.Zero, Fixed64.FromRaw(-3980925575),
  Fixed64.FromRaw(-1612133878))` and center to
  `(0.25,0.75,0.25) - n * (Fixed64.FromRaw(1918558518) + Fixed64.FromFraction(1,1024))`.
  The target has origin `(0,-4,0)`, zero yaw, half-thickness `4` and offsets
  `(-1,-0.5), (1,-0.5), (1,0.5), (-1,0.5)`.
  With `local = rotation.Conjugate().Rotate(n)`, the cylinder maximum is
  `dot(center,n) + r*sqrt(local.X*local.X+local.Z*local.Z) + h/2*abs(local.Y)`.
  The target minimum is `n.Z/2`. The independently computed clearance is
  `4194303` raw units, approximately `1/1024` unit, while the method reports
  positive penetration. This margin is far larger than the bounded fixtures'
  final fixed-point projection rounding.
- **Impact:** Mixed cone/cylinder versus AABB or polygon slabs can publish
  geometric false positives and incorrect penetration. The ordinary predicate
  must not be the sole independent oracle for exact curved-contact clearance.
- **Next action:** Complete separating-feature admission in the existing native
  curved-prism owner, reusing exact finite-feature/support geometry. Preserve
  the ordinary warmed baseline and cover separated/touching/penetrating cases,
  frame reversal, winding, dimensions and full-domain positions. Do not replace
  this with more sampled axes or loosen the exact contact certificates.

**Next issue ID:** `FMS-Issue-034`

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
