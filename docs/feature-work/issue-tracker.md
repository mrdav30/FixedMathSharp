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

### FMS-Issue-029: Strict decomposition rejects representable scaled rotations

- **Status:** Confirmed limitation; tolerance contract needs review.
- **Priority:** Medium.
- **Affected area:** `Fixed4x4.Decompose` final reconstructed-transform check.
- **Evidence:** On committed core `819dbde` (2026-10-12, Release, Windows x64,
  .NET 8.0.29), create `FixedTransform(Vector3d.Zero, q, Vector3d.One * s)`.
  For `q = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)19, (Fixed64)31,
  (Fixed64)11)`, `TryGetLocalToWorldMatrix` succeeds at `s=1` and `2,048`, but
  `Fixed4x4.Decompose` succeeds only at `1`. The normalized basis is orthogonal
  within one raw increment and its quaternion reconstruction passes. The final
  scaled reconstruction differs by `3,724` raw increments, exceeding the fixed
  absolute `Fixed64.Epsilon` of `256`. A planar rotation from
  `FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.PiOver4)` reproduces the
  same boundary: scale-one difference `3`, scale-2,048 difference `2,097`.
- **Impact:** Rotation-to-quaternion rounding is amplified by scale, rejecting
  useful representable TRS matrices even without meaningful shear. Consumers
  using strict decomposition for collider hierarchy validation can reject small
  compensated physical geometry. This follows the current strict absolute
  round-trip contract; it is not an observed saturation or contact miss.
- **Next action:** Review normalized-basis and scaled reconstruction error
  bounds together. Preserve genuine shear, perspective and representability
  rejection; do not blindly relax all tolerances or bypass decomposition in
  consumers. Cover uniform/nonuniform scale, reflection, hierarchy composition
  and planar parity before revising the public contract.

**Next issue ID:** `FMS-Issue-030`

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
