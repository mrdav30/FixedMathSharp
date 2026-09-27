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
