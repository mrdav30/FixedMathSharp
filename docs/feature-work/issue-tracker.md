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

### FMS-Issue-020: Rounded capsule contact axes reject exact corner tangency

- **Status:** Confirmed; generalized contact-depth path is not fixed by the
  separate exact upright-capsule relation.
- **Priority:** High
- **Affected area:** `FixedSegment2d.TryGetCenteredCapsuleConvexMinimumTranslation`
  and its `WideCenteredCapsule2dRelations.TryKeepAxis` classifier.
- **Evidence (2026-09-21):** Polygon vertices `(-1,-1),(0,-1),(0,0),(-1,0)`,
  capsule center `(3,4)`, Forward axis, axis length zero, radius `5`. Exact
  corner contact follows `3² + 4² = 5²`, but the existing method returns false.
  Scaling the center and radius by 10, 100 and 1000 also returns false. The
  public-API diagnostic ran against the previously built, unchanged source-stack
  binaries; this predates planar-clearance implementation.
- **Cause:** Classification projects onto a normalized, rounded `Vector2d`
  direction but treats its radial extent as exactly unit length. At the 3–4–5
  corner, component rounding makes the center projection exceed the assumed
  radius extent. Wide products cannot recover information already rounded away.
- **Impact / next action:** Exact closed contact can be lost in consumers of
  this generalized capsule/polygon contact path. Reproduce permanently at the
  owning API, then retain exact axes through classification before rounding
  contact output; cover arbitrary axes/rotations and Gravitas consumers.
  Native planar navigation uses the independently exact
  `FixedConvex2dRelations.IntersectsUprightCapsule` predicate instead. That is
  a geometry classification API, not a fix or wrapper for contact-depth output.

**Next issue ID:** `FMS-Issue-021`

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
