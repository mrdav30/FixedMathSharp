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

### FMS-Issue-028: Convex hull/capsule anchors can disagree tangentially

- **Status:** Confirmed geometry defect, 2026-10-11 during convex-hull contact
  coverage validation. Source comparison against `941e689` shows the same
  midpoint selection and one-sided rematching before the current refinements;
  the positive-core fallback still uses that rule.
- **Priority:** High
- **Affected area:** `FixedConvexHullRelations.TryGetCenteredCapsuleContact`,
  `WideOrientedBox.TryGetConvexHullCenteredCapsuleContact` and its support-anchor
  selection. Minimum exit depth and individual surface membership are correct;
  coherent paired witnesses are not.
- **Evidence:** Use a complete closed convex cube with local vertices at all
  combinations of `(-1,+1)` on X/Y/Z, its twelve boundary triangles and twelve
  physical edges. Both frames have zero origin and identity rotation. Query a
  capsule centered at `(3,0,0)`, local axis `Vector3d.Right`, core length `8` and
  radius `1/4`. The hull query returns depth `5/4`, normal `(0,0,-1)`, hull
  anchor `(1,0,-1)` and capsule anchor `(3,0,1/4)`. Removing the normal component
  from their difference leaves `(-2,0,0)`. Center `(-3,0,0)` gives the symmetric
  `(2,0,0)` tangential mismatch. The equivalent
  `FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)`
  capsule query returns the same depth with zero tangential anchor difference.
  The finite-support and whole-exit controls are maintained in
  `FixedConvexHullRelations.Tests.cs`; the reproducer requires no saved artifact.
- **Root cause:** For a normal perpendicular to the capsule axis, its support
  anchor starts at the core midpoint. Hull witness selection clips toward that
  point, but the capsule witness is not rematched to the finite hull footprint.
- **Impact:** Contact consumers receive two support points that do not form a
  coherent normal contact pair. Equal-and-opposite normal impulses at tangentially
  separated anchors can create a spurious internal angular impulse. Full host-loop
  severity has not been measured; do not equate the geometry proof with a tested
  simulation failure.
- **Next action:** Add the paired-anchor regression, reuse the existing
  cuboid/capsule finite-core rematching owner where appropriate, and verify
  transformed, endpoint, face/edge, zero-core and full-domain controls. Validate
  two-body torque/angular momentum in the consuming physics layer, preserve
  exact classification/depth and authored ties, and rerun coverage and cost gates.

**Next issue ID:** `FMS-Issue-029`

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
