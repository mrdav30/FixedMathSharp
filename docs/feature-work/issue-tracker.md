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

### FMS-Issue-019: Allocation guards reported unexplained bursts

- **Status:** Needs reproduction after test-protocol alignment. No production
  allocator or source for the observed bytes is established.
- **Priority:** Low
- **Affected area:** `Fixed64ProductComparisonTests.CompareProducts_WarmedExecution_DoesNotAllocate`.
- **Evidence:** The first covered ReleaseLean run on 2026-09-13 UTC reports
  2,208 bytes (8,298,416 before / 8,300,624 after); 2,694 other core tests pass.
  `CompareProducts` source is unchanged by the containment optimization.
  The coordinating Trailblazer checkout retains the failed log/TRX/coverage in
  `artifacts/benchmark005/vertex-verification` and source plus post-run binary
  snapshots in `vertex-allocation-failure` beside it.
- **Confirmed protocol correction:** The original guard warmed a discard-result
  loop and measured a separate accumulating loop, using direct counters despite
  the existing repository guidance. Both phases now use the same operation via
  `FixedMathTestHelper.MeasureWarmedAllocations`: 64 calls to each overload,
  unchanged inputs, zero checksum and exactly zero bytes. No retry, tolerance,
  extra warmup or runtime change was added. The aligned seven-test class passes;
  temporarily allocating a 64-byte array per iteration fails at 5,632 bytes.
  That mutation is removed. This proves guard sensitivity, not the burst's cause.
- **Next action:** Preserve any recurrence with its configuration and coverage
  context before attributing it to production code. Successful aligned runs do
  not establish that flakiness is cured; no broader test-harness work is planned.

- **Separate 2026-09-21 signal:** An uninstrumented Release solution run during
  planar-clearance work failed the unchanged
  `FixedPointAnchorTests.TryGetLocalPointIn_DoesNotAllocate` guard at 7,240 bytes
  (2,775,296 before / 2,782,536 after); 2,775 other core tests passed. Five
  isolated runs and the complete covered run subsequently passed. This is not
  evidence of the same allocator as the earlier product-comparison signal.
  The guard warmed one call but measured 256, discarding every success result.
  It now uses the existing helper for the identical 256-call operation and
  asserts successful execution as well as zero bytes. This corrects a visible
  test-protocol weakness, not an attributed allocation defect. A temporary
  64-byte array per iteration made the aligned guard fail at 22,528 bytes;
  the mutation was removed. Full standard/Lean suites and coverage subsequently
  passed, as did Linux execution; none attributes the original burst or proves
  it cannot recur. Failure and
  follow-up logs remain in the coordinating 2D plan's ignored evidence workspace.

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
