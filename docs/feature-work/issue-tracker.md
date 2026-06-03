# Feature Work Issue Tracker

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before implementing fixes, use
> superpowers:test-driven-development for runtime behavior changes, and use
> superpowers:verification-before-completion before claiming an issue is fixed.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** Active

**Goal:** Keep bugs, correctness risks, documentation defects, and
feature-work-discovered issues separate from feature design plans so each fix
can be triaged, tested, and committed independently.

**Architecture:** This document is intentionally undated. Each tracked item has
its own discovery date, source, status, affected files, and recommended
verification. Feature plans may reference this tracker instead of carrying bug
fixes inside API or design phases.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, xUnit,
BenchmarkDotNet when performance evidence is needed, FixedMathSharp core
runtime and tests.

---

## Tracker Rules

- Add new items when feature work uncovers a suspected bug, stale doc, test
  smell, performance anomaly, or correctness risk.
- Keep each item scoped tightly enough to fix and verify independently.
- Record the date on the item, not in this filename.
- Move an item to `Resolved Issues` only after the fix has tests or documented
  verification evidence.
- Do not use this tracker as a substitute for tests, benchmarks, or release
  notes.

## Active Issues

### FMS-Issue-002: `Vector3d.Direction` docs appear to swap pitch and yaw semantics

**Discovered:** 2026-06-03

**Source:** Coordinate-convention review of vector direction helpers.

**Status:** Proposed

**Affected files:**

- Review: `src/FixedMathSharp/Numerics/Vectors/Vector3d.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`

**Problem:**

`Vector3d.Direction` documentation says `X` is yaw and `Y` is pitch, while the
formula appears to make `X` control signed vertical pitch and `Y` control
horizontal yaw around `+Y`.

**Recommended approach:**

- [ ] Add tests for zero angles, positive yaw, negative yaw, positive pitch, and
  negative pitch using canonical `+Z` forward expectations.
- [ ] Decide whether the right fix is XML documentation only or additive helper
  APIs such as `DirectionFromPitchYaw` / `FromPitchYaw`.
- [ ] Preserve the existing public API unless a breaking-change phase is
  explicitly approved.
- [ ] If helper APIs are added, benchmark only if they are expected to be used
  in hot paths.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector3dTests"
dotnet test FixedMathSharp.slnx --configuration Debug
```

## Performance Investigation Queue

Performance issues should stay in the benchmark plan unless they become a
confirmed runtime defect. Current queue:

- 2026-06-03: Investigate allocations in scalar/trigonometry benchmark paths.
  See `docs/feature-work/2026-06-03-benchmark-hot-path-followups.md`.
- 2026-06-03: Investigate vector normalization and quaternion creation
  allocations after scalar/trigonometry allocation sources are known.

## Resolved Issues

### FMS-Issue-001: `FixedQuaternion.FromDirection(Vector3d.Backward)` returned identity

**Discovered:** 2026-06-03

**Resolved:** 2026-06-03

**Source:** Coordinate-convention review of `Vector3d.Forward`/`Backward` and
quaternion direction helpers.

**Resolution:**

`FixedQuaternion.FromDirection` now detects the anti-parallel zero-cross-axis
case and returns a deterministic 180-degree rotation around `Vector3d.Up`
instead of `FixedQuaternion.Identity`.

**Completed work:**

- [x] Added a failing test for `FixedQuaternion.FromDirection(Vector3d.Backward)`.
- [x] Added near-forward and near-backward tests so edge behavior is documented.
- [x] Preserved `FixedQuaternion.FromDirection(Vector3d.Forward) ==
  FixedQuaternion.Identity`.
- [x] Used a deterministic `Vector3d.Up` fallback axis for the anti-parallel
  case.
- [x] Re-ran focused quaternion tests.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedQuaternionTests"
```

Result on 2026-06-03: passed, 72 tests.
