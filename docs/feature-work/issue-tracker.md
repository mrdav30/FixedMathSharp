# Feature Work Issue Tracker

## Purpose

This document tracks active correctness, determinism, API, documentation, build,
and tooling issues that fall outside an active implementation plan, and indexes
their verified resolutions or documented investigation closures.

Measured performance and allocation concerns belong in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md).
Work that requires staged implementation belongs in a focused feature-work plan.

## Tracker Rules

- Add unresolved concerns with concrete evidence to `Active Issues`. Include the
  affected area, reproducer, source revision, impact, priority and next action.
- Use repository-relative paths and portable commands. Do not record
  developer-specific drive letters, home directories or dependency locations.
- Keep active entries self-contained. Ignored artifacts are supplemental;
  reproducing or continuing an issue must not require their survival.
- Link an active implementation plan instead of duplicating its task details.
- After verification, replace the active entry with a compact `Resolved Issues`
  row: the same ID, resolution/closure date, brief outcome and an evidence link.
  Keep detailed root causes, decisions and validation in an existing completion
  record, regression or fix commit rather than duplicating them in this index.
- Distinguish verified fixes, mitigations and investigations closed without a
  reproduced defect. A closed investigation does not establish a runtime fix or
  an attributed cause; preserve its limits and recurrence guidance in the
  record.
- Resolved means verified in source or closed by documented decision, not
  released. Add release/version claims only when separately confirmed.
- Preserve every assigned ID through closure; never delete or reuse it. Retain
  prior evidence when new findings change an issue's status. Unknown historical
  IDs stay reserved rather than acquiring an inferred cause.

## Active Issues

**Next issue ID:** `FMS-Issue-042`

The entries below capture runtime behavior observed during the API documentation
pass at revision `1a9a270db3a6c79d4f501fc849883ef6ffd6ea12`. Use .NET 10 and the
Release package dependency graph. To run the examples from the repository root,
create a disposable console project, add the core project reference, and replace
its `Program.cs` with the case under investigation. Include
`using FixedMathSharp;` and `using FixedMathSharp.Geometry;` as needed.

```bash
dotnet new console --framework net10.0 --output artifacts/issue-repro
dotnet add artifacts/issue-repro/issue-repro.csproj reference src/FixedMathSharp/FixedMathSharp.csproj
dotnet run --project artifacts/issue-repro/issue-repro.csproj -c Release -p:UseLocalLsfStack=false
```

### FMS-Issue-040: Geometry query ties, shear bounds, and representability

- **Status:** Confirmed behavior. The frustum plane test is a conservative
  classification, and the listed protocols need individual acceptance checks.
- **Priority:** High for conservative bounds and invalid hit points, Medium for
  closest features and extreme plane normals
- **Affected area:**
  [Box](../../src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs),
  [sphere](../../src/FixedMathSharp/Geometry/Bounds/FixedBoundSphere.cs),
  [frustum](../../src/FixedMathSharp/Geometry/Bounds/FixedBoundFrustum.cs),
  [ray](../../src/FixedMathSharp/Geometry/Primitives/Rays/FixedRay.cs), and
  [plane](../../src/FixedMathSharp/Geometry/Primitives/FixedPlane.cs) queries.
- **Evidence:**
  - For `FixedBoundBox.FromMinMax(new(-2, -2, -2), new(2, 2, 2))`,
    `ClosestPointOnSurface(Vector3d.Zero)` returns `(-2, -2, -2)`. Squared
    distance is 12, versus 4 for a face point. For the same box and B with min
    `(3, -1, -1)` and max `(4, 1, 1)`, `FindClosestPointsBetweenBoxes(A, B)`
    searches B's corners and returns the single point `(2, -1, -1)` on A.
  - Start with a unit sphere at zero and set `M12 = Fixed64.One` on
    `Fixed4x4.Identity`. `sphere.Transform(matrix)` returns radius about 1.4142.
    The source point `(0.75, 0.625, 0)` is inside, but `TransformPoint` maps it
    to `(0.75, 1.375, 0)`, outside the result.
  - Construct a `FixedBoundFrustum` from six planes with normals
    `-Z, +Z, -X, +X, +Y, -Y` in constructor order and `D = -1` for each. This is
    the cube `[-1, 1]³`. A sphere at `(2, 2, 0)` with radius 1.25 passes the
    plane test. Its squared distance to the cube is 2, versus squared radius
    1.5625.
  - A ray from zero with direction `(Fixed64.FromRaw(1), 0, 0)` reports
    `Fixed64.MaxValue` against a box from `(1, -1, -1)` to `(2, 1, 1)`.
    `GetPoint(MaxValue)` is approximately `(0.5, 0, 0)`, outside the box.
  - Call `FixedPlane.GetNormalized` with normal `(Fixed64.FromRaw(1), 0, 0)` and
    `D = 0`. The output normal is `(0.5, 0, 0)` because the reciprocal
    saturates.
- **Impact:** These results matter when callers require minimum distance, a
  conservative transformed bound, a usable hit witness, or unit plane normals.
- **Next action:** Add exact face, shear, slab, and tiny-normal regression
  oracles in the respective owners. Check broad-phase callers before changing
  the conservative frustum test.

### FMS-Issue-041: Fixed-point rounding and vector or quaternion edge behavior

- **Status:** Confirmed behavior. Compatibility decision pending.
- **Priority:** High for rounding wraparound, Medium for the other numeric cases
- **Affected area:** [FixedMath](../../src/FixedMathSharp/Core/FixedMath.cs),
  [Fixed64 conversions](../../src/FixedMathSharp/Numerics/Scalars/Fixed64.Conversions.cs),
  vector operations, and
  [FixedQuaternion rotation](../../src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.cs).
- **Evidence:**
  - `RoundToPrecision(Fixed64.FromDecimal(0.2m), 1)` returns raw `858993460`,
    one unit above the input. With `Fixed64.MaxValue` instead, the returned raw
    value is `-922337203685477580`. `ToInt(-Fixed64.Half)` returns `-1`, while
    `(int)(-Fixed64.Half)` returns `0`.
  - `Vector3d.Angle(new(1000000, 0, 0), new(1, 1000000, 0))` returns zero under
    saturation. `Vector3d.Reflect(Vector3d.Right, new(2, 0, 0))` returns
    `(-7, 0, 0)`, demonstrating the unit-normal requirement. The scalar cross
    products of `Vector2d.Forward, Vector2d.Right` and
    `Vector3d.Up, Vector3d.Right` both return `-1`.
  - `FixedQuaternion.Zero.Rotate(Vector3d.Forward)` returns the input.
    `FixedQuaternion.Zero.TryRotate(Vector3d.Forward, out var result)` succeeds
    with a zero result. For a positive quarter turn about Up, `ToDirection().X`
    is negative and `Rotate(Vector3d.Forward).X` is positive.
  - `Vector4d.Lerp(new Vector4d(Fixed64.MinValue), new Vector4d(Fixed64.MaxValue), Fixed64.One)`
    misses the end value because subtraction saturates. For all components equal
    to `-1`, `AllComponentsGreaterThanEpsilon` is true in 2D and 3D and false in
    4D.
- **Impact:** Rounding can change sign at the representable endpoint. The other
  cases affect conversions and agreement between related math APIs.
- **Next action:** Start with an exact raw-integer rounding oracle for
  `RoundToPrecision`. Review caller expectations before aligning overloads or
  dimensional variants. Confirm which of the observed differences are intended.

## Resolved Issues

Historical rows come from committed tracker entries and linked completion
records. Dates reflect recorded resolution/closure, which can precede the commit
date. Evidence describes verification at closure; it does not claim a fresh
rerun or package release.

| Issue                                                                                                      | Resolved/closed | Outcome                                                                                                                                                                                                   | Evidence                                                                                             |
| ---------------------------------------------------------------------------------------------------------- | --------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| FMS-Issue-039                                                                                              | 2026-11-15      | Verified caller requirements and preserved normalization and comparison contracts with characterization tests and guidance.                                                                               | [Record](done/2026-11-15-matrix-normalization-and-comparison.md)                                     |
| FMS-Issue-038                                                                                              | 2026-11-15      | Corrected rotated camera view mapping, perspective coefficients, and rounded half-angle validation.                                                                                                       | [Record](done/2026-11-15-camera-view-and-perspective.md)                                             |
| FMS-Issue-037                                                                                              | 2026-10-31      | Aligned Chronicler and shim source/package identities to 1.0.0; reproduced the companion load failure and passed source/package Release and Lean validation.                                              | [Record](done/2026-10-31-chronicler-v1-dependency-alignment.md)                                      |
| <a id="fms-issue-036-spherecapsule-prism-witnesses-can-miss-the-opposing-finite-support"></a>FMS-Issue-036 | 2026-10-25      | Matched finite sphere/capsule prism supports, completed missing rim/vertex contacts and retained exact radial scaling; downstream torque/replay and coordinated coverage pass.                            | [Record](done/2026-10-25-prism-capsule-finite-witnesses.md)                                          |
| FMS-Issue-035                                                                                              | 2026-10-20      | Corrected contained sphere/capsule prism exit orientation and paired support signs; whole-shape clearance and downstream host response/replay pass, with separate finite witness matching tracked as 036. | [Record](done/2026-10-20-contained-prism-exit-orientation.md)                                        |
| FMS-Issue-034                                                                                              | 2026-10-20      | Completed exact cone/cylinder-prism minimum depth, normal and finite witnesses; bounded throughput refinements are committed, with residual penetrating oblique cost tracked in the benchmark backlog.    | [Record](done/2026-10-19-curved-prism-contact-depth.md)                                              |
| FMS-Issue-033                                                                                              | 2026-10-19      | Completed closed cylinder/cone-prism admission; the separate contact depth/normal defect was resolved under FMS-Issue-034.                                                                                | [Record](done/2026-10-19-curved-prism-admission.md)                                                  |
| FMS-Issue-031                                                                                              | 2026-10-14      | Preserved finite point/segment polygon extents and rejected false containment and initial sweep overlap.                                                                                                  | [Record](done/2026-10-14-degenerate-convex-relations.md)                                             |
| FMS-Issue-030                                                                                              | 2026-10-14      | Preserved convex sweep normals and exact witnesses independently of polygon-origin placement.                                                                                                             | [Record](done/2026-10-14-convex-sweep-normal-witness.md)                                             |
| FMS-Issue-029                                                                                              | 2026-10-14      | Made strict scaled-rotation decomposition scale-aware while retaining shear, perspective and representability rejection.                                                                                  | [Record](done/2026-10-14-scaled-rotation-decomposition.md)                                           |
| FMS-Issue-028                                                                                              | 2026-10-11      | Rematched finite hull/capsule and box/capsule support witnesses to remove tangential disagreement.                                                                                                        | [7ca6c70](https://github.com/mrdav30/FixedMathSharp/commit/7ca6c707173eabe50582c6d660a77600e5002f5c) |
| FMS-Issue-027                                                                                              | 2026-09-28      | Completed triangle/capsule-slab finite cap/rim/edge contacts, retaining positive-core and zero-core geometry.                                                                                             | [3f7a606](https://github.com/mrdav30/FixedMathSharp/commit/3f7a606c000b032d22e42022051ea1a44b0cd9ea) |
| FMS-Issue-026                                                                                              | 2026-09-24      | Bounded exact cylinder-pair depth rounding with a midpoint-proven binary-search fallback; geometric completeness was separate issue 024.                                                                  | [Record](done/2026-09-24-cylinder-pair-depth-rounding.md)                                            |
| FMS-Issue-025                                                                                              | 2026-09-27      | Completed exact box/cylinder contacts, including cap-clipped radial separation.                                                                                                                           | [fd1cb8f](https://github.com/mrdav30/FixedMathSharp/commit/fd1cb8ff014c64af2863f6c0b5a6d4fa80c9a2ce) |
| FMS-Issue-024                                                                                              | 2026-09-26      | Completed finite-cylinder pair cap/rim contact classification, depth ranking and witnesses.                                                                                                               | [Record](done/2026-09-25-cylinder-pair-contact-design.md)                                            |
| FMS-Issue-023                                                                                              | 2026-09-25      | Completed cylinder/capsule cap-rim classification, minimum depth and finite witness selection.                                                                                                            | [Record](done/2026-09-24-cylinder-capsule-contact-design.md)                                         |
| FMS-Issue-022                                                                                              | 2026-09-21      | Rematched capsule-side witnesses to opposing polygon features, eliminating tangential anchor offsets.                                                                                                     | [Record](done/2026-09-21-capsule-side-contact-witness.md)                                            |
| FMS-Issue-021                                                                                              | 2026-09-21      | Closed an unreproduced capsule benchmark crash investigation; no runtime fix or native-cause claim.                                                                                                       | [Record](done/2026-09-21-capsule-benchmark-crash.md)                                                 |
| FMS-Issue-020                                                                                              | 2026-09-21      | Retained exact capsule/convex projection axes through closed admission before final contact rounding.                                                                                                     | [Record](done/2026-09-21-exact-capsule-convex-contacts.md)                                           |
| FMS-Issue-019                                                                                              | 2026-09-21      | Mitigated .NET 8 background-GC allocation-counter artifacts in the test host while preserving strict zero-byte guards.                                                                                    | [Record](done/2026-09-21-allocation-counter-accounting.md)                                           |
| FMS-Issue-018                                                                                              | 2026-09-21      | Closed unreproduced containment benchmark crashes and fixed launcher failure reporting; original native/NRE cause remains unattributed.                                                                   | [Record](done/2026-09-11-convex-point-containment-performance.md#2026-09-21-fms-issue-018-closure)   |
| FMS-Issue-017                                                                                              | 2026-07-21      | Completed exact finite-cone segment intervals, point containment and conservative bounds.                                                                                                                 | [aa2ad5a](https://github.com/mrdav30/FixedMathSharp/commit/aa2ad5a0c9efa366045ba9592b6d276ef3aafd8a) |
| FMS-Issue-016                                                                                              | 2026-07-20      | Hardened area/box centers, sizes and extents, atomic failures and explicit domain-clipped factories.                                                                                                      | [a5a406c](https://github.com/mrdav30/FixedMathSharp/commit/a5a406cccf4b966fbd2ba4d6f37c344e0808587d) |
| FMS-Issue-015                                                                                              | 2026-07-20      | Made sphere construction/merge containment-conservative across the full domain, with explicit overflow rejection.                                                                                         | [390b0d2](https://github.com/mrdav30/FixedMathSharp/commit/390b0d217e9e73c1725caba8aca5e414f5274cbc) |
| FMS-Issue-014                                                                                              | 2026-07-19      | Added exact finite-axis capsule/cylinder intervals with wide clipping before final rounding; committed 2026-07-20.                                                                                        | [8effdb1](https://github.com/mrdav30/FixedMathSharp/commit/8effdb12b4147034ea2290f08afa9d708bd5f450) |
| FMS-Issue-013                                                                                              | 2026-07-18      | Completed full-domain radial predicates, bounded ray intervals and sphere/slab cross-section queries.                                                                                                     | [Record](done/2026-07-18-full-domain-radial-query-plan.md)                                           |
| FMS-Issue-012                                                                                              | 2026-06-30      | Closed broad in-process benchmark crash investigation; adopted out-of-process smoke checks without claiming a runtime fix or proven native cause.                                                         | [9ce101e](https://github.com/mrdav30/FixedMathSharp/commit/9ce101e16b2f01181300341359a7a6516eb3460d) |
| FMS-Issue-011                                                                                              | 2026-06-08      | Guarded floating-point conversion range/rounding and retired obsolete floating-point APIs.                                                                                                                | [e7ca0bc](https://github.com/mrdav30/FixedMathSharp/commit/e7ca0bc767bfa426bd747bb671c15235c10613bd) |
| FMS-Issue-010                                                                                              | 2026-06-06      | Removed ambiguous long arithmetic overloads and bounded explicit integer conversion.                                                                                                                      | [5b62303](https://github.com/mrdav30/FixedMathSharp/commit/5b623032a4ff70d8c4a9fd560e0e38d807cebc29) |
| FMS-Issue-009                                                                                              | 2026-06-05      | Aligned matrix point transforms, quaternion conversion and composition with the row-vector affine convention.                                                                                             | [7877e01](https://github.com/mrdav30/FixedMathSharp/commit/7877e01df5804b25d9deaf7bec2337d9923365ea) |
| FMS-Issue-008                                                                                              | 2026-06-04      | Isolated restore/intermediate outputs by host and configuration, retaining the benchmark-generated-project exception.                                                                                     | [Record](2026-06-04-benchmark-regression-followups.md)                                               |
| FMS-Issue-007                                                                                              | 2026-06-03      | Retired FixedThrowHelper in favor of direct guarded throws that defer message construction.                                                                                                               | [Record](done/2026-06-03-benchmark-hot-path-followups.md)                                            |
| FMS-Issue-006                                                                                              | 2026-06-04      | Resolved Lean MemoryPack shim warnings caused by stale conditional assets, using matching restore and output isolation.                                                                                   | [Record](2026-06-04-benchmark-regression-followups.md)                                               |
| FMS-Issue-005                                                                                              | 2026-06-04      | Mitigated Coverlet restoration failures by making local coverage opt-in; no upstream-engine fix claimed.                                                                                                  | [Record](2026-06-04-benchmark-regression-followups.md)                                               |
| FMS-Issue-004                                                                                              | 2026-06-03      | Reused static Pow10Lookup backing storage behind the existing span API.                                                                                                                                   | [Record](done/2026-06-03-benchmark-hot-path-followups.md)                                            |
| FMS-Issue-003                                                                                              | 2026-06-03      | Removed eager diagnostic-message allocations from valid scalar/trigonometry paths.                                                                                                                        | [Record](done/2026-06-03-benchmark-hot-path-followups.md)                                            |
| FMS-Issue-002                                                                                              | 2026-06-03      | Corrected Vector3d.Direction pitch/yaw documentation and signed-direction regressions.                                                                                                                    | [9c288c8](https://github.com/mrdav30/FixedMathSharp/commit/9c288c8245be6e8b7d8d80afbb45261762f64460) |
| FMS-Issue-001                                                                                              | 2026-06-03      | Fixed backward-direction quaternion construction with deterministic antiparallel handling.                                                                                                                | [c02069e](https://github.com/mrdav30/FixedMathSharp/commit/c02069eae77ba2eca190f520978f59c9166bb9b6) |

**Reserved historical ID:** `FMS-Issue-032`.
[Commit 9145a7a](https://github.com/mrdav30/FixedMathSharp/commit/9145a7a20fcaae500eaf8c568ebfc2351d6bfacc)
advances the counter from 032 to 033, but no committed numbered entry or
verified identity survives. Keep this slot reserved; do not infer a fix or
assign it to an unrelated closed diagnostic.

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
