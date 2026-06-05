# Vector Normalization Follow-Ups

**Status:** In Progress

**Source:** Extracted from Phase 6 of
`docs/feature-work/done/2026-06-04-benchmark-hotspot-optimization-plan.md`.

**Goal:** Keep vector normalization deterministic while tracking future
optimization ideas that need deeper proof than the 2026-06-05 pass justified.

## Phase 1: Exact Reciprocal Normalization Research

**Status:** Completed on 2026-06-05

**Why:** Replacing repeated component divisions with a shared reciprocal looked
like the obvious speed shape:

```csharp
Fixed64 invMag = Fixed64.One / mag;
value.X * invMag
```

However, the first implementation changed rounding behavior:

- `Vector2d(3,4).Normal` drifted by a few raw units.
- `Vector3d.SpeedLerp_DoesNotOvershoot` overshot to `5.000000004656613`.

**Outcome:** `Fixed64.One / mag` plus `FixedMath.FastMul(component, invMag)` was
tested and rejected. It still changed raw results because the shared reciprocal
rounds once and then each component multiply rounds from that already-rounded
value. The accepted approach is an internal `Fixed64.DivideByPositive` helper
for normalization paths where the divisor is already known to be a positive
magnitude. This keeps the same quotient and round-half-to-even behavior as
`Fixed64.operator /` while skipping the generic negative-divisor path.

Focused exactness tests now cover representative 2D, 3D, and 4D normalization
inputs by asserting the public normalization result matches explicit
component-divided-by-magnitude output. The existing no-overshoot movement test
also remains green.

Fresh short-run benchmark samples after adding the helper:

- `Vector3d.Normal`: 63.782 us, 0 allocated.
- `Vector3d.NormalizeInPlace`: 63.362 us, 0 allocated.
- `Vector3d.GetNormalized`: 67.960 us, 0 allocated.
- `Vector4d.Normal`: 74.526 us, 0 allocated.
- `Vector4d.NormalizeInPlace`: 79.324 us, 0 allocated.
- `Vector4d.GetNormalized`: 76.340 us, 0 allocated.

- [x] Only revisit if an exact deterministic reciprocal/multiply path can match
  current per-component division raw results or if the public contract
  intentionally changes.
- [x] Include exact 3-4-5 vectors, fractional vectors, huge vectors, tiny raw
  vectors, and no-overshoot movement tests before benchmarking.
- [x] Keep any accepted approach allocation-free and verify `netstandard2.1`.

## Phase 1-b: Fast Helper Candidate Audit

**Status:** Completed on 2026-06-05

**Why:** `Fixed64.DivideByPositive` proved useful for vector normalization, but
that does not mean every positive-looking divide or every `Fixed64` operator
should move to a fast helper. Audit candidate hot paths one by one, keep only
measured wins, and reject changes that alter rounding, saturating behavior, or
public operator semantics.

**Current findings:**

- `FixedMath.FastMul` is not a safe replacement for exact normalization through
  a shared reciprocal. It changes raw results because the reciprocal is rounded
  once and the multiply rounds from that already-rounded value.
- `FixedMath.FastAdd` and `FixedMath.FastSub` should not replace normal public
  scalar/vector/matrix arithmetic unless local invariants prove overflow is
  impossible and saturating semantics are intentionally not needed.
- Quaternion normalization and direction/axis normalization were tested with
  `DivideByPositive`; correctness held, but the short benchmark result was not
  a clear win versus the prior Phase 3 numbers. Runtime changes were rejected
  for now; the exact quaternion normalization guardrail tests remain useful.
- Ray/sphere intersection has a strictly positive `Direction.SqrMagnitude`
  divisor after the zero-direction guard. Switching that final hit-parameter
  divide to `DivideByPositive` preserved exact behavior and measured as a small
  short-run win: `RayIntersectsSphere` moved from 22.263 us to 22.010 us.
- Sphere merge/expansion center-step divisors are strictly positive after the
  containment and outside-point guards. Switching those divides to
  `DivideByPositive` preserved exact behavior and gave the clearest win in this
  pass: `SphereCreateFromBoundingBox` moved from 92.206 us to 87.484 us.
  `SphereTransform` was mixed/noisy at 105.448 us to 106.443 us, so the
  accepted evidence is the direct merge-heavy benchmark plus exact tests.
- Plane normalization should keep the existing single reciprocal plus multiply
  shape. Replacing it with per-component positive divides would add division
  work and could change raw multiplication/division rounding shape.
- `FixedMath.FastMod` did not surface a clear hot-path candidate in this pass.
- Vector projection, vector angle, closest-point helpers, and matrix
  extraction/decomposition have positive-divisor candidates, but they lack
  focused benchmark coverage today. Defer runtime substitutions until those
  methods are benchmarked directly.

- [x] Add exact quaternion normalization guardrails matching component division
  by magnitude.
- [x] Test `DivideByPositive` in quaternion normalization and direction/axis
  normalization paths.
- [x] Reject quaternion runtime helper substitutions unless longer benchmarks
  show a clear win.
- [x] Audit plane, ray, matrix extraction/decomposition, and bounds expansion
  positive-divisor candidates.
- [x] Record any accepted helper substitutions with before/after benchmark
  snippets and targeted correctness tests.

## Phase 2: Longer-Run Vector4d Normalization Confirmation

**Why:** The fresh short run still shows `Vector4d` normalization paths as the
largest vector normalization costs:
`GetNormalized` 76.879 us, `Normal` 78.904 us, and `NormalizeInPlace` 82.420 us.
The costs are expected because the path is square-root plus four fixed
divisions, but the spread between `GetNormalized` and `NormalizeInPlace` should
be treated as noise unless reproduced in a longer run.

- [ ] Re-run `Vector4d` normalization in a longer benchmark job before opening
  runtime work.
- [ ] Do not trade away raw-value precision for the sake of reducing fixed
  divisions.
- [ ] Prefer new squared-length consumer APIs or guidance only when callers do
  not need actual magnitude or normalized components.

## Phase 3: Focused Positive-Divisor Candidate Benchmarks

**Why:** Phase 1-b found more methods with locally positive divisors, but these
paths are not directly covered by the current benchmark suite. Add benchmarks
before changing runtime code so helper substitutions stay evidence-driven.

- [ ] Add vector benchmarks for `Project`, `ProjectOnPlane`, `Angle`,
  `ClosestPointOnLineSegment`, and `ClosestPointsOnTwoLines`.
- [ ] Add matrix benchmarks for `ExtractRotation` and `Decompose`.
- [ ] Test `DivideByPositive` substitutions only after those baselines exist.
- [ ] Keep `FixedPlane.Normalize` on its reciprocal-plus-multiply shape unless a
  benchmark proves another exact approach is faster.
