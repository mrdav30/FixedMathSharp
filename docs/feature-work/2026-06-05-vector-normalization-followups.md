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
