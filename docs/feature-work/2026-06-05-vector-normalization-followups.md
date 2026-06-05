# Vector Normalization Follow-Ups

**Status:** Proposed

**Source:** Extracted from Phase 6 of
`docs/feature-work/done/2026-06-04-benchmark-hotspot-optimization-plan.md`.

**Goal:** Keep vector normalization deterministic while tracking future
optimization ideas that need deeper proof than the 2026-06-05 pass justified.

## Phase 1: Exact Reciprocal Normalization Research

**Why:** Replacing repeated component divisions with a shared reciprocal looked
like the obvious speed shape:

```csharp
Fixed64 invMag = Fixed64.One / mag;
value.X * invMag
```

However, the first implementation changed rounding behavior:

- `Vector2d(3,4).Normal` drifted by a few raw units.
- `Vector3d.SpeedLerp_DoesNotOvershoot` overshot to `5.000000004656613`.

- [ ] Only revisit if an exact deterministic reciprocal/multiply path can match
  current per-component division raw results or if the public contract
  intentionally changes.
- [ ] Include exact 3-4-5 vectors, fractional vectors, huge vectors, tiny raw
  vectors, and no-overshoot movement tests before benchmarking.
- [ ] Keep any accepted approach allocation-free and verify `netstandard2.1`.

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
