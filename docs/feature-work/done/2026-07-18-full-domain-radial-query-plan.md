# Full-Domain Radial Query Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development or superpowers:executing-plans,
> superpowers:test-driven-development, and
> superpowers:verification-before-completion. Steps use checkbox (`- [ ]`)
> syntax for tracking.

**Status:** Complete on 2026-07-18

**Goal:** Make circle/sphere predicates, bounded ray intervals, and sphere
cross-sections exact across the complete finite Q32.32 input domain, then move
the matching Gravitas query reducers onto those lower-stack contracts.

**Architecture:** Keep `Signed192` and `Signed320` internal. Existing bound
methods route through one exact radial-distance comparison; `FixedRay2d` and
`FixedRay` expose allocation-free bounded interval methods with two `out`
parameters; `FixedMath` exposes the scalar circle-cross-section radius needed
by mixed-dimensional geometry. The existing first-hit ray path remains
independent so callers that do not need an exit root do not pay for one.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, internal fixed-width wide arithmetic,
xUnit v3, BenchmarkDotNet, `netstandard2.1`, and `net8.0`.

## Global Constraints

- Preserve deterministic nearest-even conversion at the single public
  `Fixed64` boundary.
- Perform radius addition, squared-distance comparison, discriminant work,
  root ordering, and interval clipping before representable narrowing.
- Keep hot paths allocation-free and do not expose internal wide-number types.
- Keep 2D and 3D public contracts behaviorally identical.
- Treat this as a clean v7 API: remove squared-radius properties that cannot
  represent their advertised domain instead of retaining misleading aliases.
- Do not claim arbitrary finite-segment capsule/cylinder projections are fixed
  by a circle/sphere interval. That separate lower-stack task must own its wide
  projection arithmetic.

---

## Design Decision

The selected API is:

```csharp
public bool TryGetIntersectionInterval(
    FixedBoundCircle circle,
    Fixed64 maxParameter,
    out Fixed64 entry,
    out Fixed64 exit);

public bool TryGetIntersectionInterval(
    FixedBoundCircle circle,
    Fixed64 radiusExpansion,
    Fixed64 maxParameter,
    out Fixed64 entry,
    out Fixed64 exit);
```

`FixedRay` mirrors the two overloads for `FixedBoundSphere`.

The result is the exact closed overlap interval clipped to
`[0, maxParameter]`. Tangency has equal endpoints; a start inside has
`entry == 0`; an exact zero direction inside returns the whole bounded
interval; a boundary point moving outward returns `[0, 0]`; negative maximums
return `false`; negative radius expansion throws. Exact clipping happens before
nearest-even conversion. A non-empty sub-raw interval may therefore return two
equal representable endpoints while the Boolean remains authoritative.

`FixedRange` was rejected because its existing half-open range semantics do
not match boundary-inclusive contact. A public generic quadratic solver was
rejected because callers could saturate coefficients before invoking it and
because it would expose numeric machinery instead of geometry intent.

## Task 1: Exact Radial Predicates

**Files:**

- Modify: `src/FixedMathSharp/Numerics/Wide/WideGeometry.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.Extensions.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector3d.Extensions.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundCircle.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundSphere.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundCircle.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundSphere.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundArea.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundBox.Tests.cs`

- [x] Add failing extreme-domain point, pair, strict-overlap, closest-point,
      and centered-extent containment regressions in both dimensions.
- [x] Add internal exact distance-versus-combined-radius and centered-extent
      predicates.
- [x] Route existing public predicates through them while preserving inclusive,
      strict, zero-radius, and negative-threshold behavior.
- [x] Remove `FixedBoundCircle.RadiusSquared` and
      `FixedBoundSphere.RadiusSquared`; update migration/docs and downstream
      callers to retain actual radius ownership.
- [x] Run focused tests and existing predicate benchmarks with zero allocation.

**Completion summary (2026-07-18):** Existing vector, circle/sphere, area, and
box predicates now compare exact endpoint-difference squares with exact radius
sums, including the one-raw-unit separation at opposite scalar limits. Centered
extent containment no longer consumes saturated derived bounds. The misleading
public squared-radius properties were removed and the v7 migration guide now
directs callers to retain actual radius ownership. Exact predicate admission
also exposed and corrected an ordinary tilted-frustum construction drift by
using the same comparator for construction admission and final radius
correction; full-domain sphere construction/update arithmetic remains deferred
to `FMS-Issue-015`. The six focused regressions, all 408 affected vector/bound
tests, and the seven existing predicate benchmark rows passed; BenchmarkDotNet
reported zero managed allocation for every measured row.

## Task 2: Exact Bounded Entry/Exit Intervals

**Files:**

- Modify: `src/FixedMathSharp/Geometry/Primitives/WideRayIntersection.cs`
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedRay2d.cs`
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- Test: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedRay2d.Tests.cs`
- Test: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedRay.Tests.cs`
- Benchmark: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`

- [x] Add failing interval tests for ordinary/non-unit crossings, clipping,
      starts-inside, full containment, tangency, zero direction, exact boundary
      motion, radius-sum overflow, sub-raw intervals, half-even entry/exit
      roots, equivalent scaling, and exact roots just inside/outside the bound.
- [x] Refactor coefficient/discriminant ownership so the first-root and
      interval solvers share exact setup without forcing first-hit callers to
      refine an unused exit.
- [x] Implement ordered lower- and upper-root correction with exact polynomial
      and derivative evaluation in `Signed320`.
- [x] Expose the two bounded interval overloads on both public ray types.
- [x] Add 2D and 3D interval benchmark rows and prove the existing first-root
      rows do not materially regress.

**Completion summary (2026-07-18):** `FixedRay2d` and `FixedRay` now expose
closed bounded entry/exit intervals with exact radius expansion, root clipping,
and independent nearest-even conversion. Regressions cover non-unit and
scale-equivalent motion, starts inside, zero motion, outward/inward boundaries,
tangency, exact bounded admission, radius-sum overflow, irrational correction,
half-even endpoints, and non-empty sub-raw intervals. ShortRun medians were
183.8 ns/112.4 ns for the unchanged 2D/3D first-hit rows and 340.2 ns/127.4 ns
for the corresponding interval rows; every row allocated 0 B.

## Task 3: Exact Sphere Cross-Section Radius

**Files:**

- Create: `src/FixedMathSharp/Core/FixedMath.Geometry.cs`
- Create: `src/FixedMathSharp/Numerics/Wide/WideRadialGeometry.cs`
- Test: `tests/FixedMathSharp.Tests/Core/FixedMathGeometry.Tests.cs`

- [x] Add failing tests for ordinary `3-4-5`, signed offsets, tangent/outside,
      negative-radius rejection, `100000/60000 -> 80000`, and maximum-raw
      difference-of-squares rounding.
- [x] Implement `FixedMath.TryGetCircleCrossSectionRadius(radius, offset, out
      crossSectionRadius)` with a wide difference of squares and exact
      nearest-even square root.
- [x] Add `TryGetSphereSlabCrossSectionRadius` so opposite-domain sphere/slab
      centers remain exact until the nearest in-slab plane is selected.
- [x] Document zero-allocation domain and failure behavior.

**Completion summary (2026-07-18):** `FixedMath` now owns exact sphere-slice
reduction through `TryGetCircleCrossSectionRadius`. The difference of squares
and integer root remain wide through nearest conversion, including the
`100000/60000 -> 80000` regression, maximum-raw near-tangent rounding, signed
offsets, tangency, misses, and the otherwise unrepresentable absolute value of
`Fixed64.MinValue`. The centered-slab helper also rejects opposite-domain false
overlaps without narrowing center separation or half-thickness projection.

## Task 4: Gravitas Circle/Sphere Consumer Migration

**Files:**

- Modify: `../Gravitas/src/Gravitas/Queries/3D/RaycastSegmentWorker.cs`
- Modify: `../Gravitas/src/Gravitas/Queries/Mixed/GravitasQueryMixedService.SphereSlab.cs`
- Modify: `../Gravitas/src/Gravitas/Queries/Mixed/GravitasQueryMixedService.Support.cs`
- Modify: `../Gravitas/src/Gravitas/Queries/Mixed/GravitasQueryMixedService.CircleGeometry.cs`
- Modify: `../Gravitas/docs/feature-work/issue-tracker.md`
- Test: `../Gravitas/tests/Gravitas.Tests/Queries/RaycastSegmentWorkerTests.cs`
- Test: `../Gravitas/tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`
- Benchmark: `../Gravitas/tests/Gravitas.Benchmarks/Queries/RadialRaycastBenchmarks.cs`

- [x] Add ordinary/extreme crossings, a nonzero segment whose square rounds to
      zero, authored-endpoint, intersections-disabled, and mixed-slab parity
      regressions; retain the broader coefficient/interval edge matrix in the
      lower-stack tests that own the exact solver.
- [x] Replace duplicated circle/sphere quadratics with the FixedMathSharp
      interval and cross-section APIs, passing radius expansion separately.
- [x] Preserve spatial-distance units, deterministic feature order, exact
      start/end containment, and authored endpoint reconstruction.
- [x] Do not route arbitrary segment/mesh-edge projections through already
      rounded perpendicular vectors; keep them queued for the dedicated
      finite-axis geometry task.
- [x] Run full Release/ReleaseLean, focused allocation regressions, and relevant
      query benchmarks.

**Completion summary (2026-07-18):** Gravitas's ambiguous raw sphere-segment
overload was replaced by explicit `FixedBoundSphere` ownership. It evaluates
the authored segment on `[0, 1]` and reconstructs authored endpoints from exact
bounded interval parameters. The
mixed circle-slab side reducer passes radius expansion separately to the exact
2D interval primitive with authored-segment admission, and mixed sphere slicing
uses the exact lower-stack sphere-vs-slab cross-section helper. Direct reducer
regressions cover rounded side/cap endpoints, opposite-domain vertical
separation, and the `400000 / 60000 / 100000` extreme crossing without forcing
broad-phase enumeration across an artificial giant world. Release passed 2,790
tests and ReleaseLean passed 2,751 tests; focused mixed steady-state allocation
remained 0 B. Gravitas ShortRun medians were 1.425/1.669 us for sphere segments
and 3.326/4.217 us for mixed circle slabs at scales 1 and 100,000 respectively,
all at 0 B. Finite-axis projections and conic quadratics remain explicit active
issues.

## Task 5: Closure

- [x] Update `MIGRATION.md`, complexity exceptions, both issue trackers, and
      the Gravitas active queue with exact completed/deferred scope.
- [x] Re-achieve 100% FixedMathSharp line/branch/method coverage without
      coverage-only or API-shape tests.
- [x] Obtain an independent correctness/performance review.
- [x] Commit FixedMathSharp and Gravitas independently, excluding Gravitas
      local-link project files.

**Closure evidence (2026-07-18):** Debug coverage passed 1,460 core plus 8
Chronicler tests and reached exactly 9,408/9,408 lines, 3,064/3,064 branches,
and 1,528/1,528 ReportGenerator methods. Release passed the same 1,460 plus 8;
ReleaseLean passed 1,439 plus 8. The only CRAP scores above 30 remain the four
fully covered complexity floors already registered in
`docs/complexity-exceptions.md`. FixedMathSharp committed the final slab
projection hardening as `38344d2`; Gravitas committed its breaking explicit
bounds migration as `01749ec`. The three Gravitas local-link project files
remain unstaged validation scaffolding.

## Explicit Follow-Ups

- Full-domain finite-segment capsule/cylinder and mesh-edge projection needs a
  dedicated lower-stack primitive; its coefficients exceed the current radial
  solver widths if formed naïvely.
- `FixedBoundSphere` construction/merge paths still contain saturated distance
  and radius-update arithmetic.
- Gravitas cone quadratics are conic rather than radial and remain a separate
  hardening problem.
