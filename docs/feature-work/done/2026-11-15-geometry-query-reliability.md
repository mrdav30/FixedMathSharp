# FMS-Issue-040 Geometry Query Reliability

## Reproduced defects and fixes

Reproduced the original cases at `c5c10e253eced333bf346d86a3d5031b3657259d` with
.NET SDK 10.0.302 on Windows, using both package dependency graphs.

- **Box surface.** The center of `[-2, 2]³` projected to `(-2, -2, -2)` because
  the routine snapped every tied axis. Its squared distance was 12 where the
  minimum is 4. It now returns `(-2, 0, 0)`. Distances use the full unsigned raw
  span so differences above `Fixed64.MaxValue` still rank correctly. Ties favor
  X, then Y, then Z and the minimum face within the selected axis.
  [`FixedBoundBox.Tests.cs`](../../../tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundBox.Tests.cs)
- **Ray slabs.** A ray from zero with X direction one raw unit reported a hit at
  `Fixed64.MaxValue` for the X slab `[1, 2]`. The point at that parameter was
  `(0.5, 0, 0)`. Disjoint per-axis intervals could also appear to meet when both
  endpoints saturated or rounded to the same parameter. Both ray overloads now
  intersect the exact slab ratios within the closed representable forward
  interval and round the admitted entry once, to nearest even. This preserves a
  real hit at `Fixed64.MaxValue` and retains a hit at 1,500,000,000 for origin X
  = −1,500,000,000, direction X = 2, and target X = 1,500,000,000.
  [`RaySlab.Tests.cs`](../../../tests/FixedMathSharp.Tests/Geometry/Primitives/RaySlab.Tests.cs)
- **Sphere transform.** For a unit sphere at zero and identity with `M12 = 1`,
  the old radius was about 1.4142. The interior point `(0.75, 0.625, 0)` maps to
  `(0.75, 1.375, 0)`, outside that bound. An orthogonal scale of 100,000 also
  underbounded because basis squares saturated. The new radius uses the largest
  absolute row sum of the exact Gram matrix as a squared stretch bound. It
  retains wide intermediates until outward radius rounding and adds an allowance
  for center rounding. The guarantee covers the mathematical affine image before
  individual sample points are rounded. Projective transforms and
  unrepresentable computed bounds throw.
  [`FixedBoundSphere.Tests.cs`](../../../tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundSphere.Tests.cs)
- **Plane normalization.** A normal `(Fixed64.FromRaw(1), 0, 0)` with `D = 0`
  normalized to `(0.5, 0, 0)` because its reciprocal saturated. A right frustum
  plane scaled to one raw unit then admitted a sphere at `(2.25, 0, 0)` of
  radius `0.75` even though it cleared that face. Both coefficients now divide
  by the same exact normal length and round to nearest even. A zero normal still
  throws. An out-of-range rounded constant term throws before an in-place plane
  is replaced.
  [`FixedPlane.Tests.cs`](../../../tests/FixedMathSharp.Tests/Geometry/Primitives/FixedPlane.Tests.cs),
  [`FixedBoundFrustum.Tests.cs`](../../../tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundFrustum.Tests.cs)

## Preserved query contracts

- The frustum with outward normals `−Z, +Z, −X, +X, +Y, −Y` and `D = −1`
  describes `[-1, 1]³`. A sphere at `(2, 2, 0)` with radius 1.25 passes every
  plane even though its squared distance to the cube is 2 and its squared radius
  is 1.5625. This is a conservative broad-phase result.
- For box A `[-2, 2]³` and box B from `(3, −1, −1)` to `(4, 1, 1)`,
  `FindClosestPointsBetweenBoxes(A, B)` finds the minimizing point `(2, −1, −1)`
  on A. The documented search visits B's corners. For B from `(−1, 0, −0.5)` to
  `(1, 100, 0.5)`, the boxes overlap and the corner search can still return a
  candidate with positive distance. A true minimum-distance pair would need a
  separate query with that contract.

## Verification

The initial focused regression run produced 17 expected failures and four
passing controls before any runtime edit. The focused post-fix run passed all 69
cases. Full solution builds produced no compiler warnings for either target
framework. The solution passed 7,393 core and 49 Chronicler tests in `Release`
and 7,372 core and 49 Chronicler tests in `ReleaseLean`. DocFX built with
warnings as errors, and the Pages checks passed for all 69 HTML pages. The
generated reference and both target frameworks' package XML agree on the updated
contracts. Focused Coverlet runs passed 296 tests in `Release` and 290 in
`ReleaseLean`, with 142 of 142 instrumented lines and 78 of 78 branches covered
across the 16 affected runtime methods in each configuration.

To rerun the core regressions:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter 'FullyQualifiedName~RaySlabTests|FullyQualifiedName~FixedBoundBoxTests|FullyQualifiedName~FixedBoundSphereTests|FullyQualifiedName~FixedPlaneTests|FullyQualifiedName~FixedBoundFrustumTests'
```

## Performance check

The existing `BoundsBenchmarks` fixtures ran before and after the fix on Windows
x64 with .NET 10.0.10. BenchmarkDotNet ran one out-of-process launch with three
warmups and three measured iterations per case. All eight children completed
successfully and reported zero managed allocation. These means describe the
sampled fixtures on this host.

| Fixture                |    Before |     After |
| ---------------------- | --------: | --------: |
| `Ray2dIntersectsArea`  |  82.76 ns |   9.00 ns |
| `RayIntersectsBox`     |  87.66 ns |   8.58 ns |
| `SphereTransform`      | 199.34 ns | 414.09 ns |
| `FrustumConstructOnly` |   2.14 µs |   4.35 µs |

## Sphere transform performance follow-up

The `SphereTransform` fixture applies 256 deterministic TRS matrices and merges
each transformed sphere into an accumulator. A split probe of the `adcd7be`
implementation measured about 30–40 ns per merge with precomputed transformed
spheres after warmup. The transform accounted for most of the measured cost.

The optimized transform retains the same Gram row-sum bound and outward
rounding. It obtains the leading root bits with single-word integer Newton steps
and recovers the remaining bits by comparing exact squares against the full
radicand. On `net10.0`, the Gram calculation uses signed 128-bit arithmetic when
the coefficient widths prove every row sum fits. Other matrices and
`netstandard2.1` use the existing wide Gram calculation.

A fixed-seed probe compared 32,768 affine cases with the `adcd7be` baseline. The
`net10.0` and `netstandard2.1` outputs matched the baseline center and radius
bits and overflow outcomes in all cases. The regression tests check both sides
of the compact Gram cutoff and the floor and ceiling overflow boundaries.

On the same Windows x64 and .NET 10.0.10 setup, four out-of-process runs
alternated baseline, optimized, optimized, baseline. Each used four warmups and
five measured iterations. The baseline means were 410.3 and 410.4 ns per
transform plus merge. The optimized means were 261.7 and 266.6 ns, about 36%
lower. All four children completed and reported zero managed allocation.

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net10.0/FixedMathSharp.Benchmarks.dll \
  bounds -j Short --launchCount 1 --warmupCount 4 --iterationCount 5 \
  --filter '*SphereTransform'
```

Release and ReleaseLean solution tests passed. After switching the dot product
to direct `Math.BigMul` calls, focused coverage exercised all 65 instrumented
lines and 30 branch outcomes across the eight affected methods in each
configuration.

## Frustum construction performance follow-up

A split probe of `65a71c6` attributed most of the perspective construction time
to normalizing its six planes. Axis-aligned normals now derive their length
directly from the sole nonzero component. For irrational lengths, qualifying
coefficients reuse the floor of the normal length to estimate the rounded
magnitude. The estimate is at most one raw unit high. An exact squared midpoint
comparison corrects it when necessary. Tiny normals and offsets outside the
bound continue through the exact square-ratio path.

A fixed-seed comparison against `65a71c6` matched raw coefficients and failure
outcomes for 65,536 plane cases on `net10.0` and `netstandard2.1`. The same
comparison matched plane and corner outputs for 256 rotated and translated
view-projection matrices. Regression tests cover estimates on the wrong side of
a midpoint, rounding to zero, and the minimum signed axial component.

Four out-of-process BenchmarkDotNet runs on Windows x64 and .NET 10.0.10
alternated baseline, optimized, optimized, baseline. Each used four warmups and
five measured iterations of `FrustumConstructOnly` over the existing 256
perspective matrices. The baseline means were 4.218 and 4.217 µs per
construction. The optimized means were 2.812 and 2.814 µs, about 33% lower. All
four children completed and reported zero managed allocation.

```bash
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net10.0/FixedMathSharp.Benchmarks.dll \
  bounds -j Short --launchCount 1 --warmupCount 4 --iterationCount 5 \
  --filter '*FrustumConstructOnly'
```

The separate rotated-view probe showed no improvement. Its per-process medians
were about 7.60 µs before and 7.64–7.66 µs after. Both full solution suites
passed. Focused coverage exercised all 61 instrumented lines and 30 branch
outcomes across the six affected plane normalization methods in both
configurations.
