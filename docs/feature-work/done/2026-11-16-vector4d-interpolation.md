# FMS-Issue-043 Vector4d Interpolation

At `ec4b934`, both 4D interpolation methods formed the component difference with
saturating subtraction before multiplying by the interpolation factor. This lost
part of a difference spanning the full `Fixed64` range. The case was split from
FMS-Issue-041 after reproduction at `2a65963`.

For example, let `a = new Vector4d(Fixed64.MinValue)` and
`b = new Vector4d(Fixed64.MaxValue)`. At a factor of one, both `Lerp` and
`UnclampedLerp` returned raw `-1` in every component, in either endpoint order.
At one half, `Lerp(a, b, Fixed64.Half)` returned `-1073741824` in every
component. The exact midpoint is half a raw unit below zero, which rounds to
zero with ties to even.

Clamped `Lerp` now uses `FixedMath.Lerp` for each component. The scalar
operation preserves the entire difference, clamps the fraction, and rounds the
final result to nearest with ties to even. Final-result rounding can change the
last raw bit even when the difference was already representable and the
interpolation lands on a half-unit tie. This aligns 4D with the clamped 2D and
3D interpolators.

`UnclampedLerp` now returns the ending vector exactly at one. Zero already
returned the starting vector exactly. For other fractions it retains its
documented saturating component arithmetic, including extrapolation. The factor
immediately above one can still return raw `-1` for the extreme case above. Use
clamped `Lerp` for interpolation within the segment. The receiver extensions
forward to the static methods and document the same contracts.

The
[regression tests](../../../tests/FixedMathSharp.Tests/Numerics/Vectors/Vector4d.Tests.cs)
cover both endpoint orders, out-of-range clamping, interpolated values at one
half and one quarter, ordinary raw-unit ties, and preserved extrapolation at
saturated differences. Five endpoint and midpoint cases failed before the
runtime change.

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter 'FullyQualifiedName~Vector4dTests'
```

The full solution suites passed 7,479 tests in `Release` and 7,458 in
`ReleaseLean`. Focused coverage reached all 10 instrumented lines and both
branch outcomes across the four interpolation entry points in each build. A
separate integer oracle checked 10,810 clamped lerps against each Lean binary
for `net10.0` and `netstandard2.1`, using extreme raw values, boundary factors,
and seeded samples. DocFX built with warnings treated as errors and rendered the
updated contracts for the static methods and receiver extensions.

On Windows with .NET SDK 10.0.302, the existing `Vector4dBenchmarks.Lerp`
averaged 4.843 µs at the baseline and 3.444 µs after the fix, about 29% less
time per batch of 256 midpoint interpolations. The benchmark uses ordinary
component magnitudes. Each build completed two out-of-process launches of five
measurement iterations, with zero allocations in all four launches.

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net10.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net10.0/FixedMathSharp.Benchmarks.dll all \
  -j Short --launchCount 2 --warmupCount 4 --iterationCount 5 --filter '*Vector4dBenchmarks.Lerp'
```
