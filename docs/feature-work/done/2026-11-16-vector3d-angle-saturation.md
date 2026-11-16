# FMS-Issue-042 Vector3d Angle Saturation

At `5c4118efad67e252172c5a122f552ac58cd4a82a`, `Vector3d.Angle` estimates the
cosine from the dot product and a product of squared lengths. Saturation can
understate the denominator enough to clamp the computed cosine to `1` or `-1`.
This defect was split from FMS-Issue-041 after reproduction at `2a65963`.

| Vector pair                                      | Previous angle |  Corrected angle |
| ------------------------------------------------ | -------------: | ---------------: |
| `(256, 0, 0)` and `(256, 256, 0)`                |             0° |              45° |
| `(-256, 0, 0)` and `(256, 256, 0)`               |           180° |             135° |
| `(Fixed64.MinValue, 0, 0)` and `(0.25, 0.25, 0)` |           180° |             135° |
| `(1000000, 0, 0)` and `(1, 1000000, 0)`          |             0° | About 89.999943° |

The method now checks whether either squared length or their product reaches
`Fixed64.MaxValue`. In that case it evaluates the original cosine formula on
scale-safe normalized directions. The small-length-product check still uses the
original vectors and runs first, including when one squared length underflows to
zero while the other saturates. This retains the zero result for missing surface
normals at existing slope callers. The ordinary path uses the original
fixed-point operations and rounding.

The
[regression tests](../../../tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs)
cover both saturation paths, both cosine signs, operand symmetry, the
million-scale case, and the tiny-input zero policy. Three saturation cases
failed before the runtime change. The ordinary right-angle case and tiny-length
case passed as controls.

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter 'FullyQualifiedName~Vector3dTests.Angle_'
```

The full solution suites passed 7,473 tests in `Release` and 7,452 in
`ReleaseLean`. Focused coverage reached all 15 instrumented lines and 10 branch
outcomes across the static method and extension in each configuration. DocFX
built with warnings treated as errors and rendered the updated remarks on both
Angle members.

Trailblazer at `2f0fbfb` passed all 28 tests in `TransitSlopeTests` and
`MoveLocomotionTests` in both `Release` and `ReleaseLean`. The test projects
used the local FixedMathSharp source build, confirmed by matching DLL hashes and
the local package identities in the runtime dependency manifests.

The 256 nonsaturating benchmark input pairs produced identical raw results with
the baseline and the corrected `net10.0` and `netstandard2.1` builds. On Windows
with .NET 10, two paired out-of-process runs of `Vector3dBenchmarks.Angle`
measured baseline means of 136.47 and 137.35 µs per 256 calls, and corrected
means of 138.09 and 138.41 µs. The difference between the average means was
about 5 ns per call. All four launches completed five measurement iterations and
allocated zero bytes. These inputs measure the ordinary path.

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net10.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net10.0/FixedMathSharp.Benchmarks.dll all \
  -j Short --launchCount 1 --warmupCount 4 --iterationCount 5 --filter '*Vector3dBenchmarks.Angle'
```
