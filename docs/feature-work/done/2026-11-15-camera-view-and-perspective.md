# FMS-Issue-038 Camera View Mapping and Perspective Depth

## Reproduction

The failures were reproduced on `10d699b` with .NET SDK 10.0.302 in Windows
package mode. The eight added camera and perspective regression cases failed
against the original implementation.

- `CreateLookAt` with eye `(1, 2, 3)`, target `(2, 2, 3)` and up `+Y` mapped the
  eye to `(6, 0, -2)` and the target to `(6, 0, -3)`. The camera basis was in
  rows, while the translation already used column projections for the
  world-to-view mapping.
- All three perspective factories used a narrowed `near * far` before dividing
  by the clip depth. With near `100000` and far `200000`, the centered factory
  projected the near and far planes to approximately `1.7852516351` and
  `1.8926258176`. The expected depths are zero and one.
- A near distance and width of `1500000000` produced a horizontal scale of about
  `1.43166` instead of two. Off-center bounds with X edges
  `(-1500000000, 1500000000)` and Y edges `(1200000000, 1600000000)` also
  saturated intermediate spans or sums.
- A field of view of one raw Q32.32 unit reached division by zero after its half
  angle rounded to zero. With a field of view of two raw units and aspect ratio
  four, the saturated vertical coefficient was divided again and produced a
  horizontal scale of `536870912` instead of `1073741824`.

## Correction

The camera basis occupies columns in the view matrix so row-vector
multiplication projects a world point onto each camera axis and subtracts the
eye's projection. `CreateWorld` continues to store the local-to-world basis in
rows. Exact axis cases now map the eye to zero and put the target on positive
view Z. The oblique test checks the composed world and view matrices against
identity.

The three perspective factories share the depth coefficients and form them
without intermediate product saturation. Centered scale uses fused multiply and
divide. Off-center scale and offset use exact endpoint spans and sums. The
field-of-view factory derives both scale factors directly from the fixed-point
sine and cosine samples and rejects a rounded half angle of zero or pi/2 with
`ArgumentOutOfRangeException` naming `fieldOfView`.

Intermediate arithmetic in the perspective factories now remains wide until each
final coefficient is rounded. Coefficients outside the representable range still
saturate. Point transforms retain their existing arithmetic range and can
saturate when applying a matrix, even if its coefficients fit. Camera basis and
translation arithmetic likewise retain their existing range. View and projection
matrix values can differ from earlier releases where the layout, rounding, or
saturation differed.

## Verification

The Windows package-mode solution builds passed with zero warnings and errors.
The full suites passed 7,409 tests in `Release` and 7,388 in `ReleaseLean`.
Focused coverage runs passed 158 and 157 tests, respectively. In both
configurations, the four camera and perspective factories and their three
perspective helpers covered all 68 instrumented lines and 26 branches. Run the
regressions from the repository root with the restored package graph.

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter 'FullyQualifiedName~Fixed4x4Tests|FullyQualifiedName~FixedBoundFrustum'
```

An independent integer-ratio oracle checked 512 deterministic inputs for each
perspective factory, including reversed off-center spans and saturated
coefficients. The Release `net10.0` and `netstandard2.1` assemblies produced
identical raw coefficients for those inputs. The field-of-view oracle used the
library's sampled sine and cosine to check coefficient arithmetic independently
of trigonometric approximation.

See the focused regressions in
[`Fixed4x4.Tests.cs`](../../../tests/FixedMathSharp.Tests/Numerics/Matrices/Fixed4x4.Tests.cs)
and the caller contract in
[`coordinate-conventions.md`](../../api/guides/coordinate-conventions.md#camera-and-projection-matrices).
