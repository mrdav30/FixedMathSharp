# FixedMathSharp.FluentAssertions

`FixedMathSharp.FluentAssertions` adds custom FluentAssertions helpers for `FixedMathSharp` types.

Main library:
[FixedMathSharp on GitHub](https://github.com/mrdav30/FixedMathSharp)

It includes assertions for:

- `Fixed64`
- `Vector2d`
- `Vector3d`
- `FixedQuaternion`
- `Fixed3x3`
- `Fixed4x4`

## Example

```csharp
using FixedMathSharp;
using FixedMathSharp.Assertions;

new Fixed64(1.25).Should().BeApproximately(new Fixed64(1.25));

var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
rotation.Should().BeNormalized();

var matrix = Fixed4x4.ScaleRotateTranslate(
    new Vector3d(1, 2, 3),
    rotation,
    new Vector3d(2, 2, 2));

matrix.Should().HaveTranslationApproximately(new Vector3d(1, 2, 3));
matrix.Should().HaveRotationApproximately(rotation);
matrix.Should().HaveScaleApproximately(new Vector3d(2, 2, 2));
```
