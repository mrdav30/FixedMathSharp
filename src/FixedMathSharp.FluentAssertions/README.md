# FixedMathSharp.FluentAssertions

`FixedMathSharp.FluentAssertions` adds readable FluentAssertions checks for
FixedMathSharp scalar, vector, quaternion, and matrix values.

## Install

```bash
dotnet add package FixedMathSharp.FluentAssertions
```

Use
[`FixedMathSharp.FluentAssertions.Lean`](https://www.nuget.org/packages/FixedMathSharp.FluentAssertions.Lean)
when your tests reference `FixedMathSharp.Lean`.

## Example

```csharp
using FixedMathSharp;
using FixedMathSharp.Assertions;

Fixed64 actual = Fixed64.FromDecimal(1.2501m);
Fixed64 expected = Fixed64.FromDecimal(1.25m);

actual.Should().BeApproximately(
    expected,
    Fixed64.FromDecimal(0.001m));

FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
    Vector3d.Up,
    Fixed64.HalfPi);

rotation.Should().BeNormalized();

Fixed4x4 matrix = Fixed4x4.ScaleRotateTranslate(
    new Vector3d(1, 2, 3),
    rotation,
    new Vector3d(2, 2, 2));

matrix.Should().HaveTranslationApproximately(new Vector3d(1, 2, 3));
matrix.Should().HaveRotationApproximately(rotation);
matrix.Should().HaveScaleApproximately(new Vector3d(2, 2, 2));
```

Approximate assertions use `Fixed64.Epsilon` when no tolerance is supplied.
Pass an explicit tolerance when the expected error budget is part of the test.

The package includes assertions for:

- `Fixed64`
- `Vector2d` and `Vector3d`
- `FixedQuaternion`
- `Fixed3x3` and `Fixed4x4`

See the [main FixedMathSharp repository](https://github.com/mrdav30/FixedMathSharp)
and [API reference](https://mrdav30.github.io/FixedMathSharp/) for the numeric
contracts behind these assertions.
