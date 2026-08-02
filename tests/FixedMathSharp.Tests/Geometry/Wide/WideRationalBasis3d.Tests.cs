//=======================================================================
// WideRationalBasis3d.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class WideRationalBasis3dTests
{
    [Fact]
    public void CreateRelative_SameBasisProducesExactIdentity()
    {
        var basis = new WideRationalBasis3d(
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)27,
                (Fixed64)(-41),
                (Fixed64)13));

        WideRationalBasis3d relative =
            WideRationalBasis3d.CreateRelative(basis, basis);
        Signed192 expectedDenominator = Signed192.NarrowProven(
            WideArithmetic.MultiplySigned192(
                basis.Denominator,
                basis.Denominator));

        Assert.True(relative.Denominator.Equals(expectedDenominator));
        Assert.True(relative.Xx.Equals(expectedDenominator));
        Assert.True(relative.Yy.Equals(expectedDenominator));
        Assert.True(relative.Zz.Equals(expectedDenominator));
        Assert.True(relative.Xy.IsZero);
        Assert.True(relative.Xz.IsZero);
        Assert.True(relative.Yx.IsZero);
        Assert.True(relative.Yz.IsZero);
        Assert.True(relative.Zx.IsZero);
        Assert.True(relative.Zy.IsZero);
    }
}
