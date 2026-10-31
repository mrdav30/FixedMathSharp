using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class WideRationalBasis3dTests
{
    [Fact]
    public void ExactIdentity_ShouldUseUnitScaleAndPreserveFullDomainCoordinates()
    {
        var basis = new WideRationalBasis3d(FixedQuaternion.Identity);
        // This exact representation removes artificial common powers before
        // downstream coordinate products, without changing the rotation.
        Assert.Equal(Signed192.Signed(1), basis.Denominator);
        var point = new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.FromRaw(1));
        WideAxis3 transformed = WideRigidProjection.TransformLocalAxis(basis,
            Signed192.Raw(point.X), Signed192.Raw(point.Y), Signed192.Raw(point.Z));
        Assert.Equal(Signed320.ExtendValue(Signed192.Raw(point.X)), transformed.X);
        Assert.Equal(Signed320.ExtendValue(Signed192.Raw(point.Y)), transformed.Y);
        Assert.Equal(Signed320.ExtendValue(Signed192.Raw(point.Z)), transformed.Z);
    }

    [Fact]
    public void CreateRelative_IdentityShouldPreserveTheOtherExactBasisAndOrthogonality()
    {
        var identity = new WideRationalBasis3d(FixedQuaternion.Identity);
        var other = new WideRationalBasis3d(FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)27, (Fixed64)(-41), (Fixed64)13));
        WideRationalBasis3d relative = WideRationalBasis3d.CreateRelative(identity, other);
        Assert.Equal(other.Denominator, relative.Denominator);
        for (int index = 0; index < 3; index++)
            Assert.Equal(other.GetAxis(index), relative.GetAxis(index));
        WideRationalBasis3d inverse = WideRationalBasis3d.CreateRelative(other, identity);
        Signed576 squaredScale = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
            other.Denominator, other.Denominator));
        for (int first = 0; first < 3; first++)
            for (int second = 0; second < 3; second++)
                Assert.Equal(first == second ? squaredScale : default,
                    WideAxis3.Dot(inverse.GetAxis(first), inverse.GetAxis(second)));
    }

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
