using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderCapsuleWideLengthTests
{
    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void MaximumHalfLength_PreservesTheEndpointRimBoundary(long radiusOffset, bool expected)
    {
        // The upper cap is MaxValue-4; the sphere center at (4,MaxValue,0)
        // has radial/axial excesses 3 and 4, hence exact rim distance 5.
        bool hit = WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
            new Vector3d(0, -4, 0), FixedQuaternion.Identity, Vector3d.Up,
            MaximumFullLength, Fixed64.One,
            new Vector3d((Fixed64)4, Fixed64.MaxValue, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero,
            (Fixed64)5 + Fixed64.FromRaw(radiusOffset),
            out Vector3d normal, out Fixed64 depth, out bool clamped);

        Assert.Equal(expected, hit);
        Assert.Equal(expected
            ? new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero)
            : Vector3d.Zero, normal);
        Assert.Equal(expected ? Fixed64.FromRaw(radiusOffset) : Fixed64.Zero, depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void MaximumHalfLength_DistinguishesExactMaximumDepthFromClamping(long radiusRaw, bool expectedClamped)
    {
        Assert.True(WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            MaximumFullLength, Fixed64.MaxValue,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero,
            Fixed64.FromRaw(radiusRaw), out Vector3d normal, out Fixed64 depth, out bool clamped));

        Assert.Equal(Vector3d.Up, normal);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.Equal(expectedClamped, clamped);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void MaximumHalfLength_ZeroRadiusCylinderRetainsSegmentDistance(long radiusOffset, bool expected)
    {
        // The parallel cores reach MaxValue-4. The capsule lies 3 right and
        // 4 above that endpoint; doubling the cylinder length must be exact.
        bool hit = WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
            new Vector3d(0, -5, 0), FixedQuaternion.Identity, Vector3d.Up,
            MaximumFullLength, Fixed64.Zero,
            new Vector3d((Fixed64)3, Fixed64.MaxValue, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two,
            (Fixed64)5 + Fixed64.FromRaw(radiusOffset),
            out Vector3d normal, out Fixed64 depth, out bool clamped);

        Assert.Equal(expected, hit);
        Assert.Equal(expected
            ? new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero)
            : Vector3d.Zero, normal);
        Assert.Equal(expected ? Fixed64.FromRaw(radiusOffset) : Fixed64.Zero, depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(false, -1L, false)]
    [InlineData(false, 0L, true)]
    [InlineData(false, 1L, true)]
    [InlineData(true, -1L, false)]
    [InlineData(true, 0L, true)]
    [InlineData(true, 1L, true)]
    public void MaximumHalfLength_PreservesTheObliqueInteriorRimBoundary(
        bool permute, long radiusOffset, bool expected)
    {
        // Exact capsule axis b=(12,-9,20)/25 is perpendicular to n=(3,4,0)/5.
        // The upper cylinder rim p=(625,500,0) and capsule-core center
        // q=(706,608,0)=p+135*n certify distance 135. Extending only the
        // lower cylinder cap cannot change this unique supporting pair.
        const long scale = 607_400_100L;
        FixedQuaternion cylinderRotation = permute
            ? new FixedQuaternion(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half)
            : FixedQuaternion.Identity;
        FixedQuaternion capsuleRotation = permute
            ? new FixedQuaternion(Fixed64.FromRaw(3 * scale), Fixed64.FromRaw(6 * scale),
                Fixed64.FromRaw(-2 * scale), Fixed64.FromRaw(scale))
            : new FixedQuaternion(Fixed64.FromRaw(5 * scale), Fixed64.Zero,
                Fixed64.FromRaw(-3 * scale), Fixed64.FromRaw(4 * scale));
        Vector3d cylinderCenter = new(Fixed64.Zero, -Fixed64.MaxValue + (Fixed64)500, Fixed64.Zero);
        Vector3d capsuleCenter = new(706, 608, 0);
        Vector3d expectedNormal = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero);
        if (permute)
        {
            cylinderCenter = new Vector3d(cylinderCenter.Z, cylinderCenter.X, cylinderCenter.Y);
            capsuleCenter = new Vector3d(capsuleCenter.Z, capsuleCenter.X, capsuleCenter.Y);
            expectedNormal = new Vector3d(expectedNormal.Z, expectedNormal.X, expectedNormal.Y);
        }

        bool hit = WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
            cylinderCenter, cylinderRotation, Vector3d.Up, MaximumFullLength, (Fixed64)625,
            capsuleCenter, capsuleRotation, Vector3d.Up, (Fixed64)2000,
            (Fixed64)135 + Fixed64.FromRaw(radiusOffset),
            out Vector3d normal, out Fixed64 depth, out bool clamped);

        Assert.Equal(expected, hit);
        Assert.Equal(expected ? expectedNormal : Vector3d.Zero, normal);
        Assert.Equal(expected ? Fixed64.FromRaw(radiusOffset) : Fixed64.Zero, depth);
        Assert.False(clamped);
    }

    [Fact]
    public void MaximumHalfLength_WarmedPenetrationDoesNotAllocate()
    {
        int contacts = 0;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            contacts = 0;
            for (int iteration = 0; iteration < 8; iteration++)
            {
                if (WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
                        Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
                        MaximumFullLength, Fixed64.One,
                        new Vector3d(Fixed64.One, Fixed64.MaxValue - Fixed64.One, Fixed64.Zero),
                        FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.Half,
                        out _, out _, out _))
                    contacts++;
            }
        });

        Assert.Equal(8, contacts);
        Assert.Equal(0L, allocated);
    }

    private static Signed192 MaximumFullLength => WideArithmetic.AddSigned192(
        Signed192.Raw(Fixed64.MaxValue), Signed192.Raw(Fixed64.MaxValue));
}
