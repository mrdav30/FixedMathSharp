using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedOrientedBoxCylinderEdgeFeatureTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void EdgeRim_InwardNormalOffsetPreservesExactSquaredDepthAndReflectedSupport(int sign)
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d((Fixed64)10, Fixed64.Half, Fixed64.Half));
        // At the original center(-5,31/2,41/2), R=25 is exact rim/edge
        // touch with normal(0,15,16)/sqrt(481). Moving inward by that
        // unnormalized vector/1024 retains its stationary normal and gives
        // depth²=481/2^20, an exact dyadic value. The move is <1/32, well
        // inside the edge's length and the projected ellipse's minimum
        // curvature radius16, so this remains the nearest boundary feature.
        Vector3d center = new Vector3d((Fixed64)(-5),
            Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024),
            Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024)) * (Fixed64)sign;
        Assert.True(box.TryGetCenteredCylinderContact(center, ThreeFourAxisRotation(), Vector3d.Up,
            (Fixed64)10, (Fixed64)25, out FixedContactAnchors contact));

        Assert.False(contact.DepthIsClamped);
        // Nearest-even sqrt(481*2^44), derived by integer square comparisons.
        Assert.Equal(Fixed64.FromRaw(91_988_268L), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.Zero,
            Fixed64.FromRaw(sign * 2_937_504_781L), Fixed64.FromRaw(sign * 3_133_338_433L)), contact.Normal);
        Assert.Equal(new Vector3d((Fixed64)(-10), (Fixed64)sign * Fixed64.Half,
            (Fixed64)sign * Fixed64.Half), contact.FirstAnchor.LocalPoint);
        Assert.Equal(center, contact.SecondAnchor.Origin);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EdgeRim_RanksMultipleStationaryValuesWithoutChangingThePositiveOverlap(bool offPrincipal)
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(10, 10, 10));
        Vector3d center = new(Fixed64.Zero, (Fixed64)6,
            (Fixed64)10 + (offPrincipal ? Fixed64.Quarter : Fixed64.Zero));
        // On the positive-cap/+Y/+Z edge-X cone, the projected ellipse has
        // squared semiaxes400,625 and offset(7,-1/4) or(7,0).
        // The former's derivative is negative at t=0, positive at t=1/2,
        // and negative at t=1: two distinct positive stationary gaps share
        // one chart. The latter's squared stationary values include169,729
        // before the nonprincipal6850/9; choosing the first value blindly
        // would associate the wrong depth with the retained normal.
        Assert.True(box.TryGetCenteredCylinderContact(center, ThreeFourAxisRotation(), Vector3d.Up,
            (Fixed64)10, (Fixed64)25, out FixedContactAnchors contact));

        // The box contains the origin-centered radius10 ball. The cylinder
        // contains the origin-centered radius1 ball: axial distance18/5 has
        // margin7/5, while its center distance is below12 and radius is25.
        // The cylinder cap axis independently bounds depth above by77/5.
        Assert.InRange(contact.Depth.m_rawValue, ((Fixed64)11).m_rawValue,
            Fixed64.FromFraction(77, 5).m_rawValue);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(center, contact.SecondAnchor.Origin);
    }

    [Fact]
    public void EdgeRim_ClampedAnalyticDepthRetainsMultipleNonwinningStationaryValues()
    {
        Fixed64 maximum = Fixed64.MaxValue;
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(maximum, maximum, maximum));
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(maximum.m_rawValue / 64), Fixed64.Zero);
        const long scale = 784_150_157L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(scale), Fixed64.Zero,
            Fixed64.FromRaw(-2 * scale), Fixed64.FromRaw(5 * scale));
        // Contained balls give depth >= (3/2-1/64)*MaxValue, proving clamping.
        // In the +cap/+X/+Y edge-Z chart, the exact derivative is positive
        // at t=0, negative at t=3/4, and positive at t=1: two admitted roots
        // share one chart. Both lose to the analytic Y face. A clamped depth
        // cannot bound them, so construction reuse and exact ranking remain.
        Assert.True(box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            maximum, maximum, out FixedContactAnchors contact));
        Assert.Equal(maximum, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(center, contact.SecondAnchor.Origin);
    }

    [Fact]
    public void FaceTouch_RemainsZeroWhileNonwinningEdgeStationaryGapsArePositive()
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        // The negative-X support of this cylinder is exactly(1,0,0), in
        // the interior of the box's +X face. Other edge-cone stationary
        // gaps are positive and must not replace this exact zero minimum.
        Assert.True(box.TryGetCenteredCylinderContact(new Vector3d(20, -17, 0),
            ThreeFourAxisRotation(), Vector3d.Up, (Fixed64)10, (Fixed64)25,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ProjectedCapCenter_UsesPrincipalDirectionWhenStationaryEliminationDenominatorVanishes()
    {
        const long quaternionScale = 784_150_157L;
        // This exact quaternion ratio rotates +Y to(2,2,1)/3. With
        // half-axis(2,2,1), cap+ and edge-X(+Y,+Z) have projected offset0.
        // K=q-p*t vanishes identically; the principal normal has t=1/2.
        FixedQuaternion rotation = new(Fixed64.FromRaw(quaternionScale), Fixed64.Zero,
            Fixed64.FromRaw(-2 * quaternionScale), Fixed64.FromRaw(5 * quaternionScale));
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(3, 3, 3));
        Assert.True(box.TryGetCenteredCylinderContact(new Vector3d(0, 5, 4), rotation,
            Vector3d.Up, (Fixed64)6, (Fixed64)10, out FixedContactAnchors contact));

        // Both shapes contain a radius1/4 ball at(0,2,2). The cap-axis
        // support gap is10/3, giving independent nonzero finite bounds.
        Assert.InRange(contact.Depth.m_rawValue, Fixed64.Half.m_rawValue,
            Fixed64.FromFraction(10, 3).m_rawValue);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void EdgeRim_IntegerInwardOffsetHasExactUnitDepthAndRationalNormal()
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(100, 1, 1));
        const long scale = 106_545_026L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(2 * scale), Fixed64.Zero,
            Fixed64.FromRaw(-10 * scale), Fixed64.FromRaw(39 * scale));
        // a=(60,109,12)/125, n=(0,3/5,4/5), and radial unit
        // u=(-45,12,116)/125. With H=125a and R=100, the center
        // (0,1,1)+H+R*u-n=(24,119,105) lies exactly one unit inward
        // from rim/edge touch. The projected minimum curvature radius
        // is 100*(12/25)^2>23, greater than this displacement.
        Assert.True(box.TryGetCenteredCylinderContact(new Vector3d(24, 119, 105),
            rotation, Vector3d.Up, (Fixed64)250, (Fixed64)100,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5)), contact.Normal);
        Assert.Equal(new Vector3d(-100, 1, 1), contact.FirstAnchor.LocalPoint);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void EdgeRim_ChartBoundaryTouchRetainsZeroAcrossBothCharts()
    {
        const long scale = 1_753_413_056L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(scale), Fixed64.Zero,
            Fixed64.FromRaw(-scale), Fixed64.FromRaw(2 * scale));
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(10, 1, 1));
        // a=(2,1,2)/3, H=(2,1,2). For the unnormalized normal
        // (0,1,1), |P_a*n|=1 and R*P_a*n=(-2,2,1). Thus the
        // cylinder's negative support is exactly the edge point(0,1,1).
        // Both closed chart endpoints represent this t=1 minimum.
        Assert.True(box.TryGetCenteredCylinderContact(new Vector3d(0, 4, 4), rotation,
            Vector3d.Up, (Fixed64)6, (Fixed64)3, out FixedContactAnchors contact));

        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(3_037_000_500L),
            Fixed64.FromRaw(3_037_000_500L)), contact.Normal);
        Assert.Equal(new Vector3d(-10, 1, 1), contact.FirstAnchor.LocalPoint);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ProjectedPrincipalOffset_AdmitsAnUnprunedZeroEliminationDenominator()
    {
        const long scale = 784_150_157L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(scale), Fixed64.Zero,
            Fixed64.FromRaw(-2 * scale), Fixed64.FromRaw(5 * scale));
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(3, 3, 3));
        // For a=(2,2,1)/3 and H=(2,2,1), the positive-cap/+Y/+Z
        // edge has offset(-2,-1). Its t=1/2 principal direction makes
        // K=L=0, while negative offset coordinates prevent cone pruning.
        Assert.True(box.TryGetCenteredCylinderContact(new Vector3d(0, 7, 5), rotation,
            Vector3d.Up, (Fixed64)6, (Fixed64)10, out FixedContactAnchors contact));

        // Both shapes contain a radius1/4 ball at(2,5/2,5/2).
        // The cylinder cap-axis independently gives gap5/3.
        Assert.InRange(contact.Depth.m_rawValue, Fixed64.Half.m_rawValue,
            Fixed64.FromFraction(5, 3).m_rawValue);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ThreeComponentAxisFaceTouch_PreservesTheAnalyticZeroMinimum()
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        // H=(20,9,12), R=125 and negative-X radial support offset
        // (-75,60,80) put the cylinder's support exactly at(1,0,0).
        Assert.True(box.TryGetCenteredCylinderContact(new Vector3d(96, -51, -68),
            RationalThreeComponentAxisRotation(), Vector3d.Up, (Fixed64)50, (Fixed64)125,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    private static FixedQuaternion RationalThreeComponentAxisRotation()
    {
        const long scale = 208_336_516L;
        return new FixedQuaternion(Fixed64.FromRaw(6 * scale), Fixed64.Zero,
            Fixed64.FromRaw(-10 * scale), Fixed64.FromRaw(17 * scale));
    }

    private static FixedQuaternion ThreeFourAxisRotation()
    {
        const long scale = 1_920_767_767L;
        return new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-scale), Fixed64.FromRaw(2 * scale));
    }
}
