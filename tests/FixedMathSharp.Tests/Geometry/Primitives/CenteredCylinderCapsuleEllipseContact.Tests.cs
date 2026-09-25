using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderCapsuleEllipseContactTests
{
    private const long QuaternionScale = 607_400_100L;

    [Theory]
    [InlineData(135, false, false, false)]
    [InlineData(135, true, false, false)]
    [InlineData(135, false, true, false)]
    [InlineData(135, true, true, false)]
    [InlineData(135, false, false, true)]
    [InlineData(135, true, false, true)]
    [InlineData(135, false, true, true)]
    [InlineData(135, true, true, true)]
    [InlineData(180, false, false, false)]
    [InlineData(180, true, false, false)]
    [InlineData(180, false, true, false)]
    [InlineData(180, true, true, false)]
    [InlineData(180, false, false, true)]
    [InlineData(180, true, false, true)]
    [InlineData(180, false, true, true)]
    [InlineData(180, true, true, true)]
    public void CoreInside_UsesTheUniqueEllipseMinimumBeforeRounding(
        int clearance, bool negateCylinderAxis, bool negateCapsuleAxis, bool permute)
    {
        // Exact b=(12,-9,20)/25 and n=(3,4,0)/5 satisfy b.n=0.
        // The cylinder rim p=(625,500,0) supports n. Set q=p-clearance*n.
        // Projection onto b-perpendicular gives ellipse semiaxes 625,225.
        // At clearance135 its relative minor coordinate is zero and its
        // major coordinate is80*sqrt(34). The nearest-ellipse distance is
        // 225*sqrt(1-(80^2*34)/(625^2-225^2))=135 exactly.
        // At clearance180 the same rim point is the outward-arc local
        // minimum (curvature radius375>180), while the major boundary is
        // farther: (625-180)^2*34-2585^2=50625>0. The minor boundary and
        // cap/side strata are also farther. This case has two positive
        // stationary roots: choosing the first would select a maximum.
        Vector3d center = new(625 - 3 * clearance / 5, 500 - 4 * clearance / 5, 0);
        Fixed64 radius = Fixed64.FromFraction(1, 4);
        Assert.True(Contact(center, radius, negateCylinderAxis, negateCapsuleAxis,
            permute, false, out FixedContactAnchors contact));

        Assert.Equal((Fixed64)clearance + radius, contact.Depth);
        Assert.Equal(Permute(Normal, permute), contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false, -1L, false)]
    [InlineData(false, 0L, true)]
    [InlineData(false, 1L, true)]
    [InlineData(true, -1L, false)]
    [InlineData(true, 0L, true)]
    [InlineData(true, 1L, true)]
    public void CoreOutside_PreservesTheNegativeGapAndOneRawRadiusBoundary(
        bool permute, long radiusOffset, bool expected)
    {
        // q=(706,608,0)=p+135*n. The feasible cylinder/core pair p,q and
        // their common supporting plane prove distance135 independently.
        // This is the negative-Q ellipse family; the two inside fixtures
        // above exercise Q=0 and Q>0 respectively.
        bool hit = Contact(new Vector3d(706, 608, 0),
            (Fixed64)135 + Fixed64.FromRaw(radiusOffset), false, false,
            permute, true, out FixedContactAnchors contact);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(radiusOffset), contact.Depth);
            Assert.Equal(Permute(Normal, permute), contact.Normal);
            Assert.False(contact.DepthIsClamped);
        }
    }

    [Theory]
    [InlineData(0L)]
    [InlineData(1L)]
    [InlineData(long.MaxValue)]
    public void CoreExactlyOnRim_PreservesZeroGapAndUnclampedMaximum(long radiusRaw)
    {
        // The core center is exactly p=(625,500,0), and b.n=0. The
        // supporting plane proves zero core gap, so depth is exactly radius.
        Assert.True(Contact(new Vector3d(625, 500, 0), Fixed64.FromRaw(radiusRaw),
            false, false, false, false, out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromRaw(radiusRaw), contact.Depth);
        Assert.Equal(Normal, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CoreEndpointExactlyOnRim_RetainsTheEarlierZeroGapNormal()
    {
        // Center=p+1000*b puts the negative core endpoint at the same rim.
        // Both the side normal and the ellipse normal now have zero gap.
        // Exact ties retain the earlier analytic side direction, not the
        // capsule-interior ellipse direction used when p is inside the core.
        Assert.True(Contact(new Vector3d(1105, 140, 800), Fixed64.Zero,
            false, false, false, false, out FixedContactAnchors contact));

        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CoreInside_ReportsConceptualDepthBeyondMaximum()
    {
        // The independently established inside gap is 135, so adding the
        // largest radius exceeds the output range before any rounding.
        Assert.True(Contact(new Vector3d(544, 392, 0), Fixed64.MaxValue,
            false, false, false, false, out FixedContactAnchors contact));

        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.Equal(Normal, contact.Normal);
        Assert.True(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    public void CoreInside_DistinguishesMaximumDepthFromOneRawOverflow(long offset)
    {
        // The exact gap is 135, although the radius-only search bound uses
        // the larger cylinder radius 625. A bound beyond MaxValue must not
        // imply that the actual depth clamps: compare the exact depth first.
        long gapRaw = ((Fixed64)135).m_rawValue;
        Fixed64 radius = Fixed64.FromRaw(long.MaxValue - gapRaw + offset);
        Assert.True(Contact(new Vector3d(544, 392, 0), radius,
            false, false, false, false, out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromRaw(offset < 0 ? long.MaxValue - 1 : long.MaxValue), contact.Depth);
        Assert.Equal(Normal, contact.Normal);
        Assert.Equal(offset > 0, contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReflectedCoreInside_ReflectsTheUniqueNormal(bool permute)
    {
        // Both centered shapes are centrally symmetric. Reflecting the
        // relative center preserves the 135 gap and reverses its unique normal.
        Assert.True(Contact(new Vector3d(-544, -392, 0), Fixed64.Zero,
            false, false, permute, false, out FixedContactAnchors contact));

        Assert.Equal((Fixed64)135, contact.Depth);
        Assert.Equal(-Permute(Normal, permute), contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(0L, 2L)]
    [InlineData(1L, 4L)]
    public void OddRawCylinderLength_RoundsHalfRawEllipseDepthToEven(
        long radiusRaw, long expectedDepthRaw)
    {
        // In raw units p=(625,500.5,0), n=(4,3,0)/5 and q=(623,499,0)
        // give p-q=2.5*n. Quaternion ratio(20,0,-9,13) gives the exact
        // capsule direction b=(9,-12,20)/25, perpendicular to n. The
        // projected ellipse has minimum curvature radius 144, so this 2.5
        // inward normal offset has p as its unique closest boundary point.
        // Radius 0/1 gives depths 2.5/3.5, exercising both nearest-even ties.
        const long scale = 168_462_477L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(20 * scale), Fixed64.Zero,
            Fixed64.FromRaw(-9 * scale), Fixed64.FromRaw(13 * scale));
        Assert.True(rotation.IsNormalized());
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.FromRaw(1001), Fixed64.FromRaw(625),
            new Vector3d(Fixed64.FromRaw(623), Fixed64.FromRaw(499), Fixed64.Zero),
            rotation, Vector3d.Up, Fixed64.FromRaw(2000), Fixed64.FromRaw(radiusRaw),
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromRaw(expectedDepthRaw), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5),
            Fixed64.FromFraction(3, 5), Fixed64.Zero), contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PrincipalAxisQuery_HasAnAnalyticUniqueMinimum(bool negateCapsuleAxis)
    {
        // The projected query is the center of the positive cap ellipse.
        // Only its outward minor vertex is admissible, at distance225.
        // The major/cap alternatives have gaps625 and360 respectively.
        Assert.True(Contact(new Vector3d(0, 500, 0), Fixed64.Zero,
            false, negateCapsuleAxis, false, false, out FixedContactAnchors contact));
        Assert.Equal((Fixed64)225, contact.Depth);
        Assert.True(contact.Normal.Y > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DoubleCriticalPoint_DoesNotReplaceTheShallowerBoundary(bool negateAxis)
    {
        // Quaternion ratio(0,0,-1,2) gives exact b=(4,3,0)/5.
        // On its perpendicular plane the ellipse semiaxes are3125,1875;
        // the positive cap shift is720 and the major query coordinate1024.
        // The derivative has an exact double zero at normal ratio5/4:
        // Q/R=144/625, x/R=1024/3125. It does not change derivative sign.
        // The true minimum remains the major boundary3125-1024=2101.
        const long scale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-scale), Fixed64.FromRaw(2 * scale));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            (Fixed64)1800, (Fixed64)3125,
            new Vector3d(0, 0, 1024), rotation,
            negateAxis ? -Vector3d.Up : Vector3d.Up,
            (Fixed64)10000, Fixed64.Zero, out FixedContactAnchors contact));

        Assert.Equal((Fixed64)2101, contact.Depth);
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    private static Vector3d Normal => new(
        Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero);

    private static bool Contact(Vector3d center, Fixed64 radius,
        bool negateCylinderAxis, bool negateCapsuleAxis, bool permute,
        bool negateQuaternion, out FixedContactAnchors contact)
    {
        FixedQuaternion cylinderRotation = permute
            ? new FixedQuaternion(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half)
            : FixedQuaternion.Identity;
        // Exact composition with the cyclic permutation(x,y,z)->(z,x,y)
        // changes integer quaternion ratio(5,0,-3,4) to(3,6,-2,1).
        int sign = negateQuaternion ? -1 : 1;
        FixedQuaternion capsuleRotation = permute
            ? new FixedQuaternion(Fixed64.FromRaw(sign * 3 * QuaternionScale),
                Fixed64.FromRaw(sign * 6 * QuaternionScale),
                Fixed64.FromRaw(-sign * 2 * QuaternionScale),
                Fixed64.FromRaw(sign * QuaternionScale))
            : new FixedQuaternion(Fixed64.FromRaw(sign * 5 * QuaternionScale), Fixed64.Zero,
                Fixed64.FromRaw(-sign * 3 * QuaternionScale),
                Fixed64.FromRaw(sign * 4 * QuaternionScale));
        Assert.True(cylinderRotation.IsNormalized());
        Assert.True(capsuleRotation.IsNormalized());
        Vector3d translation = new(100, -200, 300);
        return FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            translation, cylinderRotation, negateCylinderAxis ? -Vector3d.Up : Vector3d.Up,
            (Fixed64)1000, (Fixed64)625,
            translation + Permute(center, permute), capsuleRotation,
            negateCapsuleAxis ? -Vector3d.Up : Vector3d.Up,
            (Fixed64)2000, radius, out contact);
    }

    private static Vector3d Permute(Vector3d value, bool permute) =>
        permute ? new Vector3d(value.Z, value.X, value.Y) : value;
}
