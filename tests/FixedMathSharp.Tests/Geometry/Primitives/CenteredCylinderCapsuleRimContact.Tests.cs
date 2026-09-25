using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderCapsuleRimContactTests
{
    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void Contact_ZeroRadiusCoreEndpointsMeetWithoutInventingANormal(int sign)
    {
        // The two length-2 perpendicular cores meet at (0,sign,0).
        // Their Minkowski sum is a flat rectangle and the queried center is
        // its corner: the minimum offset depth is exactly the capsule radius.
        // The coincident endpoint is not an additional normal direction.
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Zero,
            new Vector3d(sign, sign, 0), FixedQuaternion.Identity, Vector3d.Right,
            Fixed64.Two, Fixed64.Half, out FixedContactAnchors contact));

        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.Equal(sign * Vector3d.Up, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_CoreInsideCylinderDoesNotExceedAKnownSeparatingTranslation()
    {
        // Quaternion ratio (0,0,-1,2) gives the exact axis (4/5,3/5,0).
        // The core contains (11/20,-9/10,1/4), strictly inside the cylinder.
        // Along n=(3,-4,0)/5, the cylinder support is 7/5, the center
        // projection is 21/20, and the capsule core contributes zero.
        // Including its radius 1/4, a 3/5 translation already separates:
        // the minimum translation cannot exceed that independently proved bound.
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion capsuleRotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One,
            new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.Zero, Fixed64.FromFraction(1, 4)),
            capsuleRotation, Vector3d.Up, (Fixed64)10, Fixed64.FromFraction(1, 4),
            out FixedContactAnchors contact));

        Assert.InRange(contact.Depth.m_rawValue, 0L, Fixed64.FromFraction(3, 5).m_rawValue);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void Contact_PreservesOneRawDepthAtAnObliqueInteriorRim(
        long radiusRawOffset, bool expectedContact)
    {
        // The exact core-to-cylinder distance is five, independently of the
        // capsule radius. This distinguishes an exact one-raw penetration
        // from contact classification based on a rounded normal or depth.
        const long quaternionScale = 607_400_100L;
        FixedQuaternion capsuleRotation = new(
            Fixed64.FromRaw(5 * quaternionScale), Fixed64.Zero,
            Fixed64.FromRaw(-3 * quaternionScale), Fixed64.FromRaw(4 * quaternionScale));
        bool hit = FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One,
            new Vector3d(4, 5, 0), capsuleRotation, Vector3d.Up,
            Fixed64.Two, (Fixed64)5 + Fixed64.FromRaw(radiusRawOffset),
            out FixedContactAnchors contact);

        Assert.Equal(expectedContact, hit);
        if (expectedContact)
        {
            Assert.Equal(Fixed64.FromRaw(radiusRawOffset), contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5),
                Fixed64.FromFraction(4, 5), Fixed64.Zero), contact.Normal);
        }
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(1L, false)]
    public void Contact_ClassifiesObliqueInteriorRimTangencyBeforeRounding(
        long verticalRawOffset, bool expectedContact)
    {
        // This exact quaternion ratio gives the local +Y axis (12,-9,20)/25.
        // At zero offset, the core midpoint q=(4,5,0) is five units from the
        // cylinder rim p=(1,1,0), along n=(3,4,0)/5 perpendicular to the core.
        // The common support plane is 3x+4y=7. Moving q upward by one raw
        // separates that plane; moving it downward makes the squared distance
        // to p equal 25-8e+(544/625)e^2 < 25, with an interior core witness.
        const long quaternionScale = 607_400_100L;
        FixedQuaternion capsuleRotation = new(
            Fixed64.FromRaw(5 * quaternionScale), Fixed64.Zero,
            Fixed64.FromRaw(-3 * quaternionScale), Fixed64.FromRaw(4 * quaternionScale));

        bool hit = FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One,
            new Vector3d((Fixed64)4, (Fixed64)5 + Fixed64.FromRaw(verticalRawOffset), Fixed64.Zero),
            capsuleRotation, Vector3d.Up, Fixed64.Two, (Fixed64)5,
            out _);

        Assert.Equal(expectedContact, hit);
    }

    [Theory]
    [InlineData(false, 5)]
    [InlineData(true, 5)]
    [InlineData(false, 6)]
    [InlineData(true, 6)]
    public void Contact_RetainsTheRimNormalDepthAndSurfaceWitnesses(
        bool interiorCoreFeature, int radiusQuarters)
    {
        // The nearest core point is (43/4, 2, 0), at distance 5/4 from
        // the rim point (10, 1, 0). Its outward unit normal is (3/5, 4/5, 0).
        // A +X core starts there; a +Z core crosses there at its midpoint.
        // Both give the same minimum translation and surface witnesses.
        Vector3d capsuleCenter = new(
            Fixed64.FromFraction(interiorCoreFeature ? 43 : 83, 4),
            Fixed64.Two,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, (Fixed64)10,
            capsuleCenter, FixedQuaternion.Identity,
            interiorCoreFeature ? Vector3d.Forward : Vector3d.Right,
            (Fixed64)20, Fixed64.FromFraction(radiusQuarters, 4),
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromFraction(radiusQuarters - 5, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5), Fixed64.Zero), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d cylinderPoint));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d capsulePoint));
        AssertPointWithinTwoRawUnits(new Vector3d(10, 1, 0), cylinderPoint);
        // (43/4, 2, 0) - radius * (3/5, 4/5, 0), derived independently.
        Vector3d expectedCapsulePoint = radiusQuarters == 5
            ? new Vector3d(10, 1, 0)
            : new Vector3d(Fixed64.FromFraction(197, 20),
                Fixed64.FromFraction(4, 5), Fixed64.Zero);
        AssertPointWithinTwoRawUnits(expectedCapsulePoint, capsulePoint);
    }

    private static void AssertPointWithinTwoRawUnits(Vector3d expected, Vector3d actual)
    {
        // Public anchor coordinates are narrowed after normal normalization
        // and radius multiplication; classification and depth above are exact.
        Assert.InRange(actual.X.m_rawValue, expected.X.m_rawValue - 2, expected.X.m_rawValue + 2);
        Assert.InRange(actual.Y.m_rawValue, expected.Y.m_rawValue - 2, expected.Y.m_rawValue + 2);
        Assert.Equal(expected.Z, actual.Z);
    }
}
