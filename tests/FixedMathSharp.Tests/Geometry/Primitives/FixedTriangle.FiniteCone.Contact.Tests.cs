using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteConeContactTests
{
    [Theory]
    [InlineData(-3, 4, 1, 4, 1)]
    [InlineData(7, 8, 1, 8, -1)]
    public void HorizontalFace_ReturnsMinimumExitAndPairedWitnesses(
        int heightNumerator, int heightDenominator, int depthNumerator, int depthDenominator, int normalY)
    {
        Fixed64 y = Fixed64.FromFraction(heightNumerator, heightDenominator);
        FixedTriangle triangle = HorizontalTriangle(y);
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)normalY, Fixed64.Zero), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(depthNumerator, depthDenominator), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(y, first.Y);
        Assert.Equal((Fixed64)(-normalY), second.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
        Assert.Equal(contact.Normal * contact.Depth, first - second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SideFace_RetainsTheBaseRimWitnessRatherThanCenterSample(bool useZ)
    {
        Fixed64 x = Fixed64.FromFraction(3, 4);
        var triangle = new FixedTriangle(new Vector3d(x, (Fixed64)(-16), (Fixed64)(-16)),
            new Vector3d(x, (Fixed64)16, (Fixed64)(-16)), new Vector3d(x, Fixed64.Zero, (Fixed64)16));
        if (useZ)
            triangle = new FixedTriangle(Swap(triangle.A), Swap(triangle.B), Swap(triangle.C));
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(useZ ? Vector3d.Backward : Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Vector3d first = new(x, -Fixed64.One, Fixed64.Zero), second = new(1, -1, 0);
        AssertPoints(contact, useZ ? Swap(first) : first, useZ ? Swap(second) : second);
        static Vector3d Swap(Vector3d value) => new(value.Z, value.Y, value.X);
    }

    [Fact]
    public void VerticalEdge_KZeroStationaryFeatureBeatsTheTriangleFace()
    {
        // The base-rim point (1,-1,0) projects onto AB at (1/2,-1,0).
        // Its depth 1/2 is smaller than the triangle-face candidate 3/5.
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Half, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Half, (Fixed64)2, Fixed64.Zero), new Vector3d(2, 0, 2));
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        AssertPoints(contact, new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.Zero), new Vector3d(1, -1, 0));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void ApexFace_ClassifiesOneRawNeighborsBeforeRounding(long rawOffset, bool hit)
    {
        Assert.Equal(hit, Contact(HorizontalTriangle(Fixed64.One + Fixed64.FromRaw(rawOffset)),
            out FixedContactAnchors contact));
        if (hit)
        {
            Assert.Equal(Fixed64.FromRaw(-rawOffset), contact.Depth);
            Assert.Equal(Vector3d.Down, contact.Normal);
            AssertPoints(contact, new Vector3d(Fixed64.Zero, Fixed64.One + Fixed64.FromRaw(rawOffset), Fixed64.Zero), Vector3d.Up);
        }
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void BaseFace_ClassifiesOneRawNeighborsBeforeRounding(long rawOffset, bool hit)
    {
        Assert.Equal(hit, Contact(HorizontalTriangle(-Fixed64.One + Fixed64.FromRaw(rawOffset)),
            out FixedContactAnchors contact));
        if (hit)
        {
            Assert.Equal(Fixed64.FromRaw(rawOffset), contact.Depth);
            Assert.Equal(Vector3d.Up, contact.Normal);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(rawOffset), Fixed64.Zero), first - second);
        }
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void OddRawHeight_PreservesExactHalfHeightUntilFinalRounding(long rawHeight, long roundedDepth)
    {
        Assert.True(HorizontalTriangle(Fixed64.Zero).TryGetCenteredFiniteConeContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.FromRaw(rawHeight), Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(roundedDepth), contact.Depth);
        var roundedOnly = new FixedPointAnchor(contact.SecondAnchor.Origin, contact.SecondAnchor.Rotation,
            contact.SecondAnchor.LocalPoint, contact.SecondAnchor.LocalDisplacement);
        Assert.NotEqual(0, contact.SecondAnchor.CompareLocalFeature(roundedOnly));
    }

    [Fact]
    public void RadiusZero_UsesTheSameExactSegmentRelationAsCylinder()
    {
        FixedTriangle horizontal = HorizontalTriangle(Fixed64.Zero);
        var vertical = new FixedTriangle(new Vector3d(0, -16, -16), new Vector3d(0, 16, -16), new Vector3d(0, 0, 16));
        foreach (FixedTriangle triangle in new[] { horizontal, vertical })
        {
            Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out FixedContactAnchors cone));
            Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out FixedContactAnchors cylinder));
            Assert.Equal(cylinder, cone);
            Assert.Equal(triangle.Equals(horizontal) ? Fixed64.One : Fixed64.Zero, cone.Depth);
        }
        Assert.False(horizontal.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(30, 0, 0), FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out _));
        // Coplanar separation needs the segment's in-plane support boundaries.
        Assert.False(vertical.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(0, 0, 30), FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out _));
    }

    [Fact]
    public void InvalidFramesDimensionsAndDegenerateTriangles_FollowExistingContracts()
    {
        FixedTriangle triangle = HorizontalTriangle(Fixed64.Zero);
        Assert.Throws<ArgumentException>(() => triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero,
            default, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentException>(() => triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, default, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, -Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, -Fixed64.MinIncrement, out _));
        Assert.False(Contact(new FixedTriangle(Vector3d.Zero, Vector3d.Up, Vector3d.Up), out _));
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void SquaredNormalThreshold_UsesTheEstablishedInclusiveDegeneracyContract(long rawOffset, bool admitted)
    {
        // |Right x (0,0,2^-12)|^2 = 2^-24 = Epsilon exactly.
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw((1L << 20) + rawOffset)));
        Assert.Equal(!admitted, triangle.IsDegenerate);
        Assert.Equal(admitted, Contact(triangle, out FixedContactAnchors contact));
        if (admitted)
            Assert.True(contact.Depth > Fixed64.Zero);
    }

    private static FixedTriangle HorizontalTriangle(Fixed64 y) =>
        new(new Vector3d((Fixed64)(-16), y, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, y, (Fixed64)(-16)), new Vector3d(Fixed64.Zero, y, (Fixed64)16));

    private static bool Contact(FixedTriangle triangle, out FixedContactAnchors contact) =>
        triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out contact);

    private static void AssertPoints(FixedContactAnchors contact, Vector3d first, Vector3d second)
    {
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualFirst));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d actualSecond));
        Assert.Equal(first, actualFirst);
        Assert.Equal(second, actualSecond);
    }
}
