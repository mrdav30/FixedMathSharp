using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteCylinderTests
{
    [Fact]
    public void Cylinder_VertexBeyondRim_RejectsAdmittedSmoothCornerSeparator()
    {
        // Every vertex has x+y>=21/2, whereas the cylinder has x+y<=10.
        // Both cardinal intervals overlap; the nearest corner residual at A
        // is admitted by both incident triangle edges.
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.FromFraction(21, 4), Fixed64.FromFraction(21, 4), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(19, 4), (Fixed64)6, Fixed64.Zero),
            new Vector3d((Fixed64)6, Fixed64.FromFraction(19, 4), Fixed64.Zero));
        Assert.False(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out _));
    }

    [Fact]
    public void Cylinder_CapPoleAtTriangleVertex_PreservesEachSupportMask()
    {
        Vector3d point = new(Fixed64.Zero, Fixed64.FromFraction(19, 4), Fixed64.Zero);
        var triangle = new FixedTriangle(point, new Vector3d(1, 6, 0), new Vector3d(0, 6, 1));
        for (int order = 0; order < 3; order++)
        {
            AssertCapWitness(triangle, point, new Vector3d(0, 5, 0), false);
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Cylinder_CapPoleAtSupportEdge_ClampsBothProjectionEndpoints(bool reverse)
    {
        Vector3d a = new((Fixed64)2, Fixed64.FromFraction(19, 4), (Fixed64)2);
        Vector3d b = new((Fixed64)3, Fixed64.FromFraction(19, 4), (Fixed64)2);
        var triangle = reverse ? new FixedTriangle(b, a, new Vector3d(2, 6, 3))
            : new FixedTriangle(a, b, new Vector3d(2, 6, 3));
        AssertCapWitness(triangle, a, new Vector3d(2, 5, 2), false);
    }

    [Fact]
    public void Cylinder_CapPoleAtSupportEdge_ProjectsIntoEdgeInterior()
    {
        Fixed64 y = Fixed64.FromFraction(19, 4);
        var triangle = new FixedTriangle(new Vector3d(-Fixed64.One, y, (Fixed64)2),
            new Vector3d(Fixed64.One, y, (Fixed64)2), new Vector3d(0, 6, 3));
        AssertCapWitness(triangle, new Vector3d(Fixed64.Zero, y, (Fixed64)2), new Vector3d(0, 5, 2), false);
    }

    [Fact]
    public void Cylinder_RimAtTriangleVertex_PreservesEachSupportMask()
    {
        // The cylinder support for n=(3,5,4) is p=(3,5,4), with n.p=50.
        // A and B have n.v=125/2, so the triangle touches only at C. Their
        // meridional edge supplies this analytic principal direction.
        Vector3d point = new(3, 5, 4);
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.FromFraction(43, 4), Fixed64.FromFraction(5, 4), (Fixed64)6),
            new Vector3d(Fixed64.FromFraction(19, 4), Fixed64.FromFraction(45, 4), (Fixed64)(-2)), point);
        for (int order = 0; order < 3; order++)
        {
            Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
                out FixedContactAnchors contact, out bool capFace));
            Assert.False(capFace);
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(point, first); Assert.Equal(point, second);
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        }
    }

    [Theory]
    [InlineData(0L, 0L)]
    [InlineData(1L, 2L)]
    public void Cylinder_AnalyticRimWitnessHalfRawTie_RoundsToEven(long offsetRaw, long expectedRaw)
    {
        // The sole touch is the edge midpoint (5 raw,1/2 raw,0), since
        // x+2y=6 raw is the cylinder support plane. Whole-unit endpoints keep
        // the triangle above the established degeneracy threshold. Translating
        // one raw unit changes the floor parity, not the geometry.
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)(-2) + Fixed64.FromRaw(4), Fixed64.One + Fixed64.FromRaw(offsetRaw + 1), Fixed64.Zero),
            new Vector3d((Fixed64)2 + Fixed64.FromRaw(6), -Fixed64.One + Fixed64.FromRaw(offsetRaw), Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2 + Fixed64.FromRaw(offsetRaw), Fixed64.Zero));
        var center = new Vector3d(Fixed64.Zero, Fixed64.FromRaw(offsetRaw), Fixed64.Zero);
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.MinIncrement, Fixed64.FromRaw(5), out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        var expected = new Vector3d(Fixed64.FromRaw(5), Fixed64.FromRaw(expectedRaw), Fixed64.Zero);
        Assert.Equal(expected, first); Assert.Equal(expected, second);
    }

    [Theory]
    [InlineData(0L, 0L)]
    [InlineData(1L, 2L)]
    public void Cylinder_RootRimWitnessHalfRawTie_RoundsToEven(long offsetRaw, long expectedRaw)
    {
        Fixed64 x = (Fixed64)3, y = (Fixed64)5 + Fixed64.FromRaw(offsetRaw), z = (Fixed64)4;
        Fixed64 epsilon = Fixed64.MinIncrement;
        var triangle = new FixedTriangle(new Vector3d(x - (Fixed64)2 - epsilon, y + Fixed64.One + epsilon, z - (Fixed64)2 - epsilon),
            new Vector3d(x + (Fixed64)2 + epsilon, y - Fixed64.One, z + (Fixed64)2 + epsilon),
            new Vector3d(x + (Fixed64)3, y + (Fixed64)14 + epsilon, z + (Fixed64)4));
        var center = new Vector3d(Fixed64.Zero, Fixed64.FromRaw(offsetRaw), Fixed64.Zero);
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, (Fixed64)10 + epsilon, (Fixed64)5, out FixedContactAnchors contact));
        // The edge direction (2,-1,2) is perpendicular to n=(3,14,4).
        // Its exact midpoint p=(3,5+1/2 raw,4) is the sole rim touch and the
        // genuine interior chart root t=2/7, not a principal direction.
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        var expected = new Vector3d(x, (Fixed64)5 + Fixed64.FromRaw(expectedRaw), z);
        Assert.Equal(expected, first); Assert.Equal(expected, second);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void Cylinder_CapFace_ProjectsIntoEveryVertexAndEdgeRegion(int order, bool edgeRegion)
    {
        Fixed64 y = Fixed64.FromFraction(19, 4);
        Vector3d a = edgeRegion ? new((Fixed64)(-1), y, (Fixed64)2) : new((Fixed64)2, y, (Fixed64)2);
        Vector3d b = edgeRegion ? new(Fixed64.One, y, (Fixed64)2) : new((Fixed64)3, y, (Fixed64)2);
        Vector3d c = edgeRegion ? new(Fixed64.Zero, y, (Fixed64)3) : new((Fixed64)2, y, (Fixed64)3);
        var triangle = new FixedTriangle(a, b, c);
        for (int turn = 0; turn < order; turn++)
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        Vector3d expected = edgeRegion ? new(Fixed64.Zero, y, (Fixed64)2) : a;
        AssertCapWitness(triangle, expected, new Vector3d(expected.X, (Fixed64)5, expected.Z), true);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Cylinder_ObtuseCapFace_RejectsWrongEdgeVoronoiRegions(int variant)
    {
        Fixed64 y = Fixed64.FromFraction(19, 4);
        Vector3d a, b, c, expected;
        if (variant == 2)
        {
            a = new Vector3d(Fixed64.Half, y, Fixed64.FromFraction(1, 4));
            b = new Vector3d(Fixed64.FromFraction(3, 2), y, Fixed64.FromFraction(1, 4));
            c = new Vector3d(-Fixed64.Half, y, Fixed64.FromFraction(5, 4));
            expected = new Vector3d(Fixed64.FromFraction(3, 8), y, Fixed64.FromFraction(3, 8));
        }
        else
        {
            a = new Vector3d((Fixed64)(-2), y, Fixed64.Half);
            b = new Vector3d(-Fixed64.One, y, Fixed64.Half);
            c = new Vector3d(Fixed64.Zero, y, Fixed64.FromFraction(3, 2));
            expected = new Vector3d(-Fixed64.FromFraction(3, 4), y, Fixed64.FromFraction(3, 4));
            if (variant == 1)
                (b, c) = (c, b);
        }
        AssertCapWitness(new FixedTriangle(a, b, c), expected,
            new Vector3d(expected.X, (Fixed64)5, expected.Z), true);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void Cylinder_SideFace_AxialBoundaryRetainsExactCapSupport(int sign)
    {
        var triangle = new FixedTriangle(new Vector3d(4, sign * -10, -4),
            new Vector3d(4, sign * 10, -4), new Vector3d(4, 0, 4));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal((Fixed64)4, first.X); Assert.Equal((Fixed64)5, second.X);
        Assert.Equal(Fixed64.Zero, first.Z); Assert.Equal(Fixed64.Zero, second.Z);
        Assert.Equal(first.Y, second.Y); Assert.Equal((Fixed64)5, FixedMath.Abs(first.Y));
    }

    [Fact]
    public void Cylinder_EdgeThroughCapCenter_RetainsKZeroPrincipalDirections()
    {
        // For AB through the top cap center, P is identically zero in its
        // normal plane and every quartic stationary root has K=0. The unit
        // ball at B lies in the cylinder, and the top cap attains depth one.
        var triangle = new FixedTriangle(new Vector3d(1, 6, 1), new Vector3d(-1, 4, -1), new Vector3d(0, 5, -1));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(new Vector3d(-1, 4, -1), first);
    }

    [Fact]
    public void Cylinder_RimStationaryRootOnTriangleFaceBoundary_RetainsFaceWinner()
    {
        FixedTriangle source = CreateInteriorRimTriangle();
        Vector3d q = new(Fixed64.FromFraction(765, 256), Fixed64.FromFraction(317, 64), Fixed64.FromFraction(255, 64));
        // The certified support edge is unchanged, but the third vertex is
        // now tangent to n=(-3,-12,-4)/13. The stationary edge root coincides
        // exactly with the earlier triangle-face feature, not a new minimum.
        var triangle = new FixedTriangle(source.A, source.B,
            q + new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, -Fixed64.FromFraction(3, 16)));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact, out bool capFace));
        Assert.False(capFace);
        Assert.Equal(new Vector3d(-Fixed64.FromFraction(3, 13), -Fixed64.FromFraction(12, 13), -Fixed64.FromFraction(4, 13)), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(13, 256), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(q, first); Assert.Equal(new Vector3d(3, 5, 4), second);
    }

    private static void AssertCapWitness(FixedTriangle triangle, Vector3d expectedFirst,
        Vector3d expectedSecond, bool expectedCapFace)
    {
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact, out bool capFace));
        Assert.Equal(expectedCapFace, capFace);
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(expectedFirst, first); Assert.Equal(expectedSecond, second);
    }
}
