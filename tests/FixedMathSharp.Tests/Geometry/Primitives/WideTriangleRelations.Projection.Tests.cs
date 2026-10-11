using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class WideTriangleProjectionTests
{
    private static readonly FixedTriangle Triangle = new(
        Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(0, 2, 0));

    [Theory]
    [InlineData(4, 1, 8, true, -3)]
    [InlineData(-3, 1, 8, true, 3)]
    [InlineData(0, 1, 8, true, 0)]
    [InlineData(0, 3, 8, false, 0)]
    [InlineData(3, 1, 2, false, 0)]
    public void ProjectedCore_ClipsTheClosedFaceAndChoosesNearestMidpoint(
        int x, int y, int length, bool expected, int axial)
    {
        Vector3d center = new(x, y, 7);
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)length, Fixed64.Zero, -Vector3d.Forward, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)length, Fixed64.Zero, -Vector3d.Forward, 1);
        Assert.Equal(expected, WideTriangleRelations.TryGetProjectedCoreParameter(
            Triangle, Vector3d.Zero, FixedQuaternion.Identity, first, second,
            center, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)length, out var n, out var d));
        if (expected)
            Assert.Equal(Vector3d.Right * (Fixed64)axial, WideTriangleRelations.MaterializeCorePoint(Vector3d.Right, n, d));
    }

    [Theory]
    [InlineData(1, -1, 1, 1, true, 1)]
    [InlineData(1, 2, 1, 3, false, 0)]
    [InlineData(1, 0, 2, 0, true, 1)]
    [InlineData(2, 0, 1, 0, true, 1)]
    [InlineData(-2, 0, -1, 0, true, -1)]
    [InlineData(-1, 0, -2, 0, true, -1)]
    [InlineData(-1, 0, 1, 0, true, 0)]
    [InlineData(3, 0, 4, 0, false, 0)]
    [InlineData(1, 0, 1, 0, true, 1)]
    [InlineData(-1, 0, -1, 0, true, -1)]
    [InlineData(0, 0, 0, 0, true, 0)]
    [InlineData(3, 0, 3, 0, false, 0)]
    public void ProjectedCore_HandlesFiniteEdgesAndVertices(
        int ax, int ay, int bx, int by, bool expected, int axial)
    {
        Vector3d start = new(ax, ay, 7), end = new(bx, by, 7);
        var feature = new FixedTriangle(start, end, end);
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)4, Fixed64.Zero, -Vector3d.Forward, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)4, Fixed64.Zero, -Vector3d.Forward, 1);
        Assert.Equal(expected, WideTriangleRelations.TryGetProjectedCoreParameter(
            feature, Vector3d.Zero, FixedQuaternion.Identity, first, second,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)4, out var n, out var d));
        if (expected)
            Assert.Equal(Vector3d.Right * (Fixed64)axial, WideTriangleRelations.MaterializeCorePoint(Vector3d.Right, n, d));
    }

    [Fact]
    public void ProjectedCore_RetainsAnOddRawEndpointOnATinyFace()
    {
        // At y=one raw this face ends at x=1.5 raw, exactly the core's left
        // endpoint. Rounding endpoints before clipping would lose the contact.
        var triangle = new FixedTriangle(new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(2), Fixed64.FromRaw(2), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(2), Fixed64.Zero));
        Vector3d center = new(Fixed64.FromRaw(4), Fixed64.MinIncrement, Fixed64.One);
        Fixed64 length = Fixed64.FromRaw(5);
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, Vector3d.Right, length, Fixed64.Zero, -Vector3d.Forward, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, Vector3d.Right, length, Fixed64.Zero, -Vector3d.Forward, 1);
        Assert.True(WideTriangleRelations.TryGetProjectedCoreParameter(
            triangle, Vector3d.Zero, FixedQuaternion.Identity, first, second,
            center, FixedQuaternion.Identity, Vector3d.Right, length, out var n, out var d));
        Assert.Equal(-1, n.Sign);
        Signed832 twiceN = WideArithmetic.AddSigned832(n, n);
        Assert.True(WideArithmetic.AddSigned832(twiceN,
            WideArithmetic.MultiplySigned832(d, Signed192.Raw(length))).IsZero);
    }

    [Fact]
    public void CertifiedTinyFace_ShouldMaterializeItsInteriorRatherThanAnEdge()
    {
        Fixed64 unit = Fixed64.FromRaw(1024);
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right * unit, Vector3d.Up * unit);
        Vector3d local = new(unit / (Fixed64)4, unit / (Fixed64)4, Fixed64.One);
        var point = new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, local);
        Assert.True(WideTriangleRelations.ContainsProjection(triangle, Vector3d.Zero, FixedQuaternion.Identity, point));
        FixedPointAnchor closest = WideTriangleRelations.GetClosestPointAnchor(
            triangle, Vector3d.Zero, FixedQuaternion.Identity, point, preserveExactFace: true);
        Assert.Equal(new Vector3d(local.X, local.Y, Fixed64.Zero), closest.LocalPoint);
    }

    [Fact]
    public void ProjectedCore_ShouldRejectDisjointFaceHalfspaceIntervals()
    {
        // The infinite diagonal crosses each edge halfspace separately, but
        // their admitted finite intervals never intersect inside the triangle.
        Vector3d center = new(0, 3, 7), axis = new Vector3d(1, 1, 0).Normalized;
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, axis, (Fixed64)8, Fixed64.Zero, -Vector3d.Forward, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, axis, (Fixed64)8, Fixed64.Zero, -Vector3d.Forward, 1);
        Assert.False(WideTriangleRelations.TryGetProjectedCoreParameter(
            Triangle, Vector3d.Zero, FixedQuaternion.Identity, first, second,
            center, FixedQuaternion.Identity, axis, (Fixed64)8, out _, out _));
    }

    [Fact]
    public void ProjectedCore_ShouldRetainCoincidentLowerFaceBounds()
    {
        Vector3d center = new(-3, -3, 7), axis = new Vector3d(1, 1, 0).Normalized;
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, axis, (Fixed64)12, Fixed64.Zero, -Vector3d.Forward, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(center,
            FixedQuaternion.Identity, axis, (Fixed64)12, Fixed64.Zero, -Vector3d.Forward, 1);
        Assert.True(WideTriangleRelations.TryGetProjectedCoreParameter(
            Triangle, Vector3d.Zero, FixedQuaternion.Identity, first, second,
            center, FixedQuaternion.Identity, axis, (Fixed64)12, out var n, out var d));
        Vector3d projected = center + WideTriangleRelations.MaterializeCorePoint(axis, n, d);
        Assert.True(FixedMath.Abs(projected.X) <= Fixed64.MinIncrement);
        Assert.True(FixedMath.Abs(projected.Y) <= Fixed64.MinIncrement);
    }

    [Theory]
    [InlineData(1, 3, 2)]
    [InlineData(2, 1, 3)]
    [InlineData(1, 2, 3)]
    public void ProjectedCore_ShouldUseTheCompleteCollinearFeature(int a, int b, int c)
    {
        var triangle = new FixedTriangle(new Vector3d(a, 0, 7), new Vector3d(b, 0, 7), new Vector3d(c, 0, 7));
        FixedPointAnchor first = WideGeometry.GetCenteredCapsuleSupportAnchor(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)4, Fixed64.Zero, -Vector3d.Forward, -1);
        FixedPointAnchor second = WideGeometry.GetCenteredCapsuleSupportAnchor(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Right, (Fixed64)4, Fixed64.Zero, -Vector3d.Forward, 1);
        Assert.True(WideTriangleRelations.TryGetProjectedCoreParameter(
            triangle, Vector3d.Zero, FixedQuaternion.Identity, first, second,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)4, out var n, out var d));
        Assert.Equal(Vector3d.Right, WideTriangleRelations.MaterializeCorePoint(Vector3d.Right, n, d));
    }

    [Theory]
    [InlineData(0, 0, true)]
    [InlineData(2, 0, true)]
    [InlineData(0, 2, true)]
    [InlineData(1, 1, true)]
    [InlineData(1, 0, true)]
    [InlineData(-1, 1, false)]
    [InlineData(1, -1, false)]
    [InlineData(2, 1, false)]
    public void ContainsProjection_IsInclusiveAndIgnoresPlaneDistance(int x, int y, bool expected)
    {
        var point = new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(x, y, 7));
        Assert.Equal(expected, WideTriangleRelations.ContainsProjection(
            Triangle, Vector3d.Zero, FixedQuaternion.Identity, point));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainsProjection_CancelsScalarFaceOriginsWithRationalRotation(bool positive)
    {
        Fixed64 edge = positive ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector3d origin = new(edge, edge, edge);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        var point = new FixedPointAnchor(origin, rotation,
            new Vector3d(Fixed64.Half, Fixed64.Half, (Fixed64)7));
        Assert.True(WideTriangleRelations.ContainsProjection(Triangle, origin, rotation, point));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ContainsProjection_RejectsSubRawOutsideEveryEdge(int edge)
    {
        // Exact half-raw residuals materialize to the even boundary coordinate.
        // The projection decision must retain their signed side of each edge.
        Vector3d basePoint = edge switch
        {
            0 => new Vector3d(Fixed64.Zero, Fixed64.Half, (Fixed64)7),
            1 => new Vector3d(Fixed64.Half, Fixed64.Zero, (Fixed64)7),
            _ => new Vector3d(1, 1, 7)
        };
        Vector3d residualDirection = edge == 0 ? Vector3d.Left * Fixed64.MinIncrement
            : edge == 1 ? Vector3d.Down * Fixed64.MinIncrement
            : Vector3d.Up * Fixed64.MinIncrement;
        FixedPointAnchorTerm3d term = FixedPointAnchorTerm3d.CreateRadialSupport(
            residualDirection, Fixed64.Half, Vector3d.Zero);
        var point = new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity,
            basePoint, Vector3d.Zero, term);
        Assert.False(WideTriangleRelations.ContainsProjection(
            Triangle, Vector3d.Zero, FixedQuaternion.Identity, point));
    }

    [Fact]
    public void ContainsProjection_UsesIndependentLocalTranslation()
    {
        var point = new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(-1, 0, 7)).WithLocalTranslation(new Vector3d(2, 0, 0));
        Assert.True(WideTriangleRelations.ContainsProjection(
            Triangle, Vector3d.Zero, FixedQuaternion.Identity, point));
    }

    [Fact]
    public void ContainsProjection_RejectsDegenerateTriangle()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Right * (Fixed64)2);
        var point = new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero);
        Assert.False(WideTriangleRelations.ContainsProjection(
            triangle, Vector3d.Zero, FixedQuaternion.Identity, point));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosestPointAnchor_RetainsTranslationAndHalfRawResidualAtScalarFace(bool positive)
    {
        Fixed64 edge = positive ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector3d origin = new(edge, edge, edge);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        // At x = one raw + half a raw, nearest-even materialization is two raw.
        // Omitting either the translation or the residual instead returns zero/one.
        FixedPointAnchorTerm3d term = FixedPointAnchorTerm3d.CreateRadialSupport(
            Vector3d.Right * Fixed64.MinIncrement, Fixed64.Half, Vector3d.Zero);
        var point = new FixedPointAnchor(origin, rotation,
            new Vector3d(Fixed64.Zero, Fixed64.Half, (Fixed64)7), Vector3d.Zero, term)
            .WithLocalTranslation(Vector3d.Right * Fixed64.MinIncrement);
        FixedPointAnchor closest = Triangle.GetClosestPointAnchor(origin, rotation, point);
        Assert.Equal(Fixed64.FromRaw(2), closest.LocalPoint.X);
        Assert.Equal(Fixed64.Half, closest.LocalPoint.Y);
        Assert.Equal(Fixed64.Zero, closest.LocalPoint.Z);
    }
}
