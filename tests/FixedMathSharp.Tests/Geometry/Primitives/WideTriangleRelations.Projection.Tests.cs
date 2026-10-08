using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class WideTriangleProjectionTests
{
    private static readonly FixedTriangle Triangle = new(
        Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(0, 2, 0));

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
