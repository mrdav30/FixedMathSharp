using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleFiniteConeConvexPatchTests
{
    [Theory]
    [InlineData(-1, false)]
    [InlineData(-1, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    public void SmallQuad_CompleteNormalFanIgnoresInternalDiagonal(int side, bool reverse)
    {
        Fixed64 extent = Fixed64.One / (Fixed64)5;
        Vector3d[] vertices = Quad(extent);
        int[] corners = reverse ? new[] { 0, 3, 2, 1 } : new[] { 0, 1, 2, 3 };
        var seed = new FixedTriangle(vertices[corners[0]], vertices[corners[1]], vertices[corners[2]]);
        Assert.True(TriangleConeContact.TryGetConvexPatchContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Up * (side * Fixed64.Quarter), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up * (Fixed64)side, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Zero, first);
        Assert.Equal(first - contact.Normal * contact.Depth, second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TrueCorner_RemainsACompleteCurvedFeature(bool reverse)
    {
        Fixed64 extent = Fixed64.One / (Fixed64)5;
        Vector3d[] vertices = Quad(extent);
        int[] corners = reverse ? new[] { 0, 3, 2, 1 } : new[] { 0, 1, 2, 3 };
        var seed = new FixedTriangle(vertices[0], vertices[2], vertices[3]);
        Fixed64 offset = (Fixed64)7 / (Fixed64)25;
        Assert.True(TriangleConeContact.TryGetConvexPatchContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            vertices, corners, new Vector3d(offset, -Fixed64.Quarter, offset), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.True(contact.Normal.X > Fixed64.Zero && contact.Normal.Y < Fixed64.Zero);
        Assert.Equal(contact.Normal.X, contact.Normal.Z);
        Fixed64 radial = FixedMath.Sqrt(Fixed64.Two) * (offset - extent);
        Fixed64 expected = (Fixed64.One / (Fixed64)8 - radial)
            / FixedMath.Sqrt(Fixed64.One + Fixed64.Quarter);
        Assert.True(FixedMath.Abs(contact.Depth - expected) <= Fixed64.MinIncrement * (Fixed64)2);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(new Vector3d(extent, Fixed64.Zero, extent), first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Vector3d paired = first - contact.Normal * contact.Depth;
        Assert.True((paired - second).Magnitude <= Fixed64.MinIncrement * (Fixed64)3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(4)]
    public void FaceGenerator_UsesCompleteFanBeyondWinningCornerEar(int firstCorner)
    {
        // 2X+Y=0, with a generator-limited support. The seed near the base's
        // outer Z edge intersects the cone but misses the generator at Z=0.
        // The whole convex face, rather than that seed, supplies its witness.
        Vector3d[] vertices =
        {
            new(-2, 4, -2), new(2, -4, -2), new(3, -6, 0),
            new(2, -4, 2), new(-2, 4, 2), new(-3, 6, 0)
        };
        var corners = new int[vertices.Length];
        for (int index = 0; index < corners.Length; index++)
            corners[index] = (index + firstCorner) % corners.Length;
        Fixed64 tiny = Fixed64.One / (Fixed64)20;
        var seed = new FixedTriangle(new Vector3d(-tiny, tiny * (Fixed64)2, (Fixed64)2 / (Fixed64)5),
            new Vector3d(tiny, -tiny * (Fixed64)2, (Fixed64)2 / (Fixed64)5),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)9 / (Fixed64)20));
        Assert.True(TriangleConeContact.TryGetContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out _));
        Assert.True(TriangleConeContact.TryGetConvexPatchContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.One / FixedMath.Sqrt((Fixed64)5)) <= Fixed64.MinIncrement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.True(FixedMath.Abs(first.Z) <= Fixed64.MinIncrement);
        Assert.True((first - contact.Normal * contact.Depth - second).Magnitude <= Fixed64.MinIncrement * (Fixed64)3);
        Assert.True(FixedMath.Abs(second.X * (Fixed64)2 + second.Y - Fixed64.One) <= Fixed64.MinIncrement * (Fixed64)3);
    }

    [Theory]
    [InlineData(0, false, 0L, false)]
    [InlineData(1, false, 0L, false)]
    [InlineData(2, false, 0L, false)]
    [InlineData(1, true, 0L, false)]
    [InlineData(1, false, -1L, false)]
    [InlineData(1, false, 0L, true)]
    [InlineData(1, false, 1L, false)]
    public void ObliqueExposedEdge_RetainsExactRimRootAcrossCornerCharts(int firstCorner, bool reverse, long separationRaw, bool touching)
    {
        Vector3d[] vertices =
        {
            new(Fixed64.FromFraction(93, 16), (Fixed64)(-4), Fixed64.FromFraction(5, 4)),
            new(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(-11, 2), Fixed64.FromFraction(-5, 4)),
            new(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(-35, 4), Fixed64.Zero)
        };
        var corners = new int[3];
        for (int index = 0; index < corners.Length; index++)
            corners[index] = (firstCorner + (reverse ? 3 - index : index)) % 3;
        var seed = new FixedTriangle(vertices[0], vertices[1], vertices[2]);
        Vector3d origin = separationRaw == 0 && !touching ? Vector3d.Zero
            : new Vector3d(Fixed64.FromFraction(3, 16) + Fixed64.FromRaw(separationRaw),
                Fixed64.FromFraction(-1, 4), Fixed64.Zero);
        bool hit = TriangleConeContact.TryGetConvexPatchContact(seed, origin, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact);
        Assert.Equal(separationRaw <= 0, hit);
        if (!hit)
            return;
        Assert.Equal(touching ? Fixed64.Zero : separationRaw == 0 ? Fixed64.FromFraction(5, 16) : Fixed64.MinIncrement, contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero), contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        // At the touch translation, shifting the oblique edge left by one
        // raw moves its stationary parameter by -8/25 raw. AB.Z=-5/2 then
        // moves the rim witness by +4/5 raw in Z, rounding to one raw.
        Assert.Equal(new Vector3d((Fixed64)5, (Fixed64)(-5),
            separationRaw < 0 ? Fixed64.MinIncrement : Fixed64.Zero), second);
    }

    [Fact]
    public void CurvedInteriorRimRoot_PreservesMatchedEdgeWitnessAndExactDepth()
    {
        Vector3d[] vertices = InteriorRimVertices();
        int[] corners = { 2, 0, 1 };
        var seed = new FixedTriangle(vertices[0], vertices[1], vertices[2]);
        Assert.True(TriangleConeContact.TryGetConvexPatchContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(13, 256), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(-3, 13), Fixed64.FromFraction(12, 13), Fixed64.FromFraction(-4, 13)), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(765, 256), Fixed64.FromFraction(-317, 64), Fixed64.FromFraction(255, 64)), first);
        Assert.Equal(new Vector3d(3, -5, 4), second);
    }

    [Fact]
    public void CurvedInteriorRimRoot_OneRawBeyondTouchRejectsWholePolygon()
    {
        Vector3d[] vertices = InteriorRimVertices();
        int[] corners = { 2, 0, 1 };
        var seed = new FixedTriangle(vertices[0], vertices[1], vertices[2]);
        // This translation puts AB's exact rim witness at (3,-5,4).
        // One raw lower Y separates it at an interior stationary rim normal;
        // the analytic fan alone still admits the polygon.
        var origin = new Vector3d(Fixed64.FromFraction(3, 256),
            Fixed64.FromFraction(-12, 256) - Fixed64.MinIncrement, Fixed64.FromFraction(4, 256));
        Assert.False(TriangleConeContact.TryGetConvexPatchContact(seed, origin, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out _));
    }

    [Fact]
    public void FaceGenerator_FinalFanTriangleSuppliesSliceWhenEarlierTriangleDeclines()
    {
        Fixed64 top = Fixed64.One / (Fixed64)10;
        Vector3d[] vertices =
        {
            new(-2, 4, -2), new(2, -4, -2),
            new((Fixed64)2, (Fixed64)(-4), top), new((Fixed64)(-2), (Fixed64)4, top)
        };
        int[] corners = { 0, 1, 2, 3 };
        Fixed64 tiny = Fixed64.One / (Fixed64)20;
        var seed = new FixedTriangle(new Vector3d(-tiny, tiny * (Fixed64)2, (Fixed64)(-2) / (Fixed64)5),
            new Vector3d(tiny, -tiny * (Fixed64)2, (Fixed64)(-2) / (Fixed64)5),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-9) / (Fixed64)20));
        // At Z=0, the first fan triangle only reaches X near its right end,
        // beyond the whole projected generator X interval [-2/5,1/10].
        Assert.True(TriangleConeContact.TryGetConvexPatchContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(-2, -1, 0).Normalized, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.One / FixedMath.Sqrt((Fixed64)5)) <= Fixed64.MinIncrement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.True(FixedMath.Abs(first.Z) <= Fixed64.MinIncrement);
        Assert.True((first - contact.Normal * contact.Depth - second).Magnitude <= Fixed64.MinIncrement * (Fixed64)3);
    }

    [Fact]
    public void SmallQuad_ExplicitWitnessBoundsPreserveExactContact()
    {
        Fixed64 extent = Fixed64.One / (Fixed64)5;
        Vector3d[] vertices = Quad(extent);
        int[] corners = { 0, 1, 2, 3 };
        var seed = new FixedTriangle(vertices[0], vertices[1], vertices[2]);
        ReadOnlySpan<Vector3d> bounds = new[]
        {
            new Vector3d(-extent, Fixed64.Zero, -extent), new Vector3d(extent, Fixed64.Zero, extent)
        };
        Assert.True(TriangleConeContact.TryGetConvexPatchContact(seed, Vector3d.Zero, FixedQuaternion.Identity,
            vertices, corners, Vector3d.Down * Fixed64.Quarter, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact, bounds));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Zero, first);
        Assert.Equal(Vector3d.Up * Fixed64.Quarter, second);
    }

    private static Vector3d[] Quad(Fixed64 extent) => new[]
    {
        new Vector3d(-extent, Fixed64.Zero, -extent), new Vector3d(extent, Fixed64.Zero, -extent),
        new Vector3d(extent, Fixed64.Zero, extent), new Vector3d(-extent, Fixed64.Zero, extent)
    };

    private static Vector3d[] InteriorRimVertices() => new[]
    {
        new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(-309, 64), Fixed64.FromFraction(231, 64)),
        new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(-325, 64), Fixed64.FromFraction(279, 64)),
        new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(-365, 64), Fixed64.FromFraction(271, 64))
    };
}
