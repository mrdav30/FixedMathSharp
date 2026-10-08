using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleFiniteConePatchFaceTests
{
    [Theory]
    [InlineData(-1, false)]
    [InlineData(-1, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    public void HorizontalPatch_InternalDiagonalDoesNotShortenExactFaceExit(int side, bool reverse)
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)2);
        FixedTriangle seed = Seed(vertices, reverse);
        int[] boundary = Boundary(reverse);
        Vector3d center = Vector3d.Up * (side * Fixed64.Quarter);

        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, boundary, center, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up * (Fixed64)side, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Zero, first);
        Assert.Equal(-contact.Normal * Fixed64.Quarter, second);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 1)]
    public void CardinalSidePatch_CertifiesBaseDiameterAndKeepsBaseRimWitness(bool useZ, int side)
    {
        Vector3d[] vertices =
        {
            new(0, -2, -2), new(0, 2, -2), new(0, 2, 2), new(0, -2, 2)
        };
        if (useZ)
        {
            for (int i = 0; i < vertices.Length; i++)
                vertices[i] = new Vector3d(vertices[i].Z, vertices[i].Y, vertices[i].X);
        }
        Vector3d normal = (useZ ? Vector3d.Forward : Vector3d.Right) * (Fixed64)side;
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(new FixedTriangle(vertices[0], vertices[1], vertices[3]),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            normal * Fixed64.Quarter, FixedQuaternion.Identity, Fixed64.One, Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(normal, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Down * Fixed64.Half, first);
        Assert.Equal(first - normal * Fixed64.Quarter, second);
    }

    [Fact]
    public void ObliqueApexLimitedFace_CertifiesActualAxialChord()
    {
        // X+Y=0. The apex and base center lie on opposite plane sides with
        // clearances 1/(4sqrt(2)) and 3/(4sqrt(2)). The full perimeter is far
        // from both projected endpoints, including the diagonal between seeds.
        Vector3d[] vertices =
        {
            new(-2, 2, -2), new(2, -2, -2), new(2, -2, 2), new(-2, 2, 2)
        };
        var seed = new FixedTriangle(vertices[0], vertices[1], vertices[3]);
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false), Vector3d.Down * Fixed64.Quarter,
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(-1, -1, 0).Normalized, contact.Normal);
        Fixed64 expected = Fixed64.Quarter / FixedMath.Sqrt(Fixed64.Two);
        Assert.True(FixedMath.Abs(contact.Depth - expected) <= Fixed64.MinIncrement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(-Fixed64.One / (Fixed64)8, Fixed64.One / (Fixed64)8, Fixed64.Zero), first);
        Assert.Equal(Vector3d.Up * Fixed64.Quarter, second);
    }

    [Fact]
    public void ObliqueFace_WhenAxialChordIsTooShort_WholeProjectionStillCertifies()
    {
        Vector3d[] vertices =
        {
            new(-2, 2, -2), new(2, -2, -2), new(2, -2, 2), new(-2, 2, 2)
        };
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(
            new FixedTriangle(vertices[0], vertices[1], vertices[3]), Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false), Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(-1, -1, 0).Normalized, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(3, 4) / FixedMath.Sqrt(Fixed64.Two))
            <= Fixed64.MinIncrement);
    }

    [Fact]
    public void ObliqueChord_WhenOnlyBaseProjectionApproachesPerimeter_DeclinesCertificate()
    {
        Fixed64 right = Fixed64.FromFraction(9, 20);
        Vector3d[] vertices =
        {
            new(-Fixed64.Two, Fixed64.Two, -Fixed64.Two),
            new(right, -right, -Fixed64.Two),
            new(right, -right, Fixed64.Two),
            new(-Fixed64.Two, Fixed64.Two, Fixed64.Two)
        };
        // The apex projection (-1/8,1/8,0) has ample clearance, but the
        // base-center projection (3/8,-3/8,0) is too close to the right edge.
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            Vector3d.Down * Fixed64.Quarter, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ExposedBoundaryCloserThanFaceDepth_DeclinesCertificate()
    {
        Vector3d[] vertices = HorizontalQuad(Fixed64.One / (Fixed64)8);
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            Vector3d.Down * Fixed64.Quarter, FixedQuaternion.Identity, Fixed64.One, Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void SeedProjectionOutsideTriangle_DeclinesCertificate()
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)4);
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d((Fixed64)(-2), -Fixed64.Quarter, (Fixed64)2), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void FaceSeparatedFromCone_DeclinesCertificate()
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)2);
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            Vector3d.Down * Fixed64.Two, FixedQuaternion.Identity, Fixed64.One, Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void MissingPerimeterOrDegenerateSeed_DeclinesCertificate()
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)2);
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, ReadOnlySpan<int>.Empty,
            Vector3d.Down * Fixed64.Quarter, FixedQuaternion.Identity, Fixed64.One, Fixed64.Half,
            out FixedContactAnchors missing));
        Assert.Equal(default, missing);
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(
            new FixedTriangle(vertices[0], vertices[0], vertices[1]), Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false), Vector3d.Down * Fixed64.Quarter,
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors degenerate));
        Assert.Equal(default, degenerate);
    }

    [Fact]
    public void ConcaveLInterior_OutsideBoundaryHalfplaneKernelStillCertifies()
    {
        Vector3d[] vertices =
        {
            new(-4, 0, -4), new(4, 0, -4), new(4, 0, -1),
            new(-1, 0, -1), new(-1, 0, 4), new(-4, 0, 4)
        };
        int[] boundary = {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 0};
        // The upper-left arm lies outside the inward halfplane of edge 2->3.
        // Its actual radius-1/4 neighborhood remains wholly within the patch.
        var seed = new FixedTriangle(vertices[3], vertices[4], vertices[5]);
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, boundary,
            new Vector3d((Fixed64)(-2), -Fixed64.Quarter, (Fixed64)2),
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
    }

    [Fact]
    public void HoleBoundaryInsideProjectedChordTube_DeclinesCertificate()
    {
        Vector3d[] vertices =
        {
            new(-4, 4, -4), new(4, -4, -4), new(4, -4, 4), new(-4, 4, 4),
            new(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half),
            new(Fixed64.Quarter, -Fixed64.Quarter, -Fixed64.Half),
            new(Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Half),
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.Half)
        };
        int[] boundary = {0, 1, 1, 2, 2, 3, 3, 0, 5, 4, 6, 5, 7, 6, 4, 7};
        var seed = new FixedTriangle(vertices[0], vertices[4], vertices[7]);
        // The apex projection starts left of the hole, while the base-center
        // projection ends right of it. Endpoint disk tests alone are insufficient.
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, boundary, Vector3d.Down * Fixed64.Quarter,
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void NonCardinalBaseRimFace_UsesExactWholeProjectionCertificate()
    {
        Vector3d[] vertices =
        {
            new(-2, -2, 2), new(2, -2, -2), new(2, 2, -2), new(-2, 2, 2)
        };
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(1, 0, 1).Normalized, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - (Fixed64.Half - Fixed64.Half / FixedMath.Sqrt(Fixed64.Two)))
            <= Fixed64.MinIncrement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Down * Fixed64.Half, first);
        Assert.True(second.X < Fixed64.Zero && second.Z < Fixed64.Zero);
    }

    [Fact]
    public void GeneratorFaceTie_KeepsExactApexSupportAndFallsBackFromShortChord()
    {
        Vector3d[] vertices =
        {
            new(-2, 4, -2), new(2, -4, -2), new(2, -4, 2), new(-2, 4, 2)
        };
        var seed = new FixedTriangle(vertices[0], vertices[1], vertices[3]);
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false), Vector3d.Up * Fixed64.Half,
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(-2, -1, 0).Normalized, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.One / FixedMath.Sqrt((Fixed64)5))
            <= Fixed64.MinIncrement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero), first);
        Assert.Equal(Vector3d.Up, second);
    }

    [Fact]
    public void TiltedCone_FaceWitnessMayLieAcrossTheSeedDiagonal()
    {
        Vector3d[] vertices =
        {
            new(-2, 0, -2), new(2, 0, -2), new(2, 0, 2), new(-2, 0, 2),
            new(0, 0, -2), new(0, 0, 2)
        };
        var seed = new FixedTriangle(vertices[0], vertices[4], vertices[5]);
        var coneRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d(Fixed64.FromFraction(-3, 5), -Fixed64.Quarter, Fixed64.Zero),
            coneRotation, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        // The center projects within a seed whose maximum X is zero. The
        // top base-rim support projects on the neighboring triangle instead.
        Assert.True(first.X > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, first.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
        Assert.Equal(contact.Depth, second.Y);

        Vector3d[] bounds = { new(-2, 0, -2), new(2, 0, 2) };
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d(Fixed64.FromFraction(-3, 5), -Fixed64.Quarter, Fixed64.Zero),
            coneRotation, Fixed64.One, Fixed64.Half, out FixedContactAnchors bounded, bounds));
        Assert.Equal(contact, bounded);
    }

    [Fact]
    public void SubdividedTiltedCone_ProjectionFitsPatchEvenWhenEnclosingBallDoesNot()
    {
        Vector3d[] vertices =
        {
            new(-2, 0, -2), new(2, 0, -2), new(2, 0, 2), new(-2, 0, 2),
            new(-Fixed64.FromFraction(3, 2), Fixed64.Zero, -Fixed64.Half),
            new(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new(-Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)
        };
        var seed = new FixedTriangle(vertices[4], vertices[5], vertices[6]);
        var coneRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Vector3d[] bounds = { new(-2, 0, -2), new(2, 0, 2) };
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d(Fixed64.FromFraction(-28, 25), -Fixed64.Quarter, Fixed64.Zero),
            coneRotation, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact, bounds));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(9, 100)) <= Fixed64.MinIncrement);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.Zero, first.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
        Assert.Equal(contact.Depth, second.Y);
        Assert.True(first.X > -Fixed64.One);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PerimeterSupportAtGeneratorEquality_UsesPointChartWithoutTriangleFeatures(bool nearBoundary)
    {
        // The patch plane is Z=0. Its upper edge Y=2X has normal
        // (2,-1,0), which is exactly a generator direction for H=1,R=1/2.
        Vector3d[] vertices =
        {
            new(-4, -8, 0), new(4, 8, 0), new(4, 2, 0), new(-4, -14, 0)
        };
        Fixed64 y = nearBoundary ? Fixed64.FromFraction(-23, 4) : (Fixed64)(-3);
        var seed = nearBoundary
            ? new FixedTriangle(vertices[0], vertices[2], vertices[3])
            : new FixedTriangle(vertices[0], vertices[1], vertices[2]);
        // The base-center projection lies outside the seed, so the axial
        // chord declines. The whole projection admits the interior case and
        // rejects the case crossing the actual lower perimeter Y=2X-6.
        bool admitted = TriangleConeContact.TryGetPatchFaceContact(seed, Vector3d.Zero,
            FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d(Fixed64.Zero, y, Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out FixedContactAnchors contact);
        Assert.Equal(!nearBoundary, admitted);
        if (admitted)
        {
            Assert.Equal(Vector3d.Forward, contact.Normal);
            Assert.Equal(Fixed64.Quarter, contact.Depth);
        }
        else
            Assert.Equal(default, contact);
    }

    [Theory]
    [InlineData(2L)]
    [InlineData(5L)]
    public void AxialChord_RetainsTouchingAndOddRawHeights(long heightRaw)
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)2);
        Fixed64 offset = Fixed64.FromRaw(-1);
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d(Fixed64.Zero, offset, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.FromRaw(heightRaw), Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromRaw(heightRaw == 2 ? 0 : 2), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.FromRaw(heightRaw == 2 ? 0 : 2), second.Y);
    }

    [Fact]
    public void PatchCertificate_PreservesRigidPoseAndExtremeOrigins()
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)2);
        var origin = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.MinValue);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero);
        var center = origin + Vector3d.Up * Fixed64.Quarter;
        Assert.True(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            origin, rotation, vertices, Boundary(false), center, rotation,
            Fixed64.One, Fixed64.Zero, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(origin, first);
        Assert.Equal(origin + Vector3d.Down * Fixed64.Quarter, second);
    }

    [Fact]
    public void TouchingAtExposedBoundary_DeclinesZeroClearanceCertificate()
    {
        Vector3d[] vertices = HorizontalQuad((Fixed64)2);
        Assert.False(TriangleConeContact.TryGetPatchFaceContact(Seed(vertices, false),
            Vector3d.Zero, FixedQuaternion.Identity, vertices, Boundary(false),
            new Vector3d((Fixed64)2, -Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    private static Vector3d[] HorizontalQuad(Fixed64 halfExtent) =>
    new Vector3d[]
    {
        new(-halfExtent, Fixed64.Zero, -halfExtent),
        new(halfExtent, Fixed64.Zero, -halfExtent),
        new(halfExtent, Fixed64.Zero, halfExtent),
        new(-halfExtent, Fixed64.Zero, halfExtent)
    };

    private static FixedTriangle Seed(Vector3d[] vertices, bool reverse) => reverse
        ? new FixedTriangle(vertices[0], vertices[2], vertices[1])
        : new FixedTriangle(vertices[0], vertices[1], vertices[2]);

    private static int[] Boundary(bool reverse) => reverse
        ? new int[] {1, 0, 2, 1, 3, 2, 0, 3}
        : new int[] {0, 1, 1, 2, 2, 3, 3, 0};
}
