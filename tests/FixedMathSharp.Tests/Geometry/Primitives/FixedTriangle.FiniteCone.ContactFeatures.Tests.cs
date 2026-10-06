using FixedMathSharp.Geometry;
using FixedMathSharp.Random;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteConeContactTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReciprocalRimChartBoundary_KeepsTheSameExactWitness(bool reverse)
    {
        Vector3d n = new(-3, 4, 4), e = new(4, 1, 2), rim = new(3, -5, -4);
        Vector3d point = rim + n * Fixed64.FromFraction(1, 64);
        var source = new FixedTriangle(point - e * Fixed64.Quarter, point + e * Fixed64.Quarter, point - n);
        FixedTriangle triangle = reverse ? new FixedTriangle(source.A, source.C, source.B) : source;
        for (int order = 0; order < 3; order++)
        {
            Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
            // Both reciprocal charts select t=1. AB minus the base disk
            // contains the radius-sqrt(41)/64 ball: its disk trust-region
            // multiplier 64 exceeds the radial slice map's spectral bound 21.
            // These raw constants independently round sqrt(41)/64 and n/sqrt(41).
            Assert.Equal(Fixed64.FromRaw(429706394L), contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromRaw(-2012283599L), Fixed64.FromRaw(2683044799L),
                Fixed64.FromRaw(2683044799L)), contact.Normal);
            Assert.False(contact.DepthIsClamped);
            AssertPoints(contact, point, rim);
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        }
    }

    [Fact]
    public void FullDomainSideFace_RetainsExactMaximumDepthWithoutClamping()
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.MinValue),
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.MaxValue),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero));
        Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.MaxValue, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Left, contact.Normal);
        AssertPoints(contact, new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, -Fixed64.Half, Fixed64.Zero));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GeneratorEndpointTouch_RetainsTheExactSharedEndpoint(bool apex)
    {
        Vector3d endpoint = apex ? new Vector3d(0, 1, 0) : new Vector3d(-1, -1, 0);
        var triangle = new FixedTriangle(endpoint, endpoint + new Vector3d(1, 2, 1),
            endpoint + new Vector3d(-1, -2, 1));
        // The plane 2X-Y=-1 supports the negative-X generator. Z is positive
        // everywhere except this vertex, so the only shared point is its endpoint.
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        const long reciprocalRootFiveRaw = 1920767767L;
        Assert.Equal(new Vector3d(Fixed64.FromRaw(2 * reciprocalRootFiveRaw),
            Fixed64.FromRaw(-reciprocalRootFiveRaw), Fixed64.Zero), contact.Normal);
        AssertPoints(contact, endpoint, endpoint);
    }

    [Fact]
    public void TangentGeneratorNormalCircle_KeepsItsDoubleIntersection()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(1, 2, 0), Vector3d.Forward);
        // AB is parallel to a generator. Its normal plane is tangent to the
        // generator normal circle, with a repeated root rather than two crossings.
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(1920767767L), contact.Depth);
        AssertPoints(contact, Vector3d.Zero,
            new Vector3d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(1, 5), Fixed64.Zero));
    }

    [Fact]
    public void InteriorRimPrincipalDirection_DoesNotDisplaceTheSmallerGeneratorContact()
    {
        var triangle = new FixedTriangle(new Vector3d(-1, 1, -1), new Vector3d(0, 2, 0), Vector3d.Zero);
        // For AB, the base offset (-1,2,-1) is parallel to the projected-Up
        // principal direction. Both unsquared stationary factors vanish there;
        // the cone's origin-centered insphere proves the smaller generator exit.
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(1920767767L), contact.Depth);
        AssertPoints(contact, Vector3d.Zero,
            new Vector3d(Fixed64.FromFraction(-2, 5), Fixed64.FromFraction(1, 5), Fixed64.Zero));
    }

    [Fact]
    public void DeterministicFixtures_AgreeWithIndependentAxialIntersectionAndPairedExitGeometry()
    {
        var random = new DeterministicRandom(0x086C071UL);
        for (int index = 0; index < 48; index++)
        {
            var triangle = new FixedTriangle(Point(), Point(), Point());
            if (triangle.IsDegenerate)
                continue;
            bool expected = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(new Vector3d(0, 4, 0),
                Vector3d.Down, (Fixed64)8, (Fixed64)3, out _);
            bool actual = triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)8, (Fixed64)3, out FixedContactAnchors contact);
            Assert.True(expected == actual, $"Cone fixture {index}: {triangle.A}; {triangle.B}; {triangle.C}");
            if (!actual)
                continue;
            Assert.True(contact.Normal.IsNormalized());
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Vector3d residual = first - second - contact.Normal * contact.Depth;
            // Depth <= diameter/2 = sqrt(73)/2 < 4.273. Independent point,
            // depth, normal and final product rounding sum to <5 raw/component.
            Assert.InRange(residual.X.m_rawValue, -5L, 5L);
            Assert.InRange(residual.Y.m_rawValue, -5L, 5L);
            Assert.InRange(residual.Z.m_rawValue, -5L, 5L);
        }
        Vector3d Point() => new(random.Next(-9, 10), random.Next(-9, 10), random.Next(-9, 10));
    }

    [Fact]
    public void InteriorBaseRimStationaryRoot_SelectsExactPairedEdgeFeature()
    {
        // AB minus the base disk contains a radius-13/256 ball. At its
        // selected support point, the disk trust-region multiplier is 256,
        // larger than the radial slice map's squared norm 74. This proves
        // the lower bound independently of enumerating contact candidates.
        Assert.True(InteriorRimTriangle().TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(13, 256), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(-3, 13), Fixed64.FromFraction(12, 13), Fixed64.FromFraction(-4, 13)), contact.Normal);
        AssertPoints(contact, new Vector3d(Fixed64.FromFraction(765, 256), Fixed64.FromFraction(-317, 64), Fixed64.FromFraction(255, 64)),
            new Vector3d(3, -5, 4));
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void InteriorBaseRimRoot_ClassifiesOneRawAxialNeighbors(long rawY, bool hit)
    {
        var origin = new Vector3d(Fixed64.FromFraction(3, 256),
            Fixed64.FromFraction(-12, 256) + Fixed64.FromRaw(rawY), Fixed64.FromFraction(4, 256));
        Assert.Equal(hit, InteriorRimTriangle().TryGetCenteredFiniteConeContact(origin, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        if (hit)
            Assert.Equal(Fixed64.FromRaw(rawY), contact.Depth);
        if (rawY == 0)
            AssertPoints(contact, new Vector3d(3, -5, 4), new Vector3d(3, -5, 4));
    }

    [Fact]
    public void GeneratorFaceTouch_StraddlingSliceReturnsTheSharedExactMidpoint()
    {
        // Plane 4X-3Y=-6 touches the generator from (0,2,0) to (-3,-2,0).
        // Its slice extends from t=-1 to t=3, straddling the entire generator.
        var triangle = new FixedTriangle(new Vector3d(3, 6, -4),
            new Vector3d(3, 6, 4), new Vector3d(-9, -10, 0));
        Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(-3, 5), Fixed64.Zero), contact.Normal);
        var midpoint = new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.Zero, Fixed64.Zero);
        AssertPoints(contact, midpoint, midpoint);
    }

    [Fact]
    public void GeneratorFaceSlice_StraddlingEndpointsSelectsTheExactInteriorMidpoint()
    {
        // Plane 2X-Y=0 has selected normal (2,-1,0)/sqrt(5). Its azimuth
        // slice extends past both generator endpoints, so the paired witness
        // uses the generator midpoint rather than either distant slice end.
        var triangle = new FixedTriangle(new Vector3d(-10, -20, -10),
            new Vector3d(-10, -20, 10), new Vector3d(10, 20, 0));
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        const long reciprocalRootFiveRaw = 1920767767L;
        Assert.Equal(Fixed64.FromRaw(reciprocalRootFiveRaw), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(2 * reciprocalRootFiveRaw),
            Fixed64.FromRaw(-reciprocalRootFiveRaw), Fixed64.Zero), contact.Normal);
        AssertPoints(contact, new Vector3d(Fixed64.FromFraction(-1, 10), Fixed64.FromFraction(-1, 5), Fixed64.Zero),
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConstantGeneratorSupport_UsesCanonicalAzimuthAndAnInteriorGeneratorWitness(bool fullyContained)
    {
        // The origin vertex owns the complete generator normal circle:
        // unscaled upper vertices have radial support <=4/5, below their
        // 6/5 axial subtraction; scaling preserves this dominance.
        // The cone's centered insphere has radius 6/5.
        Fixed64 scale = fullyContained ? Fixed64.FromFraction(1, 8) : Fixed64.One;
        var triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(1, 2, 0) * scale, new Vector3d(0, 2, 1) * scale);
        // The smaller triangle lies wholly inside the convex cone: its upper
        // vertices have Y=1/4 and radial distance 1/8 < cross-section radius 21/16.
        // Containment still needs the same minimum exit, not a surface crossing.
        Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(6, 5), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(-3, 5), Fixed64.Zero), contact.Normal);
        AssertPoints(contact, Vector3d.Zero,
            new Vector3d(Fixed64.FromFraction(-24, 25), Fixed64.FromFraction(18, 25), Fixed64.Zero));
    }

    [Fact]
    public void VertexGenerator_UniqueRadialMinimumUsesTheInteriorLateralPoint()
    {
        Vector3d vertex = new(Fixed64.FromFraction(1, 8), Fixed64.Zero, Fixed64.Zero);
        var triangle = new FixedTriangle(vertex, new Vector3d(1, 2, 0), new Vector3d(0, 2, 1));
        Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3, out FixedContactAnchors contact));
        // The radius-11/10 ball centered on this vertex is contained in the
        // cone; the other vertices lie strictly behind this supporting normal.
        Assert.Equal(Fixed64.FromFraction(11, 10), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(-4, 5), Fixed64.FromFraction(-3, 5), Fixed64.Zero), contact.Normal);
        AssertPoints(contact, vertex,
            new Vector3d(Fixed64.FromFraction(201, 200), Fixed64.FromFraction(33, 50), Fixed64.Zero));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueBaseRim_UniqueMinimumPreservesWindingAndCyclicOrder(bool reverse)
    {
        FixedTriangle source = ObliqueRimTriangle();
        var triangle = reverse ? new FixedTriangle(source.A, source.C, source.B) : source;
        for (int index = 0; index < 3; index++)
        {
            Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
            // AB minus the base disk contains the complete ball of radius
            // 5/16 about zero; this support axis attains that lower bound.
            Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero), contact.Normal);
            AssertPoints(contact, new Vector3d(Fixed64.FromFraction(77, 16), Fixed64.FromFraction(-19, 4), Fixed64.Zero),
                new Vector3d(5, -5, 0));
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        }
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void ObliqueBaseRim_OneRawNeighborsRemainDistinct(long rawX, bool overlaps)
    {
        // The translation puts the exact edge/rim witness at (5,-5,0).
        var origin = new Vector3d(Fixed64.FromFraction(3, 16) + Fixed64.FromRaw(rawX),
            Fixed64.FromFraction(-1, 4), Fixed64.Zero);
        Assert.Equal(overlaps, ObliqueRimTriangle().TryGetCenteredFiniteConeContact(origin, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        if (rawX == 0)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            AssertPoints(contact, new Vector3d(5, -5, 0), new Vector3d(5, -5, 0));
        }
        else if (overlaps)
            Assert.Equal(Fixed64.MinIncrement, contact.Depth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedRationalRotation_RetainsAuthoredFramesEvenWhenWorldWitnessOverflows(bool extremeOrigin)
    {
        // The exact normalized frame is the rational XY rotation [3,-4;4,3]/5.
        const long k = 1920767767L;
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(k), Fixed64.FromRaw(2 * k));
        Assert.True(rotation.IsNormalized());
        var origin = extremeOrigin ? new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue) : new Vector3d(7, -11, 13);
        Assert.True(ObliqueRimTriangle().TryGetCenteredFiniteConeContact(origin, rotation,
            origin, rotation, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(origin, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(rotation, contact.SecondAnchor.Rotation);
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetLocalPointIn(origin, rotation, out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetLocalPointIn(origin, rotation, out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(77, 16), Fixed64.FromFraction(-19, 4), Fixed64.Zero), first);
        Assert.Equal(new Vector3d(5, -5, 0), second);
        Assert.Equal(!extremeOrigin, contact.FirstAnchor.TryGetPoint(out _));
        Assert.Equal(!extremeOrigin, contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void DistinctRationalFrames_RoundTheTriangleWitnessInItsAuthoredFrameOnlyOnce()
    {
        const long k = 1920767767L;
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(k), Fixed64.FromRaw(2 * k));
        Fixed64 x = Fixed64.FromFraction(3, 8);
        // Applying [3,-4;4,3]/5 gives a world wall X=5/8 with Y=-20,20,0.
        var triangle = new FixedTriangle(new Vector3d(x - (Fixed64)16, -Fixed64.Half - (Fixed64)12, (Fixed64)(-20)),
            new Vector3d(x + (Fixed64)16, -Fixed64.Half + (Fixed64)12, (Fixed64)(-20)),
            new Vector3d(x, -Fixed64.Half, (Fixed64)20));
        Assert.True(triangle.TryGetCenteredFiniteConeContact(Vector3d.Zero, rotation,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 8), contact.Depth);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(FixedQuaternion.Identity, contact.SecondAnchor.Rotation);
        Assert.True(contact.FirstAnchor.TryGetLocalPointIn(Vector3d.Zero, rotation, out Vector3d local));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(-17, 40), Fixed64.FromFraction(-11, 10), Fixed64.Zero), local);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d world));
        // Each local coordinate rounds within 1/2 raw; each transformed
        // coordinate therefore differs by <=(3+4)/5*1/2 raw before materialization.
        Assert.InRange(world.X.m_rawValue, Fixed64.FromFraction(5, 8).m_rawValue - 1, Fixed64.FromFraction(5, 8).m_rawValue + 1);
        Assert.InRange(world.Y.m_rawValue, -Fixed64.One.m_rawValue - 1, -Fixed64.One.m_rawValue + 1);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d conePoint));
        Assert.Equal(new Vector3d(1, -1, 0), conePoint);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnrepresentableRelativeCenter_DoesNotReplaceIntersectionWithFrameAdmission(bool separated)
    {
        var triangle = new FixedTriangle(new Vector3d(-3, -1, 0), new Vector3d(3, -1, 0),
            new Vector3d(separated ? 6 : 0, 1, 0));
        var triangleOrigin = new Vector3d(3, 0, 0);
        var coneCenter = new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero, Fixed64.Zero);
        Assert.False(new FixedPointAnchor(coneCenter, FixedQuaternion.Identity, Vector3d.Zero)
            .TryGetLocalPointIn(triangleOrigin, FixedQuaternion.Identity, out _));
        Assert.Equal(!separated, triangle.TryGetCenteredFiniteConeContact(triangleOrigin, FixedQuaternion.Identity,
            coneCenter, FixedQuaternion.Identity, Fixed64.One, Fixed64.MaxValue, out FixedContactAnchors contact));
        if (!separated)
        {
            Assert.Equal(triangleOrigin, contact.FirstAnchor.Origin);
            Assert.Equal(coneCenter, contact.SecondAnchor.Origin);
            Assert.True(contact.Normal.IsNormalized());
            Assert.False(contact.DepthIsClamped);
        }
    }

    [Fact]
    public void SideIntrusion_MissedByCenterSampling_HasExactVerticalEdgeContact()
    {
        var triangle = new FixedTriangle(new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(4, 5), (Fixed64)(-2), Fixed64.Zero), new Vector3d(2, -2, 0));
        Assert.True(Contact(triangle, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One - Fixed64.FromFraction(4, 5), contact.Depth);
        Assert.Equal(Vector3d.Left, contact.Normal);
        AssertPoints(contact, new Vector3d(Fixed64.FromFraction(4, 5), -Fixed64.One, Fixed64.Zero), new Vector3d(1, -1, 0));
    }

    private static FixedTriangle ObliqueRimTriangle() => new(
        new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)(-4), Fixed64.FromFraction(5, 4)),
        new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(-11, 2), Fixed64.FromFraction(-5, 4)),
        new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(-35, 4), Fixed64.Zero));

    private static FixedTriangle InteriorRimTriangle() => new(
        new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(-309, 64), Fixed64.FromFraction(231, 64)),
        new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(-325, 64), Fixed64.FromFraction(279, 64)),
        new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(-365, 64), Fixed64.FromFraction(271, 64)));
}
