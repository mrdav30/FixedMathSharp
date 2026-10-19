using System;
using System.Numerics;
using System.Collections.Generic;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ScaledRigidFiniteSectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineConeBoundary_ReconstructsIsolatedAndContinuousFiniteWitnesses(bool segment)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)19, (Fixed64)31), Vector3d.Zero,
            new Vector3d((Fixed64)1.5, (Fixed64).75, (Fixed64).625), FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d((Fixed64)1.25, (Fixed64)1.5, (Fixed64).75));
        FixedSegment line = segment
            ? new(new Vector3d(-1, 1, 0), new Vector3d(1, -1, 1))
            : new(Vector3d.Zero, Vector3d.Zero);
        Vector3d center = new(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero);
        Span<SegmentConeSurfaceCandidate> candidates = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(candidates);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(line, source,
            center, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, ref selection));
        WideAxis3 worldUp = new(default, Signed320.ExtendValue(Signed192.One), default);
        int samples = 0;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family == ConeSurfaceFamily.None)
            {
                FixedContactAnchors contact = candidate.GetContact();
                Assert.True(contact.Depth >= Fixed64.Zero);
                Assert.Equal(center, contact.FirstAnchor.Origin);
                Assert.True(candidate.TryGetNormalDotSign(worldUp, out int sign));
                if (contact.Normal.Y != Fixed64.Zero)
                    Assert.True(contact.Normal.Y.m_rawValue.CompareTo(0L) == sign,
                        $"{candidate.Feature} chart={candidate.Chart} ordinal={candidate.RootOrdinal}: rounded Y={contact.Normal.Y.m_rawValue}, exact sign={sign}");
                samples++;
                continue;
            }
            int count = candidate.GetMaximumFamilyEventCount(0);
            Span<ConeSurfaceFamilyEvent> events = new ConeSurfaceFamilyEvent[count];
            var families = new ConeSurfaceFamilySelection(events);
            for (int location = 1; location <= 3; location++)
                candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty, (ConeSurfacePointLocation)location, ref families);
            for (int item = 0; item < families.Count; item++)
            {
                FixedContactAnchors contact = events[item].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty);
                Assert.Equal(center, contact.FirstAnchor.Origin);
                Assert.True(contact.Depth >= Fixed64.Zero);
                samples++;
            }
        }
        Assert.True(samples > 0);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    [InlineData(1, true)]
    public void AffineCylinderStrictOverlap_UsesExactHalfRawGap(long planeRaw, bool expected)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d((Fixed64)1.5, Fixed64.Half, Fixed64.One), FixedQuaternion.Identity,
            Vector3d.Zero, Vector3d.One);
        Fixed64 y = Fixed64.FromRaw(planeRaw);
        FixedTriangle triangle = new(new Vector3d((Fixed64)(-3), y, (Fixed64)(-3)),
            new Vector3d((Fixed64)3, y, (Fixed64)(-3)), new Vector3d(Fixed64.Zero, y, (Fixed64)3));
        Assert.Equal(expected, WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(Vector3d.Up,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, triangle, source));
    }

    [Theory]
    [InlineData(false, true, 0, true)]
    [InlineData(false, false, 0, false)]
    [InlineData(false, true, 4, true)]
    [InlineData(false, false, 4, true)]
    [InlineData(false, true, 8, false)]
    [InlineData(false, false, 8, false)]
    [InlineData(false, true, 3, true)]
    [InlineData(true, true, 0, true)]
    [InlineData(true, true, 8, false)]
    public void SourceCylinderStrictHull_RetainsEnclosureFiniteFacesAndSeparatedVolumes(bool rigid, bool closed, int twiceX, bool expected)
    {
        var source = rigid ? ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity)
            : new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                new Vector3d((Fixed64)1.5, Fixed64.One, Fixed64.One), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        ReadOnlySpan<Vector3d> vertices = stackalloc Vector3d[8]
        { new(-1,-1,-1), new(1,-1,-1), new(-1,1,-1), new(1,1,-1),
          new(-1,-1,1), new(1,-1,1), new(-1,1,1), new(1,1,1) };
        ReadOnlySpan<int> faces = stackalloc int[36]
        { 0,1,3,0,3,2,4,6,7,4,7,5,0,4,5,0,5,1,2,3,7,2,7,6,0,2,6,0,6,4,1,5,7,1,7,3 };
        Fixed64 radius = twiceX == 4 ? Fixed64.One : Fixed64.Quarter;
        Assert.Equal(expected, WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(new Vector3d(Fixed64.FromFraction(twiceX, 2), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.One, radius, source, vertices, faces, closed));
        FixedTriangle floor = new(new Vector3d(-4,0,-4), new(4,0,-4), new(0,0,4));
        Assert.True(WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, floor, source));
    }

    [Fact]
    public void AffineContactIngress_RejectsCollapsedFacesAndRetainsZeroRadiusCone()
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d((Fixed64)1.5, Fixed64.One, Fixed64.One), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.False(WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, source, ReadOnlySpan<Vector3d>.Empty, ReadOnlySpan<int>.Empty, true));
        FixedTriangle collapsed = new(Vector3d.Zero, Vector3d.Right, Vector3d.Right * Fixed64.Two);
        Assert.False(TriangleConeContact.TryGetContact(collapsed, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out _));
        Assert.False(TriangleCylinderContact.TryGetContact(collapsed, source, Vector3d.Zero,
            FixedQuaternion.Identity, Signed192.Raw(Fixed64.Two), Fixed64.One, out _, out _));
        Assert.False(WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, collapsed, source));
        FixedTriangle face = new(new Vector3d(-4,0,-4), new(4,0,-4), new(0,0,4));
        var collapsedPlane = new ConePlaneRayFrame(collapsed, source, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(collapsedPlane, collapsed));
        var plane = new ConePlaneRayFrame(face, source, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        Assert.False(ConePlaneRayEvents.IntersectsTriangle(plane, collapsed));
        Assert.True(TriangleConeContact.TryGetContact(face, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out FixedContactAnchors needle));
        Assert.Equal(Fixed64.One, needle.Depth);
        Assert.Throws<ArgumentOutOfRangeException>(() => TriangleConeContact.TryGetContact(face, source, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)(-1), Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => TriangleConeContact.TryGetContact(face, source, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)8193, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => TriangleConeContact.TryGetContact(face, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, (Fixed64)(-1), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => TriangleConeContact.TryGetContact(face, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, (Fixed64)4097, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConeFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConeFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)8193, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConeFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, (Fixed64)(-1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConeFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, (Fixed64)4097));
        Assert.Throws<ArgumentException>(() => new ConeFiniteSectionFrame(source, Vector3d.Zero,
            new FixedQuaternion(Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One), Fixed64.Two, Fixed64.One));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AffineCircularTriangle_UsesTheSameCompleteGeometryAsExactDyadicVertices(bool cone, bool oblique)
    {
        FixedQuaternion rotation = oblique
            ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)27, (Fixed64)41)
            : FixedQuaternion.Identity;
        var scale = new Vector3d((Fixed64)1.5, (Fixed64).75, (Fixed64).625);
        var partScale = new Vector3d((Fixed64)1.25, (Fixed64)1.5, (Fixed64).75);
        var source = new ScaledRigidFrame3d(Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)19, (Fixed64)31), Vector3d.Zero,
            scale, rotation, Vector3d.Zero, partScale);
        FixedTriangle triangle = new(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        FixedTriangle exactDyadic = new(Bake(triangle.A), Bake(triangle.B), Bake(triangle.C));
        Vector3d Bake(Vector3d value) => new(value.X * scale.X * partScale.X,
            value.Y * scale.Y * partScale.Y, value.Z * scale.Z * partScale.Z);
        Vector3d center = new(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero);
        bool actual = cone
            ? TriangleConeContact.TryGetContact(triangle, source, center, FixedQuaternion.Identity,
                Fixed64.Two, Fixed64.One, out FixedContactAnchors contact)
            : TriangleCylinderContact.TryGetContact(triangle, source, center, FixedQuaternion.Identity,
                Signed192.Raw(Fixed64.Two), Fixed64.One, out contact, out _);
        bool expected = cone
            ? TriangleConeContact.TryGetContact(exactDyadic, Vector3d.Zero, rotation, center, FixedQuaternion.Identity,
                Fixed64.Two, Fixed64.One, out FixedContactAnchors baseline)
            : TriangleCylinderContact.TryGetContact(exactDyadic, Vector3d.Zero, rotation, center, FixedQuaternion.Identity,
                Signed192.Raw(Fixed64.Two), Fixed64.One, out baseline, out _);
        Assert.True(expected); Assert.True(actual);
        Assert.Equal(baseline.Depth, contact.Depth);
        Assert.Equal(baseline.Normal, contact.Normal);
        // The affine path retains the physical witness directly; it must not
        // round an authored coordinate and amplify that error by source scale.
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.FirstAnchor.Rotation);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineStadiumTriangle_FractionalAnisotropyAndOddCoreRetainPhysicalContact(bool oblique)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)27, (Fixed64)41);
        FixedQuaternion target = oblique
            ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)11, (Fixed64)23) : rotation;
        // Tiny authored vertices and large, odd raw scale factors create an
        // ordinary 2,048-unit surface with high-precision rational coordinates.
        // The odd core length must retain its exact half-raw endpoints.
        Vector3d owner = Raw((1L << 45) + 1, (1L << 45) + 3, (1L << 45) + 5);
        Vector3d part = Raw((1L << 40) + 7, (1L << 40) + 9, (1L << 40) + 11);
        FixedTriangle triangle = new(Raw(-(1L << 22) + 1, 0, -(1L << 22) + 3),
            Raw((1L << 22) + 5, 0, -(1L << 22) + 7), Raw(0, 0, (1L << 22) + 9));
        Assert.True(triangle.IsDegenerate); // Authored epsilon area is irrelevant after physical scale.
        Vector2d core = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Fixed64 length = Fixed64.FromRaw(2 * Fixed64.One.m_rawValue + 1);
        var placement = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)19, (Fixed64)31);
        var source = new ScaledRigidFrame3d(Vector3d.Zero, placement, Vector3d.Zero,
            owner, rotation, Vector3d.Zero, part);
        Assert.True(TriangleCylinderContact.TryGetContact(triangle, source, Vector3d.Zero, target,
            Signed192.Raw(Fixed64.Two), Fixed64.One, out FixedContactAnchors contact, out _, core, length));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
        if (!oblique) Assert.Equal(Fixed64.One, contact.Depth);
        var plane = new ConePlaneRayFrame(triangle, source, Vector3d.Zero, target, Fixed64.Two, Fixed64.One);
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(plane, triangle));
        Assert.True(TriangleConeContact.TryGetContact(triangle, source, Vector3d.Zero, target,
            Fixed64.Two, Fixed64.One, out FixedContactAnchors cone));
        Assert.True(cone.Depth > Fixed64.Zero);
        Assert.True(WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(Vector3d.Zero,
            target, Fixed64.Two, Fixed64.One, triangle, source));
        Vector3d shift = new(7, -3, 11);
        var translated = new ScaledRigidFrame3d(shift, placement, Vector3d.Zero,
            owner, rotation, Vector3d.Zero, part);
        Assert.True(TriangleCylinderContact.TryGetContact(triangle, translated, shift, target,
            Signed192.Raw(Fixed64.Two), Fixed64.One, out FixedContactAnchors moved, out _, core, length));
        Assert.Equal(contact.Depth, moved.Depth);
        Assert.Equal(contact.Normal, moved.Normal);
        Assert.Equal(contact.FirstAnchor.LocalPoint, moved.FirstAnchor.LocalPoint);
        Assert.Equal(contact.SecondAnchor.LocalPoint, moved.SecondAnchor.LocalPoint);
        Assert.Equal(shift, moved.FirstAnchor.Origin);

        static Vector3d Raw(long x, long y, long z) => new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
    }

    [Fact]
    public void AffineSlabBoundary_RetainsWorldFanAndPhysicalSourceWitness()
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d((Fixed64)1.5, Fixed64.One, Fixed64.One), FixedQuaternion.Identity,
            Vector3d.Zero, new Vector3d((Fixed64)1.25, Fixed64.One, Fixed64.One));
        FixedSegment segment = new(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0));
        Vector3d center = new(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero);
        Span<WideAxis3> fan = stackalloc WideAxis3[1]
        { new(default, Signed320.ExtendValue(Signed192.One), default) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(segment, source, center,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.Half,
            ReadOnlySpan<Vector2d>.Empty, fan, true, out FixedContactAnchors contact));
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.FirstAnchor.Rotation);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.Zero, point.Y);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AffineCylinderRimWitness_ShouldMatchTheSameFiniteDyadicSurface(int feature)
    {
        bool rootWinner = feature != 0;
        FixedTriangle world = rootWinner
            ? new(new Vector3d(Fixed64.FromFraction(1021,256), Fixed64.FromFraction(309,64), Fixed64.FromFraction(231,64)),
                new(Fixed64.FromFraction(509,256), Fixed64.FromFraction(325,64), Fixed64.FromFraction(279,64)),
                new(Fixed64.FromFraction(813,256), Fixed64.FromFraction(365,64), Fixed64.FromFraction(271,64)))
            : new(new Vector3d(Fixed64.FromFraction(43,4), Fixed64.FromFraction(5,4), (Fixed64)6),
                new(Fixed64.FromFraction(19,4), Fixed64.FromFraction(45,4), (Fixed64)(-2)), new(3,5,4));
        if (feature == 2)
        {
            // The finite face itself owns the rim-stationary witness. Its third
            // vertex cuts the old edge at the exact q=(765/256,317/64,255/64).
            Vector3d q = new(Fixed64.FromFraction(765,256), Fixed64.FromFraction(317,64), Fixed64.FromFraction(255,64));
            world = new(world.A, world.B, q + new Vector3d(Fixed64.Quarter, Fixed64.Zero, -Fixed64.FromFraction(3,16)));
        }
        Vector3d Unscale(Vector3d p) => new(p.X / Fixed64.Two, p.Y, p.Z / Fixed64.Two);
        FixedTriangle authored = new(Unscale(world.A), Unscale(world.B), Unscale(world.C));
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d(2,1,2), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        for (int turn = 0; turn < 3; turn++)
        {
            Assert.True(TriangleCylinderContact.TryGetContact(authored, source, Vector3d.Zero,
                FixedQuaternion.Identity, Signed192.Raw((Fixed64)10), (Fixed64)5, out FixedContactAnchors actual, out _));
            Assert.True(world.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors expected));
            Assert.Equal(expected.Depth, actual.Depth);
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.True(expected.FirstAnchor.TryGetPoint(out Vector3d expectedPoint));
            Assert.True(actual.FirstAnchor.TryGetPoint(out Vector3d actualPoint));
            Assert.Equal(expectedPoint, actualPoint);
            Assert.Equal(rootWinner, WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(Vector3d.Zero,
                FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, authored, source));
            authored = new(authored.B, authored.C, authored.A);
            world = new(world.B, world.C, world.A);
        }
    }

    [Theory]
    [InlineData(0L, 0L)]
    [InlineData(1L, 2L)]
    public void AffineCylinderRim_HalfRawPhysicalWitnessShouldRoundOnceToEven(long offsetRaw, long expectedRaw)
    {
        Fixed64 x = (Fixed64)3, y = (Fixed64)5 + Fixed64.FromRaw(offsetRaw), z = (Fixed64)4;
        Fixed64 epsilon = Fixed64.MinIncrement;
        Vector3d Author(Vector3d p) => new(p.X * Fixed64.Two, p.Y, p.Z * Fixed64.Two);
        FixedTriangle triangle = new(
            Author(new Vector3d(x - Fixed64.Two - epsilon, y + Fixed64.One + epsilon, z - Fixed64.Two - epsilon)),
            Author(new Vector3d(x + Fixed64.Two + epsilon, y - Fixed64.One, z + Fixed64.Two + epsilon)),
            Author(new Vector3d(x + (Fixed64)3, y + (Fixed64)14 + epsilon, z + (Fixed64)4)));
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Half), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var center = new Vector3d(Fixed64.Zero, Fixed64.FromRaw(offsetRaw), Fixed64.Zero);
        Assert.True(TriangleCylinderContact.TryGetContact(triangle, source, center, FixedQuaternion.Identity,
            Signed192.Raw((Fixed64)10 + epsilon), (Fixed64)5, out FixedContactAnchors contact, out _));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        // Affine source witnesses round once in the final physical shape
        // chart. Its exact local Y=5+1/2 raw rounds to even5 for both origins;
        // the cap independently retains its exact half-height term. An odd
        // origin can therefore separate the final world samples by one raw.
        Assert.Equal(new Vector3d(x, (Fixed64)5, z), contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(x, (Fixed64)5 + Fixed64.FromRaw(offsetRaw), z), first);
        var expected = new Vector3d(x, (Fixed64)5 + Fixed64.FromRaw(expectedRaw), z);
        Assert.Equal(expected, second);
        Assert.InRange((first.Y - expected.Y).m_rawValue, -1L, 1L);
    }

    [Fact]
    public void ExactPointInverse_ShouldRetainSignedScaleAndRejectSingularOrUnrepresentableSourceCoordinates()
    {
        const int words = 40;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var x = ContactQuadratic.At(values, signs, 0, words); var y = ContactQuadratic.At(values, signs, 1, words);
        var z = ContactQuadratic.At(values, signs, 2, words); var d = ContactQuadratic.At(values, signs, 3, words);
        x.Set(Signed576.ExtendValue(Signed192.Raw(-Fixed64.One)));
        y.Set(Signed576.ExtendValue(Signed192.Raw((Fixed64)(-6))));
        z.Set(Signed576.ExtendValue(Signed192.Raw(Fixed64.FromFraction(15,4)))); d.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d(-Fixed64.Half, Fixed64.Two, Fixed64.FromFraction(3,4)), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.True(QuadraticPointMaterialization.TryGetPointInScaledFrame(Vector3d.Zero, FixedQuaternion.Identity,
            frame, x, y, z, d, ReadOnlySpan<ulong>.Empty, out Vector3d source));
        Assert.Equal(new Vector3d(2,-3,5), source);
        d.MultiplySign(-1);
        Assert.False(QuadraticPointMaterialization.TryGetPointInScaledFrame(Vector3d.Zero, FixedQuaternion.Identity,
            frame, x, y, z, d, ReadOnlySpan<ulong>.Empty, out _));
        d.MultiplySign(-1);
        var singular = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d(0,1,1), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.False(QuadraticPointMaterialization.TryGetPointInScaledFrame(Vector3d.Zero, FixedQuaternion.Identity,
            singular, x, y, z, d, ReadOnlySpan<ulong>.Empty, out _));
        var tiny = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.One), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.False(QuadraticPointMaterialization.TryGetPointInScaledFrame(Vector3d.Zero, FixedQuaternion.Identity,
            tiny, x, y, z, d, ReadOnlySpan<ulong>.Empty, out _));
    }

    [Fact]
    public void WideClippedConeGenerator_ShouldRetainTheSameFinitePlaneAndFamilySamples()
    {
        var source = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        FixedTriangle triangle = new(new Vector3d(-3,-3,0), new(3,-3,0), new(0,3,0));
        var frame = new ConePlaneRayFrame(triangle, source, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        FixedSegment segment = new(new Vector3d(-1,-1,0), Vector3d.Up);
        Signed832 unit = new(0,0,0,0,0,0,1,0,0,0,0,0,0);
        Signed832 twice = WideArithmetic.AddSigned832(unit, unit),
            three = WideArithmetic.AddSigned832(twice, unit), four = WideArithmetic.AddSigned832(twice, twice);
        var wide = new RationalSegmentInterval(unit, four, three, four);
        var narrow = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(4)),
            Signed320.ExtendValue(Signed192.Signed(3)), Signed320.ExtendValue(Signed192.Signed(4)));
        Assert.True(wide.HasWideBounds);
        Assert.Equal(Collect(narrow, frame), Collect(wide, frame));
        // A joined cohort can normalize a frame that already owns wide banks.
        // Repeating that normalization must leave every physical event intact.
        var normalized = frame.WithWideParameters();
        Assert.Equal(Collect(wide, frame), Collect(wide, normalized.WithWideParameters()));
        List<Vector3d> Collect(RationalSegmentInterval clip, ConePlaneRayFrame inputFrame)
        {
            var points = new List<Vector3d>();
            void Visit(in ConePlaneRayEventSource scope, in ConePlaneRayFrame visited,
                ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
                scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
            {
                Assert.True(ConePlaneRayPointMaterialization.TryGetPointInFrame(visited, point, root, Vector3d.Zero, out Vector3d p));
                Assert.InRange(p.Y.m_rawValue, -Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue);
                points.Add(p);
            }
            Assert.True(ConePlaneRayEvents.VisitEvents(new ConePlaneRayEventSource(segment), inputFrame, clip, Visit));
            Assert.NotEmpty(points);
            return points;
        }
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment, source,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, ref selection));
        int samples = 0;
        for (int index = 0; index < selection.Count; index++)
        {
            var candidate = selection[index];
            if (candidate.Family != ConeSurfaceFamily.SegmentInterval) continue;
            Span<ConeSurfaceFamilyEvent> events = new ConeSurfaceFamilyEvent[candidate.GetMaximumFamilyEventCount(0)];
            var family = new ConeSurfaceFamilySelection(events);
            candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty, ConeSurfacePointLocation.Interior, wide, ref family);
            for (int item = 0; item < family.Count; item++)
            {
                Assert.True(family[item].TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, wide,
                    out Vector3d a, out Vector3d b));
                Assert.InRange(a.Y.m_rawValue, -Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue);
                Assert.Equal(a, b);
                Assert.Equal(Fixed64.Zero, family[item].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, wide).Depth);
                samples++;
            }
        }
        Assert.True(samples > 0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineConePlane_RetainsExactPlaneAdmissionAndMaterializesAuthoredWitnessOnce(bool oblique)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)27, (Fixed64)41);
        FixedQuaternion placement = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)19, (Fixed64)31);
        var source = new ScaledRigidFrame3d(Vector3d.Zero, placement, Vector3d.Zero,
            new Vector3d((Fixed64)1.5, (Fixed64).75, (Fixed64).625), rotation, Vector3d.Zero,
            new Vector3d((Fixed64)1.25, (Fixed64)1.5, (Fixed64).75));
        FixedTriangle triangle = new(new Vector3d(-3, 0, -3), new Vector3d(3, 0, -3), new Vector3d(0, 0, 3));
        var frame = new ConePlaneRayFrame(triangle, source, Vector3d.Zero,
            oblique ? FixedQuaternion.Identity : rotation, Fixed64.Two, Fixed64.One);
        Vector3d worldNormal = frame.GetWorldNormal(rotation);
        Vector3d expectedNormal = rotation.Rotate(Vector3d.Up).Normalized;
        Assert.True(Vector3d.Dot(worldNormal, expectedNormal) >= Fixed64.One - Fixed64.FromRaw(8));
        Assert.True(frame.IsCoplanar(frame.Transform(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half))));
        Assert.False(frame.IsCoplanar(frame.Transform(new Vector3d(Fixed64.Half, Fixed64.Epsilon, Fixed64.Half))));
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(frame, triangle));
        Span<ulong> positiveValues = stackalloc ulong[frame.SelectionStorageWords],
            negativeValues = stackalloc ulong[frame.SelectionStorageWords],
            pointValues = stackalloc ulong[frame.PointStorageWords], root = stackalloc ulong[frame.RootWords];
        Span<int> positiveSigns = stackalloc int[ConePlaneRaySelection.SignCount],
            negativeSigns = stackalloc int[ConePlaneRaySelection.SignCount], pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var positive = new ConePlaneRaySelection(positiveValues, positiveSigns, frame.FieldWords, frame.RootWords);
        var negative = new ConePlaneRaySelection(negativeValues, negativeSigns, frame.FieldWords, frame.RootWords);
        var point = new ConePlaneRayPoint(pointValues, pointSigns, frame.FieldWords);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Axis), point, root, ref positive, ref negative));
        Assert.True(positive.GetRoundedMaximumDepth() > Fixed64.Zero);
        Assert.True(negative.GetRoundedMaximumDepth() > Fixed64.Zero);
        if (!oblique)
        {
            Assert.Equal(Fixed64.One, positive.GetRoundedMaximumDepth());
            Assert.Equal(Fixed64.One, negative.GetRoundedMaximumDepth());
        }
        Assert.True(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, point, root, out Vector3d witness));
        Assert.Equal(Vector3d.Zero, witness);
        int events = 0;
        void Visit(in ConePlaneRayEventSource scope, in ConePlaneRayFrame visitedFrame,
            ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint candidate, scoped ReadOnlySpan<ulong> candidateRoot,
            scoped in ConePlaneRaySelection above, scoped in ConePlaneRaySelection below)
        {
            events++;
            Assert.True(ConePlaneRayPointExits.ContainsPoint(scope, visitedFrame, candidate, candidateRoot));
            Assert.True(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(visitedFrame, candidate, candidateRoot, out Vector3d local));
            Assert.Equal(Fixed64.Zero, local.Y);
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(ConePlaneRayEventSource.Plane, frame, Visit));
        Assert.True(events > 0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineSections_RetainCompoundDenominatorsAndExactSegmentRoots(bool cone)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)27, (Fixed64)41);
        FixedQuaternion placement = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)7, (Fixed64)19, (Fixed64)31);
        Vector3d scale = new((Fixed64)1.5, (Fixed64).75, (Fixed64).625);
        Vector3d partScale = new((Fixed64)1.25, (Fixed64)1.5, (Fixed64).75);
        var source = new ScaledRigidFrame3d(Vector3d.Zero, placement, Vector3d.Zero,
            scale, rotation, Vector3d.Zero, partScale);
        var capsule = new CapsuleFiniteSectionFrame(source, Vector3d.Zero, rotation, Fixed64.Two, Fixed64.One);
        var finiteCone = new ConeFiniteSectionFrame(source, Vector3d.Zero, rotation, Fixed64.Two, Fixed64.One);
        FixedSegment segment = new(new Vector3d(-3, 0, 0), new Vector3d(3, 0, 0));
        Span<ulong> values = stackalloc ulong[8 * FiniteSectionPoint.AffineWords + FiniteSectionPoint.AffineRootWords];
        Span<int> signs = stackalloc int[FiniteSectionPoint.SignCount];
        var point = new FiniteSectionPoint(values, signs,
            FiniteSectionPoint.AffineWords, FiniteSectionPoint.AffineRootWords);
        if (cone)
        {
            Assert.True(finiteCone.ContainsTransformedPoint(finiteCone.Transform(Vector3d.Zero)));
            Assert.False(finiteCone.ContainsTransformedPoint(finiteCone.Transform(new Vector3d(1, 0, 0))));
            Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(segment, finiteCone, point));
        }
        else
        {
            Assert.True(capsule.ContainsPoint(Vector3d.Zero));
            Assert.False(capsule.ContainsPoint(new Vector3d(1, 0, 0)));
            Assert.True(capsule.IntersectsSegment(segment));
            Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(segment, capsule, point));
        }
        Fixed64 x = ContactQuadratic.RoundRatio(point.Coordinate(0), point.Denominator, point.Root);
        // Rotation cancels exactly; X scale is 15/8. At Y=0 the capsule
        // radius is 1 and the cone radius is 1/2, so source X is -8/15 or -4/15.
        BigInteger numerator = -(cone ? 4 : 8) * (BigInteger.One << 32);
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), 15, out BigInteger remainder);
        if (2 * remainder > 15 || 2 * remainder == 15 && !quotient.IsEven)
            quotient++;
        Assert.Equal(-(long)quotient, x.m_rawValue);
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RigidContactIngress_PreservesTheExistingFiniteWitness(bool cone)
    {
        FixedTriangle triangle = new(new Vector3d(-4,0,-4), new(4,0,-4), new(0,0,4));
        Vector3d origin = new(7,-3,11), center = origin + Vector3d.Up * Fixed64.Quarter;
        var frame = ScaledRigidFrame3d.Rigid(origin, FixedQuaternion.Identity);
        bool actual = cone
            ? TriangleConeContact.TryGetContact(triangle, frame, center, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out FixedContactAnchors a)
            : TriangleCylinderContact.TryGetContact(triangle, frame, center, FixedQuaternion.Identity, Signed192.Raw(Fixed64.Two), Fixed64.One, out a, out _);
        bool expected = cone
            ? TriangleConeContact.TryGetContact(triangle, origin, FixedQuaternion.Identity, center, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, out FixedContactAnchors b)
            : TriangleCylinderContact.TryGetContact(triangle, origin, FixedQuaternion.Identity, center, FixedQuaternion.Identity, Signed192.Raw(Fixed64.Two), Fixed64.One, out b, out _);
        Assert.True(expected); Assert.True(actual);
        Assert.Equal(b.Depth, a.Depth); Assert.Equal(b.Normal, a.Normal);
        Assert.Equal(b.FirstAnchor, a.FirstAnchor); Assert.Equal(b.SecondAnchor, a.SecondAnchor);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AffineConePlane_CanonicalOrientationRetainsEveryPrincipalPlane(int axis)
    {
        Vector3d Map(Vector3d v) => axis == 0 ? new(v.Y,v.X,v.Z) : axis == 2 ? new(v.X,v.Z,v.Y) : v;
        FixedTriangle triangle = new(Map(new(-3,0,-3)), Map(new(3,0,-3)), Map(new(0,0,3)));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            new Vector3d(2,3,4), FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var plane = new ConePlaneRayFrame(triangle, frame, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        Assert.Equal(Map(Vector3d.Up), plane.GetWorldNormal(FixedQuaternion.Identity));
        Assert.True(plane.IsCoplanar(plane.Transform(Map(new(Fixed64.Half,Fixed64.Zero,Fixed64.Half)))));
        Assert.False(plane.IsCoplanar(plane.Transform(Map(Vector3d.Up))));
        Assert.True(ConePlaneRayEvents.IntersectsTriangle(plane, triangle));
    }

    [Fact]
    public void WideNormalization_RetainsZeroNegativeAndDominantLastComponents()
    {
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        Signed832 minus = Signed832.ExtendValue(Signed192.Signed(-1));
        Assert.Equal(Vector2d.Zero, WideNormalization.GetNormalized(default(Signed832), default(Signed832)));
        Vector2d diagonal = WideNormalization.GetNormalized(one, minus);
        Assert.True(diagonal.X > Fixed64.Zero); Assert.Equal(-diagonal.X, diagonal.Y);
        Assert.Equal(Vector3d.Forward, WideNormalization.GetNormalized(default(Signed832), default(Signed832), one));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WideConePlaneCohort_RetainsTheStationarySideExit(bool affine)
    {
        FixedTriangle physical = new(new Vector3d(0,-5,-3), new(0,-5,3), new(-2,1,0));
        FixedTriangle authored = affine ? new(physical.A / Fixed64.Two, physical.B / Fixed64.Two, physical.C / Fixed64.Two) : physical;
        var frame = affine
            ? new ConePlaneRayFrame(authored, new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, Vector3d.One * Fixed64.Two, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One),
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two)
            : new ConePlaneRayFrame(authored, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two).WithWideParameters();
        Fixed64 maximum = Fixed64.Zero;
        void Visit(in ConePlaneRayEventSource scope, in ConePlaneRayFrame input,
            ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection above, scoped in ConePlaneRaySelection below)
        {
            Assert.True(ConePlaneRayPointExits.ContainsPoint(scope, input, point, root));
            if (above.HasValue) maximum = FixedMath.Max(maximum, above.GetRoundedMaximumDepth());
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(ConePlaneRayEventSource.Plane, frame, Visit));
        Assert.Equal(Fixed64.FromRaw(11641610684L), maximum); // 6*sqrt(10)/7
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AffineFaceMinimumCertificate_DeclinesOutsideProjectionWeights(int order)
    {
        // A valid cap-plane contact need not contain the cap pole. Every
        // vertex permutation must keep the same finite closest feature.
        Fixed64 y = Fixed64.FromFraction(19,4);
        FixedTriangle physical = new(new Vector3d((Fixed64)(-2),y,Fixed64.Half),
            new((Fixed64)(-1),y,Fixed64.Half),new(Fixed64.Zero,y,(Fixed64)1.5));
        for(int turn=0;turn<order;turn++) physical=new(physical.B,physical.C,physical.A);
        FixedTriangle authored = new(physical.A/Fixed64.Two,physical.B/Fixed64.Two,physical.C/Fixed64.Two);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            Vector3d.One*Fixed64.Two,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        var geometry = new TriangleCircularGeometry(authored,frame,Vector3d.Zero,FixedQuaternion.Identity,
            Signed192.Raw((Fixed64)10),(Fixed64)5);
        Assert.False(geometry.HasFaceMinimumCertificate());
        Assert.True(TriangleCylinderContact.TryGetContact(authored,frame,Vector3d.Zero,FixedQuaternion.Identity,
            Signed192.Raw((Fixed64)10),(Fixed64)5,out FixedContactAnchors contact,out _));
        Assert.Equal(Fixed64.Quarter,contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(new Vector3d((Fixed64)(-.75),y,(Fixed64).75),first);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullDomainRimFace_EndpointPaddingDoesNotOverflowScalarBounds(bool negative)
    {
        Vector3d a = new(Fixed64.FromFraction(1021,256),Fixed64.FromFraction(309,64),Fixed64.FromFraction(231,64));
        Vector3d b = new(Fixed64.FromFraction(509,256),Fixed64.FromFraction(325,64),Fixed64.FromFraction(279,64));
        Vector3d q = new(Fixed64.FromFraction(765,256),Fixed64.FromFraction(317,64),Fixed64.FromFraction(255,64));
        Vector3d c = q + new Vector3d(Fixed64.Quarter,Fixed64.Zero,-Fixed64.FromFraction(3,16));
        long translation = negative ? long.MinValue + a.X.m_rawValue : long.MaxValue - a.X.m_rawValue;
        Vector3d Move(Vector3d v) => new(Fixed64.FromRaw(translation + (negative ? -v.X.m_rawValue : v.X.m_rawValue)),v.Y,v.Z);
        FixedTriangle triangle = new(Move(a),Move(b),Move(c));
        Vector3d center = new(Fixed64.FromRaw(translation),Fixed64.Zero,Fixed64.Zero);
        Assert.Equal(negative ? Fixed64.MinValue : Fixed64.MaxValue,triangle.A.X);
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero,FixedQuaternion.Identity,
            center,FixedQuaternion.Identity,(Fixed64)10,(Fixed64)5,out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(13,256),contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));Assert.Equal(Move(q),first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));Assert.Equal(Move(new(3,5,4)),second);
    }

    [Theory]
    [InlineData(0L)]
    [InlineData(1L)]
    public void AffineAnalyticRimEdge_RetainsTheHalfRawFiniteFoot(long offsetRaw)
    {
        // x+2y=6 raw supports the cylinder at one finite edge midpoint.
        // Its meridional principal normal wins analytically, without a quartic
        // or whole-face witness. The translated local chart rounds y=1/2 raw
        // once; the cap anchor retains its independent exact half-raw term.
        FixedTriangle physical = new(
            new Vector3d((Fixed64)(-2)+Fixed64.FromRaw(4),Fixed64.One+Fixed64.FromRaw(offsetRaw+1),Fixed64.Zero),
            new Vector3d((Fixed64)2+Fixed64.FromRaw(6),-Fixed64.One+Fixed64.FromRaw(offsetRaw),Fixed64.Zero),
            new Vector3d((Fixed64)2,(Fixed64)2+Fixed64.FromRaw(offsetRaw),Fixed64.Zero));
        Vector3d Author(Vector3d v) => new(v.X*Fixed64.Two,v.Y,v.Z*Fixed64.Two);
        FixedTriangle triangle = new(Author(physical.A),Author(physical.B),Author(physical.C));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
            new Vector3d(Fixed64.Half,Fixed64.One,Fixed64.Half),FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
        Vector3d center = new(Fixed64.Zero,Fixed64.FromRaw(offsetRaw),Fixed64.Zero);
        Assert.True(TriangleCylinderContact.TryGetContact(triangle,frame,center,FixedQuaternion.Identity,
            Signed192.Raw(Fixed64.MinIncrement),Fixed64.FromRaw(5),out FixedContactAnchors contact,out bool cap));
        Assert.False(cap); Assert.Equal(Fixed64.Zero,contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(5),Fixed64.Zero,Fixed64.Zero),contact.FirstAnchor.LocalPoint);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.FromRaw(5),first.X); Assert.Equal(first.X,second.X);
        Assert.InRange(first.Y.m_rawValue-second.Y.m_rawValue,-1L,1L);
        Assert.Equal(Fixed64.Zero,first.Z); Assert.Equal(first.Z,second.Z);
    }

    [Theory]
    [InlineData(0, 0, 0, true)]
    [InlineData(0, 3, 0, false)]
    [InlineData(0, -3, 0, false)]
    [InlineData(0, 0, 1, false)]
    [InlineData(2, 0, 0, true)]
    [InlineData(2, 3, 0, false)]
    [InlineData(2, -3, 0, false)]
    [InlineData(2, 0, 1, true)]
    [InlineData(2, 0, 2, false)]
    [InlineData(2, -2, 2, true)]
    [InlineData(2, 2, 1, false)]
    public void ConePointAdmission_RetainsFiniteAxisAndOffAxisSections(int radius, int y, int x, bool expected)
    {
        // The source plane contains every tested point, so rejection must
        // come from the actual finite cone rather than a coplanarity shortcut.
        for (int affine = 0; affine < 2; affine++)
        {
            Fixed64 factor = affine == 0 ? Fixed64.One : Fixed64.Half;
            FixedTriangle triangle = new(new Vector3d(-4,y,-4)*factor,
                new Vector3d(4,y,-4)*factor,new Vector3d(0,y,4)*factor);
            var source = new ScaledRigidFrame3d(Vector3d.Zero,FixedQuaternion.Identity,Vector3d.Zero,
                Vector3d.One/factor,FixedQuaternion.Identity,Vector3d.Zero,Vector3d.One);
            var frame = new ConePlaneRayFrame(triangle,source,Vector3d.Zero,FixedQuaternion.Identity,(Fixed64)4,(Fixed64)radius);
            Span<ulong> values = new ulong[frame.PointStorageWords];
            Span<int> signs = new int[ConePlaneRayPoint.SignCount];
            var point = new ConePlaneRayPoint(values,signs,frame.FieldWords);
            point.Set(frame.Transform(new Vector3d(x,y,0)*factor));
            Assert.Equal(expected,ConePlaneRayPointExits.ContainsPoint(ConePlaneRayEventSource.Plane,frame,point,ReadOnlySpan<ulong>.Empty));
            // Both transverse axes obey the same finite radius constraint.
            point.Set(frame.Transform(new Vector3d(0,y,x)*factor));
            Assert.Equal(expected,ConePlaneRayPointExits.ContainsPoint(ConePlaneRayEventSource.Plane,frame,point,ReadOnlySpan<ulong>.Empty));
        }
    }

}
