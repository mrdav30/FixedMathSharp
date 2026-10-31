using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ScaledAxisTriangleTests
{
    private const long Q = 1L << 32;
    private static readonly FixedTriangle Plane = new(new Vector3d((Fixed64)(-4), Fixed64.Zero, (Fixed64)(-4)), new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)(-4)), new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4));
    private static readonly Vector3d Scale = new(Fixed64.FromRaw(2863311531), Fixed64.FromRaw(2576980378), Fixed64.One);

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ParallelClippedCore_ShouldRetainItsInteriorFamilyAcrossEdgeReversal(bool reverse, bool lowerOutside)
    {
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector3d a = new(0,-4,0), b = new(0,4,0), c = new(2,0,0);
        FixedTriangle triangle = reverse ? new(b,a,c) : new(a,b,c);
        Signed832 unit = new(0,0,0,0,0,0,1,0,0,0,0,0,0), twice = WideArithmetic.AddSigned832(unit, unit);
        Signed832 four = WideArithmetic.AddSigned832(twice, twice), three = WideArithmetic.AddSigned832(unit, twice);
        var interval = new RationalSegmentInterval(unit, four, three, four);
        Vector3d center = new(Fixed64.Quarter, lowerOutside ? Fixed64.Quarter : -Fixed64.Quarter, Fixed64.Quarter);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(frame, center, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, Vector3d.Up, interval, out var contact, out var feature, out int endpoint, out _));
        Assert.Equal(1, feature.EdgeMask); Assert.Equal(0, endpoint); Assert.True(contact.Depth > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d physical));
        Assert.Equal(Fixed64.Zero, physical.X); Assert.Equal(Fixed64.Zero, physical.Z);
        Assert.True(physical.Y >= -Fixed64.One && physical.Y <= Fixed64.One);
        Assert.True(physical.Y >= center.Y - Fixed64.One && physical.Y <= center.Y + Fixed64.One);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ClippedSkewCore_ShouldRejectTheUnconstrainedFootBeyondEitherRetainedEndpoint(int side)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        FixedTriangle triangle = new(new(-4,0,0), new(4,0,0), new(0,0,4));
        Signed320 one = Signed320.ExtendValue(Signed192.Signed(1)), three = Signed320.ExtendValue(Signed192.Signed(3)),
            four = Signed320.ExtendValue(Signed192.Signed(4));
        var interval = new RationalSegmentInterval(one, four, three, four);
        Vector3d center = new(side * Fixed64.FromFraction(3,2), Fixed64.Quarter, -Fixed64.Quarter);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(source, center, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.One, Vector3d.Up, interval, out var contact, out _, out int endpoint, out _));
        Assert.Equal(side, endpoint); Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d physical));
        Assert.Equal(new Vector3d(side, 0, 0), physical);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ObliqueFiniteCore_ShouldClampItsPlaneFootToTheProjectedTriangleInterval(int side)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        FixedTriangle triangle = new(new(-2,0,-2), new(2,0,-2), new(0,0,2));
        foreach (bool downward in new[] { false, true })
        {
            FixedQuaternion rotation = downward
                ? new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(3968032378), Fixed64.FromRaw(1643612817))
                : new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(1643612817), Fixed64.FromRaw(3968032378));
            Assert.True(triangle.TryGetCenteredCapsuleContact(source, new Vector3d(Fixed64.Zero, (Fixed64)side, Fixed64.Zero),
                rotation, (Fixed64)4, Fixed64.One, side > 0 ? Vector3d.Up : Vector3d.Down,
                out var contact, out var feature, out _));
            Assert.True(contact.Depth > Fixed64.Zero); Assert.NotEqual(0, feature.EdgeMask);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point)); Assert.Equal(Fixed64.Zero, point.Y);
        }
    }

    [Fact]
    public void GenericClippedContact_ShouldRejectASeparationWhosePrimitiveDirectionExceedsItsStorage()
    {
        Signed832 d = new(1UL << 32,0,0,0,0,0,0,0,0,0,0,0,1);
        Signed832 lower = new(1UL << 31,0,0,0,0,0,0,0,0,0,0,0,1);
        Signed832 upper = new(3UL << 30,0,0,0,0,0,0,0,0,0,0,0,1);
        var interval = new RationalSegmentInterval(lower, d, upper, d);
        var half = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var triangle = new FixedTriangle(new(-2,0,0), new(2,0,0), new(0,0,2));
        Vector3d center = new(Fixed64.Zero, Fixed64.One + Fixed64.MinIncrement, Fixed64.Zero);
        // The retained endpoint is x.Raw=Q/(2^800+1). Separation from
        // y.Raw=Q+1 has coprime direction (Q,(Q+1)*(2^800+1),0),
        // requiring 833 magnitude bits. It cannot be narrowed to Signed832.
        BigInteger denominator = (BigInteger.One << 800) + 1, component = (Q + 1) * denominator;
        Assert.Equal(BigInteger.One, BigInteger.GreatestCommonDivisor(Q, component));
        Assert.True(component >= (BigInteger.One << 832));
        Assert.Throws<OverflowException>(() => triangle.TryGetClippedCenteredCapsuleContact(half, center,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Two, Vector3d.Up, interval, out _, out _, out _, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WideClippedNonzeroCore_ShouldRetainItsInteriorEdgeFoot(bool rigid)
    {
        var source = rigid ? ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity)
            : new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var triangle = new FixedTriangle(new(-4,0,0), new(4,0,0), new(0,0,8));
        Signed832 unit = new(0,0,0,0,0,0,1,0,0,0,0,0,0), twice = WideArithmetic.AddSigned832(unit, unit);
        Signed832 four = WideArithmetic.AddSigned832(twice, twice), three = WideArithmetic.AddSigned832(unit, twice);
        var clip = new RationalSegmentInterval(unit, four, three, four);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(source, new(Fixed64.Zero, Fixed64.Quarter, -Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.One, Fixed64.One, Vector3d.Up, clip, out var contact, out var feature, out int endpoint, out bool local));
        Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth); Assert.Equal(-Vector3d.Forward, contact.Normal);
        Assert.Equal(1, feature.EdgeMask); Assert.Equal(0, endpoint); Assert.False(local);
        Assert.Equal(new Vector3d(Fixed64.Zero, -Fixed64.Quarter, Fixed64.Quarter), contact.FirstAnchor.LocalPoint);
    }

    [Fact]
    public void DegenerateAffineTriangle_ShouldRetainItsClosestPointWithoutInventingSurfaceOwnership()
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        FixedTriangle point = new(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.True(point.TryGetCenteredCapsuleContact(source, Vector3d.Up * Fixed64.Half,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Vector3d.Up, out var contact, out var feature, out _));
        Assert.Equal(7, feature.EdgeMask); Assert.Equal(Vector3d.Up, contact.Normal); Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Fact]
    public void AffineCapsuleContact_ShouldRejectAnUnrepresentableSourceAnchorAndClampedCoincidentDepth()
    {
        var twice = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Two, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector3d authored = new(Fixed64.Zero, Fixed64.FromRaw((long.MaxValue / 2) + 1), Fixed64.Zero);
        FixedTriangle outsideSource = new(authored, authored, authored);
        Assert.False(outsideSource.TryGetCenteredCapsuleContact(twice, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, Vector3d.Up, out _, out _, out _));
        var half = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        FixedTriangle coincident = new(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Assert.False(coincident.TryGetCenteredCapsuleContact(half, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, Vector3d.Up, out _, out _, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineCapsuleContact_ShouldRejectAnUnrepresentableOppositeAnchorBeforePublishing(bool coincident)
    {
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(1643612817), Fixed64.FromRaw(3968032378));
        Vector3d axis = rotation.Rotate(Vector3d.Up).Normalized, normal = new(-axis.Y, axis.X, Fixed64.Zero);
        // An integral offset preserves exact collinearity with the retained
        // scalar core axis even at a near-maximum core length.
        Vector3d physical = axis * (Fixed64)1073741823;
        if (!coincident) physical += normal;
        Vector3d authored = physical * Fixed64.Two;
        var half = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        FixedTriangle point = new(authored, authored, authored);
        Assert.False(point.TryGetCenteredCapsuleContact(half, Vector3d.Zero, rotation,
            Fixed64.MaxValue, Fixed64.MaxValue - (Fixed64)16, normal, out _, out _, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RigidSourceIngress_ShouldPreserveTheExistingFiniteAndClippedContactCharts(bool sphere)
    {
        var triangle = new FixedTriangle(new(-2,0,0), new(2,0,0), new(0,0,4));
        var source = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, -Fixed64.Quarter);
        Fixed64 length = sphere ? Fixed64.Zero : Fixed64.Two;
        Assert.True(triangle.TryGetCenteredCapsuleContact(source, center, FixedQuaternion.Identity,
            length, Fixed64.One, Vector3d.Up, out FixedContactAnchors actual, out _, out bool actualChart));
        Assert.True(triangle.TryGetCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity, center, FixedQuaternion.Identity,
            length, Fixed64.One, Vector3d.Up, out FixedContactAnchors expected, out _, out bool expectedChart));
        Assert.Equal(expectedChart, actualChart); Assert.Equal(expected.Depth, actual.Depth); Assert.Equal(expected.Normal, actual.Normal);
        var one = Signed320.ExtendValue(Signed192.Signed(1));
        var three = Signed320.ExtendValue(Signed192.Signed(3)); var four = Signed320.ExtendValue(Signed192.Signed(4));
        var interval = new RationalSegmentInterval(one, four, three, four);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(source, center, FixedQuaternion.Identity,
            length, Fixed64.One, Vector3d.Up, interval, out actual, out _, out _, out _));
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity, center,
            FixedQuaternion.Identity, length, Fixed64.One, Vector3d.Up, interval, out expected, out _, out _));
        Assert.Equal(expected.Depth, actual.Depth); Assert.Equal(expected.Normal, actual.Normal);
        Assert.Equal(expected.FirstAnchor.LocalPoint, actual.FirstAnchor.LocalPoint);
    }

    [Fact]
    public void RigidFeatureOrientation_ShouldPreserveItsExactWorldProjectionAndOwnership()
    {
        var feature = new AxisTriangleContactFeature(3, Signed576.ExtendValue(Signed192.Signed(2)),
            Signed576.ExtendValue(Signed192.Signed(-3)), Signed576.ExtendValue(Signed192.Signed(5)), 1);
        var unchanged = feature.GetWorldFeatureFromRigidLocal(FixedQuaternion.Identity);
        Assert.Equal(1, unchanged.CompareSeparationProjection(Signed192.Signed(1), default, default));
        FixedQuaternion quarter = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var rotated = feature.GetWorldFeatureFromRigidLocal(quarter);
        Assert.Equal(3, rotated.EdgeMask); Assert.Equal(1, rotated.TriangleVertex);
        // A quarter turn maps (2,-3,5) to (3,2,5). These two exact
        // projections distinguish the rotation from its inverse.
        Assert.Equal(0, rotated.CompareSeparationProjection(Signed192.Signed(2), Signed192.Signed(-3), default));
        Assert.Equal(1, rotated.CompareSeparationProjection(default, Signed192.Signed(1), default));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedAffineCapsuleSection_ShouldCertifyItsInteriorWithoutBoundaryCrossings(bool vertical)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var finite = new CapsuleFiniteSectionFrame(source, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        FixedTriangle triangle = vertical ? new(new(0,-16,-16), new(0,16,-16), new(0,0,16))
            : new(Plane.A * (Fixed64)4, Plane.B * (Fixed64)4, Plane.C * (Fixed64)4);
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(finite, triangle));
        var degenerate = new FixedTriangle(new(8,8,8), new(9,9,9), new(10,10,10));
        Assert.False(CapsulePlaneRayEvents.IntersectsTriangle(finite, degenerate));
    }

    [Fact]
    public void WideClippedCapsuleEvents_ShouldRetainTheClosedPhysicalSectionAndItsEndpoints()
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var finite = new CapsuleFiniteSectionFrame(source, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(Plane, source, finite);
        Signed832 unit = new(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
        Signed832 twice = WideArithmetic.AddSigned832(unit, unit), four = WideArithmetic.AddSigned832(twice, twice);
        var interval = new RationalSegmentInterval(unit, four, WideArithmetic.AddSigned832(twice, unit), four);
        var edge = new CapsulePlaneRayEventSource(new FixedSegment(new(-4,0,0), new(4,0,0)));
        int count = 0;
        void Visit(in CapsulePlaneRayEventSource descriptorSource, in CapsulePlaneRayFrame descriptorFrame,
            CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection point)
        {
            Assert.True(point.TryGetPointInRigidFrame(descriptorFrame, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, out Vector3d physical));
            Assert.InRange(physical.X.m_rawValue, -Q, Q); Assert.Equal(Fixed64.Zero, physical.Y);
            count++;
        }
        Assert.True(CapsulePlaneRayEvents.VisitEvents(edge, frame, 1, interval, Visit));
        Assert.True(count >= 2);
    }

    [Fact]
    public void AffineCapsuleSection_ShouldValidateItsFiniteContractAndRetainWideClipAdmission()
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.Throws<ArgumentOutOfRangeException>(() => new CapsuleFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, -Fixed64.FromRaw(1), Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CapsuleFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)8193, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CapsuleFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.One, -Fixed64.FromRaw(1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CapsuleFiniteSectionFrame(source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.One, (Fixed64)4097));
        Assert.Throws<ArgumentException>(() => new CapsuleFiniteSectionFrame(source, Vector3d.Zero,
            new FixedQuaternion(Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One), Fixed64.One, Fixed64.One));
        Signed832 unit = new(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
        Signed832 twice = WideArithmetic.AddSigned832(unit, unit), four = WideArithmetic.AddSigned832(twice, twice),
            eight = WideArithmetic.AddSigned832(four, four), seven = WideArithmetic.SubtractSigned832(eight, unit);
        Span<ulong> values = stackalloc ulong[4 * 64]; Span<int> signs = stackalloc int[4];
        Span<ulong> root = stackalloc ulong[128];
        foreach (bool rigid in new[] { false, true })
        foreach (Fixed64 length in new[] { Fixed64.Zero, Fixed64.Two })
        {
            var frame = new CapsuleFiniteSectionFrame(rigid ? ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity) : source,
                Vector3d.Zero, FixedQuaternion.Identity, length, Fixed64.One);
            var segment = new FixedSegment(new(-4, 0, 0), new(4, 0, 0));
            var middle = new RationalSegmentInterval(unit, twice, seven, eight);
            var before = new RationalSegmentInterval(default(Signed832), unit, unit, eight);
            var after = new RationalSegmentInterval(seven, eight, unit, unit);
            Assert.True(frame.IntersectsSegment(segment, middle));
            Assert.False(frame.IntersectsSegment(segment, before)); Assert.False(frame.IntersectsSegment(segment, after));
            var n = ContactQuadratic.At(values, signs, 0, 64); var d = ContactQuadratic.At(values, signs, 1, 64);
            Assert.True(frame.TryGetFirstSegmentParameter(new(Vector3d.Zero, new(4, 0, 0)), n, d, root));
            Assert.Equal(0, n.Sign(root)); Assert.True(d.Sign(root) > 0);
            Assert.False(frame.TryGetFirstSegmentParameter(new(new(0, 5, 0), new(0, 6, 0)), n, d, root));
            Assert.False(frame.TryGetFirstSegmentParameter(new(new(5, 0, 0), new(8, 0, 0)), n, d, root));
        }
    }

    [Theory]
    [InlineData(16, -8, 3, 0, 1, 1, 1, true)]
    [InlineData(16, -8, 3, 0, 1, 1, 8, false)]
    [InlineData(16, -8, 3, 7, 8, 1, 1, false)]
    [InlineData(16, -8, 4, 0, 1, 1, 1, true)]
    [InlineData(16, -8, 5, 0, 1, 1, 1, false)]
    [InlineData(0, 0, 1, 0, 1, 1, 1, false)]
    [InlineData(-1, -1, 4, 0, 1, 1, 1, false)]
    [InlineData(1, 1, 1, 0, 1, 1, 1, false)]
    [InlineData(1, 0, -1, 0, 1, 1, 1, true)]
    [InlineData(1, -1, 1, 0, 1, 1, 1, true)]
    public void WideQuadraticSegmentAdmission_ShouldRetainClosedRootsAndFiniteVertexBounds(
        int a, int b, int c, int ln, int ld, int un, int ud, bool expected)
    {
        // 16u�-16u+3 has closed roots at 1/4 and 3/4. The
        // tangent/positive-gap neighbors distinguish minimization from
        // merely inspecting the interval endpoints.
        Signed832 Integer(int value) => Signed832.ExtendValue(Signed192.Signed(value));
        Assert.Equal(expected, CapsuleFiniteSectionFrame.IntersectsQuadratic(Integer(a), Integer(b), Integer(c),
            Integer(ln), Integer(ld), Integer(un), Integer(ud)));
    }

    [Theory]
    [InlineData(0, true, 1)]
    [InlineData(1, false, 1)]
    [InlineData(0, true, -1)]
    [InlineData(1, false, -1)]
    public void ExactCapsuleAdmission_ShouldRejectEveryPositiveGap(long gapRaw, bool expected, int side)
    {
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Scale,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(side * (Q + gapRaw)), Fixed64.Zero);
        Span<ulong> candidateValues = stackalloc ulong[WideFiniteAxisIntersection.ScaledAxisTriangleCandidate.StorageWords];
        Span<int> candidateSigns = stackalloc int[WideFiniteAxisIntersection.ScaledAxisTriangleCandidate.SignCount];
        var candidate = new WideFiniteAxisIntersection.ScaledAxisTriangleCandidate(candidateValues, candidateSigns);
        WideFiniteAxisIntersection.GetClosestScaledAxisTriangleCandidate(Plane, frame, center,
            Vector3d.Up, Fixed64.One, ref candidate);
        Assert.Equal(Fixed64.FromRaw(-side * (Q + gapRaw)), ContactQuadratic.RoundRatio(candidate.TriangleCoordinate(1),
            candidate.Denominator, ReadOnlySpan<ulong>.Empty));
        Assert.Equal(Fixed64.FromRaw(-side * Q / 2), ContactQuadratic.RoundRatio(candidate.AxisCoordinate(1),
            candidate.Denominator, ReadOnlySpan<ulong>.Empty));
        bool found = Plane.TryGetCenteredCapsuleContact(frame,
            center, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.FromRaw(Q / 2), Vector3d.Up, out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out bool local);
        Assert.Equal(expected, found);
        if (!expected) return;
        Assert.False(local); Assert.False(feature.IsZeroSeparation);
        Assert.Equal(side > 0 ? Vector3d.Up : Vector3d.Down, contact.Normal); Assert.Equal(Fixed64.Zero, contact.Depth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedAuthoredPlane_ShouldKeepCapsuleContactAcrossIndependentCenters(bool rotated)
    {
        FixedQuaternion rotation = rotated ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)(-23), (Fixed64)45) : FixedQuaternion.Identity;
        Vector3d center = rotation.Rotate(Vector3d.Up * Fixed64.FromRaw(Q / 2));
        Vector3d fallback = rotation.Rotate(Vector3d.Up).Normalized;
        FixedContactAnchors expected = default;
        for (int index = 0; index < 3; index++)
        {
            Vector3d reference = new(Fixed64.FromRaw(index * 3865470566L), Fixed64.Zero, Fixed64.FromRaw(-index * Q));
            var frame = new ScaledRigidFrame3d(Vector3d.Zero, rotation, reference, Scale,
                rotation, reference, Vector3d.One);
            Assert.True(Plane.TryGetCenteredCapsuleContact(frame, center, rotation,
                Fixed64.FromRaw(Q + 1), Fixed64.FromRaw(Q / 4), fallback, out FixedContactAnchors contact, out _, out _));
            if (index == 0) { expected = contact; continue; }
            Assert.Equal(expected.Normal, contact.Normal); Assert.Equal(expected.Depth, contact.Depth);
            Assert.Equal(expected.FirstAnchor.LocalPoint, contact.FirstAnchor.LocalPoint);
            Assert.Equal(expected.SecondAnchor.LocalPoint, contact.SecondAnchor.LocalPoint);
        }
    }

    [Fact]
    public void ExactCapsulePlaneBanks_ShouldRetainPhysicalSamplingCoordinates()
    {
        Vector3d center = Vector3d.Up * Fixed64.FromRaw(Q / 2);
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Scale,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var finite = new CapsuleFiniteSectionFrame(source, center, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.FromRaw(Q / 4));
        var frame = new CapsulePlaneRayFrame(Plane, source, finite);
        Span<ulong> values = stackalloc ulong[frame.SelectionStorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns, frame.FieldWords, frame.RootWords, frame.ValueWords);
        Assert.False(selection.TryGetPointInRigidFrame(frame, center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, out _));
        Assert.True(CapsulePlaneRayEvents.Accumulate(CapsulePlaneRayEventSource.Plane, frame, -1, ref selection));
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth));
        Assert.Equal(Fixed64.FromRaw(Q / 4), depth);
        Assert.True(selection.TryGetPointInRigidFrame(frame, center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(-Fixed64.FromRaw(Q / 2), point.Y);
        Assert.True(frame.UsesWideBanks); Assert.Equal(frame.SelectionStorageWords, values.Length);
        Assert.Equal(frame.FieldWords, frame.WithWideParameters().FieldWords);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AffinePlaneWalls_ShouldRetainExactFiniteMembershipAndWorldNormal(bool reverse, bool oblique)
    {
        FixedQuaternion rotation = oblique ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17,
            (Fixed64)(-23), (Fixed64)45) : FixedQuaternion.Identity;
        var source = new ScaledRigidFrame3d(Vector3d.Zero, rotation, Vector3d.Zero, Scale,
            rotation, Vector3d.Zero, Vector3d.One);
        var finite = new CapsuleFiniteSectionFrame(source, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Two);
        FixedTriangle triangle = reverse ? new(Plane.B, Plane.A, Plane.C) : Plane;
        var plane = new CapsulePlaneRayFrame(triangle, source, finite);
        Assert.InRange((plane.GetWorldNormal(rotation) - rotation.Rotate(Vector3d.Up).Normalized).Magnitude.m_rawValue, 0, 8);
        Span<ulong> values = stackalloc ulong[8 * plane.FieldWords]; Span<int> signs = stackalloc int[8];
        var x = ContactQuadratic.At(values, signs, 0, plane.FieldWords);
        var y = ContactQuadratic.At(values, signs, 1, plane.FieldWords);
        var z = ContactQuadratic.At(values, signs, 2, plane.FieldWords);
        var d = ContactQuadratic.At(values, signs, 3, plane.FieldWords);
        d.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        foreach (Vector3d point in new[] { Vector3d.Zero, new Vector3d(0, 0, 5), Vector3d.Up })
        {
            WideAxis3 transformed = finite.Transform(point);
            x.Set(Signed576.ExtendValue(transformed.X)); y.Set(Signed576.ExtendValue(transformed.Y));
            z.Set(Signed576.ExtendValue(transformed.Z));
            Assert.Equal(point == Vector3d.Zero, plane.ContainsTrianglePoint(triangle, x, y, z, d, ReadOnlySpan<ulong>.Empty));
        }
        var rigid = ScaledRigidFrame3d.Rigid(Vector3d.Zero, rotation);
        var rigidFinite = new CapsuleFiniteSectionFrame(rigid, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One);
        var ordinary = new CapsulePlaneRayFrame(triangle, rigid, rigidFinite);
        Assert.False(ordinary.UsesWideBanks); Assert.True(ordinary.WithWideParameters().UsesWideBanks);
        Assert.Equal(ordinary.AuthoredNormal, ordinary.WithWideParameters().AuthoredNormal);
    }

    [Theory]
    [InlineData(0L, Q / 2, 0, -1)]
    [InlineData(0L, -Q / 4, 1, -1)]
    [InlineData(0L, 0L, 1, -1)]
    [InlineData(5 * Q / 4, 0L, 3, 1)]
    [InlineData(-5 * Q / 4, 0L, 5, 0)]
    [InlineData(0L, 5 * Q / 4, 6, 2)]
    public void ExactClosestFeature_ShouldPreserveFaceEdgeAndVertexIdentity(
        long xRaw, long zRaw, int expectedMask, int expectedVertex)
    {
        FixedTriangle triangle = new(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0), new Vector3d(0, 0, 2));
        Vector3d half = Vector3d.One * Fixed64.FromRaw(Q / 2);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, half,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector3d center = new(Fixed64.FromRaw(xRaw), Fixed64.FromRaw(Q / 2), Fixed64.FromRaw(zRaw));
        Assert.True(triangle.TryGetCenteredCapsuleContact(frame, center, FixedQuaternion.Identity,
            Fixed64.FromRaw(Q / 2), Fixed64.One, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.Equal(expectedMask, feature.EdgeMask); Assert.Equal(expectedVertex, feature.TriangleVertex);
        Assert.True(feature.CompareSeparationProjection(Signed192.Raw(contact.Normal.X),
            Signed192.Raw(contact.Normal.Y), Signed192.Raw(contact.Normal.Z)) > 0);
        Assert.True(feature.CompareSeparationProjection(Vector3d.Zero, contact.Normal) > 0);
        Assert.True(feature.CompareSeparationProjection(new WideAxis3(Signed320.ExtendValue(Signed192.Raw(contact.Normal.X)),
            Signed320.ExtendValue(Signed192.Raw(contact.Normal.Y)), Signed320.ExtendValue(Signed192.Raw(contact.Normal.Z)))) > 0);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 1)]
    public void WideClippedEdge_ShouldOwnItsPhysicalEndpointBeforeSphereMaterialization(bool rigid, int side)
    {
        FixedTriangle triangle = new(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0), new Vector3d(0, 0, 2));
        var frame = rigid ? ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity)
            : new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Signed832 unit = new(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
        Signed832 twice = WideArithmetic.AddSigned832(unit, unit),
            three = WideArithmetic.AddSigned832(twice, unit), four = WideArithmetic.AddSigned832(twice, twice);
        var interval = new RationalSegmentInterval(unit, four, three, four);
        Fixed64 reach = rigid ? Fixed64.One : Fixed64.Half;
        Vector3d center = new(side * (reach + Fixed64.Quarter), Fixed64.Quarter, -Fixed64.Quarter);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(frame, center, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Vector3d.Up, interval, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out int endpoint, out bool local));
        Assert.False(local); Assert.Equal(side, endpoint); Assert.Equal(1, feature.EdgeMask);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(side * reach, Fixed64.Zero, Fixed64.Zero), source);
        Vector3d expected = (center - source).Normalized;
        Assert.InRange((contact.Normal - expected).Magnitude.m_rawValue, 0, 2);
        Assert.True(feature.CompareSeparationProjection(Vector3d.Zero, contact.Normal) > 0);
        Assert.False(triangle.TryGetClippedCenteredCapsuleContact(frame, center, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.FromRaw(1), Vector3d.Up, interval, out _, out _, out _, out _));
    }

    [Fact]
    public void CollapsedExactCapsule_ShouldRetainAnInclusivePointTouchWithoutInventingAGapContact()
    {
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Scale,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Assert.True(Plane.TryGetCenteredCapsuleContact(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.Zero, Vector3d.Up, out FixedContactAnchors contact,
            out AxisTriangleContactFeature feature, out _));
        Assert.True(feature.IsZeroSeparation); Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.False(Plane.TryGetCenteredCapsuleContact(frame, Vector3d.Up * Fixed64.FromRaw(1),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero, Vector3d.Up, out _, out _, out _));
    }

    [Fact]
    public void ParallelFaceFamily_ShouldKeepTheSameFiniteWitnessUnderCoreReversal()
    {
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Scale,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Fixed64 diagonal = Fixed64.FromRaw(3037000500);
        Vector3d center = Vector3d.Up * Fixed64.Quarter;
        foreach (int sign in new[] { -1, 1 })
        {
            FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, sign * diagonal, diagonal);
            Assert.True(Plane.TryGetCenteredCapsuleContact(frame, center, rotation, Fixed64.One, Fixed64.One,
                Vector3d.Up, out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out _));
            Assert.Equal(0, feature.EdgeMask); Assert.False(feature.IsZeroSeparation);
            Assert.Equal(Vector3d.Up, contact.Normal); Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point)); Assert.Equal(Vector3d.Zero, point);
        }
    }

    [Fact]
    public void ParallelClosestFamily_ShouldKeepInteriorWitnessUnderSourceReversal()
    {
        Vector3d half = Vector3d.One * Fixed64.FromRaw(Q / 2);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, half,
            FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Fixed64 diagonal = Fixed64.FromRaw(3037000500);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, diagonal, diagonal);
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(Q / 4), Fixed64.FromRaw(-Q / 4));
        Vector3d a = new(-2, 0, 0), b = new(2, 0, 0), c = new(0, 0, 2);
        FixedContactAnchors previous = default;
        for (int reversal = 0; reversal < 2; reversal++)
        {
            FixedTriangle triangle = reversal == 0 ? new(a, b, c) : new(b, a, c);
            Assert.True(triangle.TryGetCenteredCapsuleContact(frame, center, rotation, Fixed64.One, Fixed64.One,
                Vector3d.Up, out FixedContactAnchors contact, out AxisTriangleContactFeature feature, out _));
            Assert.Equal(1, feature.EdgeMask); Assert.Equal(-1, feature.TriangleVertex);
            if (reversal == 0) previous = contact;
            else
            {
                Assert.Equal(previous.Normal, contact.Normal); Assert.Equal(previous.Depth, contact.Depth);
                Assert.Equal(previous.FirstAnchor.LocalPoint, contact.FirstAnchor.LocalPoint);
                Assert.Equal(previous.SecondAnchor.LocalPoint, contact.SecondAnchor.LocalPoint);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactSlabPlaneBanks_ShouldRetainPhysicalSamplingCoordinates(bool rigid)
    {
        Vector3d center = Vector3d.Up * Fixed64.FromRaw(Q / 2);
        var source = rigid ? ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity)
            : new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Scale,
                FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var frame = new FiniteSlabRayFrame(Plane, source, center, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Fixed64.One);
        frame = frame.WithWideParameters();
        Span<ulong> values = stackalloc ulong[frame.SelectionStorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var selection = new FiniteSlabRaySelection(values, signs, frame.FieldWords);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, Plane, ReadOnlySpan<Vector2d>.Empty, -1, ref selection);
        Assert.True(selection.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped, out _, out _));
        Assert.False(clamped); Assert.Equal(Fixed64.FromRaw(Q / 2), depth);
        Assert.True(selection.TryGetSourcePointInSamplingFrame(frame, out Vector3d point));
        Assert.Equal(-Fixed64.FromRaw(Q / 2), point.Y);
        Assert.True(frame.UsesWideBanks); Assert.Equal(frame.FieldWords, frame.WithWideParameters().FieldWords);
    }
}
