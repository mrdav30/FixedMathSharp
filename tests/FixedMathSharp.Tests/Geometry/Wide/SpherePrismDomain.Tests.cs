using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class SpherePrismDomainTests
{
    private static readonly Vector2d[] Rectangle = { new(-2, -1), new(2, -1), new(2, 1), new(-2, 1) };

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExposureMaySelectTheFartherOrientedExit(bool far)
    {
        Vector3d center = new(7 * Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero);
        bool Admit(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
            scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d) =>
            normal.Y.IsZero && normal.Z.IsZero && normal.X.Sign == (far ? 1 : -1);
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(center, Fixed64.Half, Vector3d.Zero,
            Fixed64.Zero, Rectangle, Fixed64.Two, Admit, out var contact, out _, out var depth, out bool overlaps));
        Assert.True(overlaps); Assert.False(contact.DepthIsClamped);
        Assert.Equal(far ? Vector3d.Right : Vector3d.Left, contact.Normal);
        Assert.Equal(far ? 17 * Fixed64.Quarter : 3 * Fixed64.Quarter, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(far ? -2 : 2, 0, 0), source);
        Assert.Equal(!far, WideConvexPrismRelations.IsSphereProjectionWithinRadius(depth));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RejectedSupportsDoNotChangeCompleteSolidAdmission(bool separated)
    {
        int visited = 0;
        bool Reject(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
            scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d)
        { visited++; return false; }
        Assert.False(WideConvexPrismRelations.TryGetSphereContact(separated ? new Vector3d(3, 0, 0) : Vector3d.Zero,
            Fixed64.Half, Vector3d.Zero, Fixed64.Zero, Rectangle, Fixed64.Two, Reject,
            out var contact, out var axis, out _, out bool overlaps));
        Assert.Equal(!separated, overlaps); Assert.Equal(Vector3d.Zero, contact.Normal); Assert.True(axis.IsZero);
        if (!separated) Assert.True(visited > 0);
    }

    [Fact]
    public void NativeYawAdmissionRetainsExactLocalRatiosBeforeMaterialization()
    {
        Fixed64 yaw = Fixed64.Deg2Rad * (Fixed64)31;
        Vector3d origin = new(1_900_000_000, 0, -1_900_000_000);
        Vector3d center = origin + new Vector3d(Fixed64.Quarter, 7 * Fixed64.Quarter, Fixed64.Half);
        BigInteger c = FixedMath.Cos(yaw).m_rawValue, s = FixedMath.Sin(yaw).m_rawValue;
        BigInteger metric = c * c + s * s, q = Fixed64.One.m_rawValue;
        int visited = 0;
        bool Admit(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
            scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d)
        {
            if (!normal.X.IsZero || !normal.Z.IsZero || normal.Y.Sign >= 0) return false;
            visited++;
            Assert.Equal((Fixed64.Quarter.m_rawValue * c + Fixed64.Half.m_rawValue * s) * q * Integer(d), Integer(x) * metric);
            Assert.Equal((-Fixed64.Quarter.m_rawValue * s + Fixed64.Half.m_rawValue * c) * q * Integer(d), Integer(z) * metric);
            Assert.Equal((BigInteger)Fixed64.Two.m_rawValue * Integer(d), Integer(y));
            return true;
        }
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(center, Fixed64.Half, origin, yaw,
            Rectangle, Fixed64.Two, Admit, out var contact, out _, out _, out bool overlaps));
        Assert.True(overlaps); Assert.True(visited > 0);
        Assert.Equal(Vector3d.Down, contact.Normal); Assert.Equal(3 * Fixed64.Quarter, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(center.X, Fixed64.Two, center.Z), source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteCornerRetainsItsRadialWitnessAndTranslation(bool largeWorld)
    {
        Vector3d origin = largeWorld ? new Vector3d(1_900_000_000, -1_900_000_000, 1_900_000_000) : Vector3d.Zero;
        Vector3d center = origin + new Vector3d(9 * Fixed64.Quarter, Fixed64.Zero, 5 * Fixed64.Quarter);
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(center, Fixed64.Half, origin, Fixed64.Zero,
            Rectangle, Fixed64.Two, AdmitAll, out var contact, out _, out var depth, out bool overlaps));
        Assert.True(overlaps); Assert.True(WideConvexPrismRelations.IsSphereProjectionWithinRadius(depth));
        Assert.Equal(contact.Normal.X, contact.Normal.Z); Assert.True(contact.Normal.X < Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.Equal(Fixed64.FromRaw(Fixed64.Half.m_rawValue - NearestRoot(2 * BigInteger.Pow(Fixed64.Quarter.m_rawValue, 2))), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(origin + new Vector3d(2, 0, 1), source);
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(FixedConvexPrismRelations.TryGetSphereContact(center - contact.Normal * (contact.Depth + Fixed64.FromRaw(64)),
            Fixed64.Half, origin, Fixed64.Zero, Rectangle, Fixed64.Two, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HorizontalRimAxisKeepsFiniteAdmissionBeyondSeparateFaceTests(bool separated)
    {
        Fixed64 offset = separated ? Fixed64.FromFraction(3, 8) : Fixed64.Quarter;
        Vector3d center = new(Fixed64.Zero, Fixed64.Two + offset, Fixed64.One + offset);
        bool found = WideConvexPrismRelations.TryGetSphereContact(center, Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            Rectangle, Fixed64.Two, AdmitAll, out var contact, out _, out _, out bool overlaps);
        Assert.Equal(!separated, found); Assert.Equal(!separated, overlaps);
        if (!found) return;
        Assert.Equal(Fixed64.Zero, contact.Normal.X); Assert.Equal(contact.Normal.Y, contact.Normal.Z);
        Assert.True(contact.Normal.Y < Fixed64.Zero);
        Assert.Equal(Fixed64.FromRaw(Fixed64.Half.m_rawValue - NearestRoot(2 * BigInteger.Pow(offset.m_rawValue, 2))), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(new Vector3d(0, 2, 1), source);
    }

    [Fact]
    public void BoundedLargeFeaturesKeepTheirNearWorldLimitSupportExact()
    {
        Vector2d[] offsets = { new(-1024, -1024), new(1024, -1024), new(1024, 1024), new(-1024, 1024) };
        Vector3d origin = new(Fixed64.MaxValue - (Fixed64)2048, Fixed64.Zero, Fixed64.MinValue + (Fixed64)2048);
        Vector3d center = origin + Vector3d.Right * (1024 * Fixed64.One + Fixed64.Quarter);
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(center, Fixed64.One, origin, Fixed64.Zero, offsets,
            (Fixed64)1024, AdmitAll, out var contact, out _, out _, out bool overlaps));
        Assert.True(overlaps); Assert.Equal(Vector3d.Left, contact.Normal); Assert.Equal(3 * Fixed64.Quarter, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(origin + Vector3d.Right * (Fixed64)1024, source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollapsedAabbFootprintRetainsItsFiniteSphereContact(bool point)
    {
        Vector2d[] offsets = point ? new[] { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero, Vector2d.Zero }
            : new[] { new Vector2d(0, -1), new(0, -1), new(0, 1), new(0, 1) };
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(Vector3d.Right * Fixed64.Quarter, Fixed64.Half,
            Vector3d.Zero, Fixed64.Zero, offsets, Fixed64.Two, AdmitAll,
            out var contact, out _, out _, out bool overlaps));
        Assert.True(overlaps); Assert.Equal(Vector3d.Left, contact.Normal); Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Vector3d.Zero, source);
    }

    [Fact]
    public void AdmittedDepthOrderingUsesTheUnroundedProjection()
    {
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(Vector3d.Zero, Fixed64.Half, Vector3d.Zero,
            Fixed64.Zero, Rectangle, Fixed64.Two, AdmitAll, out _, out _, out var farther, out _));
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(Vector3d.Forward * Fixed64.Quarter, Fixed64.Half, Vector3d.Zero,
            Fixed64.Zero, Rectangle, Fixed64.Two, AdmitAll, out _, out _, out var nearer, out _));
        Assert.True(WideConvexPrismRelations.CompareProjectionDepths(farther, nearer) > 0);
        Assert.True(WideConvexPrismRelations.CompareProjectionDepths(nearer, farther) < 0);
        Assert.Equal(0, WideConvexPrismRelations.CompareProjectionDepths(nearer, nearer));
        Assert.True(WideConvexPrismRelations.TryGetSphereContact(Vector3d.Forward * Fixed64.Quarter, 3 * Fixed64.Quarter, Vector3d.Zero,
            Fixed64.Zero, Rectangle, Fixed64.Two, AdmitAll, out _, out _, out var largerRadius, out _));
        Assert.Equal(0, WideConvexPrismRelations.CompareProjectionDepths(farther, largerRadius));
        Assert.True(WideConvexPrismRelations.CompareProjectionDepths(nearer, largerRadius) < 0);
        Assert.True(WideConvexPrismRelations.CompareProjectionDepths(largerRadius, nearer) > 0);
        Assert.False(WideConvexPrismRelations.IsSphereProjectionWithinRadius(farther));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClippedConcaveFaceKeepsItsFiniteSpherePeak(bool reverse)
    {
        Vector2d a = reverse ? new(0, 2) : new(0, -2), b = reverse ? new(0, -2) : new(0, 2);
        RationalSegmentInterval clip = reverse ? Interval(0, 1, 1, 2) : Interval(1, 2, 1, 1);
        Assert.True(Boundary(new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter), Fixed64.Half,
            a, b, new(-2, 0), clip, -Fixed64.Two, Fixed64.Two, out var contact, out _, out _));
        Assert.Equal(Vector3d.Left, contact.Normal);
        long expected = Fixed64.Quarter.m_rawValue + NearestRoot(3 * BigInteger.Pow(Fixed64.Quarter.m_rawValue, 2));
        Assert.Equal(Fixed64.FromRaw(expected), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Vector3d.Zero, source);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(-contact.Depth, Fixed64.Zero, Fixed64.Zero), point);
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(0)]
    [InlineData(3)]
    public void TangentProjectionClampsToTheRetainedParameterInterval(int x)
    {
        Assert.True(Boundary(new((Fixed64)x, Fixed64.Zero, Fixed64.Quarter), (Fixed64)3,
            new(-2, 0), new(2, 0), new(0, -2), Interval(1, 4, 3, 4), -Fixed64.One, Fixed64.One,
            out var contact, out _, out _));
        int selected = Math.Clamp(x, -1, 1);
        long expected = NearestRoot(BigInteger.Pow(3 * Fixed64.One.m_rawValue, 2)
            - BigInteger.Pow((x - selected) * Fixed64.One.m_rawValue, 2)) - Fixed64.Quarter.m_rawValue;
        Assert.Equal(Fixed64.FromRaw(expected), contact.Depth); Assert.Equal(Vector3d.Backward, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(new Vector3d(selected, 0, 0), source);
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(0)]
    [InlineData(3)]
    public void HeightClampsToTheRetainedBand(int y)
    {
        Assert.True(Boundary(new(Fixed64.Zero, (Fixed64)y, Fixed64.Quarter), (Fixed64)3,
            new(-2, 0), new(2, 0), new(0, -2), default, -Fixed64.One, Fixed64.One,
            out var contact, out _, out Fixed64 selectedHeight));
        int selected = Math.Clamp(y, -1, 1);
        long expected = NearestRoot(BigInteger.Pow(3 * Fixed64.One.m_rawValue, 2)
            - BigInteger.Pow((y - selected) * Fixed64.One.m_rawValue, 2)) - Fixed64.Quarter.m_rawValue;
        Assert.Equal(Fixed64.FromRaw(expected), contact.Depth); Assert.Equal((Fixed64)selected, selectedHeight);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(new Vector3d(0, selected, 0), source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveHeightStepCapKeepsItsHorizontalFinitePeak(bool bottom)
    {
        Fixed64 x = -Fixed64.FromFraction(2, 5), height = bottom ? -Fixed64.One : Fixed64.One;
        Fixed64 distance = Fixed64.FromFraction(9, 10), y = bottom ? -distance : distance;
        Assert.True(Boundary(new(x, y, -Fixed64.One), Fixed64.Half, new(0, -2), new(0, 0), new(2, -1),
            default, height, height, out var contact, out _, out _, verticalNormal: bottom ? 1 : -1));
        long expected = Fixed64.One.m_rawValue - distance.m_rawValue
            + NearestRoot(BigInteger.Pow(Fixed64.Half.m_rawValue, 2) - BigInteger.Pow(x.m_rawValue, 2));
        Assert.Equal(Fixed64.FromRaw(expected), contact.Depth); Assert.Equal(bottom ? Vector3d.Up : Vector3d.Down, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(new Vector3d(Fixed64.Zero, height, -Fixed64.One), source);
    }

    [Fact]
    public void FiniteFaceNativeYawRetainsThePhysicalRayMetric()
    {
        Vector3d origin = new(1_900_000_000, 0, -1_900_000_000);
        Fixed64 yaw = Fixed64.Deg2Rad * (Fixed64)31;
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(WideConvexPrismRelations.TryGetSphereBoundaryFaceContact(origin, Fixed64.Half, origin, yaw,
            new(-2, 0), new(2, 0), new(0, -2), default, -Fixed64.One, Fixed64.One, ref selection,
            out _, out var contact, out _, out _));
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Fixed64.Zero, contact.Normal.Y);
        Assert.True(contact.Normal.X > Fixed64.Zero); Assert.True(contact.Normal.Z < Fixed64.Zero);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(origin, source);
    }

    [Fact]
    public void WideCutKeepsItsUnrepresentableEndpointUntilFinalRounding()
    {
        Signed832 denominator = new(0, 0, 0, 0, 0, 0, 1UL << 16, 0, 0, 0, 0, 0, 0);
        Signed832 lower = new(0, 0, 0, 0, 0, 0, 1UL << 15, 0, 0, 0, 0, 0, 1);
        var clip = new RationalSegmentInterval(lower, denominator, denominator, denominator);
        Assert.True(clip.HasWideBounds);
        var values = new ulong[CapsulePlaneRaySelection.GetStorageWords(192, 120, 544)];
        var signs = new int[CapsulePlaneRaySelection.SignCount];
        var valueSigns = new sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns, 192, 120, 544);
        Assert.True(WideConvexPrismRelations.TryGetSphereBoundaryFaceContact(
            new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter), Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new(0, -2), new(0, 2), new(-2, 0), clip, -Fixed64.One, Fixed64.One, ref selection,
            out _, out var contact, out var selected, out _));
        Assert.True(selected.WideLowerNumerator.Equals(lower)); Assert.True(selected.WideLowerDenominator.Equals(denominator));
        Assert.Equal(Fixed64.FromRaw(Fixed64.Quarter.m_rawValue + NearestRoot(3 * BigInteger.Pow(Fixed64.Quarter.m_rawValue, 2))), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Vector3d.Zero, source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteFaceAdmissionSeesTheExactPointBeforeRaySelection(bool admit)
    {
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        int visits = 0;
        bool Admit(in RationalSegmentInterval parameter, Fixed64 height, in WideDirection3d normal)
        {
            visits++;
            Signed832 one = Signed832.ExtendValue(Signed192.Signed(1)), two = Signed832.ExtendValue(Signed192.Signed(2));
            Assert.Equal(0, RationalSegmentInterval.CompareParameters(parameter.WideLowerNumerator,
                parameter.WideLowerDenominator, one, two));
            Assert.Equal(0, RationalSegmentInterval.CompareParameters(parameter.WideUpperNumerator,
                parameter.WideUpperDenominator, one, two));
            Assert.Equal(Fixed64.Zero, height); Assert.Equal(Vector3d.Left, normal.GetNormal(false));
            return admit;
        }
        bool found = WideConvexPrismRelations.TryGetSphereBoundaryFaceContact(
            new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter), Fixed64.Half, Vector3d.Zero, Fixed64.Zero,
            new(0, -2), new(0, 2), new(-2, 0), Interval(1, 2, 1, 1), -Fixed64.One, Fixed64.One,
            ref selection, out _, out var contact, out _, out _, admission: Admit);
        Assert.Equal(1, visits); Assert.Equal(admit, found); Assert.Equal(admit, selection.HasValue);
        if (!admit)
        {
            Assert.Equal(Vector3d.Zero, contact.Normal); Assert.Equal(Fixed64.Zero, contact.Depth);
            return;
        }
        Assert.Equal(Fixed64.FromRaw(Fixed64.Quarter.m_rawValue + NearestRoot(3 * BigInteger.Pow(Fixed64.Quarter.m_rawValue, 2))), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Vector3d.Zero, source);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void InvalidOrSeparatedFaceHasNoAdmittedPeak(int shape)
    {
        Vector2d start = new(-2, 0), end = shape == 0 ? start : new Vector2d(2, 0);
        Vector2d inside = shape == 1 ? new Vector2d(0, 0) : new Vector2d(0, -2);
        Fixed64 lower = shape == 2 ? Fixed64.One : -Fixed64.One;
        Fixed64 upper = shape == 2 ? -Fixed64.One : Fixed64.One;
        Vector3d center = shape == 3 ? Vector3d.Forward * Fixed64.Two : Vector3d.Zero;
        Assert.False(Boundary(center, Fixed64.Half, start, end, inside, default, lower, upper, out _, out _, out _));
    }

    private static bool Boundary(Vector3d center, Fixed64 radius, Vector2d start, Vector2d end, Vector2d inside,
        in RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper, out FixedContactAnchors contact,
        out RationalSegmentInterval parameter, out Fixed64 height, int verticalNormal = 0)
    {
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        bool found = WideConvexPrismRelations.TryGetSphereBoundaryFaceContact(center, radius, Vector3d.Zero, Fixed64.Zero,
            start, end, inside, clip, lower, upper, ref selection, out _, out contact, out parameter, out height, verticalNormal);
        Assert.Equal(found, selection.HasValue);
        return found;
    }

    private static RationalSegmentInterval Interval(int ln, int ld, int un, int ud) =>
        new(Signed320.ExtendValue(Signed192.Signed(ln)), Signed320.ExtendValue(Signed192.Signed(ld)),
            Signed320.ExtendValue(Signed192.Signed(un)), Signed320.ExtendValue(Signed192.Signed(ud)));

    private static bool AdmitAll(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d) => true;

    private static BigInteger Integer(scoped ContactQuadratic value)
    {
        Assert.Equal(0, value.Signs[1]);
        BigInteger result = BigInteger.Zero;
        for (int word = value.Rational.Length - 1; word >= 0; word--) result = (result << 64) + value.Rational[word];
        return result * value.Signs[0];
    }

    private static long NearestRoot(BigInteger square)
    {
        long lower = 0, upper = 4 * Fixed64.One.m_rawValue;
        while (lower < upper)
        {
            long middle = lower + ((upper - lower + 1) >> 1);
            if ((BigInteger)middle * middle <= square) lower = middle;
            else upper = middle - 1;
        }
        int comparison = (4 * square).CompareTo(BigInteger.Pow(2 * (BigInteger)lower + 1, 2));
        return lower + (comparison > 0 || comparison == 0 && (lower & 1) != 0 ? 1 : 0);
    }
}
