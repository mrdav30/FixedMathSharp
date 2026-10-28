// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class CurvedPrismDomainTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GeneratorFamilyFindsInitialContactBetweenTwoOutsideEndpoints(bool minimum)
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Assert.True(rotation.IsNormalized());
        Vector3d center = new(-1, 0, -2);
        Vector2d[] footprint = { new(0, -8), new(2, -8), new(2, 0), new(0, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)3, Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)5, true, vertices);
        FixedContactAnchors contact; bool physical;
        if (minimum)
        {
            Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 3, default, (Fixed64)(-5), (Fixed64)5,
                center, rotation, Vector3d.Zero, false, out FixedContactAnchors supplied, out bool initial));
            Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(3, supplied, initial) };
            var domain = new PrismCurvedContactDomain(default, default, certificates: certificates);
            Assert.True(CurvedPrismContact.TryGetContact(geometry, domain, center, rotation, Vector3d.Zero,
                out contact, out int face, out physical, out _)); Assert.Equal(3, face);
        }
        else Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 3, default, (Fixed64)(-5), (Fixed64)5,
            center, rotation, Vector3d.Zero, false, out contact, out physical));
        Assert.True(physical); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(1, 5)) <= Fixed64.FromRaw(2));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        // For p(t)=(3t,2-4t), the inward translated generator is inside the
        // original cone exactly on [7/600,97/100]. Its first physical point
        // has world Y=37/24; the old t=0 witness was outside the cone.
        Assert.True(FixedMath.Abs(source.X) <= Fixed64.FromRaw(3));
        Assert.True(FixedMath.Abs(source.Y - Fixed64.FromFraction(37, 24)) <= Fixed64.FromRaw(4));
        Assert.Equal((Fixed64)(-2), source.Z);
    }

    [Fact]
    public void FarBoundaryPassRetainsAGenuineWindowOutsideTheOriginalCone()
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(-1, 0, -2);
        Vector2d[] footprint = { new(0, -8), new(2, -8), new(2, 0), new(0, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)3, Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)5, true, vertices);
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 3, default,
            Fixed64.FromFraction(-17, 5), Fixed64.FromFraction(-33, 10), center, rotation, Vector3d.Zero, true,
            out FixedContactAnchors supplied, out bool initial)); Assert.False(initial);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(3, supplied, false) };
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(default, default, certificates: certificates), center, rotation, Vector3d.Zero,
            out _, out _, out _, out _));
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(default, default, includeFarBoundary: true, certificates: certificates), center, rotation, Vector3d.Zero,
            out FixedContactAnchors contact, out int face, out bool physical, out _));
        Assert.Equal(3, face); Assert.False(physical);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(1, 5)) <= Fixed64.FromRaw(2));
    }

    [Fact]
    public void PoleChoosesARealInitialSectionPointBeforeALargerSupportDiskVertex()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        Vector3d origin = new(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero);
        Vector2d[] footprint = { new(Fixed64.Quarter, Fixed64.FromFraction(-3, 8)), new(Fixed64.FromFraction(3, 8), Fixed64.FromFraction(-3, 8)),
            new(Fixed64.FromFraction(3, 8), Fixed64.FromFraction(3, 8)), new(Fixed64.Quarter, Fixed64.FromFraction(3, 8)) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.Two, true, vertices);
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, -2, default, Fixed64.Two, Fixed64.Two,
            center, FixedQuaternion.Identity, origin, false, out FixedContactAnchors contact, out bool physical));
        Assert.True(physical); Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(Fixed64.Zero, source.Y);
        Assert.True(source.X * source.X + source.Z * source.Z <= Fixed64.FromFraction(25, 256) + Fixed64.FromRaw(3));
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
    }
    [Fact]
    public void OpenPoleEdgeKeepsItsWitnessInsideTheSmallerInitialConeSection()
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(Fixed64.FromFraction(-57, 50), Fixed64.FromFraction(-38, 25), Fixed64.Zero);
        Vector3d origin = new(0, -1, 0);
        Vector2d[] footprint = { new(-2, 0), new(0, 0), new(0, 1), new(-2, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)3, origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(4, default, false, 3, 0, 0) };
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, origin,
            out FixedContactAnchors contact, out int face, out bool physical, out int certificate));
        Assert.True(physical); Assert.Equal(-3, face); Assert.Equal(-1, certificate);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        // At source axial +1.9 the cone section radius is .075, although its
        // translated support disk has radius 3. A full-disk interior blend
        // would choose z about .486 and incorrectly lose physical priority.
        Assert.True(FixedMath.Abs(source.X) <= Fixed64.FromRaw(4)); Assert.True(FixedMath.Abs(source.Y) <= Fixed64.FromRaw(4));
        Assert.True(source.Z > Fixed64.Zero && source.Z < Fixed64.FromFraction(2, 25));
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(39, 10)) <= Fixed64.FromRaw(4));
    }

    [Fact]
    public void SamePlaneSiblingSupportSuppliesTheExactNativeMinimumBeforeWindowFiltering()
    {
        Vector3d center = new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter);
        Vector2d[] left = { new(-2, -2), new(0, -2), new(0, 0), new(-2, 0) };
        Vector2d[] right = { new(0, -2), new(2, -2), new(2, 0), new(0, 0) };
        Span<WideDirection3d> leftVertices = stackalloc WideDirection3d[8], rightVertices = stackalloc WideDirection3d[8];
        var own = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            (Fixed64)6, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, left, Fixed64.One, false, leftVertices);
        var sibling = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            (Fixed64)6, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, right, Fixed64.One, false, rightVertices);
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(own, 2, default, -Fixed64.One, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out _, out _));
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(sibling, 2, default, -Fixed64.One, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out FixedContactAnchors supplied, out bool suppliedPhysical));
        Assert.True(suppliedPhysical);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(2, supplied, true) };
        Assert.True(CurvedPrismContact.TryGetContact(own, new(default, default, certificates: certificates),
            center, FixedQuaternion.Identity, Vector3d.Zero, out FixedContactAnchors contact, out int face, out bool physical, out int certificate));
        Assert.True(physical); Assert.Equal(2, face); Assert.Equal(0, certificate);
        Assert.Equal(Fixed64.Quarter, contact.Depth); Assert.Equal(Vector3d.Backward, contact.Normal);
        Assert.Equal(supplied.FirstAnchor, contact.FirstAnchor); Assert.Equal(supplied.SecondAnchor, contact.SecondAnchor);
    }
    [Fact]
    public void RimWithoutAnAdmittedAnalyticBoundRetainsItsPositiveStationaryMinimum()
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024),
            Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024));
        Vector2d[] footprint = { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        // Axis=(4,3,0)/5 and radial=(-9,12,20)/25 give a supporting rim
        // normal (0,15,16)/sqrt(481). This inward displacement has exact
        // depth sqrt(481)/1024; an absent analytic bound is not zero depth.
        CurvedPrismRimContacts.GetContact(geometry, default, Fixed64.Zero, false,
            out var rim, out Fixed64 depth, out bool clamped, hasAnalytic: false);
        Assert.True(rim.HasValue); Assert.False(clamped); Assert.Equal(Fixed64.FromRaw(91_988_268), depth);
        CurvedPrismRimContacts.GetMaterials(geometry, rim, center, rotation, Vector3d.Zero,
            out _, out _, out FixedPointAnchor source);
        Assert.True(source.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half), point);
    }

    [Theory]
    [InlineData(false, false, 1, false)]
    [InlineData(true, false, 1, false)]
    [InlineData(false, true, 1, false)]
    [InlineData(true, true, 1, false)]
    [InlineData(true, false, 0, false)]
    [InlineData(true, false, 2, false)]
    [InlineData(false, false, 1, true)]
    [InlineData(true, false, 1, true)]
    public void EdgeOnlyDomainQualifiesTheStationaryRimMinimum(bool singleton, bool rejectFan, int cut, bool admitFan)
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024),
            Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024));
        Vector2d[] footprint = { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        var interval = singleton ? Interval(cut + 1, 4, cut + 1, 4) : default;
        Assert.True(geometry.TryGetEdgeNormalCone(7, out var first, out var last));
        var covector = CurvedPrismContact.Difference(first, -last); if (rejectFan) covector = -covector;
        Span<WideDirection3d> fan = stackalloc WideDirection3d[1] { covector };
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(7, interval, singleton, 5, 0, rejectFan || admitFan ? 1 : 0) };
        bool found = CurvedPrismContact.TryGetContact(geometry, new(edges, fan), center, rotation, Vector3d.Zero,
            out FixedContactAnchors contact, out int face, out bool physical, out int certificate);
        Assert.Equal(!rejectFan && cut == 1, found);
        if (!found) return;
        Assert.Equal(-3, face); Assert.Equal(-1, certificate); Assert.True(physical);
        // This edge's stationary ellipse point is x=0, lambda=1/2. No face
        // certificate is available; native MIN must retain the quartic owner.
        Assert.False(contact.DepthIsClamped); Assert.Equal(Fixed64.FromRaw(91_988_268), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half), source);
    }

    [Theory]
    [InlineData(false, 1, false)]
    [InlineData(false, -1, false)]
    [InlineData(true, 1, false)]
    [InlineData(true, -1, false)]
    [InlineData(false, 1, true)]
    [InlineData(false, -1, true)]
    [InlineData(true, 1, true)]
    [InlineData(true, -1, true)]
    public void SidePoleUsesTheNearestPointInTheRetainedHeightAndTangentWindow(bool cone, int sign, bool oneSided)
    {
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        Vector3d center = new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), origin = Vector3d.Right;
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            origin, Fixed64.Zero, footprint, Fixed64.One, cone, vertices);
        var clip = sign > 0 ? Interval(3, 8, 7, 16) : Interval(9, 16, 5, 8);
        Fixed64 lower = Fixed64.FromFraction(sign > 0 ? 1 : -2, 8), upper = Fixed64.FromFraction(sign > 0 ? 2 : -1, 8);
        if (oneSided) { if (sign > 0) upper = Fixed64.One; else lower = -Fixed64.One; }
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 3, clip, lower, upper,
            center, rotation, origin, false, out FixedContactAnchors contact, out bool physical));
        Assert.True(physical); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(3, 4)) <= Fixed64.FromRaw(4));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(FixedMath.Abs(source.X) <= Fixed64.FromRaw(4));
        Assert.True(FixedMath.Abs(source.Y - Fixed64.FromFraction(sign, 8)) <= Fixed64.FromRaw(4));
        Assert.True(FixedMath.Abs(source.Z - Fixed64.FromFraction(sign, 8)) <= Fixed64.FromRaw(4));
    }

    [Theory]
    [InlineData(false, false, -1)]
    [InlineData(false, false, 1)]
    [InlineData(false, true, -1)]
    [InlineData(false, true, 1)]
    [InlineData(true, false, -1)]
    [InlineData(true, false, 1)]
    [InlineData(true, true, -1)]
    [InlineData(true, true, 1)]
    public void AxiallyDistantPoleIsFarEvenWhenItsSupportDiskMeetsTheSource(bool cone, bool cap, int sign)
    {
        FixedQuaternion rotation = cap ? FixedQuaternion.Identity
            : new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        Vector3d origin = cap ? new(Fixed64.Zero, (Fixed64)(4 * sign), Fixed64.Zero) : new((Fixed64)(4 * sign), Fixed64.Zero, Fixed64.Zero);
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(Vector3d.Zero, rotation, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            origin, Fixed64.Zero, footprint, Fixed64.One, cone, vertices);
        int face = cap ? sign > 0 ? -2 : -1 : sign > 0 ? 1 : 3;
        var clip = cap ? default : Interval(1, 4, 3, 4);
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(geometry, face, clip, -Fixed64.Half, Fixed64.Half,
            Vector3d.Zero, rotation, origin, false, out _, out _));
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, face, clip, -Fixed64.Half, Fixed64.Half,
            Vector3d.Zero, rotation, origin, true, out FixedContactAnchors contact, out bool physical));
        Assert.False(physical);
        Assert.True(FixedMath.Abs(contact.Depth - (Fixed64)6) <= Fixed64.FromRaw(4));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(FixedMath.Abs((cap ? source.Y : source.X) - (Fixed64)(5 * sign)) <= Fixed64.FromRaw(4));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void OpenPoleClipsBothEndsAgainstTheSmallerInitialSection(bool upper, bool clipped)
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(Fixed64.FromFraction(-57, 50), Fixed64.FromFraction(-38, 25), upper ? Fixed64.One : Fixed64.Zero);
        Vector3d origin = new(0, -1, 0);
        Vector2d[] footprint = { new(-2, 0), new(0, 0), new(0, 1), new(-2, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)3, origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        var interval = clipped ? upper ? Interval(1, 2, 1, 1) : Interval(0, 1, 1, 2) : default;
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(4, interval, false, 5, 0, 0) };
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, origin,
            out FixedContactAnchors contact, out _, out bool physical, out _));
        Assert.True(physical); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Fixed64 axialDistance = upper ? Fixed64.One - source.Z : source.Z;
        Assert.True(axialDistance > Fixed64.Zero && axialDistance < Fixed64.FromFraction(2, 25));
        Assert.True(FixedMath.Abs(source.X) <= Fixed64.FromRaw(4)); Assert.True(FixedMath.Abs(source.Y) <= Fixed64.FromRaw(4));
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(39, 10)) <= Fixed64.FromRaw(4));
    }

    [Fact]
    public void TangencyAtAnExcludedEndpointDoesNotGiveTheOpenPolePhysicalPriority()
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(-Fixed64.FromFraction(3, 4), -Fixed64.One, -Fixed64.FromFraction(3, 4));
        Vector3d origin = new(0, -1, 0);
        Vector2d[] footprint = { new(-2, 0), new(0, 0), new(0, 1), new(-2, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)4, origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(4, default, false, 3, 0, 0) };
        // Source axial 1.25 gives initial cone radius .75: the original
        // cone touches only excluded z=0, not any point on the open edge.
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, origin,
            out _, out _, out _, out _));
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default, includeFarBoundary: true), center, rotation, origin,
            out FixedContactAnchors contact, out _, out bool physical, out _));
        Assert.False(physical); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(source.Z > Fixed64.Zero && source.Z < Fixed64.One);
    }

    [Fact]
    public void AnInteriorApexPoleCanSupplyAnOpenOriginalEdge()
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(-Fixed64.FromFraction(3, 4), -Fixed64.One, Fixed64.Half), origin = Vector3d.Up;
        Vector2d[] footprint = { new(0, 0), new(2, 0), new(2, 1), new(0, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)4, origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        WideDirection3d right = new(Signed576.ExtendValue(Signed192.Signed(1)), default, default);
        Span<WideDirection3d> fan = stackalloc WideDirection3d[2] { right, -right };
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(9, default, false, 6, 0, 2) };
        // These two incident restrictions retain the pole direction. Without
        // them the same edge also admits a shallower cone-generator normal.
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, fan), center, rotation, origin,
            out FixedContactAnchors contact, out _, out bool physical, out _));
        Assert.True(physical); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(FixedMath.Abs(source.X) <= Fixed64.FromRaw(4)); Assert.True(FixedMath.Abs(source.Y) <= Fixed64.FromRaw(4));
        Assert.Equal(Fixed64.Half, source.Z);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(3, 4)) <= Fixed64.FromRaw(4));
    }

    [Fact]
    public void IncidentFanCanRejectAnOtherwisePhysicalOpenPole()
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(-Fixed64.FromFraction(3, 4), -Fixed64.One, Fixed64.Half), origin = new(0, -1, 0);
        Vector2d[] footprint = { new(-2, 0), new(0, 0), new(0, 1), new(-2, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)4, origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        // All inward normals of this upper/right edge have negative dot
        // with primitive Up. A neighbor on that side disqualifies the fan.
        Span<WideDirection3d> fan = stackalloc WideDirection3d[1] { new(geometry.Axis) };
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(4, default, false, 3, 0, 1) };
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(edges, fan, includeFarBoundary: true), center, rotation, origin,
            out _, out _, out _, out _));
    }

    [Fact]
    public void ClockwiseFootprintStillSuppliesTheInwardSidePlane()
    {
        Vector3d center = new(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.Zero);
        Vector2d[] footprint = { new(-1, -1), new(-1, 1), new(1, 1), new(1, -1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, false, vertices);
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 2, default, -Fixed64.One, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out FixedContactAnchors contact, out bool physical));
        Assert.True(physical); Assert.Equal(Vector3d.Left, contact.Normal); Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source)); Assert.Equal(Fixed64.One, source.X);
    }

    private static RationalSegmentInterval Interval(int lowerN, int lowerD, int upperN, int upperD) => new(
        Signed320.ExtendValue(Signed192.Signed(lowerN)), Signed320.ExtendValue(Signed192.Signed(lowerD)),
        Signed320.ExtendValue(Signed192.Signed(upperN)), Signed320.ExtendValue(Signed192.Signed(upperD)));

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 0)]
    public void BoundaryCompetitionRanksExactDepthsBeforeTheirCommonRoundedValue(bool cone, int delta)
    {
        Fixed64 rawDelta = Fixed64.FromRaw(delta), radius = Fixed64.FromFraction(5, 8);
        Fixed64 height = cone ? Fixed64.FromFraction(7, 8) : Fixed64.One;
        Fixed64 half = cone ? Fixed64.FromFraction(7, 16) : Fixed64.FromFraction(3, 8) + rawDelta;
        Vector3d center = new(-Fixed64.FromFraction(3, 8), Fixed64.Zero, -Fixed64.FromFraction(3, 8) + rawDelta);
        Vector2d[] footprint = { new(-2, -2), new(0, -2), new(0, 2), new(-2, 2) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            height, radius, Vector3d.Zero, Fixed64.Zero, footprint, half, cone, vertices);
        int cap = cone ? -1 : -2;
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, cap, default, -half, half,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out FixedContactAnchors proof, out bool initial));
        Assert.True(initial);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(cap, proof, true) };
        Fixed64 lineY = cone ? -half : Fixed64.Zero;
        Span<PrismCurvedBoundaryLine> lines = stackalloc PrismCurvedBoundaryLine[1] { new(1, Interval(1, 2, 1, 1), lineY, lineY) };
        int visits = 0, order = 0; FixedContactAnchors cut = default;
        void Visit(int ordinal, in FixedContactAnchors contact, int comparison)
        { Assert.Equal(0, ordinal); visits++; cut = contact; order = comparison; }
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(default, default, certificates: certificates),
            center, FixedQuaternion.Identity, Vector3d.Zero, out FixedContactAnchors native, out _, out _, out _,
            lines, Visit, cone ? new ConePrismBoundaryContacts() : null));
        Assert.Equal(1, visits); Assert.Equal(-delta, Math.Sign(order));
        Assert.Equal(native.Depth, cut.Depth);
        // With radius 5/8 and transverse distance 3/8, the exact rim is
        // sqrt(25/64-9/64)=1/2, giving depth 7/8. Moving z one raw unit
        // changes it by about 3/4 raw; changing the cap by one raw changes
        // the native exit by one. Both round alike but their order differs.
        Assert.Equal(Fixed64.FromFraction(7, 8) + rawDelta, cut.Depth);
        Assert.True(cut.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(Fixed64.Zero, lineY, Fixed64.Zero), source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryCompetitionComparesTheFiniteRayMaximumWithTheNativeRimValueRoot(bool touching)
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Fixed64 yShift = touching ? Fixed64.Zero : Fixed64.FromFraction(15, 1024);
        Fixed64 zShift = touching ? Fixed64.Zero : Fixed64.FromFraction(16, 1024);
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - yShift,
            Fixed64.FromFraction(41, 2) - zShift);
        Vector2d[] footprint = { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(7, default, false, 5, 0, 0) };
        Span<PrismCurvedBoundaryLine> lines = stackalloc PrismCurvedBoundaryLine[1] { new(-2, default, Fixed64.Half, Fixed64.Half, 2) };
        int visits = 0, comparison = 0; FixedContactAnchors cut = default;
        void Visit(int ordinal, in FixedContactAnchors contact, int order)
        { visits++; comparison = order; cut = contact; }
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, Vector3d.Zero,
            out FixedContactAnchors native, out _, out _, out _, lines, Visit));
        Assert.Equal(1, visits); Assert.Equal(touching ? 0 : 1, Math.Sign(comparison));
        Assert.Equal(touching ? Fixed64.Zero : Fixed64.FromRaw(91_988_268), native.Depth);
        Fixed64 z = (Fixed64)20 - zShift;
        Fixed64 radial = FixedMath.Sqrt((Fixed64)625 - z * z);
        Fixed64 expected = (Fixed64)(-12) + yShift + Fixed64.FromFraction(4, 5) * radial;
        Assert.True(FixedMath.Abs(cut.Depth - expected) <= Fixed64.FromRaw(8));
        Assert.Equal(Vector3d.Down, cut.Normal);
        Assert.True(cut.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(Fixed64.Half, source.Y); Assert.Equal(Fixed64.Half, source.Z);
        Assert.True(FixedMath.Abs(source.X - ((Fixed64)(-9) + Fixed64.FromFraction(3, 5) * radial)) <= Fixed64.FromRaw(12));
    }

    [Theory]
    [InlineData(-1, -1)]
    [InlineData(-1, 0)]
    [InlineData(-1, 1)]
    [InlineData(1, -1)]
    [InlineData(1, 0)]
    [InlineData(1, 1)]
    [InlineData(0, 1)]
    public void RimComparisonPreservesTheRatioWhenACommonQuadraticFactorCancels(int radicalSign, int delta)
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024),
            Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024));
        Vector2d[] footprint = { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        CurvedPrismRimContacts.GetContact(geometry, default, Fixed64.Zero, false, out var rim, out _, out _, hasAnalytic: false);
        Assert.True(rim.HasValue);
        const int words = 8;
        Span<ulong> fields = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(fields, signs, 0, words); var denominator = ContactQuadratic.At(fields, signs, 1, words);
        numerator.Clear(); denominator.Clear();
        numerator.Rational[1] = (ulong)(3 * (481 + delta)); numerator.Radical[1] = radicalSign == 0 ? 0UL : (ulong)(481 + delta);
        numerator.Signs[0] = 1; numerator.Signs[1] = radicalSign;
        denominator.Rational[0] = 3UL << 20; denominator.Radical[0] = radicalSign == 0 ? 0UL : 1UL << 20;
        denominator.Signs[0] = 1; denominator.Signs[1] = radicalSign;
        ReadOnlySpan<ulong> radical = stackalloc ulong[1] { 2 };
        // The native squared RAW gap is (481/2^20)*2^64=481*2^44.
        // Both 3+sqrt(2) and
        // 3-sqrt(2) are positive, so canceling the common factor leaves
        // an exact order of -delta, including a genuinely irrational tie.
        int comparison = CurvedPrismRimContacts.CompareSquaredDepth(geometry, rim, numerator, denominator, radical);
        Assert.Equal(-delta, Math.Sign(comparison));
        if (radicalSign == 0)
            Assert.Equal(-delta, Math.Sign(CurvedPrismRimContacts.CompareSquaredDepth(geometry, rim, numerator, denominator, ReadOnlySpan<ulong>.Empty)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExplicitLineReportsItsOwnDepthWithoutChangingTheAdmittedNativeMinimum(bool cone)
    {
        Vector3d center = new(Fixed64.FromFraction(3, 8), Fixed64.Zero, Fixed64.Half);
        Vector2d[] footprint = { new(-2, -2), new(0, -2), new(0, 2), new(-2, 2) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.FromFraction(5, 8), Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, cone, vertices);
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 1, Interval(1, 2, 1, 1), -Fixed64.Two, Fixed64.Two,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out var proof, out bool physical));
        Assert.True(physical); Assert.Equal(Fixed64.Quarter, proof.Depth);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(1, proof, true) };
        Fixed64 y = cone ? -Fixed64.One : Fixed64.Zero;
        Span<PrismCurvedBoundaryLine> lines = stackalloc PrismCurvedBoundaryLine[1] { new(1, Interval(1, 2, 1, 2), y, y) };
        int visits = 0, order = 0; FixedContactAnchors finite = default;
        void Visit(int ordinal, in FixedContactAnchors contact, int comparison) { visits++; order = comparison; finite = contact; }
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(default, default, certificates: certificates),
            center, FixedQuaternion.Identity, Vector3d.Zero, out var native, out _, out _, out _,
            lines, Visit, cone ? new ConePrismBoundaryContacts() : null));
        Assert.Equal(1, visits); Assert.Equal(Fixed64.Zero, finite.Depth); Assert.True(order < 0);
        Assert.Equal(Fixed64.Quarter, native.Depth);
        // The line at z=0 only touches the initial circle because
        // (3/8)^2+(1/2)^2=(5/8)^2. Math reports both values exactly; source
        // topology decides whether this line is an independent alternative.
        Assert.Equal(Vector3d.Left, native.Normal);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AnIndependentFiniteLineSurvivesWhenItsParentFaceIsNotAdmitted(bool cone, bool fullSupport)
    {
        Vector3d center = new(Fixed64.FromFraction(3, 8), Fixed64.Zero, Fixed64.Half);
        Vector2d[] footprint = { new(-2, -2), new(0, -2), new(0, 2), new(-2, 2) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.FromFraction(5, 8), Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, cone, vertices);
        Fixed64 y = cone ? -Fixed64.One : Fixed64.Zero;
        var clip = fullSupport ? Interval(5, 8, 5, 8) : Interval(1, 2, 1, 2);
        Span<PrismCurvedBoundaryLine> lines = stackalloc PrismCurvedBoundaryLine[1] { new(1, clip, y, y) };
        int visits = 0, order = 0; FixedContactAnchors finite = default;
        void Visit(int ordinal, in FixedContactAnchors contact, int comparison) { visits++; order = comparison; finite = contact; }
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(default, default), center, FixedQuaternion.Identity,
            Vector3d.Zero, out _, out _, out _, out _, lines, Visit, cone ? new ConePrismBoundaryContacts() : null));
        Assert.Equal(1, visits); Assert.True(order < 0);
        Assert.Equal(fullSupport ? Fixed64.Quarter : Fixed64.Zero, finite.Depth);
        Assert.Equal(Vector3d.Left, finite.Normal);
        Assert.True(finite.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(Fixed64.Zero, y, fullSupport ? Fixed64.Half : Fixed64.Zero), source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void APhysicalEndpointStratumRetainsTheRadialCornerMinimum(bool upper)
    {
        Vector3d center = new(Fixed64.Quarter, upper ? Fixed64.Two : -Fixed64.Two, Fixed64.Quarter);
        Vector2d[] footprint = { new(-2, -2), new(0, -2), new(0, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, false, vertices);
        int cap = upper ? -2 : -1;
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, cap, default, -Fixed64.Two, Fixed64.Two,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out var proof, out bool physical));
        Assert.True(physical); Assert.Equal(Fixed64.One, proof.Depth);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(cap, proof, true) };
        var endpoint = upper ? Interval(1, 1, 1, 1) : Interval(0, 1, 0, 1);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(8, endpoint, true, upper ? 5 : 4, 0, 0) };
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default, certificates: certificates), center,
            FixedQuaternion.Identity, Vector3d.Zero, out var contact, out int face, out physical, out _));
        Assert.True(physical); Assert.Equal(-3, face);
        Fixed64 expected = Fixed64.Half - FixedMath.Sqrt(Fixed64.Two) * Fixed64.Quarter;
        Assert.True(FixedMath.Abs(contact.Depth - expected) <= Fixed64.FromRaw(2));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(Fixed64.Zero, upper ? Fixed64.Two : -Fixed64.Two, Fixed64.Zero), source);
    }

    [Fact]
    public void ARealEdgeAdmitsTheObliqueConeGeneratorBeforeRimSelection()
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(-1, 0, -2), origin = new(0, 0, -2);
        Vector2d[] footprint = { new(0, 0), new(1, -1), new(2, 0), new(1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)3, origin, Fixed64.Zero, footprint, (Fixed64)5, true, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(2, default, false, 0, 0, 0) };
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, origin,
            out var contact, out int face, out bool physical, out _));
        Assert.True(physical); Assert.Equal(-3, face); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(1, 5)) <= Fixed64.FromRaw(2));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(FixedMath.Abs(source.X) <= Fixed64.FromRaw(4)); Assert.Equal((Fixed64)(-2), source.Z);
        // The open source edge permits any deterministic witness in the
        // exact initial-cone interval, not just its lower parameter endpoint.
        Assert.True(source.Y >= Fixed64.FromFraction(-13, 4) - Fixed64.FromRaw(4)
            && source.Y <= Fixed64.FromFraction(37, 24) + Fixed64.FromRaw(4));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AnExcludedGeneratorEndpointCannotMakeAnOpenSourceIntervalPhysical(bool onlyExcludedSupport)
    {
        const long k = 1_358_187_913;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(3 * k));
        Vector3d center = new(-1, 0, -2), origin = new(0, 0, -2);
        Vector2d[] footprint = { new(0, 0), new(1, -1), new(2, 0), new(1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)4, (Fixed64)3, origin, Fixed64.Zero, footprint, (Fixed64)5, true, vertices);
        // The original vertical edge has Y=-5+10*lambda. Initial cone
        // membership ends at Y=37/24 (lambda157/240); translated support
        // ends at Y=8/5 (lambda33/50). Neither excluded endpoint belongs
        // to the open retained interval above it.
        var interval = onlyExcludedSupport ? Interval(33, 50, 7, 10) : Interval(157, 240, 7, 10);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(2, interval, false, 0, 0, 0) };
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, origin,
            out _, out _, out _, out _));
        bool far = CurvedPrismContact.TryGetContact(geometry, new(edges, default, includeFarBoundary: true), center, rotation,
            origin, out var contact, out _, out bool physical, out _);
        Assert.Equal(!onlyExcludedSupport, far);
        if (far)
        {
            Assert.False(physical); Assert.Equal(Vector3d.Right, contact.Normal);
            Assert.True(FixedMath.Abs(contact.Depth - Fixed64.FromFraction(1, 5)) <= Fixed64.FromRaw(2));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
            // Exact strict admission precedes final nearest rounding. A
            // mediant arbitrarily near the open boundary may publish that
            // boundary's lattice coordinate without becoming physical.
            Assert.True(source.Y >= Fixed64.FromFraction(37, 24) && source.Y <= Fixed64.FromFraction(8, 5));
        }
    }

    [Fact]
    public void PoleWindowOutsideTheInitialSectionCanStillSupplyAFarSupport()
    {
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        Vector3d center = new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), origin = Vector3d.Right;
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        var clip = Interval(9, 32, 5, 16); // retained z=[3/8,7/16].
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(geometry, 3, clip, Fixed64.Zero, Fixed64.Zero,
            center, rotation, origin, false, out _, out _));
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 3, clip, Fixed64.Zero, Fixed64.Zero,
            center, rotation, origin, true, out var contact, out bool physical));
        // The initial section has radius 5/16 while the translated base disk
        // has radius 1/2. Its nearest z=3/8 point is a genuine far witness.
        Assert.False(physical); Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(3, 8)), source);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ADisjointFarEdgeCannotInventAnAxialSupportOutsideTheFiniteBody(bool cone, bool oblique)
    {
        Vector3d origin = new(4, 5, 0);
        FixedQuaternion rotation = oblique ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One, Fixed64.Two).Normalized : FixedQuaternion.Identity;
        Vector2d[] footprint = { new(0, 0), new(1, -1), new(2, 0), new(1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(Vector3d.Zero, rotation, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.One, cone, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(2, default, false, 0, 0, 0) };
        // This original vertical edge is y in(4,6). Every body orientation
        // fits a sphere of radius sqrt(5)/2, and horizontal translation
        // cannot move its finite support point up into this retained edge.
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(edges, default, includeFarBoundary: true),
            Vector3d.Zero, rotation, origin, out _, out _, out _, out _));
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(geometry, -1, default, -Fixed64.One, Fixed64.One,
            Vector3d.Zero, rotation, origin, true, out _, out _));
    }

    [Fact]
    public void ZeroFaceCertificateFinishesBeforeOtherAdmittedEdgeFamilies()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Half);
        Vector2d[] footprint = { new(-Fixed64.Quarter, (Fixed64)(-2)), new(Fixed64.Quarter, (Fixed64)(-2)),
            new(Fixed64.Quarter, Fixed64.Zero), new(-Fixed64.Quarter, Fixed64.Zero) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, 2, default, -Fixed64.Half, Fixed64.Half,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out var proof, out bool physical));
        Assert.True(physical); Assert.Equal(Fixed64.Zero, proof.Depth);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(2, proof, true) };
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(7, default, false, 5, 0, 0) };
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default, certificates: certificates), center,
            FixedQuaternion.Identity, Vector3d.Zero, out var contact, out int face, out physical, out int certificate));
        Assert.Equal(2, face); Assert.Equal(0, certificate); Assert.True(physical);
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Vector3d.Backward, contact.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryDepthComparisonUsesTheNonzeroComponentAfterLeadingZeros(bool vertical)
    {
        Vector3d center = new(-Fixed64.FromFraction(3, 8), Fixed64.Zero, -Fixed64.FromFraction(3, 8));
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.One, Fixed64.FromFraction(5, 8), Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.FromFraction(3, 8), false, vertices);
        Assert.True(CurvedPrismContact.TryGetFaceSupportContact(geometry, -2, default, -Fixed64.One, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Zero, false, out var proof, out bool physical));
        Assert.True(physical);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[1] { new(-2, proof, true) };
        var line = vertical ? new PrismCurvedBoundaryLine(-2, default, Fixed64.FromFraction(3, 8), Fixed64.FromFraction(3, 8), 2)
            : new PrismCurvedBoundaryLine(2, Interval(0, 1, 1, 2), Fixed64.Zero, Fixed64.Zero);
        Span<PrismCurvedBoundaryLine> lines = stackalloc PrismCurvedBoundaryLine[1] { line };
        int visits = 0;
        void Visit(int ordinal, in FixedContactAnchors contact, int comparison)
        { visits++; Assert.Equal(0, comparison); Assert.Equal(vertical ? Vector3d.Down : Vector3d.Backward, contact.Normal); Assert.Equal(Fixed64.FromFraction(7, 8), contact.Depth); }
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(default, default, certificates: certificates), center,
            FixedQuaternion.Identity, Vector3d.Zero, out _, out _, out _, out _, lines, Visit));
        Assert.Equal(1, visits);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeRimClassifiesItsWitnessAgainstTheOriginalSlopedSolid(bool includeFar)
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024),
            Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024));
        Vector2d[] footprint = { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, true, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(7, default, false, 5, 0, 0) };
        bool found = CurvedPrismContact.TryGetContact(geometry, new(edges, default, includeFarBoundary: includeFar), center,
            rotation, Vector3d.Zero, out var contact, out _, out bool physical, out _);
        Assert.Equal(includeFar, found);
        if (!found) return;
        Assert.False(physical); Assert.Equal(Fixed64.FromRaw(91_988_268), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half), source);
        // Same base rim as the cylinder fixture, but the original cone's
        // radius at axial -5+9/1024 is smaller than this source radial norm.
        Fixed64 axial = (Fixed64)(-5) + Fixed64.FromFraction(9, 1024);
        Fixed64 x = (Fixed64)15 - Fixed64.FromFraction(12, 1024), z = (Fixed64)(-20) + Fixed64.FromFraction(16, 1024);
        Fixed64 radius = Fixed64.FromFraction(5, 2) * ((Fixed64)5 - axial);
        Assert.True(x * x + z * z > radius * radius);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void ACompletelyRetainedNativeBoundaryPreservesTheOrdinaryWholeSolidMinimum(bool cone, bool oblique, bool translated)
    {
        Vector3d origin = translated ? new(Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero, Fixed64.MinValue + (Fixed64)4) : Vector3d.Zero;
        Vector3d center = origin + (oblique ? new Vector3d(-Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter)
            : new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.FromFraction(5, 4)));
        FixedQuaternion rotation = oblique ? new FixedQuaternion(Fixed64.One, Fixed64.Two, (Fixed64)3, (Fixed64)4).Normalized : FixedQuaternion.Identity;
        Fixed64 yaw = oblique ? Fixed64.FromFraction(1, 7) : Fixed64.Zero;
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Fixed64 radius = oblique ? Fixed64.FromFraction(3, 4) : Fixed64.Half;
        Fixed64 half = oblique ? Fixed64.Half : Fixed64.Two;
        Assert.True(CurvedPrismContact.TryGetContact(center, rotation, Vector3d.Up, Fixed64.Two, radius,
            origin, yaw, footprint, half, cone, out var ordinary));
        if (!oblique) Assert.True(FixedMath.Abs(ordinary.Depth - (Fixed64.Half - FixedMath.Sqrt(Fixed64.Two) * Fixed64.Quarter)) <= Fixed64.FromRaw(2));
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up, Fixed64.Two, radius,
            origin, yaw, footprint, half, cone, vertices);
        Span<PrismCurvedFaceCertificate> certificates = stackalloc PrismCurvedFaceCertificate[6]; int certificateCount = 0;
        for (int face = -2; face < 4; face++)
            if (CurvedPrismContact.TryGetFaceSupportContact(geometry, face, default, -half, half,
                center, rotation, origin, true, out var proof, out bool physical))
                certificates[certificateCount++] = new(face, proof, physical);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[36]; int count = 0;
        for (int edge = 0; edge < geometry.EdgeCount; edge++)
        {
            geometry.Edge(edge, out int first, out int last);
            edges[count++] = new(edge, Interval(0, 1, 0, 1), true, first, 0, 0);
            edges[count++] = new(edge, default, false, first, 0, 0);
            edges[count++] = new(edge, Interval(1, 1, 1, 1), true, last, 0, 0);
        }
        // With every face, real-edge interior and endpoint retained, the
        // source domain is exactly the original closed convex prism. Exact
        // admission/filtering must preserve its original minimum value.
        Assert.True(CurvedPrismContact.TryGetContact(geometry,
            new(edges[..count], default, includeFarBoundary: true, certificates: certificates[..certificateCount]),
            center, rotation, origin, out var retained, out _, out _, out _));
        Assert.Equal(ordinary.Depth, retained.Depth); Assert.False(retained.DepthIsClamped);
        Assert.True(retained.FirstAnchor.TryGetPoint(out _)); Assert.True(retained.SecondAnchor.TryGetPoint(out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FarRimWitnessCanLieBeyondEitherOriginalAxialCap(bool reverseAxis)
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-(reverseAxis ? 2 : 1) * k), Fixed64.FromRaw((reverseAxis ? 1 : 2) * k));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2), Fixed64.FromFraction(83, 4)), origin = new(0, 12, 20);
        Vector2d[] footprint = { new((Fixed64)(-20), -Fixed64.Half), new((Fixed64)20, -Fixed64.Half),
            new((Fixed64)20, Fixed64.Half), new((Fixed64)(-20), Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, origin, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1] { new(7, default, false, 5, 0, 0) };
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(edges, default), center, rotation, origin, out _, out _, out _, out _));
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, default, includeFarBoundary: true), center, rotation, origin,
            out var contact, out _, out bool physical, out _));
        Assert.False(physical);
        // In the Y/Z projection the source is at (0,-1/4) relative to an
        // ellipse of radii20/25. Its nearest admitted point has offsets
        // (-5*sqrt1295/9,-25/36), giving depth sqrt3599/3 exactly.
        Assert.True(FixedMath.Abs(contact.Depth - FixedMath.Sqrt((Fixed64)3599) / (Fixed64)3) <= Fixed64.FromRaw(2));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        // Root anchors first round primitive-local coordinates, then their
        // final world projection. An orthogonal rotation bounds each world
        // error by (sqrt(3)+1)/2 raw units, conservatively below two raw.
        Assert.True(FixedMath.Abs(source.Y - Fixed64.FromFraction(25, 2)) <= Fixed64.FromRaw(2));
        Assert.True(FixedMath.Abs(source.Z - Fixed64.FromFraction(41, 2)) <= Fixed64.FromRaw(2));
        Fixed64 axial = Fixed64.FromFraction(4, 5) * (source.X + (Fixed64)5)
            + Fixed64.FromFraction(reverseAxis ? -3 : 3, 5) * (source.Y - center.Y);
        Assert.True(reverseAxis ? axial < (Fixed64)(-5) : axial > (Fixed64)5);
    }

    [Fact]
    public void ACollinearAuthoredVertexRetainsTheGenericRimZero()
    {
        const long k = 99_855_961;
        FixedQuaternion rotation = new(Fixed64.FromRaw(16 * k), Fixed64.Zero, Fixed64.FromRaw(-15 * k), Fixed64.FromRaw(37 * k));
        Assert.True(rotation.IsNormalized());
        Vector3d center = new(36, 3, 4);
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 0), new(1, 1), new(-1, 1) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[10];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)50, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, false, vertices);
        Assert.False(geometry.TryGetEdgeNormalCone(8, out _, out _));
        CurvedPrismRimContacts.GetContact(geometry, default, Fixed64.Zero, false, out var rim, out Fixed64 depth, out bool clamped, hasAnalytic: false);
        // Axis=(15,12,16)/25. Base offset(-15,-12,-16) and radial
        // offset(-20,9,12) put the exact Left support at (1,0,0).
        Assert.True(rim.HasValue); Assert.True(rim.IsZero); Assert.Equal(Fixed64.Zero, depth); Assert.False(clamped);
        CurvedPrismRimContacts.GetMaterials(geometry, rim, center, rotation, Vector3d.Zero,
            out Vector3d normal, out _, out FixedPointAnchor source);
        Assert.Equal(Vector3d.Left, normal); Assert.True(source.TryGetPoint(out Vector3d point));
        Assert.Equal(Vector3d.Right, point);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void RimAdmissionProjectsOntoTheRetainedEdgesNonzeroCoordinate(int pivot)
    {
        const long k = 1_920_767_767;
        bool z = pivot == 2;
        FixedQuaternion rotation = z ? new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k))
            : new(Fixed64.FromRaw(k), Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(2 * k));
        Vector3d center = z ? new((Fixed64)(-5), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024), Fixed64.FromFraction(41, 2) - Fixed64.FromFraction(16, 1024))
            : new(-Fixed64.FromFraction(41, 2) + Fixed64.FromFraction(16, 1024), Fixed64.FromFraction(31, 2) - Fixed64.FromFraction(15, 1024), (Fixed64)(-5));
        Vector2d[] footprint = z ? new[] { new Vector2d((Fixed64)(-10), -Fixed64.Half), new Vector2d(Fixed64.Zero, -Fixed64.Half), new Vector2d(Fixed64.Zero, Fixed64.Half), new Vector2d((Fixed64)(-10), Fixed64.Half) }
            : new[] { new Vector2d(-Fixed64.Half, (Fixed64)(-10)), new Vector2d(Fixed64.Half, (Fixed64)(-10)), new Vector2d(Fixed64.Half, (Fixed64)10), new Vector2d(-Fixed64.Half, (Fixed64)10) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, false, vertices);
        Span<WideDirection3d> fan = stackalloc WideDirection3d[1] { geometry.Plane(0) };
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1]
            { z ? new(4, Interval(1, 1, 1, 1), true, 5, 0, 1) : new(10, default, false, 7, 0, 0) };
        // The Z-edge singleton's incident Right covector excludes the closer
        // axial cap, while admitting the zero-X rim normal. The Y-pivot case
        // is the original stationary fixture rotated90 degrees around Up.
        Assert.True(CurvedPrismContact.TryGetContact(geometry, new(edges, fan), center, rotation, Vector3d.Zero,
            out var contact, out _, out bool physical, out _));
        Assert.True(physical); Assert.Equal(Fixed64.FromRaw(91_988_268), contact.Depth);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.Equal(z ? new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Half) : new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero), source);
    }

    [Fact]
    public void RetainedLowerEdgeCutRejectsAUniqueConeSupportBelowItsWindow()
    {
        Vector3d center = new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter);
        Vector2d[] footprint = { new(-2, -2), new(0, -2), new(0, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, true, vertices);
        Span<PrismCurvedEdgeWindow> edges = stackalloc PrismCurvedEdgeWindow[1]
            { new(8, Interval(1, 2, 1, 1), false, 4, 0, 0) };
        // The original vertical corner edge spans y=[-2,2], but this open
        // retained interval is y in(0,2). Its horizontal cone supports are
        // unique base points at y=-1: both incident wall equalities match,
        // then the lower retained bound must reject the constant witness.
        Assert.False(CurvedPrismContact.TryGetContact(geometry, new(edges, default, includeFarBoundary: true), center,
            FixedQuaternion.Identity, Vector3d.Zero, out _, out _, out _, out _));
    }
}
