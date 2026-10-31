using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleCapsulePlaneRayFrameTests
{
    private const int Words = 9;
    private static ContactQuadratic Field(Span<ulong> values, Span<int> signs, int index) =>
        ContactQuadratic.At(values, signs, index, Words);
    private static void SetInteger(ContactQuadratic value, long integer) =>
        value.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(integer))));
    private static FixedTriangle Triangle => new(Vector3d.Zero, Vector3d.Right * 2, Vector3d.Forward * 2);
    private static CapsuleFiniteSectionFrame Finite => new(Vector3d.Zero, FixedQuaternion.Identity,
        Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);

    private static void SetPoint(in CapsulePlaneRayFrame frame, Vector3d authored,
        scoped ContactQuadratic x, scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d)
    {
        WideAxis3 chart = frame.Finite.Transform(authored);
        x.Set(Signed576.ExtendValue(chart.X)); y.Set(Signed576.ExtendValue(chart.Y));
        z.Set(Signed576.ExtendValue(chart.Z)); SetInteger(d, 1);
    }

    [Theory]
    [InlineData(1, 0, .5, true)]
    [InlineData(1.5, 0, 1, false)]
    [InlineData(-.25, 0, 0, false)]
    [InlineData(0, 0, -.25, false)]
    [InlineData(.25, .25, .25, false)]
    [InlineData(0, 0, 0, true)]
    [InlineData(1, 0, 1, true)]
    [InlineData(0, 0, 2, true)]
    public void FiniteTriangleAdmission_ShouldKeepClosedEdgesAndRejectPlaneOrWallViolations(
        double px, double py, double pz, bool expected)
    {
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1);
        ContactQuadratic z = Field(values, signs, 2), d = Field(values, signs, 3);
        foreach (FixedTriangle triangle in new[] { Triangle, new FixedTriangle(Triangle.A, Triangle.C, Triangle.B) })
        {
            var frame = new CapsulePlaneRayFrame(triangle, Finite);
            SetPoint(frame, new Vector3d((Fixed64)px, (Fixed64)py, (Fixed64)pz), x, y, z, d);
            Assert.Equal(expected, frame.ContainsTrianglePoint(triangle, x, y, z, d, ReadOnlySpan<ulong>.Empty));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void FiniteTriangleAdmission_ShouldRejectANonpositiveHomogeneousDenominator(int denominator)
    {
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1);
        ContactQuadratic z = Field(values, signs, 2), d = Field(values, signs, 3);
        var frame = new CapsulePlaneRayFrame(Triangle, Finite);
        SetPoint(frame, Vector3d.Zero, x, y, z, d); SetInteger(d, denominator);
        Assert.False(frame.ContainsTrianglePoint(Triangle, x, y, z, d, ReadOnlySpan<ulong>.Empty));
    }

    [Fact]
    public void DegeneratePlane_ShouldNotAdmitAFiniteTrianglePoint()
    {
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1);
        ContactQuadratic z = Field(values, signs, 2), d = Field(values, signs, 3);
        var frame = new CapsulePlaneRayFrame(default, Finite);
        SetPoint(frame, Vector3d.Zero, x, y, z, d);
        Assert.False(frame.ContainsTrianglePoint(Triangle, x, y, z, d, ReadOnlySpan<ulong>.Empty));
        Assert.Equal(Vector3d.Zero, frame.GetWorldNormal(FixedQuaternion.Identity));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PointOrdering_ShouldUseAuthoredLexicographicCoordinatesAcrossCapsuleFrames(bool rotated)
    {
        FixedQuaternion rotation = rotated
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized
            : FixedQuaternion.Identity;
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(3, -2, 1), rotation, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(Triangle, finite);
        Span<ulong> values = stackalloc ulong[16 * Words]; Span<int> signs = stackalloc int[16];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        ContactQuadratic ox = Field(values, signs, 4), oy = Field(values, signs, 5), oz = Field(values, signs, 6), od = Field(values, signs, 7);
        SetPoint(frame, Vector3d.Zero, x, y, z, d);
        foreach (Vector3d other in new[] { new Vector3d(1, 10, 10), new Vector3d(0, 1, 10), Vector3d.Forward, Vector3d.Zero })
        {
            SetPoint(frame, other, ox, oy, oz, od);
            int expected = other.IsZero ? 0 : -1;
            Assert.Equal(expected, frame.ComparePoints(x, y, z, d, ReadOnlySpan<ulong>.Empty,
                ox, oy, oz, od, ReadOnlySpan<ulong>.Empty));
            Assert.Equal(-expected, frame.ComparePoints(ox, oy, oz, od, ReadOnlySpan<ulong>.Empty,
                x, y, z, d, ReadOnlySpan<ulong>.Empty));
        }
    }

    [Fact]
    public void PointOrdering_ShouldPassAnExactIndependentRootTieToTheNextCoordinate()
    {
        var frame = new CapsulePlaneRayFrame(Triangle, Finite);
        Span<ulong> values = stackalloc ulong[16 * Words]; Span<int> signs = stackalloc int[16];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        ContactQuadratic ox = Field(values, signs, 4), oy = Field(values, signs, 5), oz = Field(values, signs, 6), od = Field(values, signs, 7);
        SetPoint(frame, Vector3d.Zero, x, y, z, d); SetPoint(frame, Vector3d.Forward, ox, oy, oz, od);
        x.Radical[0] = (ulong)(2 * Fixed64.One.m_rawValue); x.Signs[1] = 1;
        ox.Radical[0] = (ulong)Fixed64.One.m_rawValue; ox.Signs[1] = 1;
        Span<ulong> root = stackalloc ulong[1] { 2 }, otherRoot = stackalloc ulong[1] { 8 };
        // 2*sqrt(2)=sqrt(8), so the independently represented X tie must
        // preserve the exact Z ordering instead of choosing a rounded X.
        Assert.Equal(-1, frame.ComparePoints(x, y, z, d, root, ox, oy, oz, od, otherRoot));
        Assert.Equal(1, frame.ComparePoints(ox, oy, oz, od, otherRoot, x, y, z, d, root));
    }

    [Fact]
    public void SharedBodyCapSeam_ShouldBreakDepthTiesByExactPointThenPrimitive()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -4, -4), new Vector3d(0, 4, -4), new Vector3d(0, 0, 4));
        var frame = new CapsulePlaneRayFrame(triangle, Finite);
        Span<ulong> selectionValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> selectionSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(selectionValues, selectionSigns, valueSigns);
        Span<ulong> values = stackalloc ulong[12 * Words]; Span<int> signs = stackalloc int[12];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        ContactQuadratic n = Field(values, signs, 4), divisor = Field(values, signs, 5);
        SetPoint(frame, Vector3d.Up, x, y, z, d); SetInteger(n, 2 * Fixed64.One.m_rawValue); SetInteger(divisor, 1);
        // A horizontal ray at Y=+1 exits at the C1 body/cap seam. Both
        // primitive descriptions own the same point and unit exit depth.
        selection.Keep(x, y, z, d, n, divisor, ReadOnlySpan<ulong>.Empty, 2, frame);
        selection.Keep(x, y, z, d, n, divisor, ReadOnlySpan<ulong>.Empty, 0, frame);
        Assert.Equal(0, selection.Primitive);
        selection.Keep(x, y, z, d, n, divisor, ReadOnlySpan<ulong>.Empty, 2, frame);
        Assert.Equal(0, selection.Primitive);
        SetPoint(frame, Vector3d.Down, x, y, z, d);
        selection.Keep(x, y, z, d, n, divisor, ReadOnlySpan<ulong>.Empty, 1, frame);
        Assert.Equal(1, selection.Primitive);
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(Vector3d.Down, point);
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth)); Assert.Equal(Fixed64.One, depth);
        SetPoint(frame, Vector3d.Up, x, y, z, d);
        selection.Keep(x, y, z, d, n, divisor, ReadOnlySpan<ulong>.Empty, 0, frame);
        Assert.Equal(1, selection.Primitive);
        SetPoint(frame, Vector3d.Up + Vector3d.Forward * Fixed64.Half, x, y, z, d);
        n.Clear(); n.Radical[0] = (ulong)Fixed64.One.m_rawValue; n.Signs[1] = 1;
        Span<ulong> root = stackalloc ulong[1] { 3 };
        // At Z=1/2 the seam's horizontal radius is sqrt(3)/2, so this
        // smaller exit cannot replace the retained whole unit ray.
        selection.Keep(x, y, z, d, n, divisor, root, 2, frame);
        Assert.Equal(1, selection.Primitive);
        Assert.True(selection.TryGetDepth(frame, out depth)); Assert.Equal(Fixed64.One, depth);
    }

    [Fact]
    public void AuthoredPointOutsideItsFinalCoordinateDomain_ShouldBeRejected()
    {
        Vector3d origin = Vector3d.Right * Fixed64.MinValue, center = Vector3d.Right * Fixed64.MaxValue;
        var finite = new CapsuleFiniteSectionFrame(origin, FixedQuaternion.Identity, center,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(Triangle, finite);
        Span<ulong> selectionValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> selectionSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(selectionValues, selectionSigns, valueSigns);
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        SetInteger(x, 0); SetInteger(y, 0); SetInteger(z, 0); SetInteger(d, 1);
        Assert.True(CapsulePlaneRayEvents.AccumulateRationalPoint(frame, x, y, z, d, 1, ref selection));
        Assert.False(selection.TryGetAuthoredPoint(frame, origin, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(Vector3d.Zero, point);
    }

    [Theory]
    [InlineData(true, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 0)]
    [InlineData(false, 0)]
    public void IsolatedRayValue_ShouldPreserveBodyCapSeamsAndSphereAnchors(bool positiveCore, int seam)
    {
        var triangle = new FixedTriangle(new Vector3d(0, -4, -4), new Vector3d(0, 4, -4), new Vector3d(0, 0, 4));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, positiveCore ? Fixed64.Two : Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> selectionValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> selectionSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> selectionValueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(selectionValues, selectionSigns, selectionValueSigns);
        Span<ulong> values = stackalloc ulong[14 * Words]; Span<int> signs = stackalloc int[14];
        ContactQuadratic x = Field(values, signs, 0), y = Field(values, signs, 1), z = Field(values, signs, 2), d = Field(values, signs, 3);
        ContactQuadratic a = Field(values, signs, 4), b = Field(values, signs, 5), c = Field(values, signs, 6);
        Span<ulong> rayValues = stackalloc ulong[CapsulePlaneRayValue.StorageWords];
        Span<sbyte> raySigns = stackalloc sbyte[CapsulePlaneRayValue.SignCount];
        var ray = new CapsulePlaneRayValue(rayValues, raySigns);
        SetInteger(a, 0); SetInteger(b, 1); SetInteger(c, -4 * Fixed64.One.m_rawValue);
        // An isolated value can simplify to the same exact horizontal unit
        // exit as a quadratic ratio; representation must not change seam ties.
        Assert.True(ray.TrySetUpperRoot(a, b, c, ReadOnlySpan<ulong>.Empty, 34));
        Vector3d basePoint = Vector3d.Up * (Fixed64)seam;
        SetPoint(frame, basePoint, x, y, z, d);
        int primitive = positiveCore ? seam < 0 ? 1 : seam > 0 ? 2 : 0 : 2;
        selection.KeepValue(x, y, z, d, ReadOnlySpan<ulong>.Empty, ray, primitive, frame);
        if (positiveCore)
            selection.KeepValue(x, y, z, d, ReadOnlySpan<ulong>.Empty, ray, 0, frame);
        Assert.Equal(positiveCore ? 0 : 2, selection.Primitive);
        Assert.True(selection.TryGetExitAnchor(frame, 1, Vector3d.Zero, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
        Assert.False(clamped); Assert.Equal(Fixed64.One, depth);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(basePoint + Vector3d.Right, exit);
        if (positiveCore && seam != 0)
        {
            SetPoint(frame, -basePoint, x, y, z, d);
            selection.KeepValue(x, y, z, d, ReadOnlySpan<ulong>.Empty, ray, 0, frame);
            Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, out Vector3d retained));
            Assert.Equal(Vector3d.Down, retained);
        }
    }
}
