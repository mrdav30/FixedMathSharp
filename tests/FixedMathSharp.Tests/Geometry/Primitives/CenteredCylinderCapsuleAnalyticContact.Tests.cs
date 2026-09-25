using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderCapsuleAnalyticContactTests
{
    [Theory]
    [InlineData(false, -1L, false)]
    [InlineData(false, 0L, true)]
    [InlineData(false, 1L, true)]
    [InlineData(true, -1L, false)]
    [InlineData(true, 0L, true)]
    [InlineData(true, 1L, true)]
    public void ParallelAxes_PreserveTheCombinedAxialAndRadialRimBoundary(
        bool antiparallel, long radiusOffset, bool expected)
    {
        // The cylinder plus the parallel length-4 core has radius 1 and
        // half-height 3. At (4,7,0), its radial/axial excesses are 3 and 4:
        // exact distance 5, not either separate cap or side projection.
        bool hit = Contact(new Vector3d(4, 7, 0), Fixed64.Two, Fixed64.One,
            (Fixed64)4, (Fixed64)5 + Fixed64.FromRaw(radiusOffset), out FixedContactAnchors contact,
            capsuleAxis: antiparallel ? -Vector3d.Up : Vector3d.Up);

        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(radiusOffset), contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero),
                contact.Normal);
            Assert.False(contact.DepthIsClamped);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ParallelCoreInside_UsesTheNearestCapOrSide(bool capWins)
    {
        // Effective radius and half-height are both 3. The two centers have
        // (side,cap) clearances (2,3) and (3,1), before adding radius 1/2.
        Vector3d center = capWins ? new Vector3d(0, 2, 0) : Vector3d.Right;
        Assert.True(Contact(center, Fixed64.Two, (Fixed64)3, (Fixed64)4, Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(capWins ? Fixed64.FromFraction(3, 2) : Fixed64.FromFraction(5, 2), contact.Depth);
        Assert.Equal(capWins ? Vector3d.Up : Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void ZeroCylinderRadius_ParallelSegmentsRetainEndpointDistance(long radiusOffset, bool expected)
    {
        // The summed core is the segment from y=-3 to y=3. Its distance
        // from (3,7,0) is exactly 5, including the four-unit endpoint excess.
        bool hit = Contact(new Vector3d(3, 7, 0), Fixed64.Two, Fixed64.Zero,
            (Fixed64)4, (Fixed64)5 + Fixed64.FromRaw(radiusOffset), out FixedContactAnchors contact);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(radiusOffset), contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero),
                contact.Normal);
        }
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void ZeroCylinderRadius_PerpendicularCoreDistanceIsNotAnAxialProjection(long radiusOffset, bool expected)
    {
        // The Y and X segments have interior closest points separated by
        // exactly one unit in Z. Translation along +Z is uniquely minimal.
        bool hit = Contact(Vector3d.Forward, Fixed64.Two, Fixed64.Zero,
            Fixed64.Two, Fixed64.One + Fixed64.FromRaw(radiusOffset), out FixedContactAnchors contact,
            capsuleAxis: Vector3d.Right);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(radiusOffset), contact.Depth);
            Assert.Equal(Vector3d.Forward, contact.Normal);
        }
    }

    [Fact]
    public void ZeroCylinderRadius_CollinearCoreOverlapHasOnlyTheCapsuleRadiusDepth()
    {
        Assert.True(Contact(Vector3d.Up, Fixed64.Two, Fixed64.Zero,
            (Fixed64)4, Fixed64.Two, out FixedContactAnchors contact));
        // The summed core contains the center. Any radial direction escapes
        // its radius-2 offset after two units; no axial/core length is added.
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void ZeroCapsuleCore_UsesTheSphereToCylinderRimDistance(long radiusOffset, bool expected)
    {
        // Radius-3, half-height-1 cylinder: (6,5,0) is 5 from (3,1,0).
        // A zero-length core must ignore its otherwise valid authored axis.
        bool hit = Contact(new Vector3d(6, 5, 0), Fixed64.Two, (Fixed64)3,
            Fixed64.Zero, (Fixed64)5 + Fixed64.FromRaw(radiusOffset), out FixedContactAnchors contact,
            capsuleAxis: Vector3d.Right);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(radiusOffset), contact.Depth);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero),
                contact.Normal);
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ZeroCapsuleCore_InteriorSphereAddsItsRadiusToTheNearestCap(int sign)
    {
        Assert.True(Contact(new Vector3d(Fixed64.Half, sign * Fixed64.Half, Fixed64.Zero),
            Fixed64.Two, (Fixed64)3, Fixed64.Zero, Fixed64.FromFraction(1, 4),
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.Equal(sign * Vector3d.Up, contact.Normal);
    }

    [Theory]
    [InlineData(-1L, true, 1L)]
    [InlineData(0L, true, 0L)]
    [InlineData(1L, false, 0L)]
    public void ZeroCapsuleRadius_PreservesSegmentIntrusionAndSideTangency(long offset, bool expected, long depthRaw)
    {
        bool hit = Contact(new Vector3d(Fixed64.One + Fixed64.FromRaw(offset), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Two, Fixed64.One, Fixed64.Two, Fixed64.Zero, out FixedContactAnchors contact);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
            Assert.Equal(Vector3d.Right, contact.Normal);
        }
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    [InlineData(2L, false)]
    public void ZeroRadii_PreserveClosedSegmentContactAtHalfRawEndpoints(long centerRaw, bool expected)
    {
        // Both full lengths are one raw unit. Their summed half-length is
        // exactly one raw unit even though each endpoint is off the lattice.
        bool hit = Contact(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(centerRaw), Fixed64.Zero),
            Fixed64.MinIncrement, Fixed64.Zero, Fixed64.MinIncrement, Fixed64.Zero,
            out FixedContactAnchors contact);
        Assert.Equal(expected, hit);
        if (expected)
            Assert.Equal(Fixed64.Zero, contact.Depth);
    }

    [Theory]
    [InlineData(1L, 0L, 0L)]
    [InlineData(3L, 0L, 2L)]
    [InlineData(1L, 1L, 2L)]
    [InlineData(3L, 1L, 2L)]
    public void AxialDepth_RoundsTheCompleteHalfRawDepthToNearestEven(long cylinderLengthRaw, long radiusRaw, long depthRaw)
    {
        Assert.True(Contact(Vector3d.Zero, Fixed64.FromRaw(cylinderLengthRaw), Fixed64.One,
            Fixed64.Zero, Fixed64.FromRaw(radiusRaw), out FixedContactAnchors contact));
        // Exact depths are 1/2, 3/2, 3/2 and 5/2 raw units. Rounding a
        // half-length before adding the radius changes the last two answers.
        Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(1L, -1L, -1L, false)]
    [InlineData(2L, -1L, 0L, false)]
    [InlineData(1L, 0L, 0L, true)]
    public void AxialDepth_DistinguishesMaximumMinusHalfMaximumAndMaximumPlusHalf(
        long cylinderLengthRaw, long radiusOffset, long expectedOffset, bool clamped)
    {
        Assert.True(Contact(Vector3d.Zero, Fixed64.FromRaw(cylinderLengthRaw), Fixed64.MaxValue,
            Fixed64.Zero, Fixed64.FromRaw(long.MaxValue + radiusOffset), out FixedContactAnchors contact));
        // Max raw is odd: Max-1/2 rounds down to Max-1. Exact Max is not
        // clamped, whereas conceptual Max+1/2 must report clamping.
        Assert.Equal(Fixed64.FromRaw(long.MaxValue + expectedOffset), contact.Depth);
        Assert.Equal(clamped, contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(false, 1L, -1L, false, 0L)]
    [InlineData(false, -1L, 1L, true, 1L)]
    [InlineData(false, 2L, -1L, true, 0L)]
    [InlineData(false, -2L, 1L, true, 0L)]
    [InlineData(true, 1L, -1L, false, 0L)]
    [InlineData(true, -1L, 1L, true, 1L)]
    [InlineData(true, 2L, -1L, true, 0L)]
    [InlineData(true, -2L, 1L, true, 0L)]
    public void ApproximatelyUnitParallelAxes_RetainBothAuthoredHalfAxisLengths(
        bool antiparallel, long cylinderAxisOffset, long capsuleAxisOffset, bool expected, long depthRaw)
    {
        Vector3d cylinderAxis = new(Fixed64.Zero, Fixed64.One + Fixed64.FromRaw(cylinderAxisOffset), Fixed64.Zero);
        Vector3d capsuleAxis = new(Fixed64.Zero, Fixed64.One + Fixed64.FromRaw(capsuleAxisOffset), Fixed64.Zero);
        if (antiparallel)
            capsuleAxis = -capsuleAxis;
        Assert.True(cylinderAxis.IsNormalized());
        Assert.True(capsuleAxis.IsNormalized());
        // H=(2*(1+c*e)+4*(1+s*e))/2 = 3+(c+2s)*e. At y=4,
        // radius 1 leaves exact depth (c+2s)*e, not an assumed zero.
        bool hit = Contact(new Vector3d(0, 4, 0), Fixed64.Two, Fixed64.Two, (Fixed64)4,
            Fixed64.One, out FixedContactAnchors contact, cylinderAxis, capsuleAxis);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
            Assert.Equal(Vector3d.Up, contact.Normal);
        }
    }

    [Theory]
    [InlineData(-1L, true, 1L)]
    [InlineData(0L, true, 0L)]
    [InlineData(1L, false, 0L)]
    public void ApproximatelyUnitParallelAxes_DoNotScaleTheRadialDisk(long offset, bool expected, long depthRaw)
    {
        Vector3d cylinderAxis = new(Fixed64.Zero, Fixed64.One + Fixed64.MinIncrement, Fixed64.Zero);
        Vector3d capsuleAxis = new(Fixed64.Zero, Fixed64.One - Fixed64.MinIncrement, Fixed64.Zero);
        bool hit = Contact(new Vector3d((Fixed64)3 + Fixed64.FromRaw(offset), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Two, Fixed64.Two, (Fixed64)4, Fixed64.One, out FixedContactAnchors contact,
            cylinderAxis, capsuleAxis);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
            Assert.Equal(Vector3d.Right, contact.Normal);
        }
    }

    [Theory]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    public void PerpendicularCoreInside_SelectsTheNearestStraightSide(long xOffset)
    {
        // The cylinder/core sum is a radius-3 stadium along X, extruded
        // between y=-10 and y=10. At x=1, z=1 the nearest boundary is z=3.
        // Its gap is 3-1, while the +X gap is 3+1. Their squared rational
        // parts are both 10; only the opposite radical terms order them.
        // One-raw X changes straddle that exact rational-part equality.
        Assert.True(Contact(new Vector3d(Fixed64.One + Fixed64.FromRaw(xOffset), Fixed64.Zero, Fixed64.One),
            (Fixed64)20, (Fixed64)3, (Fixed64)4, Fixed64.Half,
            out FixedContactAnchors contact, capsuleAxis: Vector3d.Right));

        Assert.Equal(Fixed64.FromFraction(5, 2), contact.Depth);
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void AlmostAxialRimNormal_RoundsItsLargestComponentUpToOne(int xSign)
    {
        // The exact rim residual is (+/-1, e, 0), where e is one raw unit.
        // Its largest normalized component is strictly below 1, but less
        // than half a raw unit below it. The small component still rounds
        // to e, and 2-sqrt(1+e^2) rounds to one unit of penetration.
        Assert.True(Contact(new Vector3d((Fixed64)(2 * xSign), Fixed64.One + Fixed64.MinIncrement, Fixed64.Zero),
            Fixed64.Two, Fixed64.One, Fixed64.Zero, Fixed64.Two,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.Equal(new Vector3d((Fixed64)xSign, Fixed64.MinIncrement, Fixed64.Zero), contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(0L, long.MaxValue)]
    [InlineData(4_294_967_296L, 9_223_372_032_559_808_511L)]
    public void ZeroRadiusCylinder_PreservesMaximumRadiusMinusTheExactCoreGap(long centerRaw, long depthRaw)
    {
        // The parallel summed core is a segment. A center displaced in X
        // has exact distance centerRaw, with no radial disk contribution.
        Assert.True(Contact(new Vector3d(Fixed64.FromRaw(centerRaw), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Two, Fixed64.Zero, Fixed64.Two, Fixed64.MaxValue,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(0L, long.MaxValue)]
    [InlineData(4_611_686_018_427_387_903L, 4_611_686_018_427_387_904L)]
    public void FullRangeParallelCore_PreservesLargePositiveGapBeforeRounding(long centerRaw, long depthRaw)
    {
        // The summed cylinder has exact radius and half-height MaxValue.
        // A radial center displacement leaves MaxValue-centerRaw clearance.
        // The displaced case subtracts large radical terms: a magnitude
        // bound on its squared terms must not overflow the search domain.
        Assert.True(Contact(new Vector3d(Fixed64.FromRaw(centerRaw), Fixed64.Zero, Fixed64.Zero),
            Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero,
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void ObliqueCoreInside_RetainsPositiveAxialAndRadialGapContributions()
    {
        // Quaternion ratio (0,0,1,3) gives exact b=(-3,4,0)/5.
        // On b-perpendicular, the projected cylinder is an ellipse with
        // semiaxes 3 and 12/5, plus a minor-axis segment of half-length 3/20.
        // Its support is concave in the absolute minor normal coordinate,
        // so the minimum is an endpoint: minor 51/20 beats major 3. The
        // cap poles have gap 17/4; long capsule endpoints cannot improve it.
        const long scale = 1_358_187_913L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(scale), Fixed64.FromRaw(3 * scale));
        Assert.True(rotation.IsNormalized());
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Half, (Fixed64)3,
            Vector3d.Zero, rotation, Vector3d.Up, (Fixed64)10, Fixed64.FromFraction(1, 4),
            out FixedContactAnchors contact));

        Assert.Equal(Fixed64.FromFraction(14, 5), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(3, 5), Fixed64.Zero),
            contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    private static bool Contact(Vector3d capsuleCenter, Fixed64 cylinderLength, Fixed64 cylinderRadius,
        Fixed64 capsuleLength, Fixed64 capsuleRadius, out FixedContactAnchors contact,
        Vector3d? cylinderAxis = null, Vector3d? capsuleAxis = null) =>
        FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero, FixedQuaternion.Identity, cylinderAxis ?? Vector3d.Up, cylinderLength, cylinderRadius,
            capsuleCenter, FixedQuaternion.Identity, capsuleAxis ?? Vector3d.Up, capsuleLength, capsuleRadius, out contact);
}
