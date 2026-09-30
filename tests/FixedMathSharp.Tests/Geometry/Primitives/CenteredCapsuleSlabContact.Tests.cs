//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleSlabContactTests
{
    private static readonly FixedQuaternion Horizontal = new(Fixed64.Zero, Fixed64.Zero,
        Fixed64.FromRaw(-3_037_000_500L), Fixed64.FromRaw(3_037_000_500L));

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void StraightRim_ClassifiesExactGapBeforeRounding(long offset)
    {
        // Endpoint (43/4,2,0) is 5/4 from (10,1,0), with residual (3/4,1,0).
        bool hit = Query((Fixed64)10, Fixed64.Two, Fixed64.One,
            new Vector3d(Fixed64.FromFraction(83, 4), Fixed64.Two, Fixed64.Zero),
            Horizontal, (Fixed64)20, Fixed64.FromFraction(5, 4) + Fixed64.FromRaw(offset),
            out Vector3d normal, out Fixed64 depth, out bool clamped);
        Assert.Equal(offset >= 0, hit);
        Assert.Equal(offset >= 0
            ? new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero)
            : Vector3d.Zero, normal);
        Assert.Equal(Fixed64.FromRaw(offset >= 0 ? offset : 0), depth);
        Assert.False(clamped);
    }

    [Fact]
    public void SeparatedRim_RejectsIncompleteDirectionFalsePositive()
    {
        Assert.False(Query((Fixed64)10, Fixed64.Two, Fixed64.One,
            new Vector3d(Fixed64.FromFraction(83, 4), Fixed64.FromFraction(7, 4), Fixed64.Zero),
            Horizontal, (Fixed64)20, Fixed64.One, out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector3d.Zero, normal);
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 1)]
    [InlineData(2, -1)]
    [InlineData(2, 1)]
    public void AxisFeatures_ReturnWholeShapeMinimum(int axis, int sign)
    {
        Fixed64 distance = Fixed64.FromFraction(axis == 0 ? 7 : 11, 4) * sign;
        Vector3d center = axis == 0 ? new(distance, Fixed64.Zero, Fixed64.Zero)
            : axis == 1 ? new(Fixed64.Zero, distance, Fixed64.Zero) : new(Fixed64.Zero, Fixed64.Zero, distance);
        Vector3d expected = axis == 0 ? Vector3d.Right : axis == 1 ? Vector3d.Up : Vector3d.Forward;
        Assert.True(Query(Fixed64.One, Fixed64.Two, Fixed64.One, center, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(expected * sign, normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), depth);
        Assert.False(clamped);
    }

    [Fact]
    public void Containment_DoesNotCombineConstituentExitDepths()
    {
        Assert.True(Query(Fixed64.Two, (Fixed64)4, (Fixed64)3, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.One, Fixed64.Half, out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.FromFraction(5, 2), depth);
        Assert.Equal(Fixed64.One, normal.X.Abs());
        Assert.Equal(Fixed64.Zero, normal.Y);
        Assert.Equal(Fixed64.Zero, normal.Z);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void ObliqueInteriorRim_RadiusBridgesNegativeCoreGap(long offset)
    {
        const long scale = 858_993_459L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(scale), Fixed64.FromRaw(2 * scale),
            Fixed64.FromRaw(-4 * scale), Fixed64.FromRaw(2 * scale));
        // Axis (20,-9,-12)/25 is perpendicular to n=(0,4,-3)/5.
        // Center=(0,1,-2)+5*n: the negative end rim owns the closest point.
        bool hit = Query(Fixed64.One, Fixed64.Two, Fixed64.One, new Vector3d(0, 5, -5), rotation,
            Fixed64.Two, (Fixed64)5 + Fixed64.FromRaw(offset), out Vector3d normal, out Fixed64 depth, out bool clamped);
        Assert.Equal(offset >= 0, hit);
        Assert.Equal(offset >= 0 ? new Vector3d(Fixed64.Zero, Fixed64.FromFraction(4, 5),
            -Fixed64.FromFraction(3, 5)) : Vector3d.Zero, normal);
        Assert.Equal(Fixed64.FromRaw(offset >= 0 ? offset : 0), depth);
        Assert.False(clamped);
    }

    private static bool Query(Fixed64 radius, Fixed64 coreLength, Fixed64 halfThickness,
        Vector3d center, FixedQuaternion rotation, Fixed64 capsuleCore, Fixed64 capsuleRadius,
        out Vector3d normal, out Fixed64 depth, out bool clamped) =>
        WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
            Vector3d.Zero, Fixed64.Zero, coreLength, radius, halfThickness,
            center, rotation, capsuleCore, capsuleRadius, out normal, out depth, out clamped);

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    public void ProjectedRim_UsesWholeDiskAndCapsuleInterior(bool horizontal, long offset)
    {
        FixedQuaternion rotation = horizontal ? Proportional(2, 1, -1, 2) : Proportional(13, -4, -1, 8);
        Vector3d center = new(horizontal ? -24 : 24, 25, 37);
        // Whole-slab support at n=(±9,20,12)/25 is (±15,5,25).
        // The residual has length 25 and is perpendicular to the capsule
        // axis. Horizontal and oblique projections exercise different owners.
        bool hit = Query((Fixed64)25, (Fixed64)10, (Fixed64)5, center, rotation, Fixed64.Two,
            (Fixed64)25 + Fixed64.FromRaw(offset), out Vector3d normal, out Fixed64 depth, out bool clamped);
        Assert.Equal(offset >= 0, hit);
        Assert.Equal(offset >= 0 ? new Vector3d(Fixed64.FromFraction(horizontal ? -9 : 9, 25),
            Fixed64.FromFraction(4, 5), Fixed64.FromFraction(12, 25)) : Vector3d.Zero, normal);
        Assert.Equal(Fixed64.FromRaw(Math.Max(offset, 0)), depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(2, 1)]
    [InlineData(3, 2)]
    [InlineData(5, 2)]
    public void TinyOddCore_PreservesHalfRawDepthAndNearestEvenRounding(long core, long expectedDepth)
    {
        Assert.True(Query(Fixed64.One, Fixed64.FromRaw(core), Fixed64.Two, Vector3d.Forward,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero, out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector3d.Forward, normal);
        Assert.Equal(Fixed64.FromRaw(expectedDepth), depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ZeroCore_UsesExistingCylinderTieAndFrameContract(bool separated)
    {
        Vector3d center = new(separated ? 12 : 1, 2, -3);
        FixedQuaternion rotation = Proportional(13, -4, -1, 8);
        bool expected = WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Signed192.Raw((Fixed64)6), (Fixed64)2,
            center, rotation, Vector3d.Up, Fixed64.Two, Fixed64.One,
            out Vector3d expectedNormal, out Fixed64 expectedDepth, out bool expectedClamped);
        bool actual = WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
            Vector3d.Zero, Fixed64.FromFraction(7, 13), Fixed64.Zero, (Fixed64)2, (Fixed64)3,
            center, rotation, Fixed64.Two, Fixed64.One, out Vector3d normal, out Fixed64 depth, out bool clamped);
        Assert.Equal(!separated, expected);
        Assert.Equal(expected, actual);
        Assert.Equal(expectedNormal, normal); Assert.Equal(expectedDepth, depth); Assert.Equal(expectedClamped, clamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OppositeLimitCenters_RetainCancellationAndConceptualOverflow(bool clampedExpected)
    {
        Vector3d origin = new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);
        Fixed64 radius = clampedExpected ? Fixed64.MaxValue : Fixed64.FromRaw(long.MaxValue - Fixed64.One.m_rawValue);
        Assert.True(WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
            origin, Fixed64.Zero, Fixed64.MaxValue, radius, Fixed64.MaxValue,
            origin, FixedQuaternion.Identity, Fixed64.MaxValue, Fixed64.One,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.Equal(clampedExpected, clamped);
        Assert.Equal(Fixed64.One, normal.X.Abs());
        Assert.Equal(Fixed64.Zero, normal.Y); Assert.Equal(Fixed64.Zero, normal.Z);

        // The relative X offset exceeds scalar range; even maximum radii leave
        // a one-raw gap between centers MinValue and MaxValue.
        Assert.False(WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero), Fixed64.Zero,
            Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.MaxValue, out normal, out depth, out clamped));
        Assert.Equal(Vector3d.Zero, normal); Assert.Equal(Fixed64.Zero, depth); Assert.False(clamped);
    }

    [Fact]
    public void ZeroRadiusSlab_IsRectangleNotDisconnectedEndSegments()
    {
        Assert.True(Query(Fixed64.Zero, (Fixed64)4, Fixed64.One, Vector3d.Right,
            Proportional(2, 1, -1, 2), Fixed64.Two, Fixed64.One,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        // The core crosses the rectangle's X=0 plane after moving left 1/5;
        // adding radius gives depth 4/5 while Y/Z stay within its extents.
        Assert.Equal(Vector3d.Right, normal);
        Assert.Equal(Fixed64.FromFraction(4, 5), depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueRim_PreservesDirtyCallerAndWarmedZeroAllocation(bool largeRotated)
    {
        FixedQuaternion rotation = Proportional(13, -4, -1, 8);
        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try
            {
                var first = AssertDirtyCallerContact(rotation, 0xA55A0FF012345678UL, largeRotated);
                var second = AssertDirtyCallerContact(rotation, 0x5AA5F00FFEDCBA98UL, largeRotated);
                Assert.Equal(first, second);
            }
            catch (Exception exception) { failure = exception; }
        }, 1024 * 1024);
        worker.Start(); worker.Join(); Assert.Null(failure);
        bool hit = false;
        Fixed64 depth = default;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
            hit = Query((Fixed64)25, (Fixed64)10, (Fixed64)5, new Vector3d(24, 25, 37), rotation, Fixed64.Two,
                (Fixed64)26, out _, out depth, out _));
        Assert.True(hit); Assert.Equal(Fixed64.One, depth); Assert.Equal(0, allocated);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (Vector3d Normal, Fixed64 Depth) AssertDirtyCallerContact(FixedQuaternion rotation, ulong seed, bool largeRotated)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++) caller[index] = seed ^ (ulong)index;
        bool hit;
        Vector3d normal;
        Fixed64 depth;
        bool clamped;
        if (largeRotated)
        {
            Vector3d origin = new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);
            Fixed64 radius = Fixed64.FromRaw(long.MaxValue / 4);
            hit = WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
                origin, Fixed64.FromFraction(7, 13), Fixed64.MaxValue, radius, radius,
                origin, rotation, Fixed64.MaxValue, Fixed64.One, out normal, out depth, out clamped);
            // The core contains a radius-R ball. Its local-X support is at
            // most R+Max/2, so depth lies strictly between R and Max.
            Assert.InRange(depth.m_rawValue, radius.m_rawValue + Fixed64.One.m_rawValue, long.MaxValue - 1);
            Assert.True(normal.IsNormalized());
        }
        else
        {
            hit = Query((Fixed64)25, (Fixed64)10, (Fixed64)5, new Vector3d(24, 25, 37), rotation, Fixed64.Two,
                (Fixed64)26, out normal, out depth, out clamped);
            Assert.Equal(Fixed64.One, depth);
            Assert.Equal(new Vector3d(Fixed64.FromFraction(9, 25), Fixed64.FromFraction(4, 5), Fixed64.FromFraction(12, 25)), normal);
        }
        for (int index = 0; index < caller.Length; index++) Assert.Equal(seed ^ (ulong)index, caller[index]);
        Assert.True(hit); Assert.False(clamped);
        return (normal, depth);
    }

    private static FixedQuaternion Proportional(int x, int y, int z, int w)
    {
        Fixed64 scale = Fixed64.One / FixedMath.Sqrt((Fixed64)(x * x + y * y + z * z + w * w));
        FixedQuaternion result = new(scale * x, scale * y, scale * z, scale * w);
        Assert.True(result.IsNormalized());
        return result;
    }

    [Theory]
    [InlineData(392, 545, 135)]
    [InlineData(356, 518, 180)]
    [InlineData(500, 626, 0)]
    public void ObliqueEndRegion_RanksPositiveAndZeroStationaryGaps(int y, int z, int expected)
    {
        // Rotate the independently solved cylinder ellipse fixture by an exact
        // quarter-turn, then shift center +Z and add slab core [-Z,+Z]. The
        // support increases by |n.Z|-n.Z >=0 and is unchanged at its known
        // minimum n=(0,4,3)/5. The 180 case has two positive stationary roots.
        Assert.True(Query((Fixed64)625, Fixed64.Two, (Fixed64)500, new Vector3d(0, y, z),
            Proportional(4, -2, 1, 2), (Fixed64)2000, Fixed64.Zero,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(4, 5), Fixed64.FromFraction(3, 5)), normal);
        Assert.Equal((Fixed64)expected, depth); Assert.False(clamped);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void ObliquePositiveGap_ClampsOnlyConceptualOverflow(long offset)
    {
        Fixed64 radius = Fixed64.FromRaw(long.MaxValue - ((Fixed64)135).m_rawValue + offset);
        Assert.True(Query((Fixed64)625, Fixed64.Two, (Fixed64)500, new Vector3d(0, 392, 545),
            Proportional(4, -2, 1, 2), (Fixed64)2000, radius,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.FromRaw(long.MaxValue + Math.Min(offset, 0)), depth);
        Assert.Equal(offset > 0, clamped);
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(4, 5), Fixed64.FromFraction(3, 5)), normal);
    }

    [Theory]
    [InlineData(-1, -1, false)]
    [InlineData(-1, 1, false)]
    [InlineData(1, -1, false)]
    [InlineData(1, 1, false)]
    [InlineData(1, 1, true)]
    public void EndpointRim_AdmitsWholeShapeClosestPoint(int cap, int end, bool sphere)
    {
        // Closest support point is (0,cap,2*end), with residual (0,cap,end).
        Assert.True(Query(Fixed64.One, Fixed64.Two, Fixed64.One,
            new Vector3d(0, (sphere ? 2 : 3) * cap, 3 * end), FixedQuaternion.Identity,
            sphere ? Fixed64.Zero : Fixed64.Two, Fixed64.Two,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.Two - FixedMath.Sqrt(Fixed64.Two), depth);
        Fixed64 component = FixedMath.Sqrt(Fixed64.Half);
        Assert.Equal(new Vector3d(Fixed64.Zero, component * cap, component * end), normal);
        Assert.False(clamped);
    }

    [Fact]
    public void ObliqueEndRegion_RoundsIrrationalWholeShapeMinimum()
    {
        // The projected ellipse has radii 625 and 225 and major-axis offset
        // 40*sqrt(34). Its minimum is 225*sqrt(1-54400/340000)=45*sqrt(21).
        // Integer-square bracketing, not this solver, gives the final raw value.
        Assert.True(Query((Fixed64)625, Fixed64.Two, (Fixed64)500, new Vector3d(0, 446, 273),
            Proportional(4, -2, 1, 2), (Fixed64)2000, Fixed64.Zero,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.FromRaw(885_690_573_358L), depth);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(-934_769_957L), Fixed64.FromRaw(3_949_768_979L),
            Fixed64.FromRaw(1_404_376_806L)), normal);
        Assert.False(clamped);
    }

    [Fact]
    public void RepeatedStationaryRoot_DoesNotDisplaceShallowerBoundary()
    {
        // Adding [-Z,+Z] and shifting the known cylinder fixture by +Z
        // preserves its Forward minimum while retaining its double critical root.
        Assert.True(Query((Fixed64)3125, Fixed64.Two, (Fixed64)900, new Vector3d(0, 0, 1025),
            Proportional(0, 0, -1, 2), (Fixed64)10000, Fixed64.Zero,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector3d.Forward, normal); Assert.Equal((Fixed64)2101, depth); Assert.False(clamped);
    }

    [Theory]
    [InlineData(false, 0, 2)]
    [InlineData(false, 1, 4)]
    [InlineData(true, 2, -1)]
    [InlineData(true, 3, 0)]
    [InlineData(true, 4, 2)]
    public void ObliqueHalfRawGap_RoundsCompleteOffsetNearestEven(bool outside, long radius, long expected)
    {
        // Rim p=(0,500,625.5) raw and n=(0,4,3)/5. Both centers
        // p±2.5*n are integral raw coordinates. The complete signed depth,
        // not the independently rounded core gap, determines midpoint parity.
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(outside ? 502 : 498), Fixed64.FromRaw(outside ? 627 : 624));
        bool hit = Query(Fixed64.FromRaw(625), Fixed64.MinIncrement, Fixed64.FromRaw(500), center,
            Proportional(4, -2, 1, 2), Fixed64.FromRaw(2000), Fixed64.FromRaw(radius),
            out Vector3d normal, out Fixed64 depth, out bool clamped);
        Assert.Equal(expected >= 0, hit);
        Assert.Equal(Fixed64.FromRaw(Math.Max(expected, 0)), depth); Assert.False(clamped);
        Assert.Equal(expected >= 0 ? new Vector3d(Fixed64.Zero, Fixed64.FromFraction(4, 5), Fixed64.FromFraction(3, 5))
            : Vector3d.Zero, normal);
    }

    [Fact]
    public void ZeroRadiusRim_CertifiesDegenerateRectangleClosestPoint()
    {
        Assert.True(Query(Fixed64.Zero, Fixed64.Two, Fixed64.One, new Vector3d(1, 3, 3),
            FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)3,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.Zero, depth); Assert.False(clamped);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(1, 3), Fixed64.FromFraction(2, 3), Fixed64.FromFraction(2, 3)), normal);
    }

    [Theory]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    public void CurvedSeparator_CanCrossPositiveAnalyticGaps(long radius)
    {
        Vector3d center = new(Fixed64.Zero, (Fixed64)500 + Fixed64.FromRaw(4), (Fixed64)626 + Fixed64.FromRaw(3));
        bool hit = Query((Fixed64)625, Fixed64.Two, (Fixed64)500, center,
            Proportional(4, -2, 1, 2), (Fixed64)2000, Fixed64.FromRaw(radius),
            out Vector3d normal, out Fixed64 depth, out bool clamped);
        Assert.Equal(radius >= 5, hit); Assert.Equal(Fixed64.FromRaw(Math.Max(radius - 5, 0)), depth); Assert.False(clamped);
        Assert.Equal(radius >= 5 ? new Vector3d(Fixed64.Zero, Fixed64.FromFraction(4, 5), Fixed64.FromFraction(3, 5))
            : Vector3d.Zero, normal);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void ChartEndpointTie_PreservesEarlierExactZeroFeature(int radius)
    {
        Assert.True(Query(Fixed64.One, Fixed64.Two, Fixed64.One, new Vector3d(0, 1, 2),
            Proportional(4, 0, -7, 5), Fixed64.Two, (Fixed64)radius,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal((Fixed64)radius, depth); Assert.False(clamped);
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(3_037_000_500L), Fixed64.FromRaw(3_037_000_500L)), normal);
    }

    [Fact]
    public void EllipseCenter_PrincipalAxisOwnsSimultaneouslyStationaryTerms()
    {
        Assert.True(Query((Fixed64)625, Fixed64.Two, (Fixed64)500, new Vector3d(0, 500, 1),
            Proportional(4, -2, 1, 2), (Fixed64)4000, Fixed64.Zero,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal((Fixed64)225, depth); Assert.False(clamped);
        // (-45,136,27)/(25*sqrt(34)); exact integer squared-midpoint bounds
        // independently determine all three rounded components.
        Assert.Equal(new Vector3d(Fixed64.FromRaw(-1_325_845_466L), Fixed64.FromRaw(4_006_999_631L),
            Fixed64.FromRaw(795_507_280L)), normal);
    }

    [Fact]
    public void TwoStationaryRootsInOneChart_BothLoseToShallowerBoundary()
    {
        // Reducing the double-critical fixture's H from 900 to 899 splits
        // its repeated root. E(t)=800000*t/sqrt(9+16*t*t)-102400-57536*t
        // has signs -,+,- at 3/4,1,2 and is strictly concave. The reciprocal
        // chart contains both roots. The independent whole-shape minimum
        // remains 2101 (the support-minus-2101 Bernstein bound is positive).
        Assert.True(Query((Fixed64)3125, Fixed64.Two, (Fixed64)899, new Vector3d(-1024, 0, -1),
            Proportional(1, -2, -1, 2), (Fixed64)20000, Fixed64.Zero,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(-Vector3d.Right, normal); Assert.Equal((Fixed64)2101, depth); Assert.False(clamped);
    }

    [Fact]
    public void WideCap_RejectsFalseEndpointRimMinimumInsideRadialDisk()
    {
        Assert.True(Query((Fixed64)100, Fixed64.Two, Fixed64.One, new Vector3d(0, 5, 2),
            Proportional(4, -2, 1, 2), Fixed64.One, (Fixed64)5,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector3d.Up, normal); Assert.Equal(Fixed64.FromFraction(59, 50), depth); Assert.False(clamped);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ParallelCapsule_BetweenRoundedEndsUsesStraightSide(int sign)
    {
        Assert.True(Query(Fixed64.One, Fixed64.Two, Fixed64.One,
            new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.Zero, Fixed64.Half * sign),
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(Vector3d.Right, normal); Assert.Equal(Fixed64.FromFraction(1, 4), depth); Assert.False(clamped);
    }

    [Fact]
    public void SphereAboveStraightRim_UsesCompleteDiagonalCorrection()
    {
        Assert.True(Query(Fixed64.One, Fixed64.Two, Fixed64.One, new Vector3d(2, 2, 0),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Two,
            out Vector3d normal, out Fixed64 depth, out bool clamped));
        Fixed64 component = FixedMath.Sqrt(Fixed64.Half);
        Assert.Equal(new Vector3d(component, component, Fixed64.Zero), normal);
        Assert.Equal(Fixed64.Two - FixedMath.Sqrt(Fixed64.Two), depth); Assert.False(clamped);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RotatedTinyPositiveCore_RetainsAuthoredSideFrame(long core)
    {
        Fixed64 yaw = Fixed64.FromFraction(7, 13);
        Assert.True(WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
            Vector3d.Zero, yaw, Fixed64.Two, Fixed64.One, (Fixed64)4, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, out Vector3d expectedNormal, out Fixed64 expectedDepth, out _));
        Assert.Equal(Fixed64.FromFraction(3, 2), expectedDepth);
        Assert.True(expectedNormal.X > Fixed64.Half && expectedNormal.Z > Fixed64.Zero);
        Assert.True(WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
            Vector3d.Zero, yaw, Fixed64.FromRaw(core), Fixed64.One, (Fixed64)4, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.Half, out Vector3d normal, out Fixed64 depth, out bool clamped));
        Assert.Equal(expectedNormal, normal); Assert.Equal(expectedDepth, depth); Assert.False(clamped);
    }
}
