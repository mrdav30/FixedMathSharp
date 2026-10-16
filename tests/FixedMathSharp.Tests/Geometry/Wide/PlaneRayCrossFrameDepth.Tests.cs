using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class PlaneRayCrossFrameDepthTests
{
    private static readonly FixedTriangle Plane = new(new Vector3d(-4, 0, -4),
        new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
    private static readonly FixedQuaternion NegativeIdentity = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, -Fixed64.One);

    [Fact]
    public void ConePhysicalDepthComparisonPreservesApexReflectionAndUnequalMemberScales()
    {
        FixedTriangle triangle = new(new Vector3d(-4, 4, -4), new Vector3d(4, -4, -4), new Vector3d(0, 0, 4));
        ConePlaneRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        ConePlaneRayFrame otherFrame = new(triangle, Vector3d.Zero, NegativeIdentity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> opv = stackalloc ulong[ConePlaneRaySelection.StorageWords], onv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ops = stackalloc int[ConePlaneRaySelection.SignCount], ons = stackalloc int[ConePlaneRaySelection.SignCount];
        ConePlaneRaySelection positive = new(pv, ps), negative = new(nv, ns), otherPositive = new(opv, ops), otherNegative = new(onv, ons);
        Assert.True(ConePlaneRayTestQueries.AccumulateTriangle(triangle, frame, ref positive, ref negative));
        Assert.True(ConePlaneRayTestQueries.AccumulateTriangle(triangle, otherFrame, ref otherPositive, ref otherNegative));
        Assert.Equal(0, positive.CompareDepth(otherPositive));
        Assert.Equal(0, negative.CompareDepth(otherNegative));
        // For x+y=0, the apex reaches x+y=2 and the opposite base rim x+y=-4.
        Assert.Equal(-1, positive.CompareDepth(otherNegative));
        Assert.Equal(1, negative.CompareDepth(otherPositive));
        ulong[] retainedPositive = pv.ToArray(), retainedNegative = onv.ToArray();
        int[] retainedPositiveSigns = ps.ToArray(), retainedNegativeSigns = ons.ToArray();
        ConePlaneRayEventSource positiveSource = positive.MaximumSource, negativeSource = otherNegative.MaximumSource;
        ConePlaneRayEvent positiveEvent = positive.MaximumEvent, negativeEvent = otherNegative.MaximumEvent;
        int result = 0;
        Assert.Equal(0, FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            ConePlaneRaySelection a = new(retainedPositive, retainedPositiveSigns), b = new(retainedNegative, retainedNegativeSigns);
            a.Restore(frame, positiveSource, positiveEvent); b.Restore(otherFrame, negativeSource, negativeEvent);
            result = a.CompareDepth(b);
        }));
        Assert.Equal(-1, result);
    }

    [Fact]
    public void ConeDepthComparisonPreservesSubRawDifferenceAcrossUnequalCoordinateScales()
    {
        ConePlaneRayFrame firstFrame = new(Plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        ConePlaneRayFrame secondFrame = new(Plane, Vector3d.Zero, NegativeIdentity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> firstValues = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> secondValues = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> firstSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> secondSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        ConePlaneRaySelection first = new(firstValues, firstSigns), second = new(secondValues, secondSigns);
        Span<ulong> values = stackalloc ulong[4 * ConePlaneRaySelection.FieldWords];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, ConePlaneRaySelection.FieldWords);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 1, ConePlaneRaySelection.FieldWords);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        ConePlaneRayPoint point = new(pointValues, pointSigns); point.Set(default);
        // Physical raw depth is n/(d*2D): both start at exactly one raw unit.
        Assert.False(firstFrame.Finite.ShapeFrame.Denominator.Equals(secondFrame.Finite.ShapeFrame.Denominator));
        n.Set(Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.AddSigned192(firstFrame.Finite.ShapeFrame.Denominator,
            firstFrame.Finite.ShapeFrame.Denominator)))); d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
        first.Keep(n, d, ReadOnlySpan<ulong>.Empty, firstFrame, point);
        n.Set(Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.AddSigned192(secondFrame.Finite.ShapeFrame.Denominator,
            secondFrame.Finite.ShapeFrame.Denominator))));
        second.Keep(n, d, ReadOnlySpan<ulong>.Empty, secondFrame, point);
        Assert.Equal(0, first.CompareDepth(second));
        // Add 2^-81 raw depth. Rounded depth equality cannot establish this order.
        n.Clear(); d.Clear();
        n.Rational[1] = 1UL << 17; n.Rational[0] = 1;
        d.Rational[1] = 1UL << 16;
        n.Signs[0] = d.Signs[0] = 1;
        first = new(firstValues, firstSigns);
        first.Keep(n, d, ReadOnlySpan<ulong>.Empty, firstFrame, point);
        Assert.Equal(1, first.CompareDepth(second));
        Assert.Equal(-1, second.CompareDepth(first));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SlabDepthComparisonNormalizesConditionedFramesAndOppositeRays(bool polygon)
    {
        FixedTriangle sourcePlane = new(new Vector3d(-4, 4, -4), new Vector3d(4, -4, -4), new Vector3d(0, 0, 4));
        FiniteSlabRayFrame firstFrame = new(sourcePlane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        FiniteSlabRayFrame secondFrame = new(sourcePlane, Vector3d.Zero, NegativeIdentity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        Assert.NotEqual(firstFrame.RayNormalSquared, secondFrame.RayNormalSquared);
        Vector2d[] offsets = polygon ? new[] { new Vector2d(-2, -2), new Vector2d(2, -2),
            new Vector2d(2, 2), new Vector2d(-2, 2) } : Array.Empty<Vector2d>();
        Span<ulong> firstValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<ulong> secondValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> firstSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        Span<sbyte> secondSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection first = new(firstValues, firstSigns), second = new(secondValues, secondSigns);
        FiniteSlabRayExtrema.AccumulateTriangle(firstFrame, sourcePlane, offsets, 1, ref first);
        FiniteSlabRayExtrema.AccumulateTriangle(secondFrame, sourcePlane, offsets, -1, ref second);
        Assert.True(first.HasValue && second.HasValue);
        Assert.Equal(0, first.CompareDepthAcrossFrames(second, firstFrame, secondFrame));
        // On x+y=0, the full domain reaches x=1/2 (depth 3*sqrt(2)/2),
        // whereas this finite x<=0 domain has depth sqrt(2).
        FixedTriangle narrow = new(new Vector3d(-1, 1, -1), new Vector3d(0, 0, -1), new Vector3d(0, 0, 1));
        second = new(secondValues, secondSigns);
        FiniteSlabRayExtrema.AccumulateTriangle(secondFrame, narrow, offsets, 1, ref second);
        Assert.Equal(1, first.CompareDepthAcrossFrames(second, firstFrame, secondFrame));
        Assert.Equal(-1, second.CompareDepthAcrossFrames(first, secondFrame, firstFrame));
        ulong[] retainedFirst = firstValues.ToArray(), retainedSecond = secondValues.ToArray();
        sbyte[] retainedFirstSigns = firstSigns.ToArray(), retainedSecondSigns = secondSigns.ToArray();
        FiniteSlabRaySelectionMetadata firstMetadata = first.Metadata, secondMetadata = second.Metadata;
        int result = 0;
        Assert.Equal(0, FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            FiniteSlabRaySelection a = new(retainedFirst, retainedFirstSigns), b = new(retainedSecond, retainedSecondSigns);
            a.Restore(firstMetadata); b.Restore(secondMetadata);
            result = a.CompareDepthAcrossFrames(b, firstFrame, secondFrame);
        }));
        Assert.Equal(1, result);
    }

    [Theory]
    [InlineData(0, -1, false, false, false, false)]
    [InlineData(1, 1, false, false, false, false)]
    [InlineData(0, 0, true, false, false, false)]
    [InlineData(0, -1, false, true, false, false)]
    [InlineData(1, 1, false, true, false, false)]
    [InlineData(0, 0, false, false, true, false)]
    [InlineData(0, -1, false, false, false, true)]
    [InlineData(1, 1, false, false, false, true)]
    public void SlabDepthComparisonOrdersBothQuadraticBranchesAgainstRationalLp(int ordinal, int expected, bool repeated, bool tiny, bool linear, bool wide)
    {
        FixedTriangle sourcePlane = linear ? new(new Vector3d(-4, -4, 0), new Vector3d(4, -4, 0), new Vector3d(0, 4, 0)) : Plane;
        FiniteSlabRayFrame frame = new(sourcePlane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        const int words = FiniteSlabPolynomial.Words;
        Span<ulong> values = stackalloc ulong[9 * words]; Span<sbyte> signs = stackalloc sbyte[9];
        FiniteSlabPolynomial equation = FiniteSlabPolynomial.At(values, signs, 0, 3);
        equation.Clear(); equation.Set(repeated ? 4 : 142); equation.Set(repeated ? -12 : -384, 1); equation.Set(repeated ? 9 : 256, 2);
        if (tiny)
        {
            // 2^160(3u-2)^2-1: u=2/3 +/- 2^-80/3, so rounded depths coincide.
            equation.Clear(); equation.Values[0] = equation.Values[1] = ulong.MaxValue;
            equation.Values[2] = (4UL << 32) - 1;
            equation.Values[words + 2] = 12UL << 32;
            equation.Values[2 * words + 2] = 9UL << 32;
            equation.Signs[0] = equation.Signs[2] = 1; equation.Signs[1] = -1;
        }
        if (linear) { equation.Clear(); equation.Set(-2); equation.Set(3, 1); }
        if (wide)
        {
            // L=2^2112; P=(4L+1)u²-6Lu+2L has discriminant 4L²-8L >4096 bits.
            // P(1/2)=1/4, P(2/3)=-2L/9+4/9<0, P(1)=1: the roots straddle 2/3.
            equation.Clear(); equation.Values[33] = 2; equation.Values[words + 33] = 6;
            equation.Values[2 * words] = 1; equation.Values[2 * words + 33] = 4;
            equation.Signs[0] = equation.Signs[2] = 1; equation.Signs[1] = -1;
        }
        // Negating the polynomial preserves its ordered roots and checks the leading-sign branch convention.
        if (ordinal == 1)
            for (int index = 0; index < equation.Signs.Length; index++) equation.Signs[index] *= -1;
        Span<ulong> cells = stackalloc ulong[64]; Span<int> shifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(equation.Values, equation.Signs, cells, shifts);
        Assert.Equal(repeated || linear ? 1 : 2, roots.Count);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(equation.Values, equation.Signs)];
        FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, equation.Values, equation.Signs, cell);
        FiniteSlabPolynomial zero = FiniteSlabPolynomial.At(values, signs, 3, 2); zero.Clear();
        FiniteSlabPolynomial n = FiniteSlabPolynomial.At(values, signs, 5, 2); n.Clear(); n.Set(2);
        FiniteSlabPolynomial d = FiniteSlabPolynomial.At(values, signs, 7, 2); d.Clear(); d.Set(3);
        Span<ulong> curvedValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<ulong> lpValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> curvedSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        Span<sbyte> lpSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection curved = new(curvedValues, curvedSigns), lp = new(lpValues, lpSigns);
        curved.Keep(root, roots.Count, zero, zero, zero, d, true, n.Slice(0, 1), d.Slice(0, 1), Plane, 1, 0);
        lp.Keep(root, roots.Count, zero, zero, zero, d, false, n.Slice(0, 1), d.Slice(0, 1), Plane, -1, 0);
        // The ordinary fixture u=3/4 +/- sqrt(2)/16 straddles 2/3.
        // The map t=(2u-1)/(1-u) is increasing in all three exact fixtures.
        Assert.Equal(expected, curved.CompareDepthAcrossFrames(lp, frame, frame));
        Assert.Equal(-expected, lp.CompareDepthAcrossFrames(curved, frame, frame));
    }

    [Fact]
    public void DepthComparisonsRejectEmptySelectionsAndIncompatibleNormalMetrics()
    {
        FixedTriangle tilted = new(new Vector3d(-4, 4, -4), new Vector3d(4, -4, -4), new Vector3d(0, 0, 4));
        ConePlaneRayFrame coneFrame = new(Plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        ConePlaneRayFrame tiltedCone = new(tilted, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        ConePlaneRayFrame zeroCone = new(default, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> coneValues = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> coneSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        ConePlaneRaySelection emptyCone = new(coneValues, coneSigns), validCone = new(coneValues, coneSigns);
        ConePlaneRaySelection differentCone = new(coneValues, coneSigns), degenerateCone = new(coneValues, coneSigns);
        validCone.Restore(coneFrame, default, default); differentCone.Restore(tiltedCone, default, default);
        degenerateCone.Restore(zeroCone, default, default);
        Assert.Equal(typeof(InvalidOperationException), ConeComparisonFailure(emptyCone, validCone));
        Assert.Equal(typeof(InvalidOperationException), ConeComparisonFailure(validCone, emptyCone));
        Assert.Equal(typeof(ArgumentException), ConeComparisonFailure(validCone, differentCone));
        Assert.Equal(typeof(ArgumentException), ConeComparisonFailure(degenerateCone, validCone));

        FiniteSlabRayFrame slabFrame = new(Plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        FiniteSlabRayFrame tiltedSlab = new(tilted, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        FiniteSlabRayFrame zeroSlab = new(default, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        Span<ulong> slabValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> slabSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection emptySlab = new(slabValues, slabSigns), validSlab = new(slabValues, slabSigns);
        validSlab.Restore(new(true, false, 0, 1, 2, 1, Plane, 0, true, true, 1));
        Assert.Equal(typeof(InvalidOperationException), SlabComparisonFailure(emptySlab, validSlab, slabFrame, slabFrame));
        Assert.Equal(typeof(InvalidOperationException), SlabComparisonFailure(validSlab, emptySlab, slabFrame, slabFrame));
        Assert.Equal(typeof(ArgumentException), SlabComparisonFailure(validSlab, validSlab, slabFrame, tiltedSlab));
        Assert.Equal(typeof(ArgumentException), SlabComparisonFailure(validSlab, validSlab, zeroSlab, slabFrame));
    }

    private static Type? ConeComparisonFailure(scoped in ConePlaneRaySelection first, scoped in ConePlaneRaySelection second)
    {
        try { first.CompareDepth(second); return null; }
        catch (Exception exception) { return exception.GetType(); }
    }

    private static Type? SlabComparisonFailure(scoped in FiniteSlabRaySelection first, scoped in FiniteSlabRaySelection second,
        in FiniteSlabRayFrame frame, in FiniteSlabRayFrame otherFrame)
    {
        try { first.CompareDepthAcrossFrames(second, frame, otherFrame); return null; }
        catch (Exception exception) { return exception.GetType(); }
    }
}
