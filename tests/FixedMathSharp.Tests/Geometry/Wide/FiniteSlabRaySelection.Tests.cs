using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class FiniteSlabRaySelectionTests
{
    private static readonly FixedTriangle Floor = new(new Vector3d(-4, 0, -4), new(4, 0, -4), new(0, 0, 4));

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CrossWidthPolynomialCopy_ShouldPreserveSignedCoefficientsAndClearStaleTails(bool wideSource)
    {
        int sourceWords = wideSource ? FiniteSlabPolynomial.AffineWords : FiniteSlabPolynomial.Words;
        int targetWords = wideSource ? FiniteSlabPolynomial.Words : FiniteSlabPolynomial.AffineWords;
        Span<ulong> sourceValues = stackalloc ulong[2 * sourceWords], targetValues = stackalloc ulong[2 * targetWords];
        Span<sbyte> sourceSigns = stackalloc sbyte[2], targetSigns = stackalloc sbyte[2];
        var source = new FiniteSlabPolynomial(sourceValues, sourceSigns);
        var target = new FiniteSlabPolynomial(targetValues, targetSigns);
        source.Set(2); source.Set(-3, 1);
        targetValues.Fill(ulong.MaxValue); targetSigns.Fill(1);
        target.Copy(source);
        Assert.Equal((sbyte)1, targetSigns[0]); Assert.Equal((sbyte)(-1), targetSigns[1]);
        Assert.Equal(2UL, targetValues[0]); Assert.Equal(3UL, targetValues[targetWords]);
        for (int word = 1; word < targetWords; word++)
        {
            Assert.Equal(0UL, targetValues[word]); Assert.Equal(0UL, targetValues[targetWords + word]);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void PolynomialScale_ShouldPreserveExactSignedCoefficientsAndClearUnusedMagnitudeWords(bool wideFactor, bool wideTarget)
    {
        int targetWords = wideTarget ? FiniteSlabPolynomial.AffineWords : FiniteSlabPolynomial.Words;
        int sourceWords = wideTarget ? FiniteSlabPolynomial.Words : FiniteSlabPolynomial.AffineWords;
        Span<ulong> sourceValues = stackalloc ulong[3 * sourceWords], targetValues = stackalloc ulong[3 * targetWords];
        Span<sbyte> sourceSigns = stackalloc sbyte[3], targetSigns = stackalloc sbyte[3];
        var source = new FiniteSlabPolynomial(sourceValues, sourceSigns);
        var target = new FiniteSlabPolynomial(targetValues, targetSigns);
        sourceValues.Fill(ulong.MaxValue);
        source.Set(Signed576.ExtendValue(Signed192.Signed(2)));
        source.Set(Signed832.ExtendValue(Signed192.Signed(-3)), 1);
        // A zero sign makes the padded magnitude irrelevant. Scaling must not
        // copy those stale bytes into a retained zero coefficient.
        sourceSigns[2] = 0;
        for (int factorIndex = -2; factorIndex <= 6; factorIndex++)
        {
            Signed576 factor = factorIndex <= 2 ? Signed576.ExtendValue(Signed192.Signed(factorIndex))
                : new(0, 0, 0, 0, 0, 0, 0, factorIndex == 3 ? 1UL : 0UL, factorIndex == 3 ? 1UL : ulong.MaxValue);
            BigInteger exactFactor = factorIndex <= 2 ? factorIndex
                : factorIndex == 3 ? (BigInteger.One << 64) + 1 : ulong.MaxValue;
            targetValues.Fill(ulong.MaxValue); targetSigns.Fill(1);
            if (wideFactor)
            {
                Signed832 scalar = factorIndex < 5 ? Signed832.ExtendValue(factor)
                    : new(factorIndex == 5 ? 1UL << 62 : 1UL << 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                if (factorIndex >= 5) exactFactor = factorIndex == 5 ? BigInteger.One << 830 : -(BigInteger.One << 831);
                target.Scale(source, scalar);
            }
            else
            {
                if (factorIndex >= 5)
                {
                    factor = new(factorIndex == 5 ? 1UL << 62 : 1UL << 63, 0, 0, 0, 0, 0, 0, 0, 0);
                    exactFactor = factorIndex == 5 ? BigInteger.One << 574 : -(BigInteger.One << 575);
                }
                target.Scale(source, factor);
            }
            for (int coefficient = 0; coefficient < 3; coefficient++)
            {
                BigInteger expected = exactFactor * (coefficient == 0 ? 2 : coefficient == 1 ? -3 : 0);
                Assert.Equal((sbyte)expected.Sign, targetSigns[coefficient]);
                BigInteger magnitude = BigInteger.Abs(expected);
                for (int word = 0; word < targetWords; word++)
                {
                    Assert.Equal((ulong)(magnitude & ulong.MaxValue), targetValues[coefficient * targetWords + word]);
                    magnitude >>= 64;
                }
                Assert.Equal(BigInteger.Zero, magnitude);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SourcePointsUseExactIndependentMemberFrames(bool opposed)
    {
        FixedQuaternion targetRotation = opposed
            ? new(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero) : FixedQuaternion.Identity;
        Vector3d targetOrigin = new(3, 0, 5);
        Vector3d Local(Vector3d world) => opposed
            ? new(targetOrigin.X - world.X, world.Y, targetOrigin.Z - world.Z) : world - targetOrigin;
        FixedTriangle triangle = new(Local(Floor.C), Local(Floor.B), Local(Floor.A));
        FiniteSlabRayFrame source = new(Floor, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        FiniteSlabRayFrame target = new(triangle, targetOrigin, targetRotation,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        FixedTriangle outside = new(triangle.A + new Vector3d(20, 0, 0),
            triangle.B + new Vector3d(20, 0, 0), triangle.C + new Vector3d(20, 0, 0));
        FixedTriangle lifted = new(triangle.A + Vector3d.Up, triangle.B + Vector3d.Up, triangle.C + Vector3d.Up);
        FiniteSlabRayFrame offPlane = new(lifted, targetOrigin, targetRotation,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        Assert.False(maximum.ContainsTrianglePoint(target, triangle, source));
        Assert.False(maximum.TryGetSourcePointInSamplingFrame(source, out _));
        int count = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.ContainsTrianglePoint(target, triangle, source));
            Assert.False(candidate.ContainsTrianglePoint(target, outside, source));
            Assert.False(candidate.ContainsTrianglePoint(offPlane, lifted, source));
            FiniteSlabRayFrame degenerate = new(default, targetOrigin, targetRotation,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
            Assert.False(candidate.ContainsTrianglePoint(degenerate, default, source));
            Assert.True(candidate.TryGetSourcePointInSamplingFrame(source, out Vector3d point));
            Assert.Equal(Fixed64.Zero, point.Y);
            Assert.True(point.X.Abs() <= Fixed64.One && point.Z.Abs() <= Fixed64.One);
            count++;
        }
        FiniteSlabRayExtrema.AccumulatePlane(source, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(count >= 4);
    }

    [Fact]
    public void ExactClosedAdmissionPrecedesSubRawPointRounding()
    {
        Fixed64 raw = Fixed64.FromRaw(1);
        FiniteSlabRayFrame frame = new(Floor, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        FixedTriangle left = new(new Vector3d(0, 0, -1), new(0, 0, 1), new(-1, 0, 0));
        FixedTriangle right = new(left.A, left.B, new Vector3d(1, 0, 0));
        FixedSegment segment = new(Vector3d.Zero, new Vector3d(raw, Fixed64.Zero, Fixed64.Zero));
        RationalSegmentInterval clip = Interval(1, 3);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        int count = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetSourcePointInSamplingFrame(frame, out Vector3d point));
            Assert.Equal(Vector3d.Zero, point);
            Assert.False(candidate.ContainsTrianglePoint(frame, left, frame));
            Assert.True(candidate.ContainsTrianglePoint(frame, right, frame));
            count++;
        }
        FiniteSlabRayExtrema.AccumulateSegment(frame, segment, clip, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(count > 0);
    }

    [Fact]
    public void IrrationalSupportPointKeepsItsClosedPlaneAcrossScaledFrames()
    {
        FixedTriangle plane = new(new Vector3d((Fixed64)(-10), (Fixed64)26.5, (Fixed64)(-10)),
            new Vector3d((Fixed64)10, (Fixed64)6.5, (Fixed64)(-10)),
            new Vector3d(Fixed64.Zero, (Fixed64)(-23.5), (Fixed64)10));
        Vector3d targetOrigin = new(3, 4, 5);
        Vector3d Local(Vector3d world) => new(targetOrigin.X - world.X, world.Y - targetOrigin.Y, targetOrigin.Z - world.Z);
        FixedTriangle targetTriangle = new(Local(plane.C), Local(plane.B), Local(plane.A));
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        FiniteSlabRayFrame source = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, Fixed64.Two);
        FiniteSlabRayFrame target = new(targetTriangle, targetOrigin, rotation,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(source, plane, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum);
        Assert.True(maximum.ContainsTrianglePoint(target, targetTriangle, source));
        Assert.True(maximum.TryGetSourcePointInSamplingFrame(source, out Vector3d point));
        Assert.True(maximum.TryGetContactAnchor(source, out _, out _, out Vector3d authored, out _));
        Assert.Equal(authored, point);
        long planeResidual = point.X.m_rawValue + point.Y.m_rawValue + 2 * point.Z.m_rawValue + ((Fixed64)3.5).m_rawValue;
        Assert.InRange(planeResidual, -2L, 2L);
    }

    [Fact]
    public void UnrepresentableCoreEndPoles_ShouldFailSamplingWithoutLosingRepresentableCardinals()
    {
        // General math admits this stadium: its two Z end poles exceed the
        // scalar range, while its four side cardinals and center still fit.
        FiniteSlabRayFrame frame = new(Floor, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        int retained = 0, unrepresentable = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            if (!candidate.TryGetSourcePointInSamplingFrame(frame, out Vector3d point))
            { unrepresentable++; return; }
            Assert.Equal(Fixed64.Zero, point.Y);
            if (point != Vector3d.Zero)
            {
                Assert.Equal(Fixed64.MaxValue, point.X.Abs());
                Assert.Equal(Fixed64.MaxValue * Fixed64.Half, point.Z.Abs());
            }
            retained++;
        }
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(maximum.HasValue);
        Assert.Equal(5, retained); Assert.Equal(2, unrepresentable);
    }

    [Theory]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    [InlineData(long.MaxValue, false)]
    [InlineData(long.MinValue, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    [InlineData(long.MaxValue, true)]
    [InlineData(long.MinValue, true)]
    public void SamplingOffsetRoundsOnceWithWorldLatticeParity(long centerRaw, bool reverse)
    {
        Vector3d center = new(Fixed64.FromRaw(centerRaw), Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion rotation = reverse
            ? new(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero) : FixedQuaternion.Identity;
        FiniteSlabRayFrame frame = new(Floor, center, rotation, center,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        FixedSegment segment = new(Vector3d.Zero, new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        int count = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetSourcePointInSamplingFrame(frame, out Vector3d point));
            Assert.Equal((centerRaw & 1) == 0 ? 0 : reverse ? -1 : 1, point.X.m_rawValue);
            Assert.Equal(Fixed64.Zero, point.Y); Assert.Equal(Fixed64.Zero, point.Z);
            count++;
        }
        FiniteSlabRayExtrema.AccumulateSegment(frame, segment, Interval(1, 2), ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(count > 0);
    }

    [Fact]
    public void ReentrantVisitor_ShouldPreserveOuterCandidateAndRetainedMaximum()
    {
        FiniteSlabRayFrame frame = new(Floor, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> expectedValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> expectedSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection expected = new(expectedValues, expectedSigns);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, Floor, ReadOnlySpan<Vector2d>.Empty, 1, ref expected);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        int visited = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor exit));
            Assert.True(exit.TryGetPoint(out Vector3d point));
            FixedTriangle wall = new(new Vector3d(0, -3, -3), new(0, 3, -3), new(0, -3, 3));
            FiniteSlabRayFrame nestedFrame = new(wall, Vector3d.Zero, FixedQuaternion.Identity,
                new(Fixed64.Quarter, Fixed64.Half, Fixed64.One), FixedQuaternion.Identity,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One);
            Span<Vector2d> offsets = stackalloc Vector2d[4] { new(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
            Span<ulong> nestedValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
            Span<sbyte> nestedSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
            FiniteSlabRaySelection nested = new(nestedValues, nestedSigns);
            FiniteSlabRayExtrema.AccumulateTriangle(nestedFrame, wall, offsets, -1, ref nested);
            Assert.True(nested.HasValue);
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 afterDepth, out bool afterClamped,
                out Vector3d afterSource, out FixedPointAnchor afterExit));
            Assert.True(afterExit.TryGetPoint(out Vector3d afterPoint));
            Assert.Equal(depth, afterDepth); Assert.Equal(clamped, afterClamped);
            Assert.Equal(source, afterSource); Assert.Equal(point, afterPoint);
            visited++;
        }
        FiniteSlabRayExtrema.AccumulateTriangle(frame, Floor, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(visited > 0);
        Assert.Equal(0, maximum.CompareDepth(expected));
        Assert.True(expected.TryGetContactAnchor(frame, out Fixed64 expectedDepth, out bool expectedClamped,
            out Vector3d expectedSource, out FixedPointAnchor expectedExit));
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 actualDepth, out bool actualClamped,
            out Vector3d actualSource, out FixedPointAnchor actualExit));
        Assert.True(expectedExit.TryGetPoint(out Vector3d expectedPoint));
        Assert.True(actualExit.TryGetPoint(out Vector3d actualPoint));
        Assert.Equal(expectedDepth, actualDepth); Assert.Equal(expectedClamped, actualClamped);
        Assert.Equal(expectedSource, actualSource); Assert.Equal(expectedPoint, actualPoint);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void FreeHeightSupport_ShouldClipAgainstTheFiniteTriangleBeforeEitherCap(int orientation)
    {
        FixedTriangle triangle = new(new Vector3d(Fixed64.Zero, -Fixed64.Half, -Fixed64.One),
            new(Fixed64.Zero, Fixed64.Half, -Fixed64.One), Vector3d.Forward);
        FiniteSlabRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        var sources = new System.Collections.Generic.HashSet<Vector3d>();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped); Assert.Equal(Fixed64.One, depth);
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.Quarter).m_rawValue, Fixed64.Quarter.m_rawValue);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(new Vector3d((Fixed64)orientation, source.Y, Fixed64.Zero), exit);
            sources.Add(source);
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(frame, triangle,
            ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum, Visit);
        Assert.Equal(2, sources.Count);
        Assert.Contains(new Vector3d(Fixed64.Zero, -Fixed64.Quarter, Fixed64.Zero), sources);
        Assert.Contains(new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), sources);
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
    public void PlaneExitDepthOrder_ShouldMatchBothExactPeaksAndThePhysicalSupportMidpoint(
        bool affine, bool rotated, bool reversed)
    {
        Vector3d center = new(20, 3, -5);
        FixedQuaternion rotation = rotated ? FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4)
            : FixedQuaternion.Identity;
        Vector3d scale = affine ? new(2, 3, 4) : Vector3d.One;
        Span<Vector2d> offsets = stackalloc Vector2d[3];
        offsets[0] = reversed ? new(3, -1) : new(1, -1);
        offsets[1] = reversed ? new(1, -1) : new(3, -1);
        offsets[2] = new(1, 2);
        Span<ulong> positiveValues = stackalloc ulong[FiniteSlabRaySelection.GetStorageWords(FiniteSlabPolynomial.AffineWords)];
        Span<ulong> negativeValues = stackalloc ulong[FiniteSlabRaySelection.GetStorageWords(FiniteSlabPolynomial.AffineWords)];
        Span<sbyte> positiveSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        Span<sbyte> negativeSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        for (int shape = 0; shape < 3; shape++)
        {
            // The triangle's X support midpoint is 2, despite its authored
            // origin being 0. Source Z rotation also exercises cancelling Y caps.
            Vector3d origin = center + (shape == 2 ? Vector3d.Right * Fixed64.Two : Vector3d.Zero);
            ScaledRigidFrame3d sourceFrame = new(origin, rotation, Vector3d.Zero, scale,
                rotation, Vector3d.Zero, Vector3d.One);
            for (int side = -1; side <= 1; side++)
            {
                Fixed64 x = (Fixed64)side * Fixed64.Quarter / scale.X;
                FixedTriangle plane = new(new Vector3d(x, (Fixed64)(-3), (Fixed64)(-3)),
                    new(x, (Fixed64)3, (Fixed64)(-3)), new(x, (Fixed64)(-3), (Fixed64)3));
                if (reversed) plane = new(plane.C, plane.B, plane.A);
                FiniteSlabRayFrame frame = new(plane, sourceFrame, center, FixedQuaternion.Identity,
                    shape == 1 ? Fixed64.Two : Fixed64.Zero,
                    shape == 2 ? Fixed64.Zero : Fixed64.One, Fixed64.One);
                ReadOnlySpan<Vector2d> footprint = shape == 2 ? offsets : ReadOnlySpan<Vector2d>.Empty;
                FiniteSlabRaySelection positive = new(positiveValues, positiveSigns, frame.FieldWords);
                FiniteSlabRaySelection negative = new(negativeValues, negativeSigns, frame.FieldWords);
                FiniteSlabRayExtrema.AccumulatePlane(frame, footprint, 1, ref positive);
                FiniteSlabRayExtrema.AccumulatePlane(frame, footprint, -1, ref negative);
                Assert.True(positive.HasValue); Assert.True(negative.HasValue);
                Assert.Equal(-side, frame.ComparePlaneExitDepths(footprint));
                Assert.Equal(positive.CompareDepth(negative), frame.ComparePlaneExitDepths(footprint));
            }
        }
    }

    private static RationalSegmentInterval Interval(long numerator, long denominator) => new(
        Signed320.ExtendValue(Signed192.Signed(numerator)), Signed320.ExtendValue(Signed192.Signed(denominator)),
        Signed320.ExtendValue(Signed192.Signed(numerator)), Signed320.ExtendValue(Signed192.Signed(denominator)));
}
