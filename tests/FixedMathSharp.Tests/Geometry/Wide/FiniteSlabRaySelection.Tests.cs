using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class FiniteSlabRaySelectionTests
{
    private static readonly FixedTriangle Floor = new(new Vector3d(-4, 0, -4), new(4, 0, -4), new(0, 0, 4));

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

    private static RationalSegmentInterval Interval(long numerator, long denominator) => new(
        Signed320.ExtendValue(Signed192.Signed(numerator)), Signed320.ExtendValue(Signed192.Signed(denominator)),
        Signed320.ExtendValue(Signed192.Signed(numerator)), Signed320.ExtendValue(Signed192.Signed(denominator)));
}
