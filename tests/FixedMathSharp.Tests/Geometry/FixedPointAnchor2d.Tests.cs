//=======================================================================
// FixedPointAnchor2d.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FixedPointAnchor2dTests
{
    [Fact]
    public void LocalFeatureIdentity_OrdersEveryStoredComponent()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedPointAnchor2d baseline = new(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Zero);
        FixedPointAnchorTerm2d exactX =
            FixedPointAnchorTerm2d.CreateRadialSupport(
                new Vector2d(rawUnit, Fixed64.Zero),
                rawUnit,
                Vector2d.Zero);
        FixedPointAnchorTerm2d exactY =
            FixedPointAnchorTerm2d.CreateRadialSupport(
                new Vector2d(Fixed64.Zero, rawUnit),
                rawUnit,
                Vector2d.Zero);
        FixedPointAnchor2d[] greaterFeatures =
        {
            new(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(rawUnit, Fixed64.Zero)),
            new(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, rawUnit)),
            new(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                new Vector2d(rawUnit, Fixed64.Zero)),
            new(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                new Vector2d(Fixed64.Zero, rawUnit)),
            new(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Zero,
                exactX),
            new(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Zero,
                exactY),
        };

        Assert.Equal(0, baseline.CompareLocalFeature(baseline));
        foreach (FixedPointAnchor2d greater in greaterFeatures)
        {
            Assert.True(baseline.CompareLocalFeature(greater) < 0);
            Assert.True(greater.CompareLocalFeature(baseline) > 0);
            Assert.NotEqual(
                baseline.GetLocalFeatureHash64(),
                greater.GetLocalFeatureHash64());
        }

        FixedPointAnchor2d exactXCopy = new(
            new Vector2d(7, 8),
            Fixed64.PiOver4,
            Vector2d.Zero,
            Vector2d.Zero,
            exactX);
        Assert.Equal(0, greaterFeatures[4].CompareLocalFeature(exactXCopy));
        Assert.Equal(
            greaterFeatures[4].GetLocalFeatureHash64(),
            exactXCopy.GetLocalFeatureHash64());
        Assert.Equal(
            greaterFeatures[4].GetHashCode(),
            new FixedPointAnchor2d(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Zero,
                exactX).GetHashCode());
    }

    [Fact]
    public void ExactOperations_DeferPointNarrowing()
    {
        Vector2d localPoint = new(
            Fixed64.MaxValue,
            Fixed64.MaxValue);
        FixedPointAnchor2d first = new(
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue),
            Fixed64.PiOver4,
            localPoint);
        FixedPointAnchor2d second = new(
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue),
            Fixed64.PiOver4,
            localPoint);

        Assert.False(first.TryGetPoint(out Vector2d point));
        Assert.Equal(default, point);
        Assert.True(first.TryGetOffsetFrom(second, out Vector2d offset));
        Assert.Equal(Vector2d.Zero, offset);
        Assert.True(first.TryGetLocalPointIn(
            first.Origin,
            first.Rotation,
            out Vector2d sameFramePoint));
        Assert.Equal(first.LocalPoint, sameFramePoint);
    }

    [Fact]
    public void ExactOperations_ReturnFalseAtomicallyWhenFinalResultOverflows()
    {
        FixedPointAnchor2d first = new(
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            Fixed64.PiOver4,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue));
        FixedPointAnchor2d second = new(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            -Fixed64.PiOver4,
            new Vector2d(-Fixed64.MaxValue, -Fixed64.MaxValue));

        Assert.False(first.TryGetOffsetFrom(second, out Vector2d offset));
        Assert.Equal(default, offset);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CompositeLocalPoint_DefersSameAxisOverflowThroughOriginCancellation(
        bool positiveFace)
    {
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 displacement = positiveFace
            ? Fixed64.One
            : -Fixed64.One;
        FixedPointAnchor2d composite = new(
            new Vector2d(-displacement, Fixed64.Zero),
            Fixed64.Zero,
            new Vector2d(face, Fixed64.Zero),
            new Vector2d(displacement, Fixed64.Zero));
        FixedPointAnchor2d faceAnchor = new(
            new Vector2d(face, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Zero);

        Assert.True(composite.TryGetPoint(out Vector2d point));
        Assert.Equal(faceAnchor.Origin, point);
        Assert.True(composite.TryGetOffsetFrom(
            faceAnchor,
            out Vector2d offset));
        Assert.Equal(Vector2d.Zero, offset);
        Assert.False(composite.TryGetLocalPointIn(
            composite.Origin,
            Fixed64.Zero,
            out Vector2d unrepresentableLocalPoint));
        Assert.Equal(default, unrepresentableLocalPoint);
    }

    [Fact]
    public void FrameReexpression_UsesTheRepresentedTrigonometricNorm()
    {
        FixedPointAnchor2d source = new(
            new Vector2d(5, -3),
            Fixed64.PiOver3,
            new Vector2d(2, 1),
            new Vector2d(-Fixed64.Half, Fixed64.FromFraction(1, 4)));

        Assert.True(source.TryGetLocalPointIn(
            source.Origin,
            source.Rotation,
            out Vector2d sameFramePoint));
        Assert.Equal(
            new Vector2d(
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(5, 4)),
            sameFramePoint);

        Vector2d targetOrigin = new(1, 2);
        Fixed64 targetRotation = -Fixed64.PiOver4;
        Assert.True(source.TryGetLocalPointIn(
            targetOrigin,
            targetRotation,
            out Vector2d rebasedPoint));
        FixedPointAnchor2d rebased = new(
            targetOrigin,
            targetRotation,
            rebasedPoint);
        Assert.True(source.TryGetOffsetFrom(
            rebased,
            out Vector2d roundTripDifference));
        // Re-expression narrows once onto the target frame's Fixed64 lattice;
        // transforming that nearest lattice point back can differ by one raw
        // unit even though same-frame recovery above is exact.
        Assert.True(roundTripDifference.X.Abs() <= Fixed64.Epsilon);
        Assert.True(roundTripDifference.Y.Abs() <= Fixed64.Epsilon);
    }

    [Fact]
    public void TryReframe_ArbitraryResidualFreeFramePreservesExactPoint()
    {
        FixedPointAnchor2d source = new(
            new Vector2d(5, -3),
            Fixed64.HalfPi,
            new Vector2d(2, 1));
        Vector2d frameOrigin = new(1, 2);

        Assert.True(source.TryReframe(
            frameOrigin,
            Fixed64.HalfPi,
            out FixedPointAnchor2d reframed));
        Assert.Equal(frameOrigin, reframed.Origin);
        Assert.Equal(Fixed64.HalfPi, reframed.Rotation);
        Assert.True(source.TryGetOffsetFrom(
            reframed,
            out Vector2d difference));
        Assert.Equal(Vector2d.Zero, difference);
    }

    [Fact]
    public void TryReframe_SameFrameRetainsExactResidualIdentity()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedPointAnchor2d source = new(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Zero,
            FixedPointAnchorTerm2d.CreateRadialSupport(
                new Vector2d(rawUnit, Fixed64.Zero),
                rawUnit,
                Vector2d.Zero));

        Assert.True(source.TryReframe(
            source.Origin,
            source.Rotation,
            out FixedPointAnchor2d reframed));
        Assert.Equal(source, reframed);
        Assert.Equal(
            source.GetLocalFeatureHash64(),
            reframed.GetLocalFeatureHash64());
    }

    [Fact]
    public void TryReframe_UnpreservableResidualReturnsFalseAndDefault()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedPointAnchor2d source = new(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Zero,
            FixedPointAnchorTerm2d.CreateRadialSupport(
                new Vector2d(rawUnit, Fixed64.Zero),
                rawUnit,
                Vector2d.Zero));

        Assert.False(source.TryReframe(
            Vector2d.Right,
            Fixed64.Zero,
            out FixedPointAnchor2d reframed));
        Assert.Equal(default, reframed);
    }

    [Fact]
    public void TryReframe_UnrepresentableFrameReturnsFalseAndDefault()
    {
        FixedPointAnchor2d source = new(
            new Vector2d(
                Fixed64.MaxValue,
                Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Zero);

        Assert.False(source.TryReframe(
            new Vector2d(
                Fixed64.MinValue,
                Fixed64.Zero),
            Fixed64.Zero,
            out FixedPointAnchor2d reframed));
        Assert.Equal(default, reframed);
    }

    [Fact]
    public void EqualityAndHash_UseBothLocalFeatureComponents()
    {
        FixedPointAnchor2d value = new(
            Vector2d.One,
            Fixed64.PiOver4,
            Vector2d.Right);
        FixedPointAnchor2d equal = new(
            Vector2d.One,
            Fixed64.PiOver4,
            Vector2d.Right);
        FixedPointAnchor2d different = new(
            Vector2d.One,
            -Fixed64.PiOver4,
            Vector2d.Right);
        FixedPointAnchor2d displaced = new(
            value.Origin,
            value.Rotation,
            value.LocalPoint,
            Vector2d.Forward);

        Assert.True(value.Equals(equal));
        Assert.True(value.Equals((object)equal));
        Assert.Equal(value.GetHashCode(), equal.GetHashCode());
        int expectedHash;
        unchecked
        {
            expectedHash = 17;
            expectedHash = (expectedHash * 31) + value.Origin.GetHashCode();
            expectedHash = (expectedHash * 31) + value.Rotation.GetHashCode();
            expectedHash = (expectedHash * 31) + value.LocalPoint.GetHashCode();
            expectedHash =
                (expectedHash * 31) + value.LocalDisplacement.GetHashCode();
        }
        Assert.Equal(expectedHash, value.GetHashCode());
        Assert.True(value == equal);
        Assert.False(value != equal);
        Assert.False(value.Equals(different));
        Assert.False(value.Equals(null));
        Assert.True(value != different);
        Assert.False(value == different);
        Assert.NotEqual(value, displaced);
    }

    [Fact]
    public void ResponseProducts_MatchRepresentableVectorAlgebra()
    {
        FixedPointAnchor2d point = new(
            new Vector2d(4, -3),
            Fixed64.Zero,
            Vector2d.Zero);
        FixedPointAnchor2d origin = new(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero);
        Vector2d crossVector = new(2, 1);

        FixedLever2d lever = point.GetLeverFrom(origin);
        Assert.True(lever.TryGetVector(out Vector2d vector));
        Assert.Equal(new Vector2d(4, -3), vector);
        Assert.True(lever.TryGetCrossProduct(
            crossVector,
            out Fixed64 cross));
        Assert.Equal((Fixed64)10, cross);
        Assert.True(lever.TryGetScaledCrossProduct(
            crossVector,
            (Fixed64)3,
            Fixed64.Two,
            out Fixed64 scaledCross));
        Assert.Equal((Fixed64)15, scaledCross);
        Assert.True(lever.TryGetScaledCrossProduct(
            crossVector,
            (Fixed64)3,
            Fixed64.Two,
            (Fixed64)3,
            out Fixed64 twiceScaledCross));
        Assert.Equal((Fixed64)20, twiceScaledCross);
        Assert.True(lever.TryGetScaledSquaredCrossProduct(
            crossVector,
            Fixed64.Half,
            out Fixed64 squaredCross));
        Assert.Equal((Fixed64)50, squaredCross);
        Assert.True(lever.TryGetScaledSquaredCrossProduct(
            crossVector,
            -Fixed64.Half,
            out Fixed64 negativeSquaredCross));
        Assert.Equal((Fixed64)(-50), negativeSquaredCross);
        Assert.True(lever.TryGetScaledCrossProduct(
            Vector2d.Zero,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 zeroCross));
        Assert.Equal(Fixed64.Zero, zeroCross);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ResponseProducts_NarrowOnlyAfterExactFullDomainEvaluation(
        bool positiveFace)
    {
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 outward = positiveFace
            ? Fixed64.MinIncrement
            : -Fixed64.MinIncrement;
        FixedPointAnchor2d point = new(
            new Vector2d(face, Fixed64.Zero),
            Fixed64.Zero,
            new Vector2d(outward, Fixed64.Zero));
        FixedPointAnchor2d origin = new(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero);
        Fixed64 expected = positiveFace
            ? (Fixed64)1073741824
            : (Fixed64)(-1073741824);
        Fixed64 expectedSquare = positiveFace
            ? (Fixed64)1073741824
            : Fixed64.FromRaw(
                ((Fixed64)1073741824).m_rawValue + 1L);

        Assert.False(point.TryGetOffsetFrom(origin, out _));
        FixedLever2d lever = point.GetLeverFrom(origin);
        Assert.False(lever.TryGetVector(out _));
        Assert.False(lever.TryGetCrossProduct(
            Vector2d.Forward,
            out Fixed64 overflow));
        Assert.Equal(default, overflow);
        Assert.True(lever.TryGetScaledCrossProduct(
            Vector2d.Forward,
            Fixed64.Half,
            Fixed64.One,
            out Fixed64 scaledCross));
        Assert.Equal(expected, scaledCross);
        Assert.True(lever.TryGetScaledSquaredCrossProduct(
            Vector2d.Forward,
            Fixed64.MinIncrement,
            out Fixed64 squaredCross));
        Assert.Equal(expectedSquare, squaredCross);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ExactLever_ToXZLeverPreservesFullDomainComponents(
        bool positiveFace)
    {
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 outward = positiveFace
            ? Fixed64.MinIncrement
            : -Fixed64.MinIncrement;
        FixedPointAnchor2d point = new(
            new Vector2d(face, Fixed64.Zero),
            Fixed64.Zero,
            new Vector2d(outward, Fixed64.One));
        FixedLever2d planar = point.GetLeverFrom(
            new FixedPointAnchor2d(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero));

        Assert.False(planar.TryGetVector(out _));
        FixedLever spatial = planar.ToXZLever();
        Assert.False(spatial.TryGetVector(out _));
        Assert.True(spatial.TryGetTransformedScaledCrossProduct(
            Vector3d.Up,
            Fixed3x3.Identity,
            Fixed64.MinIncrement,
            Fixed64.One,
            out Vector3d scaledCross));
        Assert.True(planar.TryGetScaledCrossProduct(
            Vector2d.Forward,
            Fixed64.MinIncrement,
            Fixed64.One,
            out Fixed64 expected));
        Assert.Equal(
            new Vector3d(-Fixed64.MinIncrement, Fixed64.Zero, expected),
            scaledCross);
    }

    [Fact]
    public void ResponseProducts_RejectZeroDivisorAtomically()
    {
        Assert.False(default(FixedLever2d).TryGetVector(
            out Vector2d invalidVector));
        Assert.Equal(default, invalidVector);
        Assert.False(default(FixedLever2d).TryGetScaledCrossProduct(
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 result));
        Assert.Equal(default, result);
    }

    [Fact]
    public void ExactOperations_DoNotAllocateAfterWarmup()
    {
        FixedPointAnchor2d first = new(
            Vector2d.One,
            Fixed64.PiOver4,
            Vector2d.Right);
        FixedPointAnchor2d second = new(
            Vector2d.Zero,
            -Fixed64.PiOver4,
            Vector2d.Forward);
        FixedPointAnchor2d reframeSource = new(
            new Vector2d(5, -3),
            Fixed64.HalfPi,
            new Vector2d(2, 1));
        Vector2d reframeOrigin = new(1, 2);
        _ = first.TryGetPoint(out _);
        _ = first.TryGetOffsetFrom(second, out _);
        _ = first.TryGetLocalPointIn(Vector2d.Zero, Fixed64.Zero, out _);
        FixedLever2d lever = first.GetLeverFrom(second);
        _ = lever.TryGetVector(out _);
        _ = lever.TryGetCrossProduct(Vector2d.Forward, out _);
        _ = lever.TryGetScaledCrossProduct(
            Vector2d.Forward,
            Fixed64.Half,
            Fixed64.One,
            out _);
        _ = lever.TryGetScaledSquaredCrossProduct(
            Vector2d.Forward,
            Fixed64.Half,
            out _);
        Assert.True(reframeSource.TryReframe(
            reframeOrigin,
            Fixed64.HalfPi,
            out _));

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 32; iteration++)
        {
            _ = first.TryGetPoint(out _);
            _ = first.TryGetOffsetFrom(second, out _);
            _ = first.TryGetLocalPointIn(
                Vector2d.Zero,
                Fixed64.Zero,
                out _);
            lever = first.GetLeverFrom(second);
            _ = lever.TryGetVector(out _);
            _ = lever.TryGetCrossProduct(Vector2d.Forward, out _);
            _ = lever.TryGetScaledCrossProduct(
                Vector2d.Forward,
                Fixed64.Half,
                Fixed64.One,
                out _);
            _ = lever.TryGetScaledSquaredCrossProduct(
                Vector2d.Forward,
                Fixed64.Half,
                out _);
            _ = reframeSource.TryReframe(
                reframeOrigin,
                Fixed64.HalfPi,
                out _);
        }

        Assert.Equal(
            0L,
            GC.GetAllocatedBytesForCurrentThread() - before);
    }
}
