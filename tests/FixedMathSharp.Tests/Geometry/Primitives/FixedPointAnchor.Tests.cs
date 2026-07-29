using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedPointAnchorTests
{
    [Fact]
    public void LocalFeatureIdentity_OrdersEveryStoredComponent()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedPointAnchor baseline = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Vector3d.Zero);
        FixedPointAnchorTerm3d exactX =
            FixedPointAnchorTerm3d.CreateRadialSupport(
                new Vector3d(rawUnit, Fixed64.Zero, Fixed64.Zero),
                rawUnit,
                Vector3d.Zero);
        FixedPointAnchorTerm3d exactY =
            FixedPointAnchorTerm3d.CreateRadialSupport(
                new Vector3d(Fixed64.Zero, rawUnit, Fixed64.Zero),
                rawUnit,
                Vector3d.Zero);
        FixedPointAnchorTerm3d exactZ =
            FixedPointAnchorTerm3d.CreateRadialSupport(
                new Vector3d(Fixed64.Zero, Fixed64.Zero, rawUnit),
                rawUnit,
                Vector3d.Zero);
        FixedPointAnchor[] greaterFeatures =
        {
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(rawUnit, Fixed64.Zero, Fixed64.Zero)),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, rawUnit, Fixed64.Zero)),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, rawUnit)),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                new Vector3d(rawUnit, Fixed64.Zero, Fixed64.Zero)),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                new Vector3d(Fixed64.Zero, rawUnit, Fixed64.Zero)),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, rawUnit)),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                exactX),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                exactY),
            new(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                exactZ),
        };

        Assert.Equal(0, baseline.CompareLocalFeature(baseline));
        foreach (FixedPointAnchor greater in greaterFeatures)
        {
            Assert.True(baseline.CompareLocalFeature(greater) < 0);
            Assert.True(greater.CompareLocalFeature(baseline) > 0);
            Assert.NotEqual(
                baseline.GetLocalFeatureHash64(),
                greater.GetLocalFeatureHash64());
        }

        FixedPointAnchor exactXCopy = new(
            new Vector3d(7, 8, 9),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
            Vector3d.Zero,
            Vector3d.Zero,
            exactX);
        Assert.Equal(0, greaterFeatures[6].CompareLocalFeature(exactXCopy));
        Assert.Equal(
            greaterFeatures[6].GetLocalFeatureHash64(),
            exactXCopy.GetLocalFeatureHash64());
        Assert.Equal(
            greaterFeatures[6].GetHashCode(),
            new FixedPointAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Vector3d.Zero,
                exactX).GetHashCode());
    }

    [Fact]
    public void TryGetPoint_RotatesLocalPointBeforeAddingOrigin()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(5, -3, 7),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
            new Vector3d(2, 1, -4));

        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(1, -2, 5), point);
    }

    [Fact]
    public void WithLocalTranslation_PreservesOrdinaryRigidPointSemantics()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        FixedPointAnchor anchor = new(
            new Vector3d(5, -3, 7),
            rotation,
            new Vector3d(2, 1, -4),
            new Vector3d(-1, 2, 3));
        Vector3d translation = new(4, -2, 1);

        FixedPointAnchor translated =
            anchor.WithLocalTranslation(translation);

        Assert.Equal(translation, translated.LocalTranslation);
        Assert.True(translated.TryGetPoint(out Vector3d point));
        Assert.Equal(
            anchor.Origin
            + rotation.Rotate(
                anchor.LocalPoint
                + anchor.LocalDisplacement
                + translation),
            point);
        Assert.True(translated.TryGetOffsetFrom(
            anchor,
            out Vector3d worldTranslation));
        Assert.Equal(rotation.Rotate(translation), worldTranslation);
        Assert.True(translated.TryGetProjectedOffsetFrom(
            anchor,
            Vector3d.Right,
            out Fixed64 projectedTranslation));
        Assert.Equal(worldTranslation.X, projectedTranslation);
        Assert.True(anchor.TryGetProjectedOffsetFrom(
            translated,
            Vector3d.Right,
            out Fixed64 reverseProjectedTranslation));
        Assert.Equal(-worldTranslation.X, reverseProjectedTranslation);
        Assert.Equal(
            worldTranslation.X,
            translated.ProjectNonNegativeOffsetFrom(
                anchor,
                Vector3d.Right));
        Assert.True(translated.TryGetLocalPointIn(
            anchor.Origin,
            anchor.Rotation,
            out Vector3d translatedLocalPoint));
        Assert.Equal(
            anchor.LocalPoint
            + anchor.LocalDisplacement
            + translation,
            translatedLocalPoint);
        Assert.True(
            anchor.CompareSquaredDistance(anchor, translated) < 0);
        Assert.True(
            translated.CompareSquaredDistance(anchor, translated) > 0);
        Assert.True(
            anchor.CompareSquaredDistance(translated, anchor) > 0);
        Assert.True(translated.TryReframe(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out FixedPointAnchor reframed));
        Assert.True(reframed.TryGetPoint(out Vector3d reframedPoint));
        Assert.Equal(point, reframedPoint);
    }

    [Fact]
    public void WithLocalTranslation_PreservesExactCenteredSupportAtScalarFaces()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.HalfPi / Fixed64.Two);
        Vector3d direction = rotation.Rotate(
            new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
        FixedPointAnchor support =
            FixedSegment.GetCenteredFiniteConeSupportAnchor(
                Vector3d.Zero,
                rotation,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                direction);
        Vector3d localTranslation = new(3, -2, 1);

        FixedPointAnchor translated =
            support.WithLocalTranslation(localTranslation);

        Assert.False(support.TryGetPoint(out _));
        Assert.Equal(0, support.CompareLocalFeature(translated));
        Assert.True(translated.TryGetOffsetFrom(
            support,
            out Vector3d translation));
        Assert.True(rotation.TryRotate(
            localTranslation,
            out Vector3d expectedTranslation));
        Assert.Equal(expectedTranslation, translation);
    }

    [Fact]
    public void TryGetOffsetFrom_FusesBothRigidFramesAcrossScalarFaces()
    {
        FixedPointAnchor first = new(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, (Fixed64)(-6)),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4),
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero));
        FixedPointAnchor second = new(
            first.Origin,
            first.Rotation,
            first.LocalPoint);

        Assert.False(first.TryGetPoint(out _));
        Assert.False(second.TryGetPoint(out _));
        Assert.True(first.TryGetOffsetFrom(second, out Vector3d offset));
        Assert.Equal(Vector3d.Zero, offset);
    }

    [Fact]
    public void TryGetPoint_RejectsUnrepresentableWorldCoordinate()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Right);

        Assert.False(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(default, point);
    }

    [Fact]
    public void TryGetOffsetFrom_RejectsUnrepresentableRelativeCoordinate()
    {
        FixedPointAnchor high = new(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor low = new(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(high.TryGetOffsetFrom(low, out Vector3d offset));
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
        FixedPointAnchor composite = new(
            new Vector3d(
                Fixed64.Zero,
                -displacement,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, face, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, displacement, Fixed64.Zero));
        FixedPointAnchor faceAnchor = new(
            new Vector3d(Fixed64.Zero, face, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.True(composite.TryGetPoint(out Vector3d point));
        Assert.Equal(faceAnchor.Origin, point);
        Assert.True(composite.TryGetOffsetFrom(
            faceAnchor,
            out Vector3d offset));
        Assert.Equal(Vector3d.Zero, offset);
        Assert.False(composite.TryGetLocalPointIn(
            composite.Origin,
            FixedQuaternion.Identity,
            out Vector3d unrepresentableLocalPoint));
        Assert.Equal(default, unrepresentableLocalPoint);
    }

    [Fact]
    public void TryGetScaledOffsetFrom_NarrowsOnlyAfterExactScale()
    {
        FixedPointAnchor high = new(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor low = new(
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(high.TryGetOffsetFrom(low, out _));
        Assert.True(high.TryGetScaledOffsetFrom(
            low,
            Fixed64.Half,
            out Vector3d halfDifference));
        Assert.Equal(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            halfDifference);
    }

    [Fact]
    public void TryGetScaledOffsetFrom_SupportsZeroNegativeAndGeneralScale()
    {
        FixedPointAnchor anchor = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        FixedPointAnchor other = new(
            new Vector3d(-4, 0, 0),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.True(anchor.TryGetScaledOffsetFrom(
            other,
            Fixed64.Zero,
            out Vector3d zero));
        Assert.Equal(Vector3d.Zero, zero);
        Assert.True(anchor.TryGetScaledOffsetFrom(
            other,
            -Fixed64.Half,
            out Vector3d negative));
        Assert.Equal(new Vector3d(-2, 0, 0), negative);
        Assert.True(anchor.TryGetScaledOffsetFrom(
            other,
            Fixed64.FromFraction(3, 4),
            out Vector3d general));
        Assert.Equal(new Vector3d(3, 0, 0), general);
    }

    [Fact]
    public void ProjectedOffset_FusesUnrepresentableComponentCancellation()
    {
        FixedPointAnchor high = new(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor low = new(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.MinValue,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(high.TryGetOffsetFrom(low, out _));
        Assert.True(high.TryGetProjectedOffsetFrom(
            low,
            new Vector3d(
                Fixed64.One,
                -Fixed64.One,
                Fixed64.Zero),
            out Fixed64 canceled));
        Assert.Equal(Fixed64.Zero, canceled);
        Assert.False(high.TryGetProjectedOffsetFrom(
            low,
            Vector3d.Right,
            out Fixed64 overflow));
        Assert.Equal(default, overflow);
        Assert.Equal(
            Fixed64.MaxValue,
            high.ProjectNonNegativeOffsetFrom(low, Vector3d.Right));
        Assert.Equal(
            Fixed64.Zero,
            low.ProjectNonNegativeOffsetFrom(high, Vector3d.Right));
        Assert.False(default(FixedPointAnchor).TryGetProjectedOffsetFrom(
            high,
            Vector3d.Right,
            out _));
        Assert.False(high.TryGetProjectedOffsetFrom(
            default,
            Vector3d.Right,
            out _));
        Assert.Throws<InvalidOperationException>(() =>
            default(FixedPointAnchor).ProjectNonNegativeOffsetFrom(
                high,
                Vector3d.Right));
        ArgumentException invalidOther = Assert.Throws<ArgumentException>(() =>
            high.ProjectNonNegativeOffsetFrom(
                default,
                Vector3d.Right));
        Assert.Equal("other", invalidOther.ParamName);
    }

    [Fact]
    public void TryGetLocalPointIn_IdentityFrameReturnsRelativePoint()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(5, -3, 7),
            FixedQuaternion.Identity,
            new Vector3d(2, 1, -4));

        Assert.True(anchor.TryGetLocalPointIn(
            new Vector3d(1, 2, 3),
            FixedQuaternion.Identity,
            out Vector3d localPoint));
        Assert.Equal(new Vector3d(6, -4, 0), localPoint);
    }

    [Fact]
    public void TryGetLocalPointIn_RotatedFramePreservesAnchorLocalPoint()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(5, -3, 7),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
            new Vector3d(2, 1, -4));

        Assert.True(anchor.TryGetLocalPointIn(
            anchor.Origin,
            anchor.Rotation,
            out Vector3d localPoint));
        Assert.Equal(anchor.LocalPoint, localPoint);
    }

    [Fact]
    public void TryGetLocalPointIn_FusesScalarFaceCancellation()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, (Fixed64)(-6)),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4),
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero));

        Assert.False(anchor.TryGetPoint(out _));
        Assert.True(anchor.TryGetLocalPointIn(
            anchor.Origin,
            anchor.Rotation,
            out Vector3d localPoint));
        Assert.Equal(anchor.LocalPoint, localPoint);
    }

    [Fact]
    public void TryGetLocalPointIn_RejectsUnrepresentableLocalCoordinate()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(anchor.TryGetLocalPointIn(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            out Vector3d localPoint));
        Assert.Equal(default, localPoint);
    }

    [Fact]
    public void TryGetLocalPointIn_RejectsInvalidStoredAnchorAtomically()
    {
        Assert.False(default(FixedPointAnchor).TryGetLocalPointIn(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out Vector3d localPoint));
        Assert.Equal(default, localPoint);
    }

    [Fact]
    public void TryGetLocalPointIn_ReturnsFalseForNonNormalizedTargetRotation()
    {
        FixedPointAnchor anchor = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(anchor.TryGetLocalPointIn(
            Vector3d.Zero,
            default,
            out Vector3d localPoint));
        Assert.Equal(default, localPoint);
    }

    [Fact]
    public void TryReframe_ArbitraryResidualFreeFramePreservesExactPoint()
    {
        FixedPointAnchor source = new(
            new Vector3d(5, -3, 7),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.HalfPi),
            new Vector3d(2, 1, -4));
        Vector3d frameOrigin = new(1, 2, 3);
        FixedQuaternion frameRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.HalfPi);

        Assert.True(source.TryReframe(
            frameOrigin,
            frameRotation,
            out FixedPointAnchor reframed));
        Assert.Equal(frameOrigin, reframed.Origin);
        Assert.Equal(frameRotation, reframed.Rotation);
        Assert.True(source.TryGetOffsetFrom(
            reframed,
            out Vector3d difference));
        Assert.Equal(Vector3d.Zero, difference);
    }

    [Fact]
    public void TryReframe_SameFrameRetainsExactResidualIdentity()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedPointAnchorTerm3d exactTerm =
            FixedPointAnchorTerm3d.CreateRadialSupport(
                new Vector3d(rawUnit, Fixed64.Zero, Fixed64.Zero),
                rawUnit,
                Vector3d.Zero);
        FixedPointAnchor source = new(
            new Vector3d(1, 2, 3),
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Vector3d.Zero,
            exactTerm);

        Assert.True(source.TryReframe(
            source.Origin,
            source.Rotation,
            out FixedPointAnchor reframed));
        Assert.Equal(source, reframed);
        Assert.Equal(
            source.GetLocalFeatureHash64(),
            reframed.GetLocalFeatureHash64());
    }

    [Fact]
    public void TryReframe_UnpreservableResidualReturnsFalseAndDefault()
    {
        Fixed64 rawUnit = Fixed64.FromRaw(1L);
        FixedPointAnchor source = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            Vector3d.Zero,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                new Vector3d(rawUnit, Fixed64.Zero, Fixed64.Zero),
                rawUnit,
                Vector3d.Zero));

        Assert.False(source.TryReframe(
            Vector3d.Right,
            FixedQuaternion.Identity,
            out FixedPointAnchor reframed));
        Assert.Equal(default, reframed);
    }

    [Fact]
    public void TryReframe_InvalidOrUnrepresentableFrameReturnsFalseAndDefault()
    {
        FixedPointAnchor source = new(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(source.TryReframe(
            Vector3d.Zero,
            default,
            out FixedPointAnchor invalidTarget));
        Assert.Equal(default, invalidTarget);
        Assert.False(default(FixedPointAnchor).TryReframe(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out FixedPointAnchor invalidSource));
        Assert.Equal(default, invalidSource);
        Assert.False(source.TryReframe(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            out FixedPointAnchor unrepresentable));
        Assert.Equal(default, unrepresentable);
    }

    [Fact]
    public void TryGetLocalPointIn_DoesNotAllocate()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(5, -3, 7),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d(2, 1, -4));
        FixedQuaternion frameRotation =
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4);

        _ = anchor.TryGetLocalPointIn(
            new Vector3d(1, 2, 3),
            frameRotation,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int index = 0; index < 256; index++)
        {
            _ = anchor.TryGetLocalPointIn(
                new Vector3d(1, 2, 3),
                frameRotation,
                out _);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }

    [Fact]
    public void ExactAnchorAndWideLeverOperations_DoNotAllocate()
    {
        FixedPointAnchor first = new(
            new Vector3d(5, -3, 7),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d(2, 1, -4));
        FixedPointAnchor second = new(
            new Vector3d(1, 2, 3),
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4),
            new Vector3d(-2, 4, 1));

        _ = first.TryGetScaledOffsetFrom(second, Fixed64.Half, out _);
        _ = first.TryGetProjectedOffsetFrom(second, Vector3d.Right, out _);
        _ = first.ProjectNonNegativeOffsetFrom(second, Vector3d.Right);
        WideLever3dValue lever = WideLever3d.GetValue(first, second);
        _ = WideLever3d.TryGetRelativePointVelocityProjection(
            Vector3d.Zero,
            Vector3d.Forward,
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            lever,
            Vector3d.Up,
            out _);
        _ = WideLever3d.TryGetCrossProductQuadraticForm(
            lever,
            Vector3d.Up,
            Fixed3x3.Identity,
            out _);
        _ = WideLever3d.TryGetTransformedScaledCrossProduct(
            lever,
            Vector3d.Up,
            Fixed3x3.Identity,
            Fixed64.Half,
            Fixed64.One,
            Fixed64.One,
            out _);
        _ = WideLever3d.TryGetTransformedWeightedCrossProduct(
            lever,
            Vector3d.Right,
            Fixed64.One,
            Vector3d.Up,
            Fixed64.Half,
            Vector3d.Forward,
            -Fixed64.Half,
            Fixed3x3.Identity,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int index = 0; index < 256; index++)
        {
            _ = first.TryGetScaledOffsetFrom(second, Fixed64.Half, out _);
            _ = first.TryGetProjectedOffsetFrom(
                second,
                Vector3d.Right,
                out _);
            _ = first.ProjectNonNegativeOffsetFrom(
                second,
                Vector3d.Right);
            lever = WideLever3d.GetValue(first, second);
            _ = WideLever3d.TryGetRelativePointVelocityProjection(
                Vector3d.Zero,
                Vector3d.Forward,
                lever,
                Vector3d.Zero,
                Vector3d.Forward,
                lever,
                Vector3d.Up,
                out _);
            _ = WideLever3d.TryGetCrossProductQuadraticForm(
                lever,
                Vector3d.Up,
                Fixed3x3.Identity,
                out _);
            _ = WideLever3d.TryGetTransformedScaledCrossProduct(
                lever,
                Vector3d.Up,
                Fixed3x3.Identity,
                Fixed64.Half,
                Fixed64.One,
                Fixed64.One,
                out _);
            _ = WideLever3d.TryGetTransformedWeightedCrossProduct(
                lever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Up,
                Fixed64.Half,
                Vector3d.Forward,
                -Fixed64.Half,
                Fixed3x3.Identity,
                out _);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }

    [Fact]
    public void WideLeverArithmetic_MatchesRepresentableVectorAlgebra()
    {
        FixedPointAnchor point = new(
            new Vector3d(4, -3, 2),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Vector3d crossVector = new(2, 1, -1);
        Fixed3x3 transform = new(
            (Fixed64)2, -Fixed64.One, (Fixed64)5,
            Fixed64.Zero, (Fixed64)3, Fixed64.One,
            (Fixed64)(-2), (Fixed64)4, (Fixed64)4);

        WideLever3dValue lever = WideLever3d.GetValue(point, origin);
        Assert.True(point.TryGetOffsetFrom(origin, out Vector3d vector));
        Assert.Equal(new Vector3d(4, -3, 2), vector);
        Vector3d cross = Vector3d.Cross(vector, crossVector);
        Vector3d transformedCross =
            Fixed3x3.TransformDirection(transform, cross);
        Vector3d firstLinearVelocity = new(3, -1, 4);
        Vector3d firstAngularVelocity = new(1, 2, -1);
        Vector3d secondLinearVelocity = new(-2, 5, 1);
        Vector3d secondAngularVelocity = new(-1, 1, 3);
        Vector3d projectionAxis = Vector3d.Up;
        Vector3d firstPointVelocity =
            firstLinearVelocity
            + Vector3d.Cross(firstAngularVelocity, vector);
        Vector3d secondPointVelocity =
            secondLinearVelocity
            + Vector3d.Cross(secondAngularVelocity, vector);
        Assert.True(WideLever3d.TryGetRelativePointVelocityProjection(
            firstLinearVelocity,
            firstAngularVelocity,
            lever,
            secondLinearVelocity,
            secondAngularVelocity,
            lever,
            projectionAxis,
            out Fixed64 relativeVelocityProjection));
        Assert.Equal(
            Vector3d.Dot(
                secondPointVelocity - firstPointVelocity,
                projectionAxis),
            relativeVelocityProjection);
        Assert.True(WideLever3d.TryGetCrossProductQuadraticForm(
            lever,
            crossVector,
            transform,
            out Fixed64 quadraticForm));
        Assert.Equal(
            Vector3d.Dot(cross, transformedCross),
            quadraticForm);
        Assert.True(WideLever3d.TryGetCrossProductQuadraticForm(
            lever,
            crossVector,
            -Fixed3x3.Identity,
            out Fixed64 negativeQuadraticForm));
        Assert.Equal((Fixed64)(-165), negativeQuadraticForm);
        Assert.True(WideLever3d.TryGetCrossProductQuadraticForm(
            lever,
            Vector3d.Zero,
            transform,
            out Fixed64 zeroQuadraticForm));
        Assert.Equal(Fixed64.Zero, zeroQuadraticForm);
        Assert.True(WideLever3d.TryGetTransformedScaledCrossProduct(
            lever,
            crossVector,
            transform,
            (Fixed64)3,
            Fixed64.One,
            Fixed64.Two,
            out Vector3d scaledCross));
        Assert.Equal(
            transformedCross * (Fixed64)3 / Fixed64.Two,
            scaledCross);
        Assert.True(WideLever3d.TryGetTransformedScaledCrossProduct(
            lever,
            crossVector,
            transform,
            (Fixed64)3,
            Fixed64.Two,
            (Fixed64)3,
            out Vector3d twiceScaledCross));
        Assert.Equal(transformedCross * Fixed64.Two, twiceScaledCross);
        Assert.True(WideLever3d.TryGetTransformedWeightedCrossProduct(
            lever,
            Vector3d.Right,
            Fixed64.Two,
            Vector3d.Up,
            -Fixed64.One,
            Vector3d.Forward,
            Fixed64.Half,
            transform,
            out Vector3d weightedCross));
        Assert.Equal(
            transform.TransformDirection(
                Vector3d.Cross(
                    vector,
                    Vector3d.Right * Fixed64.Two
                    - Vector3d.Up
                    + Vector3d.Forward * Fixed64.Half)),
            weightedCross);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void WideLeverArithmetic_NarrowsOnlyAfterExactFullDomainEvaluation(
        bool positiveFace)
    {
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 outward = positiveFace
            ? Fixed64.MinIncrement
            : -Fixed64.MinIncrement;
        FixedPointAnchor point = new(
            new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(outward, Fixed64.Zero, Fixed64.Zero));
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Fixed64 expected = positiveFace
            ? (Fixed64)1073741824
            : (Fixed64)(-1073741824);
        Fixed64 expectedSquare = positiveFace
            ? (Fixed64)1073741824
            : Fixed64.FromRaw(
                ((Fixed64)1073741824).m_rawValue + 1L);
        Fixed3x3 tinyForwardScale = new(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.MinIncrement);

        Assert.False(point.TryGetOffsetFrom(origin, out _));
        WideLever3dValue lever = WideLever3d.GetValue(point, origin);
        Assert.True(WideLever3d.TryGetCrossProductQuadraticForm(
            lever,
            Vector3d.Up,
            tinyForwardScale,
            out Fixed64 quadraticForm));
        Assert.Equal(expectedSquare, quadraticForm);
        Assert.True(WideLever3d.TryGetTransformedScaledCrossProduct(
            lever,
            Vector3d.Up,
            Fixed3x3.Identity,
            Fixed64.Half,
            Fixed64.One,
            Fixed64.One,
            out Vector3d scaledCross));
        Assert.Equal(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, expected),
            scaledCross);
    }

    [Fact]
    public void RelativePointVelocityProjection_NarrowsOnlyTheFinalScalar()
    {
        FixedPointAnchor point = new(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Right * Fixed64.MinIncrement);
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        WideLever3dValue exactLever = WideLever3d.GetValue(point, origin);
        Assert.True(WideLever3d.TryGetRelativePointVelocityProjection(
            Vector3d.Zero,
            Vector3d.Forward,
            exactLever,
            Vector3d.Zero,
            Vector3d.Forward,
            exactLever,
            Vector3d.Up,
            out Fixed64 cancelledAngular));
        Assert.Equal(Fixed64.Zero, cancelledAngular);

        WideLever3dValue zeroLever = WideLever3d.GetValue(origin, origin);
        Assert.True(WideLever3d.TryGetRelativePointVelocityProjection(
            Vector3d.Right * Fixed64.Two,
            Vector3d.Zero,
            exactLever,
            Vector3d.Zero,
            Vector3d.Zero,
            zeroLever,
            Vector3d.Right,
            out Fixed64 ordinaryLinear));
        Assert.Equal(-Fixed64.Two, ordinaryLinear);
        Assert.True(WideLever3d.TryGetRelativePointVelocityProjection(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Zero,
            zeroLever,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.Zero),
            Vector3d.Zero,
            zeroLever,
            new Vector3d(
                Fixed64.Half,
                -Fixed64.One,
                Fixed64.Zero),
            out Fixed64 cancelledLinear));
        Assert.Equal(Fixed64.Zero, cancelledLinear);

        Assert.False(WideLever3d.TryGetRelativePointVelocityProjection(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Zero,
            zeroLever,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Zero,
            zeroLever,
            Vector3d.Right,
            out Fixed64 overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void WeightedTransformedCrossProduct_NarrowsOnlyTheFinalVector()
    {
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor point = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right * Fixed64.MinIncrement);
        WideLever3dValue lever = WideLever3d.GetValue(point, origin);

        Assert.True(WideLever3d.TryGetTransformedWeightedCrossProduct(
            lever,
            Vector3d.Up,
            Fixed64.MaxValue,
            Vector3d.Up,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Identity,
            out Vector3d recovered));
        Assert.Equal(
            Vector3d.Forward,
            recovered);
        FixedPointAnchor unitPoint = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right);
        WideLever3dValue unitLever = WideLever3d.GetValue(
            unitPoint,
            origin);
        Assert.True(WideLever3d.TryGetTransformedWeightedCrossProduct(
            unitLever,
            Vector3d.Up,
            Fixed64.MinValue,
            Vector3d.Zero,
            Fixed64.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Identity,
            out Vector3d minimumScalar));
        Assert.Equal(
            Vector3d.Forward * Fixed64.MinValue,
            minimumScalar);

        Assert.False(WideLever3d.TryGetTransformedWeightedCrossProduct(
            unitLever,
            Vector3d.Up,
            Fixed64.MaxValue,
            Vector3d.Up,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Identity,
            out Vector3d overflow));
        Assert.Equal(default, overflow);
        Assert.False(
            WideLever3d.TryGetTransformedWeightedCrossProduct(
                default,
                Vector3d.Up,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed3x3.Identity,
                out Vector3d invalid));
        Assert.Equal(default, invalid);
    }

    [Fact]
    public void RelativePointVelocityProjection_HandlesDifferentAnchorFrames()
    {
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor firstPoint = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(4, -3, 2));
        FixedPointAnchor secondPoint = new(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.PiOver4),
            new Vector3d(1, 2, 3));
        WideLever3dValue firstLever =
            WideLever3d.GetValue(firstPoint, origin);
        WideLever3dValue secondLever =
            WideLever3d.GetValue(secondPoint, origin);
        Assert.NotEqual(
            firstLever.Denominator,
            secondLever.Denominator);
        Assert.True(firstPoint.TryGetOffsetFrom(
            origin,
            out Vector3d firstOffset));
        Assert.True(secondPoint.TryGetOffsetFrom(
            origin,
            out Vector3d secondOffset));

        Vector3d firstLinearVelocity = new(3, -1, 4);
        Vector3d firstAngularVelocity = new(1, 2, -1);
        Vector3d secondLinearVelocity = new(-2, 5, 1);
        Vector3d secondAngularVelocity = new(-1, 1, 3);
        Vector3d projectionAxis = Vector3d.Up;
        Assert.True(WideLever3d.TryGetRelativePointVelocityProjection(
            firstLinearVelocity,
            firstAngularVelocity,
            firstLever,
            secondLinearVelocity,
            secondAngularVelocity,
            secondLever,
            projectionAxis,
            out Fixed64 projection));
        Assert.Equal(
            Vector3d.Dot(
                secondLinearVelocity
                + Vector3d.Cross(secondAngularVelocity, secondOffset)
                - firstLinearVelocity
                - Vector3d.Cross(firstAngularVelocity, firstOffset),
                projectionAxis),
            projection);
    }

    [Fact]
    public void WideLeverArithmetic_RejectsInvalidOrUnrepresentableResultsAtomically()
    {
        FixedPointAnchor point = new(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Right);
        FixedPointAnchor origin = new(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);

        Assert.False(
            WideLever3d.TryGetCrossProductQuadraticForm(
                default,
                Vector3d.Up,
                Fixed3x3.Identity,
                out Fixed64 invalidQuadraticForm));
        Assert.Equal(default, invalidQuadraticForm);
        Assert.False(
            WideLever3d.TryGetTransformedScaledCrossProduct(
                default,
                Vector3d.Up,
                Fixed3x3.Identity,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out Vector3d invalidTransformedCross));
        Assert.Equal(default, invalidTransformedCross);
        WideLever3dValue lever = WideLever3d.GetValue(point, origin);
        Assert.False(WideLever3d.TryGetRelativePointVelocityProjection(
            Vector3d.Zero,
            Vector3d.Zero,
            default,
            Vector3d.Zero,
            Vector3d.Zero,
            lever,
            Vector3d.Right,
            out Fixed64 invalidFirstRelativeProjection));
        Assert.Equal(default, invalidFirstRelativeProjection);
        Assert.False(WideLever3d.TryGetRelativePointVelocityProjection(
            Vector3d.Zero,
            Vector3d.Zero,
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            default,
            Vector3d.Right,
            out Fixed64 invalidSecondRelativeProjection));
        Assert.Equal(default, invalidSecondRelativeProjection);
        Assert.False(WideLever3d.TryGetTransformedScaledCrossProduct(
            lever,
            Vector3d.Up,
            Fixed3x3.Identity,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            out Vector3d zeroDivisor));
        Assert.Equal(default, zeroDivisor);
        Assert.False(WideLever3d.TryGetTransformedScaledCrossProduct(
            lever,
            Vector3d.Up,
            Fixed3x3.Identity,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            out Vector3d overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void Constructor_RejectsNonNormalizedRotation()
    {
        Assert.Throws<ArgumentException>(() =>
            new FixedPointAnchor(Vector3d.Zero, default, Vector3d.Zero));
    }

    [Fact]
    public void DefaultAnchor_ReturnsFalseAtomically()
    {
        FixedPointAnchor anchor = default;

        Assert.False(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(default, point);
        Assert.False(new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero).TryGetOffsetFrom(anchor, out Vector3d offset));
        Assert.Equal(default, offset);
        Assert.False(anchor.TryGetScaledOffsetFrom(
            new FixedPointAnchor(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            Fixed64.Half,
            out Vector3d scaledOffset));
        Assert.Equal(default, scaledOffset);
    }

    [Fact]
    public void EqualityAndHash_UseBothLocalFeatureComponents()
    {
        FixedPointAnchor anchor = new(
            new Vector3d(1, 2, 3),
            FixedQuaternion.Identity,
            new Vector3d(4, 5, 6));
        FixedPointAnchor equal = new(
            anchor.Origin,
            anchor.Rotation,
            anchor.LocalPoint);
        FixedPointAnchor otherOrigin = new(
            Vector3d.Zero,
            anchor.Rotation,
            anchor.LocalPoint);
        FixedPointAnchor otherRotation = new(
            anchor.Origin,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
            anchor.LocalPoint);
        FixedPointAnchor otherPoint = new(
            anchor.Origin,
            anchor.Rotation,
            Vector3d.Zero);
        FixedPointAnchor otherDisplacement = new(
            anchor.Origin,
            anchor.Rotation,
            anchor.LocalPoint,
            Vector3d.Right);
        FixedPointAnchor otherTranslation =
            anchor.WithLocalTranslation(Vector3d.Right);

        Assert.True(anchor == equal);
        Assert.False(anchor != equal);
        Assert.True(anchor.Equals(equal));
        Assert.True(anchor.Equals((object)equal));
        Assert.Equal(anchor.GetHashCode(), equal.GetHashCode());
        int expectedHash;
        unchecked
        {
            expectedHash = 17;
            expectedHash = (expectedHash * 31) + anchor.Origin.GetHashCode();
            expectedHash = (expectedHash * 31) + anchor.Rotation.GetHashCode();
            expectedHash = (expectedHash * 31) + anchor.LocalPoint.GetHashCode();
            expectedHash =
                (expectedHash * 31) + anchor.LocalDisplacement.GetHashCode();
            expectedHash =
                (expectedHash * 31) + anchor.LocalTranslation.GetHashCode();
        }
        Assert.Equal(expectedHash, anchor.GetHashCode());
        Assert.NotEqual(anchor, otherOrigin);
        Assert.NotEqual(anchor, otherRotation);
        Assert.NotEqual(anchor, otherPoint);
        Assert.NotEqual(anchor, otherDisplacement);
        Assert.NotEqual(anchor, otherTranslation);
        Assert.False(anchor.Equals(null));
    }
}
