//=======================================================================
// FiniteAxisDecision.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FiniteAxisIntersectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredAxisDistance_IgnoresUnrepresentableWorldEndpointAtMirroredFaces(
        bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector2d axis = maximumFace ? Vector2d.Left : Vector2d.Right;
        Vector2d center = new(face, Fixed64.Zero);
        Vector2d point = new(face, (Fixed64)3);

        Assert.True(FixedSegment2d.TryGetDistanceToCenteredAxis(
            point,
            center,
            axis,
            Fixed64.Two,
            out Fixed64 distance));
        Assert.Equal((Fixed64)3, distance);

        Assert.True(FixedSegment.TryGetDistanceToCenteredAxis(
            new Vector3d(point.X, point.Y, Fixed64.One),
            new Vector3d(center.X, center.Y, Fixed64.Zero),
            new Vector3d(axis.X, axis.Y, Fixed64.Zero),
            Fixed64.Two,
            out Fixed64 spatialDistance));
        Assert.Equal(FixedMath.Sqrt((Fixed64)10), spatialDistance);
    }

    [Fact]
    public void CenteredAxisDistance_RejectsNearestEvenResultBeyondMaximum()
    {
        Vector3d point = new(
            Fixed64.MaxValue,
            Fixed64.FromRaw(3_037_000_500L),
            Fixed64.Zero);

        Assert.False(FixedSegment.TryGetDistanceToCenteredAxis(
            point,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            out Fixed64 distance));
        Assert.Equal(Fixed64.MaxValue, distance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredAxesDistance_IgnoresUnrepresentableClosestWitnessAtMirroredFaces(
        bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector2d axis = maximumFace ? Vector2d.Left : Vector2d.Right;
        Vector2d firstCenter = new(face, Fixed64.Zero);
        Vector2d secondCenter = new(face, (Fixed64)3);

        Assert.False(FixedSegment2d.TryGetClosestPointsBetweenCenteredAxes(
            firstCenter,
            axis,
            Fixed64.Two,
            secondCenter,
            axis,
            Fixed64.Two,
            out _,
            out _));
        Assert.True(FixedSegment2d.TryGetDistanceBetweenCenteredAxes(
            firstCenter,
            axis,
            Fixed64.Two,
            secondCenter,
            axis,
            Fixed64.Two,
            out Fixed64 distance));
        Assert.Equal((Fixed64)3, distance);
    }

    [Fact]
    public void CenteredAxesDistance_CoversInteriorSingleClampAndDoubleClamp()
    {
        Assert.True(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            Vector3d.Up,
            (Fixed64)4,
            out Fixed64 interior));
        Assert.Equal(Fixed64.One, interior);

        Assert.True(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One),
            Vector3d.Up,
            (Fixed64)4,
            out Fixed64 singleClamp));
        Assert.Equal(FixedMath.Sqrt((Fixed64)5), singleClamp);

        Assert.True(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            new Vector3d((Fixed64)3, (Fixed64)3, Fixed64.One),
            Vector3d.Up,
            Fixed64.Two,
            out Fixed64 doubleClamp));
        Assert.Equal((Fixed64)3, doubleClamp);

        Assert.True(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)10,
            out Fixed64 secondClamp));
        Assert.Equal((Fixed64)2, secondClamp);
    }

    [Fact]
    public void CenteredAxesDistance_SaturatesOnlyAnUnrepresentableFinalScalar()
    {
        Assert.False(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue),
            Vector3d.Right,
            Fixed64.Zero,
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue),
            Vector3d.Right,
            Fixed64.Zero,
            out Fixed64 distance));
        Assert.Equal(Fixed64.MaxValue, distance);
    }

    [Fact]
    public void CenteredAxesDistance_RejectsInvalidInputWithExactParameterNames()
    {
        ArgumentException firstAxis = Assert.Throws<ArgumentException>(() =>
            FixedSegment2d.TryGetDistanceBetweenCenteredAxes(
                Vector2d.Zero,
                Vector2d.One,
                Fixed64.One,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                out _));
        Assert.Equal("firstAxisDirection", firstAxis.ParamName);

        ArgumentOutOfRangeException secondLength =
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedSegment2d.TryGetDistanceBetweenCenteredAxes(
                    Vector2d.Zero,
                    Vector2d.Right,
                    Fixed64.One,
                    Vector2d.Zero,
                    Vector2d.Right,
                    -Fixed64.One,
                    out _));
        Assert.Equal("secondAxisLength", secondLength.ParamName);
    }

    [Fact]
    public void CenteredCapsuleOverlap_UsesExactRadiusSumBeyondScalarDomain()
    {
        Vector2d minimum = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d tangent = new(
            Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue - 1L),
            Fixed64.Zero);
        Vector2d separated = new(Fixed64.MaxValue, Fixed64.Zero);

        Assert.True(FixedSegment2d.DoCenteredCapsulesOverlap(
            minimum,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            tangent,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue));
        Assert.False(FixedSegment2d.DoCenteredCapsulesOverlap(
            minimum,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            separated,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue));

        Assert.True(FixedSegment.DoCenteredCapsulesOverlap(
            new Vector3d(minimum.X, minimum.Y, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            new Vector3d(tangent.X, tangent.Y, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue));
    }

    [Fact]
    public void CenteredCapsuleOverlap_RejectsNegativeRadii()
    {
        ArgumentOutOfRangeException first = Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoCenteredCapsulesOverlap(
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                -Fixed64.One,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Equal("firstRadius", first.ParamName);

        ArgumentOutOfRangeException second = Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoCenteredCapsulesOverlap(
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                -Fixed64.One));
        Assert.Equal("secondRadius", second.ParamName);
    }

    [Fact]
    public void CenteredAxisDistanceAndOverlap_AreSymmetricAcrossFiniteFeatures()
    {
        Vector2d[] centers =
        {
            new(-4, -3),
            Vector2d.Zero,
            new(2, 5),
            new(6, -1),
        };
        Vector2d[] axes =
        {
            Vector2d.Right,
            Vector2d.Forward,
            new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)),
            new(Fixed64.FromFraction(-4, 5), Fixed64.FromFraction(3, 5)),
        };
        Fixed64[] lengths = { Fixed64.Zero, Fixed64.Two, (Fixed64)6 };

        foreach (Vector2d firstCenter in centers)
            foreach (Vector2d firstAxis in axes)
                foreach (Fixed64 firstLength in lengths)
                    foreach (Vector2d secondCenter in centers)
                        foreach (Vector2d secondAxis in axes)
                            foreach (Fixed64 secondLength in lengths)
                            {
                                Assert.True(FixedSegment2d.TryGetDistanceBetweenCenteredAxes(
                                    firstCenter,
                                    firstAxis,
                                    firstLength,
                                    secondCenter,
                                    secondAxis,
                                    secondLength,
                                    out Fixed64 forward));
                                Assert.True(FixedSegment2d.TryGetDistanceBetweenCenteredAxes(
                                    secondCenter,
                                    secondAxis,
                                    secondLength,
                                    firstCenter,
                                    firstAxis,
                                    firstLength,
                                    out Fixed64 reverse));
                                Assert.True(
                                    forward == reverse,
                                    $"first={firstCenter}/{firstAxis}/{firstLength}; " +
                                    $"second={secondCenter}/{secondAxis}/{secondLength}; " +
                                    $"forward={forward.m_rawValue}; reverse={reverse.m_rawValue}");
                                Assert.Equal(
                                    FixedSegment2d.DoCenteredCapsulesOverlap(
                                        firstCenter,
                                        firstAxis,
                                        firstLength,
                                        Fixed64.One,
                                        secondCenter,
                                        secondAxis,
                                        secondLength,
                                        Fixed64.Two),
                                    FixedSegment2d.DoCenteredCapsulesOverlap(
                                        secondCenter,
                                        secondAxis,
                                        secondLength,
                                        Fixed64.Two,
                                        firstCenter,
                                        firstAxis,
                                        firstLength,
                                        Fixed64.One));
                            }
    }

    [Fact]
    public void CenteredAxisDecisions_WarmedCallsAllocateNoManagedMemory()
    {
        _ = FixedSegment.TryGetDistanceBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            Vector3d.One,
            Vector3d.Up,
            Fixed64.Two,
            out _);

        long before = GC.GetAllocatedBytesForCurrentThread();
        Fixed64 checksum = Fixed64.Zero;
        for (int index = 0; index < 255; index++)
        {
            Assert.True(FixedSegment.TryGetDistanceBetweenCenteredAxes(
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Two,
                Vector3d.One,
                Vector3d.Up,
                Fixed64.Two,
                out Fixed64 distance));
            checksum += distance;
            if (FixedSegment.DoCenteredCapsulesOverlap(
                    Vector3d.Zero,
                    Vector3d.Right,
                    Fixed64.Two,
                    Fixed64.One,
                    Vector3d.One,
                    Vector3d.Up,
                    Fixed64.Two,
                    Fixed64.One))
            {
                checksum += Fixed64.One;
            }
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.NotEqual(Fixed64.Zero, checksum);
        Assert.Equal(0L, allocated);
    }

    [Fact]
    public void CenteredCapsulesContact_KeepsWideRadiusSumUntilFinalDepth()
    {
        Vector3d firstCenter = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d secondCenter = new(
            Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue - 2L),
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            firstCenter,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.FromRaw(1L), contact.Depth);

        Assert.False(FixedSegment.TryGetCenteredCapsulesContact(
            firstCenter,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector3d.Right,
            out _));
    }

    [Fact]
    public void CenteredCapsulesContact_UsesFallbackOnlyForCoincidentAxes()
    {
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Forward,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.Equal(Vector3d.Forward, contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Backward, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Fixed64.Two, contact.Depth);
    }

    [Fact]
    public void CenteredCapsulesContact_RemainsAuthoritativeWithoutWorldWitnesses()
    {
        Vector3d scalarFace = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            scalarFace,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            scalarFace,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Left, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void CenteredCapsulesContact_ExposeClampedDepthWithoutLosingOverlap()
    {
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void CenteredCapsulesContact_RejectsSeparation()
    {
        Assert.False(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.One,
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Right,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void CenteredCapsulesContact_UsesGeometricNormalWhenAxesDoNotCoincide()
    {
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Right,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Forward,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CenteredCapsulesContact_KeepsLocalFeaturesInvariantUnderRigidPose()
    {
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out FixedContactAnchors baseline));

        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)(-29),
                (Fixed64)11);
        Vector3d translation = new(7, -5, 3);
        Assert.True(rotation.TryRotate(Vector3d.Right, out Vector3d offset));
        Assert.True(FixedSegment.TryGetCenteredCapsulesContact(
            translation,
            rotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            translation + offset,
            rotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            offset.Normalized,
            out FixedContactAnchors transformed));

        Assert.Equal(
            baseline.FirstAnchor.LocalPoint,
            transformed.FirstAnchor.LocalPoint);
        Assert.True(Vector3d.Distance(
            baseline.FirstAnchor.LocalDisplacement,
            transformed.FirstAnchor.LocalDisplacement) <= Fixed64.Epsilon);
        Assert.Equal(
            baseline.SecondAnchor.LocalPoint,
            transformed.SecondAnchor.LocalPoint);
        Assert.True(Vector3d.Distance(
            baseline.SecondAnchor.LocalDisplacement,
            transformed.SecondAnchor.LocalDisplacement) <= Fixed64.Epsilon);
    }

    [Fact]
    public void CenteredCapsules2dContact_RemainsAuthoritativeWithoutWorldWitnesses()
    {
        Vector2d scalarFace = new(Fixed64.MinValue, Fixed64.Zero);

        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            scalarFace,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            scalarFace,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Right,
            out FixedContactAnchors2d contact));
        Assert.Equal(Vector2d.Right, contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(Vector2d.Left, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector2d.Right, contact.Normal);
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void CenteredCapsules2dContact_ExposesClampedDepthWithoutLosingOverlap()
    {
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Vector2d.Forward,
            out FixedContactAnchors2d contact));
        Assert.Equal(
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue),
            contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(
            new Vector2d(Fixed64.Zero, -Fixed64.MaxValue),
            contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector2d.Forward, contact.Normal);
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void CenteredCapsules2dContact_UsesTheExactAxisPointDirection()
    {
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Right,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Forward,
            out FixedContactAnchors2d contact));
        Assert.Equal(Vector2d.Right, contact.FirstAnchor.LocalDisplacement);
        Assert.Equal(Vector2d.Left, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(Vector2d.Right, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CenteredCapsules2dContact_KeepsLocalFeaturesInvariantUnderRigidRotation()
    {
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            Fixed64.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            out FixedContactAnchors2d baseline));
        Fixed64 rotation = Fixed64.HalfPi;
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            rotation,
            Vector2d.Rotate(Vector2d.Forward, rotation),
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Rotate(Vector2d.Right, rotation),
            rotation,
            Vector2d.Rotate(Vector2d.Forward, rotation),
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Rotate(Vector2d.Right, rotation),
            out FixedContactAnchors2d rotated));

        Assert.Equal(baseline.FirstAnchor.LocalPoint, rotated.FirstAnchor.LocalPoint);
        Assert.Equal(
            baseline.FirstAnchor.LocalDisplacement,
            rotated.FirstAnchor.LocalDisplacement);
        Assert.Equal(baseline.SecondAnchor.LocalPoint, rotated.SecondAnchor.LocalPoint);
        Assert.Equal(
            baseline.SecondAnchor.LocalDisplacement,
            rotated.SecondAnchor.LocalDisplacement);
        Assert.Equal(rotation, rotated.FirstAnchor.Rotation);
        Assert.Equal(rotation, rotated.SecondAnchor.Rotation);
    }

    [Fact]
    public void CenteredCapsules2dContact_RetainsUnrepresentableCompositeLocalSupports()
    {
        Assert.True(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector2d.Right,
            out FixedContactAnchors2d contact));
        Assert.Equal(Vector2d.Right, contact.Normal);
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.True(
            !contact.FirstAnchor.TryGetPoint(out _)
            || !contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void CenteredCapsules2dContact_ReturnsFalseForSeparation()
    {
        Assert.False(FixedSegment2d.TryGetCenteredCapsulesContact(
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Vector2d.Right,
            out FixedContactAnchors2d contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void CenteredCapsules2dContact_RejectsInvalidContactInputs()
    {
        static bool Contact(
            Fixed64 firstRadius,
            Fixed64 secondRadius,
            Vector2d fallbackNormal) =>
            FixedSegment2d.TryGetCenteredCapsulesContact(
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                firstRadius,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                secondRadius,
                fallbackNormal,
                out _);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Contact(-Fixed64.One, Fixed64.One, Vector2d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Contact(Fixed64.One, -Fixed64.One, Vector2d.Right));
        Assert.Throws<ArgumentException>(() =>
            Contact(Fixed64.One, Fixed64.One, Vector2d.Zero));

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.DoCenteredCapsulesOverlap(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                -Fixed64.One,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment2d.DoCenteredCapsulesOverlap(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.One,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                -Fixed64.One));
    }

    [Fact]
    public void CenteredCapsulesContact_RejectsInvalidContactInputs()
    {
        static bool Contact(
            Fixed64 firstRadius,
            Fixed64 secondRadius,
            Vector3d fallbackNormal) =>
            FixedSegment.TryGetCenteredCapsulesContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                firstRadius,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                secondRadius,
                fallbackNormal,
                out _);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Contact(-Fixed64.One, Fixed64.One, Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Contact(Fixed64.One, -Fixed64.One, Vector3d.Right));
        Assert.Throws<ArgumentException>(() =>
            Contact(Fixed64.One, Fixed64.One, Vector3d.Zero));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetCenteredCapsulesContact(
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Right,
                out _));
    }
}
