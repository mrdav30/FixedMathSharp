using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedConvexHullRelationsTests
{
    private static readonly Vector3d[] CubePoints =
    {
        new(-1, -1, -1),
        new(1, -1, -1),
        new(1, 1, -1),
        new(-1, 1, -1),
        new(-1, -1, 1),
        new(1, -1, 1),
        new(1, 1, 1),
        new(-1, 1, 1)
    };

    private static readonly int[] CubeTriangles =
    {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5,
        0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2
    };

    private static readonly int[] CubeEdges =
    {
        0, 1, 1, 2, 2, 3, 3, 0,
        4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7
    };

    [Fact]
    public void TryGetContact_ReturnsRigidLocalWitnesses()
    {
        Assert.True(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(1, -1, -1), contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(-1, -1, -1), contact.SecondAnchor.LocalPoint);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(new Vector3d(
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero), separation);
    }

    [Fact]
    public void TryGetContact_PreservesWitnessWhenAbsolutePointIsUnavailable()
    {
        Vector3d firstOrigin = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d secondOrigin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexHullRelations.TryGetContact(
            firstOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            secondOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(new Vector3d(
            Fixed64.FromFraction(7, 4),
            Fixed64.Zero,
            Fixed64.Zero), separation);
    }

    [Fact]
    public void TryGetContact_RejectsFaceAndEdgeSeparations()
    {
        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(3, 0, 0),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out _));

        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAngles(
                Fixed64.PiOver6,
                Fixed64.PiOver4,
                Fixed64.Zero),
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(3, 3, 3),
            FixedQuaternion.FromEulerAngles(
                -Fixed64.PiOver4,
                Fixed64.PiOver6,
                Fixed64.PiOver4),
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out _));
    }

    [Fact]
    public void TryGetContact_RejectsEdgeCrossSeparationBetweenRotatedHulls()
    {
        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(-27, 10),
                Fixed64.FromFraction(-3, 2),
                Fixed64.FromFraction(-19, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)15,
                (Fixed64)30,
                (Fixed64)45),
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out _));
    }

    [Fact]
    public void TryGetContact_OrientsMirroredOverlapTowardTheSecondHull()
    {
        Assert.True(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(-3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            out FixedContactAnchors contact));

        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_ReturnsExactRigidFrameContact()
    {
        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.One, contact.FirstAnchor.LocalPoint.X);
        Assert.Equal(-Fixed64.One, contact.SecondAnchor.LocalDisplacement.X);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_DistinguishesScalarFaceTangency()
    {
        Vector3d hullOrigin = new(
            Fixed64.MaxValue - (Fixed64)2,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hullOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d hullPoint));
        Assert.Equal(
            Fixed64.MaxValue - Fixed64.One,
            hullPoint.X);

        bool hasMiss = FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hullOrigin,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            (Fixed64)2,
            Fixed64.FromRaw(Fixed64.One.m_rawValue - 1L),
            out FixedContactAnchors missContact);
        Assert.False(
            hasMiss,
            $"normal={missContact.Normal}, depthRaw={missContact.Depth.m_rawValue}");
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_PreservesUnrepresentableCapsuleWitness()
    {
        var points = new Vector3d[CubePoints.Length];
        for (int index = 0; index < points.Length; index++)
        {
            points[index] = new Vector3d(
                CubePoints[index].X * Fixed64.FromFraction(1, 4),
                CubePoints[index].Y * (Fixed64)2,
                CubePoints[index].Z * (Fixed64)2);
        }

        Vector3d hullOrigin = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue - Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hullOrigin,
            FixedQuaternion.Identity,
            points,
            CubeTriangles,
            CubeEdges,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(-Vector3d.Right, contact.Normal);
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(
            contact.SecondAnchor,
            out Vector3d separation));
        Assert.Equal(contact.Depth, Vector3d.Dot(separation, contact.Normal));
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_ContainsAngledCore()
    {
        var points = new Vector3d[CubePoints.Length];
        for (int index = 0; index < points.Length; index++)
        {
            points[index] = new Vector3d(
                CubePoints[index].X * Fixed64.Half,
                CubePoints[index].Y * Fixed64.Half,
                CubePoints[index].Z * (Fixed64)2);
        }
        Vector3d segmentDirection = new(
            Fixed64.FromFraction(4, 5),
            Fixed64.FromFraction(3, 5),
            Fixed64.Zero);
        Vector3d rotationAxis =
            Vector3d.Cross(Vector3d.Up, segmentDirection);
        FixedQuaternion rotation = new FixedQuaternion(
            rotationAxis.X,
            rotationAxis.Y,
            rotationAxis.Z,
            Fixed64.One
            + Vector3d.Dot(Vector3d.Up, segmentDirection)).Normalized;

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            points,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.FromFraction(1, 10),
                Fixed64.Zero),
            rotation,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.FromFraction(1, 10),
            out FixedContactAnchors contact));

        Assert.True(contact.Depth > Fixed64.Zero);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_UsesOpenPlanarHullFace()
    {
        Vector3d[] points =
        {
            new(-1, 0, -1),
            new(1, 0, -1),
            new(1, 0, 1),
            new(-1, 0, 1),
        };
        int[] triangles = { 0, 2, 1, 0, 3, 2 };
        int[] edges = { 0, 1, 1, 2, 2, 3, 3, 0 };

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            points,
            triangles,
            edges,
            Vector3d.Up,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            points,
            triangles,
            edges,
            Vector3d.Up,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One - Fixed64.FromRaw(1),
            out _));
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_UsesClosestEdgeAxis()
    {
        FixedQuaternion capsuleRotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)(-90));

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(7, 5),
                Fixed64.FromFraction(7, 5)),
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out _));

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(27, 20),
                Fixed64.FromFraction(27, 20)),
            capsuleRotation,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Normal.Y > Fixed64.Zero);
        Assert.True(contact.Normal.Z > Fixed64.Zero);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsEdgeCrossSeparation()
    {
        Vector3d capsuleAxis = new(1, 1, 1);

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.Zero,
                -Fixed64.FromFraction(13, 10),
                Fixed64.FromFraction(13, 10)),
            WideGeometry.GetCanonicalAxisRotation(capsuleAxis),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.FromFraction(1, 10),
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsEndpointCornerSeparation()
    {
        var halfCube = new Vector3d[CubePoints.Length];
        for (int index = 0; index < halfCube.Length; index++)
            halfCube[index] = CubePoints[index] * Fixed64.Half;

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(3, 2),
                Fixed64.FromFraction(5, 2)),
            FixedQuaternion.Identity,
            halfCube,
            CubeTriangles,
            CubeEdges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Forward,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsEndpointToEdgeSeparation()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(Fixed64.Zero, diagonal, diagonal);
        Vector3d endpoint = new(
            Fixed64.Zero,
            Fixed64.FromFraction(7, 5),
            Fixed64.FromFraction(7, 5));
        Vector3d center = endpoint + axis;

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            center,
            WideGeometry.GetCanonicalAxisRotation(axis),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RejectsAConcentricDegenerateHull()
    {
        Vector3d[] pointHull =
        {
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
        };
        int[] triangle = { 0, 1, 2 };
        int[] edges = { 0, 1, 1, 2, 2, 0 };

        Assert.False(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointHull,
            triangle,
            edges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_DistinguishesMaximumDepthFromOverflow()
    {
        var exactMaximumHull = new Vector3d[CubePoints.Length];
        var overflowingHull = new Vector3d[CubePoints.Length];
        Fixed64 exactExtent = Fixed64.MaxValue - Fixed64.One;
        for (int index = 0; index < CubePoints.Length; index++)
        {
            exactMaximumHull[index] = CubePoints[index] * exactExtent;
            overflowingHull[index] = CubePoints[index] * Fixed64.MaxValue;
        }

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            exactMaximumHull,
            CubeTriangles,
            CubeEdges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors exactMaximum));
        Assert.Equal(Fixed64.MaxValue, exactMaximum.Depth);
        Assert.False(exactMaximum.DepthIsClamped);

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            overflowingHull,
            CubeTriangles,
            CubeEdges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            out FixedContactAnchors overflow));
        Assert.Equal(Fixed64.MaxValue, overflow.Depth);
        Assert.True(overflow.DepthIsClamped);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_RoundsSubLatticeCornerDepth()
    {
        Fixed64 oneRaw = Fixed64.MinIncrement;

        Assert.True(FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.One + oneRaw,
                Fixed64.One + oneRaw,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.FromRaw(2),
            out FixedContactAnchors contact));

        Assert.Equal(oneRaw, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(1, 1, 0), contact.FirstAnchor.LocalPoint);
    }

    [Fact]
    public void TryGetCenteredCapsuleContact_IsDirtyStackDeterministicAndAllocationFree()
    {
        Assert.True(TryGetExtremeCapsuleContact(
            out FixedContactAnchors expected));
        for (int iteration = 0; iteration < 8; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetExtremeCapsuleContact(
                out FixedContactAnchors actual));
            Assert.Equal(expected, actual);
        }

        _ = TryGetExtremeCapsuleContact(out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetExtremeCapsuleContact(out _))
                contacts++;
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(8, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void TryGetContact_RejectsHullsWithoutAUsableSeparatingAxis()
    {
        Vector3d[] pointHull =
        {
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero
        };
        int[] triangle = { 0, 1, 2 };
        int[] edges = { 0, 1, 1, 2, 2, 0 };

        Assert.False(FixedConvexHullRelations.TryGetContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointHull,
            triangle,
            edges,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            pointHull,
            triangle,
            edges,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ContainsPoint_UsesExactRigidFrames()
    {
        Vector3d origin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            Fixed64.PiOver4);

        Assert.True(FixedConvexHullRelations.ContainsPoint(
            origin,
            rotation,
            CubePoints,
            CubeTriangles,
            Vector3d.Zero,
            new FixedPointAnchor(
                origin,
                rotation,
                new Vector3d(
                    Fixed64.Half,
                    Fixed64.Zero,
                    Fixed64.Zero))));
        Assert.False(FixedConvexHullRelations.ContainsPoint(
            origin,
            rotation,
            CubePoints,
            CubeTriangles,
            Vector3d.Zero,
            new FixedPointAnchor(
                origin,
                rotation,
                new Vector3d(2, 0, 0))));
        Assert.False(FixedConvexHullRelations.ContainsPoint(
            origin,
            rotation,
            CubePoints,
            CubeTriangles,
            Vector3d.Zero,
            default));
    }

    [Fact]
    public void Relations_RejectMalformedHullContracts()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                default,
                CubePoints,
                CubeTriangles,
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints.AsSpan(0, 2),
                CubeTriangles,
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                Array.Empty<int>(),
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                new[] { 0, 1 },
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexHullRelations.ContainsPoint(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                new[] { 0, 1, CubePoints.Length },
                Vector3d.Zero,
                default));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                new[] { 0 },
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                Array.Empty<int>(),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                Array.Empty<int>(),
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                default,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                Array.Empty<int>(),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                -Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedConvexHullRelations.TryGetCenteredCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                CubePoints,
                CubeTriangles,
                CubeEdges,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Fixed64.One,
                -Fixed64.One,
                out _));
    }

    private static bool TryGetExtremeCapsuleContact(
        out FixedContactAnchors contact)
    {
        return FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            new Vector3d(
                Fixed64.MaxValue - (Fixed64)2,
                Fixed64.MinValue + (Fixed64)2,
                Fixed64.MaxValue - (Fixed64)2),
            FixedQuaternion.Identity,
            CubePoints,
            CubeTriangles,
            CubeEdges,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MinValue + (Fixed64)2,
                Fixed64.MaxValue - (Fixed64)2),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out contact);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void PolluteStack(ulong seed)
    {
        Span<ulong> words = stackalloc ulong[2_048];
        for (int index = 0; index < words.Length; index++)
        {
            words[index] =
                seed
                + unchecked((ulong)index * 0x9E37_79B9UL);
        }
        GC.KeepAlive(words[unchecked((int)seed) & 2_047]);
    }
}
