using System;
using System.Runtime.CompilerServices;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderCapsuleRigidFrameTests
{
    [Fact]
    public void Contact_UsesCanonicalRigidAxesAtScalarFace()
    {
        Vector3d cylinderCenter = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            cylinderCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                cylinderCenter,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Half,
                capsuleCenter,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Half,
                out _,
                out bool usedWideCandidate));
        Assert.False(usedWideCandidate);

        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            cylinderCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half - Fixed64.FromRaw(1),
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out _));
    }

    [Fact]
    public void Contact_RejectsSeparationOnTheCapsuleAndCrossAxes()
    {
        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right * (Fixed64)4,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            out _));

        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Forward * (Fixed64)3,
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void Contact_RanksAxialAndRadialCandidates()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(
                Fixed64.Half,
                Fixed64.One + Fixed64.Half,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.One + Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_CapsuleRadiusCanRecoverANegativeCylinderProjection()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Right,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_CapsuleRadiusExtendsATangentCenterline()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
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
            out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_ReportsDepthBeyondTheScalarDomain()
    {
        FixedQuaternion quarterTurn =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.HalfPi);

        Assert.True(WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Vector3d.Zero,
                quarterTurn,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                out FixedContactAnchors contact,
                out bool usedWideCandidate));

        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.False(usedWideCandidate);
    }

    [Fact]
    public void Contact_PreservesAnExactMaximumDepthWithoutClamping()
    {
        Assert.True(FixedSegment
            .TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.Zero,
                out FixedContactAnchors contact));

        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_ClampsAnExactDepthHalfARawUnitAboveMaximum()
    {
        Assert.True(FixedSegment
            .TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.MinIncrement,
                Fixed64.MaxValue,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Fixed64.MinIncrement,
                Fixed64.MaxValue,
                out FixedContactAnchors contact));

        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.True(contact.Depth >= Fixed64.Zero);
    }

    [Fact]
    public void Contact_FullDomainCancellationPreservesTangency()
    {
        Assert.True(TryGetFullDomainCancellationContact(
            offsetRaw: 1,
            mirrored: false,
            out FixedContactAnchors contact));
        Assert.True(TryGetFullDomainCancellationContact(
            offsetRaw: 1,
            mirrored: true,
            out FixedContactAnchors mirrored));

        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(contact.Depth, mirrored.Depth);
        Assert.Equal(-contact.Normal, mirrored.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.False(mirrored.DepthIsClamped);
    }

    [Fact]
    public void Contact_FullDomainCancellationPreservesOneRawUnitOverlap()
    {
        Assert.True(TryGetFullDomainCancellationContact(
            offsetRaw: 2,
            mirrored: false,
            out FixedContactAnchors contact));
        Assert.True(TryGetFullDomainCancellationContact(
            offsetRaw: 2,
            mirrored: true,
            out FixedContactAnchors mirrored));

        Assert.Equal(Fixed64.FromRaw(1), contact.Depth);
        Assert.Equal(contact.Depth, mirrored.Depth);
        Assert.Equal(-contact.Normal, mirrored.Normal);
    }

    [Fact]
    public void Contact_FullDomainCancellationIsDirtyStackDeterministicAndAllocationFree()
    {
        Assert.True(TryGetFullDomainCancellationContact(
            offsetRaw: 2,
            mirrored: false,
            out FixedContactAnchors expected));
        for (int iteration = 0; iteration < 8; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetFullDomainCancellationContact(
                offsetRaw: 2,
                mirrored: false,
                out FixedContactAnchors actual));
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.Equal(expected.Depth, actual.Depth);
        }

        for (int iteration = 0; iteration < 4; iteration++)
        {
            _ = TryGetFullDomainCancellationContact(
                offsetRaw: 2,
                mirrored: false,
                out _);
        }
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetFullDomainCancellationContact(
                    offsetRaw: 2,
                    mirrored: false,
                    out _))
            {
                contacts++;
            }
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(8, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Contact_RotatedFullDomainCancellationRoundsExactSupportDepth()
    {
        int[] angleDivisors = { 12, 10, 8, 7, 6, 5, 4, 3 };
        foreach (int divisor in angleDivisors)
        {
            FixedQuaternion rotation =
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Forward,
                    Fixed64.Pi / (Fixed64)divisor);
            Vector3d cylinderCenter = new(
                Fixed64.MinValue + Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
            Vector3d capsuleCenter = new(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero);
            Assert.True(FixedSegment
                .TryGetCenteredFiniteCylinderCapsuleContact(
                    cylinderCenter,
                    rotation,
                    Vector3d.Up,
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    capsuleCenter,
                    rotation,
                    Vector3d.Up,
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    out FixedContactAnchors contact));

            Assert.True(contact.Normal.IsNormalized());
            Assert.True(contact.Depth > Fixed64.Zero);
            Assert.False(contact.DepthIsClamped);
            Assert.True(contact.SecondAnchor.TryGetProjectedOffsetFrom(
                contact.FirstAnchor,
                contact.Normal,
                out Fixed64 projectedSeparation));
            Assert.InRange(
                (projectedSeparation + contact.Depth).Abs().m_rawValue,
                0L,
                Fixed64.One.m_rawValue);
        }
    }

    [Fact]
    public void Contact_UsesClampedCapToInteriorCandidate()
    {
        FixedQuaternion cylinderRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver6);
        FixedQuaternion capsuleRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.PiOver6);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            Vector3d.Zero,
            cylinderRotation,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            new Vector3d(-2, -2, -1),
            capsuleRotation,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            out FixedContactAnchors contact));

        // The capsule cap clamps while the cylinder feature remains interior.
        // None of the two rigid axes, their cross, or the center difference is
        // the separating-axis winner for this configuration.
        Assert.Equal(
            new Vector3d(
                Fixed64.FromRaw(-3_719_550_790L),
                Fixed64.FromRaw(-2_147_483_642L),
                Fixed64.FromRaw(-6L)),
            contact.Normal);
        Assert.Equal(Fixed64.FromRaw(575_416_507L), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_RejectsInvalidRigidFramesAndDimensions()
    {
        FixedQuaternion invalidRotation = default;
        Vector3d invalidAxis = new(Fixed64.One, Fixed64.One, Fixed64.Zero);

        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                invalidRotation,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                invalidAxis,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                invalidRotation,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                invalidAxis,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                -Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                out _));
    }

    [Fact]
    public void Contact_IrreducibleClosestCandidateUsesWideFallback()
    {
        Assert.True(TryGetIrreducibleContact(
            out FixedContactAnchors contact,
            out bool usedWideCandidate));
        Assert.True(usedWideCandidate);
        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.SecondAnchor.TryGetProjectedOffsetFrom(
            contact.FirstAnchor,
            contact.Normal,
            out Fixed64 projectedSeparation));
        // One raw direction unit can become a sub-unit projected witness
        // difference across near-domain-scale support lengths.
        Assert.InRange(
            (projectedSeparation + contact.Depth).Abs().m_rawValue,
            0L,
            Fixed64.One.m_rawValue);

        FixedQuaternion halfTurn =
            new(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
        Assert.True(TryGetIrreducibleContact(
            halfTurn,
            out FixedContactAnchors rotated,
            out bool rotatedWide));
        Assert.True(rotatedWide);
        Assert.Equal(contact.Depth, rotated.Depth);
        Assert.Equal(halfTurn * contact.Normal, rotated.Normal);
        AssertCommonRotation(halfTurn, contact, rotated);
    }

    [Fact]
    public void Contact_SeparatingAlongIrreducibleClosestAxisRejectsThePair()
    {
        Vector3d capsuleCenter = new(
            Fixed64.FromRaw(-682_964_183_777L),
            Fixed64.FromRaw(-140_355_624_925L),
            Fixed64.FromRaw(-284_939_450_016L));
        Assert.True(TryGetRelativeIrreducibleContact(
            capsuleCenter,
            out FixedContactAnchors contact,
            out bool usedWideCandidate));
        Assert.True(usedWideCandidate);

        Vector3d separatedCenter =
            capsuleCenter
            + contact.Normal * (contact.Depth + Fixed64.One);
        Assert.False(TryGetRelativeIrreducibleContact(
            separatedCenter,
            out _,
            out _));
    }

    [Fact]
    public void Contact_IrreducibleOverlapBeyondTheScalarDomainIsExplicitlyClamped()
    {
        Assert.True(TryGetClampedIrreducibleContact(
            out FixedContactAnchors contact,
            out bool usedWideCandidate));

        Assert.True(usedWideCandidate);
        Assert.True(contact.Normal.IsNormalized());
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_ExtremeTranslationMatchesTheReducedGeometryOracle()
    {
        Assert.True(TryGetFormerWideFixture(
            reduceTranslation: false,
            out FixedContactAnchors extreme,
            out bool extremeWide));
        Assert.True(TryGetFormerWideFixture(
            reduceTranslation: true,
            out FixedContactAnchors reduced,
            out bool reducedWide));

        Assert.False(extremeWide);
        Assert.False(reducedWide);
        Assert.Equal(reduced.Normal, extreme.Normal);
        Assert.Equal(reduced.Depth, extreme.Depth);
        Assert.Equal(
            reduced.DepthIsClamped,
            extreme.DepthIsClamped);
        Assert.Equal(
            0,
            reduced.FirstAnchor.CompareLocalFeature(
                extreme.FirstAnchor));
        Assert.Equal(
            0,
            reduced.SecondAnchor.CompareLocalFeature(
                extreme.SecondAnchor));
    }

    [Fact]
    public void Contact_WideFallbackIsDirtyStackDeterministic()
    {
        Assert.True(TryGetIrreducibleContact(
            out FixedContactAnchors expected,
            out bool expectedWide));
        Assert.True(expectedWide);

        for (int iteration = 0; iteration < 32; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetIrreducibleContact(
                out FixedContactAnchors actual,
                out bool usedWide));
            Assert.True(usedWide);
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.Equal(expected.Depth, actual.Depth);
            Assert.Equal(
                expected.DepthIsClamped,
                actual.DepthIsClamped);
        }
    }

    [Fact]
    public void Contact_WarmedWideFallbackDoesNotAllocate()
    {
        for (int iteration = 0; iteration < 4; iteration++)
            _ = TryGetIrreducibleContact(out _, out _);

        int contacts = 0;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 16; iteration++)
        {
            if (TryGetIrreducibleContact(out _, out _))
                contacts++;
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(16, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Contact_LowerEndpointCandidateMatchesTheRigidFrameOracle()
    {
        FixedQuaternion cylinderRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        Vector3d capsuleCenter =
            new(
                Fixed64.One + Fixed64.Half,
                -Fixed64.Half,
                Fixed64.Two + Fixed64.Half);
        Vector3d cylinderLower =
            -(cylinderRotation * Vector3d.Right);
        Vector3d capsuleLower =
            capsuleCenter - (Vector3d.Right * Fixed64.Two);
        Vector3d separation = capsuleLower - cylinderLower;
        FixedQuaternion halfTurn =
            new(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);

        Assert.True(TryGetLowerEndpointContact(
            Vector3d.Zero,
            cylinderRotation,
            capsuleCenter,
            FixedQuaternion.Identity,
            out FixedContactAnchors contact));
        AssertVectorWithinOneRawUnit(
            separation.Normalized,
            contact.Normal);
        Fixed64 oracleDepth =
            Fixed64.Two - separation.Magnitude;
        Assert.InRange(
            (oracleDepth - contact.Depth).Abs().m_rawValue,
            0L,
            8L);

        Assert.True(TryGetLowerEndpointContact(
            Vector3d.Zero,
            halfTurn * cylinderRotation,
            halfTurn * capsuleCenter,
            halfTurn,
            out FixedContactAnchors rotated));
        Assert.Equal(contact.Depth, rotated.Depth);
        Assert.Equal(halfTurn * contact.Normal, rotated.Normal);
    }

    [Fact]
    public void Contact_LowerEndpointCandidateIsDirtyStackDeterministicAndAllocationFree()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        Vector3d center =
            new(
                Fixed64.One + Fixed64.Half,
                -Fixed64.Half,
                Fixed64.Two + Fixed64.Half);
        Assert.True(TryGetLowerEndpointContact(
            Vector3d.Zero,
            rotation,
            center,
            FixedQuaternion.Identity,
            out FixedContactAnchors expected));

        for (int iteration = 0; iteration < 16; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetLowerEndpointContact(
                Vector3d.Zero,
                rotation,
                center,
                FixedQuaternion.Identity,
                out FixedContactAnchors actual));
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.Equal(expected.Depth, actual.Depth);
        }

        _ = TryGetLowerEndpointContact(
            Vector3d.Zero,
            rotation,
            center,
            FixedQuaternion.Identity,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 16; iteration++)
        {
            if (TryGetLowerEndpointContact(
                    Vector3d.Zero,
                    rotation,
                    center,
                    FixedQuaternion.Identity,
                    out _))
            {
                contacts++;
            }
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(16, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Contact_TopologyMatrixIsInvariantUnderACommonRigidRotation()
    {
        FixedQuaternion halfTurn =
            new(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
        FixedQuaternion[] rotations =
        {
            FixedQuaternion.Identity,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.PiOver6),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver6),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4),
        };
        Vector3d[] axes =
        {
            Vector3d.Up,
            Vector3d.Right,
            Vector3d.Forward,
        };
        Vector3d[] centers =
        {
            Vector3d.Zero,
            new(Fixed64.Half, Fixed64.One + Fixed64.Half, Fixed64.Zero),
            Vector3d.Right * (Fixed64)4,
            Vector3d.Up * (Fixed64)4,
            Vector3d.Forward * (Fixed64)4,
            new(-2, -2, -1),
            Vector3d.One,
            new(
                Fixed64.One + Fixed64.Half,
                -Fixed64.Half,
                Fixed64.Two + Fixed64.Half),
        };
        Fixed64[] lengths =
        {
            Fixed64.One,
            Fixed64.Two,
            (Fixed64)4,
            Fixed64.MaxValue / (Fixed64)8,
        };
        Fixed64[] radii =
        {
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Two,
        };

        for (int index = 0; index < 144; index++)
        {
            FixedQuaternion cylinderRotation =
                rotations[index % rotations.Length];
            FixedQuaternion capsuleRotation =
                rotations[(index / rotations.Length) % rotations.Length];
            Vector3d cylinderAxis =
                axes[(index / 7) % axes.Length];
            Vector3d capsuleAxis =
                axes[(index / 11) % axes.Length];
            Vector3d capsuleCenter =
                centers[(index * 5) % centers.Length];
            Fixed64 cylinderLength =
                lengths[(index * 3) % lengths.Length];
            Fixed64 capsuleLength =
                lengths[(index * 7 + 1) % lengths.Length];
            Fixed64 cylinderRadius =
                radii[(index * 5 + 1) % radii.Length];
            Fixed64 capsuleRadius =
                radii[(index * 7 + 2) % radii.Length];

            bool hit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylinderCapsuleContact(
                    Vector3d.Zero,
                    cylinderRotation,
                    cylinderAxis,
                    cylinderLength,
                    cylinderRadius,
                    capsuleCenter,
                    capsuleRotation,
                    capsuleAxis,
                    capsuleLength,
                    capsuleRadius,
                    out FixedContactAnchors contact,
                    out bool usedWide);
            bool rotatedHit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylinderCapsuleContact(
                    Vector3d.Zero,
                    halfTurn * cylinderRotation,
                    cylinderAxis,
                    cylinderLength,
                    cylinderRadius,
                    halfTurn * capsuleCenter,
                    halfTurn * capsuleRotation,
                    capsuleAxis,
                    capsuleLength,
                    capsuleRadius,
                    out FixedContactAnchors rotatedContact,
                    out bool rotatedUsedWide);

            Assert.Equal(hit, rotatedHit);
            if (!hit)
                continue;
            Assert.True(contact.Normal.IsNormalized());
            Assert.True(contact.Depth >= Fixed64.Zero);
            Assert.True(
                contact.Depth == rotatedContact.Depth,
                $"Topology index {index}: expected rotated depth {contact.Depth}, got {rotatedContact.Depth}; normals {contact.Normal} -> {rotatedContact.Normal}; wide {usedWide}/{rotatedUsedWide}.");
            Assert.Equal(
                contact.DepthIsClamped,
                rotatedContact.DepthIsClamped);
            Assert.Equal(
                halfTurn * contact.Normal,
                rotatedContact.Normal);
            AssertCommonRotation(
                halfTurn,
                contact,
                rotatedContact);
        }
    }

    [Fact]
    public void Contact_DeterministicFractionalDomainSampleIsInvariantUnderACommonRigidRotation()
    {
        FixedQuaternion halfTurn =
            new(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
        FixedQuaternion[] rotations =
        {
            FixedQuaternion.Identity,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.PiOver6),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4),
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.Pi / (Fixed64)3),
        };
        Vector3d[] axes =
        {
            Vector3d.Right,
            Vector3d.Up,
            Vector3d.Forward,
        };
        ulong state = 0xE703_7ED1_A0B4_28DBUL;
        int contacts = 0;
        for (int index = 0; index < 64; index++)
        {
            Vector3d cylinderCenter = new(
                NextSignedRaw(ref state, 2L << 32),
                NextSignedRaw(ref state, 2L << 32),
                NextSignedRaw(ref state, 2L << 32));
            Vector3d capsuleCenter = new(
                NextSignedRaw(ref state, 2L << 32),
                NextSignedRaw(ref state, 2L << 32),
                NextSignedRaw(ref state, 2L << 32));
            FixedQuaternion cylinderRotation =
                rotations[NextIndex(ref state, rotations.Length)];
            FixedQuaternion capsuleRotation =
                rotations[NextIndex(ref state, rotations.Length)];
            Vector3d cylinderAxis =
                axes[NextIndex(ref state, axes.Length)];
            Vector3d capsuleAxis =
                axes[NextIndex(ref state, axes.Length)];
            Fixed64 cylinderLength =
                NextPositiveRaw(ref state, 4L << 32);
            Fixed64 capsuleLength =
                NextPositiveRaw(ref state, 4L << 32);
            Fixed64 cylinderRadius =
                NextNonNegativeRaw(ref state, 3L << 32);
            Fixed64 capsuleRadius =
                NextNonNegativeRaw(ref state, 3L << 32);

            bool hit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylinderCapsuleContact(
                    cylinderCenter,
                    cylinderRotation,
                    cylinderAxis,
                    cylinderLength,
                    cylinderRadius,
                    capsuleCenter,
                    capsuleRotation,
                    capsuleAxis,
                    capsuleLength,
                    capsuleRadius,
                    out FixedContactAnchors contact,
                    out bool usedWide);
            bool rotatedHit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylinderCapsuleContact(
                    halfTurn * cylinderCenter,
                    halfTurn * cylinderRotation,
                    cylinderAxis,
                    cylinderLength,
                    cylinderRadius,
                    halfTurn * capsuleCenter,
                    halfTurn * capsuleRotation,
                    capsuleAxis,
                    capsuleLength,
                    capsuleRadius,
                    out FixedContactAnchors rotated,
                    out bool rotatedUsedWide);

            Assert.Equal(hit, rotatedHit);
            if (!hit)
                continue;
            contacts++;
            Assert.True(
                contact.Depth == rotated.Depth,
                $"Sample {index}: depths {contact.Depth}/{rotated.Depth}; normals {contact.Normal}/{rotated.Normal}; wide {usedWide}/{rotatedUsedWide}.");
            Assert.Equal(
                contact.DepthIsClamped,
                rotated.DepthIsClamped);
            Assert.Equal(
                halfTurn * contact.Normal,
                rotated.Normal);
            AssertCommonRotation(
                halfTurn,
                contact,
                rotated);
        }

        Assert.True(contacts > 32);
    }

    private static bool TryGetIrreducibleContact(
        out FixedContactAnchors contact,
        out bool usedWideCandidate) =>
        TryGetIrreducibleContact(
            FixedQuaternion.Identity,
            out contact,
            out usedWideCandidate);

    private static bool TryGetIrreducibleContact(
        FixedQuaternion commonRotation,
        out FixedContactAnchors contact,
        out bool usedWideCandidate)
    {
        Vector3d cylinderCenter = new(
            Fixed64.FromRaw(-9_223_371_298_328_934_912L),
            Fixed64.FromRaw(-9_223_371_221_263_323_628L),
            Fixed64.FromRaw(-9_223_371_427_101_329_995L));
        Vector3d capsuleCenter = new(
            Fixed64.FromRaw(-9_223_371_981_293_118_689L),
            Fixed64.FromRaw(-9_223_371_361_618_948_553L),
            Fixed64.FromRaw(-9_223_371_712_040_780_011L));
        FixedQuaternion cylinderRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        FixedQuaternion capsuleRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                commonRotation * cylinderCenter,
                commonRotation * cylinderRotation,
                Vector3d.Right,
                Fixed64.FromRaw(9_223_371_717_157_954_270L),
                Fixed64.FromRaw(360_447_514_427L),
                commonRotation * capsuleCenter,
                commonRotation * capsuleRotation,
                Vector3d.Forward,
                Fixed64.FromRaw(855_113_133_883L),
                Fixed64.FromRaw(9_223_371_872_063_158_705L),
                out contact,
                out usedWideCandidate);
    }

    private static bool TryGetRelativeIrreducibleContact(
        Vector3d capsuleCenter,
        out FixedContactAnchors contact,
        out bool usedWideCandidate)
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                Vector3d.Zero,
                rotation,
                Vector3d.Right,
                Fixed64.FromRaw(9_223_371_717_157_954_270L),
                Fixed64.FromRaw(360_447_514_427L),
                capsuleCenter,
                rotation,
                Vector3d.Forward,
                Fixed64.FromRaw(855_113_133_883L),
                Fixed64.FromRaw(9_223_371_872_063_158_705L),
                out contact,
                out usedWideCandidate);
    }

    private static bool TryGetFormerWideFixture(
        bool reduceTranslation,
        out FixedContactAnchors contact,
        out bool usedWideCandidate)
    {
        Vector3d cylinderCenter = reduceTranslation
            ? Vector3d.Zero
            : new Vector3d(
                Fixed64.MaxValue - (Fixed64)8,
                Fixed64.MinValue + (Fixed64)8,
                Fixed64.MaxValue - (Fixed64)16);
        Vector3d capsuleCenter = reduceTranslation
            ? new Vector3d(5, 5, 11)
            : new Vector3d(
                Fixed64.MaxValue - (Fixed64)3,
                Fixed64.MinValue + (Fixed64)13,
                Fixed64.MaxValue - (Fixed64)5);
        var cylinderRotation = new FixedQuaternion(
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(1_191_209_601L));
        FixedQuaternion capsuleRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.Pi / (Fixed64)3);
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                cylinderCenter,
                cylinderRotation,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue / (Fixed64)8,
                capsuleCenter,
                capsuleRotation,
                Vector3d.Up,
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.MaxValue / (Fixed64)8,
                out contact,
                out usedWideCandidate);
    }

    private static bool TryGetClampedIrreducibleContact(
        out FixedContactAnchors contact,
        out bool usedWideCandidate)
    {
        Vector3d cylinderCenter = new(
            Fixed64.FromRaw(-9_223_371_836_141_996_263L),
            Fixed64.FromRaw(-9_223_371_831_503_237_588L),
            Fixed64.FromRaw(-9_223_371_589_877_671_470L));
        Vector3d capsuleCenter = new(
            Fixed64.FromRaw(-9_223_371_392_634_327_554L),
            Fixed64.FromRaw(-9_223_371_627_891_749_603L),
            Fixed64.FromRaw(-9_223_371_800_738_914_912L));
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                cylinderCenter,
                FixedQuaternion.Identity,
                Vector3d.Forward,
                Fixed64.FromRaw(4_611_686_596_758_417_642L),
                Fixed64.FromRaw(4_611_686_426_396_932_581L),
                capsuleCenter,
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Forward,
                    Fixed64.Pi / (Fixed64)3),
                Vector3d.Forward,
                Fixed64.FromRaw(4_611_686_427_549_936_991L),
                Fixed64.FromRaw(9_223_371_370_890_567_190L),
                out contact,
                out usedWideCandidate);
    }

    private static bool TryGetLowerEndpointContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        out FixedContactAnchors contact) =>
        FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
            cylinderCenter,
            cylinderRotation,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.Zero,
            capsuleCenter,
            capsuleRotation,
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Two,
            out contact);

    private static bool TryGetFullDomainCancellationContact(
        long offsetRaw,
        bool mirrored,
        out FixedContactAnchors contact)
    {
        Fixed64 firstX =
            Fixed64.MinValue + Fixed64.FromRaw(offsetRaw);
        Fixed64 secondX = Fixed64.MaxValue;
        if (mirrored)
        {
            firstX = -firstX;
            secondX = -secondX;
        }
        return FixedSegment
            .TryGetCenteredFiniteCylinderCapsuleContact(
                new Vector3d(
                    firstX,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.MaxValue,
                new Vector3d(
                    secondX,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.MaxValue,
                out contact);
    }

    private static void AssertVectorWithinOneRawUnit(
        Vector3d expected,
        Vector3d actual)
    {
        Assert.InRange(
            (expected.X - actual.X).Abs().m_rawValue,
            0L,
            1L);
        Assert.InRange(
            (expected.Y - actual.Y).Abs().m_rawValue,
            0L,
            1L);
        Assert.InRange(
            (expected.Z - actual.Z).Abs().m_rawValue,
            0L,
            1L);
    }

    private static void AssertCommonRotation(
        FixedQuaternion rotation,
        in FixedContactAnchors original,
        in FixedContactAnchors rotated)
    {
        Assert.Equal(
            rotation * original.FirstAnchor.Origin,
            rotated.FirstAnchor.Origin);
        Assert.Equal(
            rotation * original.FirstAnchor.Rotation,
            rotated.FirstAnchor.Rotation);
        Assert.Equal(
            0,
            original.FirstAnchor.CompareLocalFeature(
                rotated.FirstAnchor));
        Assert.Equal(
            rotation * original.SecondAnchor.Origin,
            rotated.SecondAnchor.Origin);
        Assert.Equal(
            rotation * original.SecondAnchor.Rotation,
            rotated.SecondAnchor.Rotation);
        Assert.Equal(
            0,
            original.SecondAnchor.CompareLocalFeature(
                rotated.SecondAnchor));
    }

    private static Fixed64 NextSignedRaw(
        ref ulong state,
        long magnitudeExclusive)
    {
        ulong range = unchecked((ulong)magnitudeExclusive * 2UL);
        return Fixed64.FromRaw(
            unchecked((long)(NextUInt64(ref state) % range))
            - magnitudeExclusive);
    }

    private static Fixed64 NextPositiveRaw(
        ref ulong state,
        long maximumInclusive) =>
        Fixed64.FromRaw(
            unchecked((long)(
                NextUInt64(ref state)
                % unchecked((ulong)maximumInclusive))) + 1L);

    private static Fixed64 NextNonNegativeRaw(
        ref ulong state,
        long maximumInclusive) =>
        Fixed64.FromRaw(
            unchecked((long)(
                NextUInt64(ref state)
                % unchecked((ulong)maximumInclusive + 1UL))));

    private static int NextIndex(
        ref ulong state,
        int length) =>
        unchecked((int)(NextUInt64(ref state) % (uint)length));

    private static ulong NextUInt64(ref ulong state)
    {
        state += 0x9E37_79B9_7F4A_7C15UL;
        ulong value = state;
        value =
            (value ^ (value >> 30))
            * 0xBF58_476D_1CE4_E5B9UL;
        value =
            (value ^ (value >> 27))
            * 0x94D0_49BB_1331_11EBUL;
        return value ^ (value >> 31);
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
        GC.KeepAlive(words[seed.GetHashCode() & 2_047]);
    }
}
