using System;
using System.Runtime.CompilerServices;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderRigidFrameTests
{
    [Fact]
    public void Contact_UsesExactMultiRadicalCapToInteriorWinner()
    {
        FixedQuaternion firstRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver6);
        FixedQuaternion secondRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.PiOver6);

        Assert.True(WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero,
                firstRotation,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                new Vector3d(-2, -2, -1),
                secondRotation,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                out FixedContactAnchors contact,
                out bool usedWide,
                out bool usedMulti));

        Assert.False(usedWide);
        Assert.True(usedMulti);
        Assert.Equal(
            new Vector3d(
                Fixed64.FromRaw(-3_719_550_790L),
                Fixed64.FromRaw(-2_147_483_642L),
                Fixed64.FromRaw(-6L)),
            contact.Normal);
        Assert.Equal(
            Fixed64.FromRaw(151_880_415L),
            contact.Depth);
    }

    [Fact]
    public void Contact_RanksAxialAndRadialCandidates()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
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
        Assert.Equal(Fixed64.Half, contact.Depth);
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
            .TryGetCenteredFiniteCylindersContact(
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
                out bool usedWideCandidate,
                out _));

        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.True(contact.DepthIsClamped);
        Assert.False(usedWideCandidate);
    }

    [Fact]
    public void Contact_PreservesAnExactMaximumDepthWithoutClamping()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
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
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
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
    public void Contact_RotatedFullDomainGeometryMatchesTheScaledOracle()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.Pi / (Fixed64)12);
        Assert.True(FixedSegment
            .TryGetCenteredFiniteCylindersContact(
                new Vector3d(-100, 0, 0),
                rotation,
                Vector3d.Up,
                (Fixed64)100,
                (Fixed64)100,
                new Vector3d(100, 0, 0),
                rotation,
                Vector3d.Up,
                (Fixed64)100,
                (Fixed64)100,
                out FixedContactAnchors scaled));
        Assert.True(FixedSegment
            .TryGetCenteredFiniteCylindersContact(
                new Vector3d(
                    Fixed64.MinValue + Fixed64.One,
                    Fixed64.Zero,
                    Fixed64.Zero),
                rotation,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                rotation,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                out FixedContactAnchors fullDomain));

        Assert.True(scaled.Normal.IsNormalized());
        Assert.True(fullDomain.Normal.IsNormalized());
        Assert.Equal(scaled.Normal, fullDomain.Normal);
        Assert.True(scaled.Depth > Fixed64.Zero);
        Assert.True(fullDomain.Depth > Fixed64.Zero);
        Assert.False(scaled.DepthIsClamped);
        Assert.False(fullDomain.DepthIsClamped);
        Assert.True(fullDomain.SecondAnchor.TryGetProjectedOffsetFrom(
            fullDomain.FirstAnchor,
            fullDomain.Normal,
            out Fixed64 projectedSeparation));
        Assert.InRange(
            (projectedSeparation + fullDomain.Depth).Abs().m_rawValue,
            0L,
            Fixed64.One.m_rawValue);
    }

    [Fact]
    public void Contact_IrreducibleFullDomainCandidateUsesWideFallback()
    {
        Assert.True(TryGetIrreducibleFullDomainContact(
            reverse: false,
            out FixedContactAnchors contact,
            out bool usedWide));
        Assert.True(TryGetIrreducibleFullDomainContact(
            reverse: true,
            out FixedContactAnchors reversed,
            out bool reversedWide));
        Assert.True(usedWide);
        Assert.True(reversedWide);
        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(contact.Depth, reversed.Depth);
        Assert.Equal(-contact.Normal, reversed.Normal);
        Assert.True(contact.FirstAnchor == reversed.SecondAnchor);
        Assert.True(contact.SecondAnchor == reversed.FirstAnchor);
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

        for (int iteration = 0; iteration < 8; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetIrreducibleFullDomainContact(
                reverse: false,
                out FixedContactAnchors actual,
                out bool actualWide));
            Assert.True(actualWide);
            Assert.Equal(contact.Normal, actual.Normal);
            Assert.Equal(contact.Depth, actual.Depth);
        }

        for (int iteration = 0; iteration < 4; iteration++)
            _ = TryGetIrreducibleFullDomainContact(false, out _, out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetIrreducibleFullDomainContact(
                    reverse: false,
                    out _,
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
    public void Contact_SwappingArgumentsPreservesTheChosenTopology()
    {
        Vector3d secondCenter = new(
            Fixed64.Half,
            Fixed64.One + Fixed64.Half,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out FixedContactAnchors forward));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out FixedContactAnchors reverse));

        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.Equal(forward.Depth, reverse.Depth);
        Assert.Equal(forward.DepthIsClamped, reverse.DepthIsClamped);
        Assert.Equal(forward.FirstAnchor.LocalPoint, reverse.SecondAnchor.LocalPoint);
        Assert.Equal(forward.SecondAnchor.LocalPoint, reverse.FirstAnchor.LocalPoint);
    }

    [Fact]
    public void Contact_PreservesScalarFaceTangency()
    {
        Vector3d firstCenter = new(
            Fixed64.MaxValue - Fixed64.Two,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d secondCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            firstCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                firstCenter,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                secondCenter,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                out _,
                out bool usedWide,
                out _));
        Assert.False(usedWide);

        Assert.False(FixedSegment.TryGetCenteredFiniteCylindersContact(
            firstCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One - Fixed64.FromRaw(1),
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void Contact_RejectsSeparationOnTheSecondAndCrossAxes()
    {
        Assert.False(FixedSegment.TryGetCenteredFiniteCylindersContact(
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

        Assert.False(FixedSegment.TryGetCenteredFiniteCylindersContact(
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
    public void Contact_ScalarFaceTangencyIsDirtyStackDeterministicAndAllocationFree()
    {
        Assert.True(TryGetWideFallbackContact(
            out FixedContactAnchors expected));
        for (int iteration = 0; iteration < 8; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetWideFallbackContact(
                out FixedContactAnchors actual));
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.Equal(expected.Depth, actual.Depth);
            Assert.Equal(
                expected.DepthIsClamped,
                actual.DepthIsClamped);
        }

        _ = TryGetWideFallbackContact(out _);
        long before =
            GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetWideFallbackContact(out _))
                contacts++;
        }
        long after =
            GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(8, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Contact_WarmedOrdinaryAndIrreducibleMultiRadicalPathsDoNotAllocate()
    {
        for (int iteration = 0; iteration < 32; iteration++)
        {
            Assert.True(TryGetOrdinaryContact(
                out _,
                out bool ordinaryWide,
                out bool ordinaryMulti));
            Assert.False(ordinaryWide);
            Assert.False(ordinaryMulti);
            Assert.True(TryGetIrreducibleMultiRadicalContact(
                out _,
                out bool irreducibleWide,
                out bool irreducibleMulti));
            Assert.False(irreducibleWide);
            Assert.True(irreducibleMulti);
        }

        long ordinaryBefore = GC.GetAllocatedBytesForCurrentThread();
        int ordinaryContacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetOrdinaryContact(out _, out _, out _))
                ordinaryContacts++;
        }
        long ordinaryAfter = GC.GetAllocatedBytesForCurrentThread();

        long multiBefore = GC.GetAllocatedBytesForCurrentThread();
        int multiContacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetIrreducibleMultiRadicalContact(
                    out _,
                    out _,
                    out _))
                multiContacts++;
        }
        long multiAfter = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(8, ordinaryContacts);
        Assert.Equal(8, multiContacts);
        Assert.Equal(
            0,
            ordinaryAfter - ordinaryBefore);
        Assert.Equal(
            0,
            multiAfter - multiBefore);
    }

    [Fact]
    public void Contact_RejectsInvalidRigidFrame()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero,
                default,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                out _));
    }

    [Fact]
    public void Contact_LowerEndpointCandidateIsSymmetricAndMatchesTheRigidFrameOracle()
    {
        FixedQuaternion firstRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        Vector3d secondCenter =
            new(
                Fixed64.One + Fixed64.Half,
                -Fixed64.Half,
                Fixed64.Two + Fixed64.Half);
        Vector3d firstLower =
            -(firstRotation * Vector3d.Right);
        Vector3d secondLower =
            secondCenter - (Vector3d.Right * Fixed64.Two);
        Vector3d separation = secondLower - firstLower;

        Assert.True(TryGetLowerEndpointContact(
            Vector3d.Zero,
            firstRotation,
            Fixed64.Two,
            Fixed64.Zero,
            secondCenter,
            FixedQuaternion.Identity,
            (Fixed64)4,
            Fixed64.Two,
            out FixedContactAnchors forward));
        Assert.True(TryGetLowerEndpointContact(
            secondCenter,
            FixedQuaternion.Identity,
            (Fixed64)4,
            Fixed64.Two,
            Vector3d.Zero,
            firstRotation,
            Fixed64.Two,
            Fixed64.Zero,
            out FixedContactAnchors reverse));

        AssertVectorWithinOneRawUnit(
            separation.Normalized,
            forward.Normal);
        Assert.True(forward.Depth > Fixed64.Zero);
        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.Equal(forward.Depth, reverse.Depth);
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
            Fixed64.Two,
            Fixed64.Zero,
            center,
            FixedQuaternion.Identity,
            (Fixed64)4,
            Fixed64.Two,
            out FixedContactAnchors expected));

        for (int iteration = 0; iteration < 16; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetLowerEndpointContact(
                Vector3d.Zero,
                rotation,
                Fixed64.Two,
                Fixed64.Zero,
                center,
                FixedQuaternion.Identity,
                (Fixed64)4,
                Fixed64.Two,
                out FixedContactAnchors actual));
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.Equal(expected.Depth, actual.Depth);
        }

        _ = TryGetLowerEndpointContact(
            Vector3d.Zero,
            rotation,
            Fixed64.Two,
            Fixed64.Zero,
            center,
            FixedQuaternion.Identity,
            (Fixed64)4,
            Fixed64.Two,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 16; iteration++)
        {
            if (TryGetLowerEndpointContact(
                    Vector3d.Zero,
                    rotation,
                    Fixed64.Two,
                    Fixed64.Zero,
                    center,
                    FixedQuaternion.Identity,
                    (Fixed64)4,
                    Fixed64.Two,
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
    public void Contact_IntersectingCenterlinesChooseAStableNonzeroAxis()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
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
            out FixedContactAnchors contact));

        Assert.True(contact.Normal.IsNormalized());
        Assert.True(contact.Depth > Fixed64.Zero);
    }

    [Fact]
    public void Contact_ConcentricCoaxialOrderingIsDeterministicAndAllocationFree()
    {
        Assert.True(TryGetConcentricCoaxialContact(
            reverse: false,
            out FixedContactAnchors expected));
        Assert.True(TryGetConcentricCoaxialContact(
            reverse: true,
            out FixedContactAnchors reversed));
        Assert.True(expected.Normal.IsNormalized());
        Assert.True(reversed.Normal.IsNormalized());
        Assert.Equal(expected.Depth, reversed.Depth);
        Assert.Equal(
            expected.DepthIsClamped,
            reversed.DepthIsClamped);

        for (int iteration = 0; iteration < 8; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(TryGetConcentricCoaxialContact(
                reverse: false,
                out FixedContactAnchors actual));
            Assert.Equal(expected.Normal, actual.Normal);
            Assert.Equal(expected.Depth, actual.Depth);
        }

        for (int iteration = 0; iteration < 4; iteration++)
            _ = TryGetConcentricCoaxialContact(false, out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int contacts = 0;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (TryGetConcentricCoaxialContact(false, out _))
                contacts++;
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(8, contacts);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Contact_RawScaleGeometryIsSymmetric()
    {
        Vector3d secondCenter =
            new(
                Fixed64.FromRaw(2),
                Fixed64.FromRaw(-1),
                Fixed64.FromRaw(3));
        Fixed64 firstLength = Fixed64.FromRaw(3);
        Fixed64 secondLength = Fixed64.FromRaw(5);
        Fixed64 firstRadius = Fixed64.FromRaw(7);
        Fixed64 secondRadius = Fixed64.FromRaw(11);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right,
            firstLength,
            firstRadius,
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            secondLength,
            secondRadius,
            out FixedContactAnchors forward));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            secondCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            secondLength,
            secondRadius,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Right,
            firstLength,
            firstRadius,
            out FixedContactAnchors reverse));

        Assert.Equal(forward.Depth, reverse.Depth);
        Assert.Equal(-forward.Normal, reverse.Normal);
    }

    [Fact]
    public void Contact_TopologyMatrixIsSymmetric()
    {
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
            FixedQuaternion firstRotation =
                rotations[index % rotations.Length];
            FixedQuaternion secondRotation =
                rotations[(index / rotations.Length) % rotations.Length];
            Vector3d firstAxis =
                axes[(index / 7) % axes.Length];
            Vector3d secondAxis =
                axes[(index / 11) % axes.Length];
            Vector3d secondCenter =
                centers[(index * 5) % centers.Length];
            Fixed64 firstLength =
                lengths[(index * 3) % lengths.Length];
            Fixed64 secondLength =
                lengths[(index * 7 + 1) % lengths.Length];
            Fixed64 firstRadius =
                radii[(index * 5 + 1) % radii.Length];
            Fixed64 secondRadius =
                radii[(index * 7 + 2) % radii.Length];

            bool forwardHit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylindersContact(
                    Vector3d.Zero,
                    firstRotation,
                    firstAxis,
                    firstLength,
                    firstRadius,
                    secondCenter,
                    secondRotation,
                    secondAxis,
                    secondLength,
                    secondRadius,
                    out FixedContactAnchors forward,
                    out bool forwardUsedWide,
                    out bool forwardUsedMulti);
            bool reverseHit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylindersContact(
                    secondCenter,
                    secondRotation,
                    secondAxis,
                    secondLength,
                    secondRadius,
                    Vector3d.Zero,
                    firstRotation,
                    firstAxis,
                    firstLength,
                    firstRadius,
                    out FixedContactAnchors reverse,
                    out bool reverseUsedWide,
                    out bool reverseUsedMulti);

            Assert.Equal(forwardHit, reverseHit);
            if (!forwardHit)
                continue;
            Assert.True(forward.Normal.IsNormalized());
            Assert.True(forward.Depth >= Fixed64.Zero);
            Assert.True(
                forward.Depth == reverse.Depth,
                $"Topology index {index}: forward depth {forward.Depth}, reverse depth {reverse.Depth}; normals {forward.Normal}/{reverse.Normal}; wide {forwardUsedWide}/{reverseUsedWide}; multi {forwardUsedMulti}/{reverseUsedMulti}.");
            Assert.Equal(
                forward.DepthIsClamped,
                reverse.DepthIsClamped);
        }
    }

    [Fact]
    public void Contact_DeterministicFractionalDomainSampleIsSymmetric()
    {
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
        ulong state = 0xA076_1D64_78BD_642FUL;
        int contacts = 0;
        for (int index = 0; index < 64; index++)
        {
            Vector3d secondCenter = new(
                NextSignedRaw(ref state, 2L << 32),
                NextSignedRaw(ref state, 2L << 32),
                NextSignedRaw(ref state, 2L << 32));
            FixedQuaternion firstRotation =
                rotations[NextIndex(ref state, rotations.Length)];
            FixedQuaternion secondRotation =
                rotations[NextIndex(ref state, rotations.Length)];
            Vector3d firstAxis =
                axes[NextIndex(ref state, axes.Length)];
            Vector3d secondAxis =
                axes[NextIndex(ref state, axes.Length)];
            Fixed64 firstLength =
                NextPositiveRaw(ref state, 4L << 32);
            Fixed64 secondLength =
                NextPositiveRaw(ref state, 4L << 32);
            Fixed64 firstRadius =
                NextNonNegativeRaw(ref state, 3L << 32);
            Fixed64 secondRadius =
                NextNonNegativeRaw(ref state, 3L << 32);

            bool forwardHit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylindersContact(
                    Vector3d.Zero,
                    firstRotation,
                    firstAxis,
                    firstLength,
                    firstRadius,
                    secondCenter,
                    secondRotation,
                    secondAxis,
                    secondLength,
                    secondRadius,
                    out FixedContactAnchors forward,
                    out bool forwardWide,
                    out bool forwardMulti);
            bool reverseHit = WideConvexPrismRelations
                .TryGetCenteredFiniteCylindersContact(
                    secondCenter,
                    secondRotation,
                    secondAxis,
                    secondLength,
                    secondRadius,
                    Vector3d.Zero,
                    firstRotation,
                    firstAxis,
                    firstLength,
                    firstRadius,
                    out FixedContactAnchors reverse,
                    out bool reverseWide,
                    out bool reverseMulti);

            Assert.Equal(forwardHit, reverseHit);
            if (!forwardHit)
                continue;
            contacts++;
            Assert.True(
                forward.Depth == reverse.Depth,
                $"Sample {index}: depths {forward.Depth}/{reverse.Depth}; normals {forward.Normal}/{reverse.Normal}; wide {forwardWide}/{reverseWide}; multi {forwardMulti}/{reverseMulti}; center raw {secondCenter.X.m_rawValue},{secondCenter.Y.m_rawValue},{secondCenter.Z.m_rawValue}; rotations {firstRotation}/{secondRotation}; axes {firstAxis}/{secondAxis}; lengths raw {firstLength.m_rawValue}/{secondLength.m_rawValue}; radii raw {firstRadius.m_rawValue}/{secondRadius.m_rawValue}.");
            Assert.Equal(
                forward.DepthIsClamped,
                reverse.DepthIsClamped);
            Assert.Equal(-forward.Normal, reverse.Normal);
            Assert.True(
                forward.FirstAnchor == reverse.SecondAnchor);
            Assert.True(
                forward.SecondAnchor == reverse.FirstAnchor);
        }

        Assert.True(contacts > 32);
    }

    [Fact]
    public void Contact_FractionalCandidateRankingIsArgumentOrderInvariant()
    {
        Vector3d secondCenter = new(
            Fixed64.FromRaw(-5_948_400_167L),
            Fixed64.FromRaw(-8_199_609_096L),
            Fixed64.FromRaw(-2_152_663_550L));
        FixedQuaternion secondRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.Pi / (Fixed64)4);
        Fixed64 firstLength =
            Fixed64.FromRaw(10_531_964_516L);
        Fixed64 firstRadius =
            Fixed64.FromRaw(10_373_054_786L);
        Fixed64 secondLength =
            Fixed64.FromRaw(13_250_303_561L);
        Fixed64 secondRadius =
            Fixed64.FromRaw(12_414_260_817L);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Forward,
            firstLength,
            firstRadius,
            secondCenter,
            secondRotation,
            Vector3d.Up,
            secondLength,
            secondRadius,
            out FixedContactAnchors forward));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            secondCenter,
            secondRotation,
            Vector3d.Up,
            secondLength,
            secondRadius,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Forward,
            firstLength,
            firstRadius,
            out FixedContactAnchors reverse));

        Assert.Equal(forward.Depth, reverse.Depth);
        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.Equal(
            forward.FirstAnchor.LocalPoint,
            reverse.SecondAnchor.LocalPoint);
        Assert.Equal(
            forward.SecondAnchor.LocalPoint,
            reverse.FirstAnchor.LocalPoint);
        Assert.True(forward.FirstAnchor == reverse.SecondAnchor);
        Assert.True(forward.SecondAnchor == reverse.FirstAnchor);
    }

    private static bool TryGetWideFallbackContact(
        out FixedContactAnchors contact)
    {
        return FixedSegment
            .TryGetCenteredFiniteCylindersContact(
                new Vector3d(
                    Fixed64.MaxValue - Fixed64.Two,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                out contact);
    }

    private static bool TryGetLowerEndpointContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out FixedContactAnchors contact) =>
        FixedSegment.TryGetCenteredFiniteCylindersContact(
            firstCenter,
            firstRotation,
            Vector3d.Right,
            firstLength,
            firstRadius,
            secondCenter,
            secondRotation,
            Vector3d.Right,
            secondLength,
            secondRadius,
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
        return FixedSegment.TryGetCenteredFiniteCylindersContact(
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

    private static Fixed64 NextSignedRaw(
        ref ulong state,
        long halfRange)
    {
        ulong range = unchecked((ulong)(halfRange + halfRange));
        return Fixed64.FromRaw(
            unchecked((long)(NextUInt64(ref state) % range))
            - halfRange);
    }

    private static Fixed64 NextPositiveRaw(
        ref ulong state,
        long maximum) =>
        Fixed64.FromRaw(
            1L
            + unchecked((long)(
                NextUInt64(ref state)
                % unchecked((ulong)maximum))));

    private static Fixed64 NextNonNegativeRaw(
        ref ulong state,
        long maximum) =>
        Fixed64.FromRaw(
            unchecked((long)(
                NextUInt64(ref state)
                % unchecked((ulong)maximum))));

    private static int NextIndex(
        ref ulong state,
        int length) =>
        unchecked((int)(NextUInt64(ref state) % (uint)length));

    private static ulong NextUInt64(ref ulong state)
    {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        return state * 2685821657736338717UL;
    }

    private static bool TryGetOrdinaryContact(
        out FixedContactAnchors contact,
        out bool usedWide,
        out bool usedMulti)
    {
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
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
                out contact,
                out usedWide,
                out usedMulti);
    }

    private static bool TryGetIrreducibleFullDomainContact(
        bool reverse,
        out FixedContactAnchors contact,
        out bool usedWide)
    {
        Vector3d firstCenter = new(
            Fixed64.FromRaw(-9_223_371_836_141_996_263L),
            Fixed64.FromRaw(-9_223_371_831_503_237_588L),
            Fixed64.FromRaw(-9_223_371_589_877_671_470L));
        Vector3d secondCenter = new(
            Fixed64.FromRaw(-9_223_371_392_634_327_554L),
            Fixed64.FromRaw(-9_223_371_627_891_749_603L),
            Fixed64.FromRaw(-9_223_371_800_738_914_912L));
        FixedQuaternion firstRotation = FixedQuaternion.Identity;
        FixedQuaternion secondRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.Pi / (Fixed64)3);
        Fixed64 firstLength =
            Fixed64.FromRaw(4_611_686_769_793_122_804L);
        Fixed64 secondLength =
            Fixed64.FromRaw(9_223_371_489_993_066_109L);
        Fixed64 firstRadius =
            Fixed64.FromRaw(1_152_922_270_418_231_838L);
        Fixed64 secondRadius =
            Fixed64.FromRaw(1_152_922_104_737_467_605L);
        if (reverse)
        {
            (firstCenter, secondCenter) =
                (secondCenter, firstCenter);
            (firstRotation, secondRotation) =
                (secondRotation, firstRotation);
            (firstLength, secondLength) =
                (secondLength, firstLength);
            (firstRadius, secondRadius) =
                (secondRadius, firstRadius);
        }

        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                firstCenter,
                firstRotation,
                Vector3d.Forward,
                firstLength,
                firstRadius,
                secondCenter,
                secondRotation,
                Vector3d.Forward,
                secondLength,
                secondRadius,
                out contact,
                out usedWide,
                out _);
    }

    private static bool TryGetFormerWideFixture(
        bool reduceTranslation,
        out FixedContactAnchors contact,
        out bool usedWide)
    {
        Vector3d firstCenter = reduceTranslation
            ? Vector3d.Zero
            : new Vector3d(
                Fixed64.MaxValue - (Fixed64)8,
                Fixed64.MinValue + (Fixed64)8,
                Fixed64.MaxValue - (Fixed64)16);
        Vector3d secondCenter = reduceTranslation
            ? new Vector3d(5, 5, 11)
            : new Vector3d(
                Fixed64.MaxValue - (Fixed64)3,
                Fixed64.MinValue + (Fixed64)13,
                Fixed64.MaxValue - (Fixed64)5);
        var firstRotation = new FixedQuaternion(
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(-2_382_419_202L),
            Fixed64.FromRaw(1_191_209_601L));
        FixedQuaternion secondRotation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.Pi / (Fixed64)3);
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                firstCenter,
                firstRotation,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue / (Fixed64)8,
                secondCenter,
                secondRotation,
                Vector3d.Up,
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.MaxValue / (Fixed64)8,
                out contact,
                out usedWide,
                out _);
    }

    private static bool TryGetConcentricCoaxialContact(
        bool reverse,
        out FixedContactAnchors contact)
    {
        Fixed64 firstLength = reverse
            ? Fixed64.Two
            : Fixed64.One;
        Fixed64 firstRadius = reverse
            ? Fixed64.One
            : Fixed64.Half;
        Fixed64 secondLength = reverse
            ? Fixed64.One
            : Fixed64.Two;
        Fixed64 secondRadius = reverse
            ? Fixed64.Half
            : Fixed64.One;
        return FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            firstLength,
            firstRadius,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            secondLength,
            secondRadius,
            out contact);
    }

    private static bool TryGetIrreducibleMultiRadicalContact(
        out FixedContactAnchors contact,
        out bool usedWide,
        out bool usedMulti)
    {
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero,
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Forward,
                    Fixed64.PiOver6),
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                new Vector3d(-2, -2, -1),
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Right,
                    Fixed64.PiOver6),
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                out contact,
                out usedWide,
                out usedMulti);
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
        GC.KeepAlive(words[
            unchecked((int)seed) & 2_047]);
    }
}
