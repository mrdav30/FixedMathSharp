using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedLeverNormalResponseTests
{
    [Fact]
    public void NormalResponse_ShouldMatchRepresentableLinearImpulse()
    {
        FixedLever firstLever = CreateLever(Vector3d.Up, Vector3d.Zero);
        FixedLever secondLever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var first = new FixedLeverResponseOperand3d(
            firstLever,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Zero);
        var second = new FixedLeverResponseOperand3d(
            secondLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed3x3.Zero);

        bool resolved = FixedLever.TryGetNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d response);

        Assert.True(resolved);
        Assert.True(response.IsClosing);
        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetNormalVelocity(out Fixed64 normalVelocity));
        Assert.Equal(-Fixed64.One, normalVelocity);
        Assert.True(response.TryGetAppliedImpulse(out Fixed64 impulse));
        Assert.Equal(Fixed64.Half, impulse);
        Assert.False(response.TryGetAccumulatedImpulse(out _));
        Assert.Equal(Vector3d.Left * Fixed64.Half, response.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Zero, response.FirstAngularVelocityDelta);
        Assert.Equal(Vector3d.Right * Fixed64.Half, response.SecondLinearVelocityDelta);
        Assert.Equal(Vector3d.Zero, response.SecondAngularVelocityDelta);
    }

    [Fact]
    public void NormalResponse_ShouldRetainUnrepresentablePointSpeedAndEffectiveMass()
    {
        FixedLever firstLever = CreateFullDomainLever();
        FixedLever secondLever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var first = new FixedLeverResponseOperand3d(
            firstLever,
            Vector3d.Zero,
            -Vector3d.Forward * Fixed64.Two,
            Vector3d.Zero,
            Fixed64.One,
            Fixed3x3.Identity);
        var second = new FixedLeverResponseOperand3d(
            secondLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);

        bool resolved = FixedLever.TryGetNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d response);

        Assert.True(resolved);
        Assert.True(response.IsClosing);
        Assert.False(response.TryGetNormalVelocity(out _));
        Assert.Equal(Vector3d.Forward * Fixed64.Two, response.FirstAngularVelocityDelta);
        Assert.Equal(Vector3d.Zero, response.SecondAngularVelocityDelta);
    }

    [Fact]
    public void NormalResponse_ShouldApplyAngularDeltaWhenImpulseRoundsToZero()
    {
        FixedLever firstLever = CreateFullDomainLever();
        FixedLever secondLever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        Fixed64 maximum = Fixed64.MaxValue;
        var maximumInertia = new Fixed3x3(
            maximum, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, maximum, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, maximum);
        var first = new FixedLeverResponseOperand3d(
            firstLever,
            Vector3d.Zero,
            -Vector3d.Forward,
            Vector3d.Zero,
            Fixed64.Zero,
            maximumInertia);
        var second = new FixedLeverResponseOperand3d(
            secondLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);

        bool resolved = FixedLever.TryGetNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d response);

        Assert.True(resolved);
        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetAppliedImpulse(out Fixed64 impulse));
        Assert.Equal(Fixed64.Zero, impulse);
        Assert.Equal(Vector3d.Forward, response.FirstAngularVelocityDelta);
    }

    [Fact]
    public void NormalResponse_ShouldPermitUnrepresentableSharedImpulseWhenFinalDeltasFit()
    {
        FixedLever lever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var first = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Right * (Fixed64)6,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.MinIncrement * Fixed64.Two,
            Fixed3x3.Zero);
        var second = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.MinIncrement * Fixed64.Two,
            Fixed3x3.Zero);

        bool resolved = FixedLever.TryGetNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d response);

        Assert.True(resolved);
        Assert.False(response.TryGetAppliedImpulse(out _));
        Assert.Equal(
            Vector3d.Left * (Fixed64)3,
            response.FirstLinearVelocityDelta);
        Assert.Equal(
            Vector3d.Right * (Fixed64)3,
            response.SecondLinearVelocityDelta);

        Assert.True(FixedLever.TryGetAccumulatedNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverNormalResponse3d accumulated));
        Assert.True(accumulated.IsClosing);
        Assert.Equal(Vector3d.Left * (Fixed64)3, accumulated.FirstLinearVelocityDelta);
        Assert.False(accumulated.TryGetAppliedImpulse(out _));
        Assert.False(accumulated.TryGetAccumulatedImpulse(out _));
    }

    [Fact]
    public void AccumulatedNormalResponse_ShouldClampSeparatingImpulseAndCache()
    {
        FixedLever lever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var first = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Left,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Zero);
        var second = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed3x3.Zero);

        Assert.True(FixedLever.TryGetAccumulatedNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverNormalResponse3d response));

        Assert.False(response.IsClosing);
        Assert.True(response.TryGetAppliedImpulse(out Fixed64 impulse));
        Assert.Equal(-Fixed64.Half, impulse);
        Assert.True(response.TryGetAccumulatedImpulse(out Fixed64 accumulated));
        Assert.Equal(Fixed64.Zero, accumulated);
        Assert.Equal(Vector3d.Right * Fixed64.Half, response.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Left * Fixed64.Half, response.SecondLinearVelocityDelta);
    }

    [Fact]
    public void NormalResponse_ShouldHandleRestingAndSeparatingMovableContacts()
    {
        FixedLever lever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var resting = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Zero);
        var second = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed3x3.Zero);

        Assert.True(FixedLever.TryGetNormalResponse(
            resting,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d restingResponse));
        Assert.False(restingResponse.IsClosing);
        Assert.False(restingResponse.HasAppliedImpulse);

        var separating = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Left,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Zero);
        Assert.True(FixedLever.TryGetNormalResponse(
            separating,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d separatingResponse));
        Assert.False(separatingResponse.HasAppliedImpulse);
        Assert.Equal(Vector3d.Zero, separatingResponse.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Zero, separatingResponse.SecondLinearVelocityDelta);
    }

    [Fact]
    public void AccumulatedNormalResponse_ShouldReleaseOnlyTheRequiredCachedImpulse()
    {
        FixedLever lever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var first = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Left,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Zero);
        var second = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed3x3.Zero);

        Assert.True(FixedLever.TryGetAccumulatedNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverNormalResponse3d response));

        Assert.True(response.TryGetAppliedImpulse(out Fixed64 impulse));
        Assert.Equal(-Fixed64.Half, impulse);
        Assert.True(response.TryGetAccumulatedImpulse(out Fixed64 accumulated));
        Assert.Equal(Fixed64.Half, accumulated);

        Assert.True(FixedLever.TryGetAccumulatedNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverNormalResponse3d emptyCacheResponse));
        Assert.False(emptyCacheResponse.HasAppliedImpulse);
        Assert.True(emptyCacheResponse.TryGetAccumulatedImpulse(out Fixed64 emptyCache));
        Assert.Equal(Fixed64.Zero, emptyCache);
    }

    [Fact]
    public void NormalResponse_ShouldRejectUnrepresentableFinalAngularVelocity()
    {
        FixedLever lever = CreateLever(Vector3d.Up, Vector3d.Zero);
        var first = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            -Vector3d.Forward * Fixed64.MaxValue,
            Vector3d.Left,
            Fixed64.Zero,
            Fixed3x3.Identity);
        var second = new FixedLeverResponseOperand3d(
            CreateLever(Vector3d.Zero, Vector3d.Zero),
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed3x3.Zero);

        Assert.False(FixedLever.TryGetNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.One,
            Fixed64.Zero,
            out _));
    }

    [Fact]
    public void NormalResponse_ShouldRejectOnlyClosingImmobileOrUnrepresentableFinalStates()
    {
        FixedLever lever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var closing = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        var separating = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Left,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        var immobile = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);

        Assert.False(FixedLever.TryGetNormalResponse(
            closing,
            immobile,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out _));
        Assert.True(FixedLever.TryGetNormalResponse(
            separating,
            immobile,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverNormalResponse3d separated));
        Assert.False(separated.HasAppliedImpulse);

        var overflowing = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.MaxValue,
            Fixed3x3.Zero);
        Assert.False(FixedLever.TryGetNormalResponse(
            overflowing,
            immobile,
            Vector3d.Right,
            Fixed64.MaxValue,
            Fixed64.Zero,
            out _));
    }

    [Fact]
    public void NormalResponse_ShouldValidateSemanticInputs()
    {
        FixedLever lever = CreateLever(Vector3d.Zero, Vector3d.Zero);
        var valid = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        var invalidLever = new FixedLeverResponseOperand3d(
            default,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        var invalidMass = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            -Fixed64.One,
            Fixed3x3.Zero);

        Assert.False(FixedLever.TryGetNormalResponse(
            invalidLever, valid, Vector3d.Right, Fixed64.Zero, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            valid, invalidLever, Vector3d.Right, Fixed64.Zero, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            invalidMass, valid, Vector3d.Right, Fixed64.Zero, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            valid, invalidMass, Vector3d.Right, Fixed64.Zero, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            valid, valid, Vector3d.Zero, Fixed64.Zero, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            valid, valid, Vector3d.Right * Fixed64.Two, Fixed64.Zero, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            valid, valid, Vector3d.Right, -Fixed64.One, Fixed64.Zero, out _));
        Assert.False(FixedLever.TryGetNormalResponse(
            valid, valid, Vector3d.Right, Fixed64.Zero, -Fixed64.One, out _));
        Assert.False(FixedLever.TryGetAccumulatedNormalResponse(
            valid,
            valid,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            -Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetAccumulatedNormalResponse(
            valid,
            valid,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            -Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetAccumulatedNormalResponse(
            valid,
            valid,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            -Fixed64.One,
            out _));
    }

    [Fact]
    public void NormalResponse_ShouldNotAllocateAfterWarmup()
    {
        FixedLever lever = CreateFullDomainLever();
        var first = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            -Vector3d.Forward,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Identity);
        var second = new FixedLeverResponseOperand3d(
            CreateLever(Vector3d.Zero, Vector3d.Zero),
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        Assert.True(FixedLever.TryGetNormalResponse(
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out _));
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 16; iteration++)
        {
            Assert.True(FixedLever.TryGetNormalResponse(
                first,
                second,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _));
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }

    private static FixedLever CreateFullDomainLever() =>
        CreateLever(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero));

    private static FixedLever CreateLever(Vector3d point, Vector3d center)
    {
        var pointAnchor = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var centerAnchor = new FixedPointAnchor(
            center,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Assert.True(pointAnchor.TryGetLeverFrom(centerAnchor, out FixedLever lever));
        return lever;
    }
}
