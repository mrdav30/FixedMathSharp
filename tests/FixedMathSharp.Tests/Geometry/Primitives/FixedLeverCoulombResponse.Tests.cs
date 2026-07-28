using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedLeverCoulombResponseTests
{
    [Fact]
    public void CoulombLineResponse_ShouldRetainUnrepresentableNormalAndTangentImpulses()
    {
        FixedLever lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d firstVelocity =
            (Vector3d.Right * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)4);
        var normalFirst = new FixedLeverResponseOperand3d(
            lever,
            firstVelocity,
            Vector3d.Zero,
            Vector3d.Left,
            inverseMass,
            Fixed3x3.Zero);
        var normalSecond = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass,
            Fixed3x3.Zero);
        var constraint = new FixedLeverNormalConstraint3d(
            normalFirst,
            normalSecond,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        var tangentFirst = new FixedLeverResponseOperand3d(
            lever,
            firstVelocity,
            Vector3d.Zero,
            -Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);
        var tangentSecond = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);

        bool resolved = FixedLever.TryGetCoulombLineResponse(
            constraint,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverCoulombResponse3d response);

        Assert.True(resolved);
        Assert.True(response.HasAppliedImpulse);
        Assert.Equal(
            -Vector3d.Forward * Fixed64.Two,
            response.FirstLinearVelocityDelta);
        Assert.Equal(
            Vector3d.Forward * Fixed64.Two,
            response.SecondLinearVelocityDelta);
        Assert.False(response.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.False(response.TryGetSecondaryAccumulatedImpulse(out _));
    }

    [Fact]
    public void CoulombLineResponse_ShouldClampWideDesiredImpulseToDynamicLimit()
    {
        FixedLever lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d firstVelocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)8);
        FixedLeverNormalConstraint3d constraint =
            CreateNormalConstraint(
                lever,
                inverseMass,
                Vector3d.Up,
                firstVelocity);
        var tangentFirst = new FixedLeverResponseOperand3d(
            lever,
            firstVelocity,
            Vector3d.Zero,
            -Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);
        var tangentSecond = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);

        Assert.True(FixedLever.TryGetCoulombLineResponse(
            constraint,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.One / (Fixed64)4,
            out FixedLeverCoulombResponse3d response));

        Assert.Equal(
            -Vector3d.Forward * (Fixed64)0.75m,
            response.FirstLinearVelocityDelta);
        Assert.Equal(
            Vector3d.Forward * (Fixed64)0.75m,
            response.SecondLinearVelocityDelta);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 accumulated));
        Assert.True(accumulated > Fixed64.Zero);
    }

    [Fact]
    public void CoulombLineResponse_ShouldApplyWarmAccumulatorChangesExactly()
    {
        FixedLever lever = CreateLever();
        FixedLeverNormalConstraint3d constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            (Vector3d.Up * (Fixed64)6) + Vector3d.Forward);
        FixedLeverResponseOperand3d first = CreateTangentOperand(
            lever,
            constraint.First.LinearVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        FixedLeverResponseOperand3d second = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        AssertAccumulatedImpulse(-Fixed64.Half, Fixed64.Zero);
        AssertAccumulatedImpulse(-(Fixed64)0.25m, Fixed64.One / (Fixed64)4);
        AssertAccumulatedImpulse(-Fixed64.One, -Fixed64.Half);
        AssertAccumulatedImpulse(Fixed64.One, (Fixed64)1.5m);

        void AssertAccumulatedImpulse(
            Fixed64 previous,
            Fixed64 expected)
        {
            Assert.True(FixedLever.TryGetCoulombLineResponse(
                constraint,
                first,
                second,
                Vector3d.Forward,
                previous,
                Fixed64.One,
                Fixed64.One,
                out FixedLeverCoulombResponse3d response));
            Assert.True(response.HasAppliedImpulse);
            Assert.True(response.TryGetPrimaryAccumulatedImpulse(
                out Fixed64 accumulated));
            Assert.Equal(expected, accumulated);
        }
    }

    [Fact]
    public void CoulombLineResponse_ShouldUseCompletedNormalLoad()
    {
        FixedLever lever = CreateLever();
        Vector3d tangentVelocity = Vector3d.Forward;
        FixedLeverResponseOperand3d tangentFirst = CreateTangentOperand(
            lever,
            tangentVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        FixedLeverResponseOperand3d tangentSecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        FixedLeverNormalConstraint3d resting = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            tangentVelocity,
            accumulatedImpulse: Fixed64.One);
        Assert.True(FixedLever.TryGetCoulombLineResponse(
            resting,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverCoulombResponse3d restingResponse));
        Assert.True(restingResponse.HasAppliedImpulse);

        FixedLeverNormalConstraint3d partiallyReleased =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                -Vector3d.Up + tangentVelocity,
                accumulatedImpulse: Fixed64.One);
        Assert.True(FixedLever.TryGetCoulombLineResponse(
            partiallyReleased,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverCoulombResponse3d partialResponse));
        Assert.True(partialResponse.HasAppliedImpulse);

        FixedLeverNormalConstraint3d fullyReleased =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                -Vector3d.Up + tangentVelocity,
                accumulatedImpulse: Fixed64.One / (Fixed64)4);
        Assert.True(FixedLever.TryGetCoulombLineResponse(
            fullyReleased,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverCoulombResponse3d releasedResponse));
        Assert.False(releasedResponse.HasAppliedImpulse);

        FixedLeverNormalConstraint3d belowRestitutionThreshold =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                (Vector3d.Up * Fixed64.MinIncrement) + tangentVelocity,
                restitution: Fixed64.One,
                restitutionVelocityThreshold: Fixed64.One);
        Assert.True(FixedLever.TryGetCoulombLineResponse(
            belowRestitutionThreshold,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CoulombLineResponse_ShouldClampAgainstSignedWarmCacheAndRejectOverflow()
    {
        FixedLever lever = CreateLever();
        Vector3d movingVelocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)8);
        FixedLeverNormalConstraint3d constraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                movingVelocity);
        FixedLeverResponseOperand3d first = CreateTangentOperand(
            lever,
            movingVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        FixedLeverResponseOperand3d second = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        AssertClamped(Fixed64.One);
        AssertClamped(-Fixed64.One);

        FixedLeverNormalConstraint3d overflowConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.Two,
                Vector3d.Up,
                Vector3d.Forward * Fixed64.MinIncrement,
                accumulatedImpulse: Fixed64.MaxValue);
        first = CreateTangentOperand(
            lever,
            overflowConstraint.First.LinearVelocity,
            -Vector3d.Forward,
            Fixed64.Two);
        second = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.Two);
        Assert.False(FixedLever.TryGetCoulombLineResponse(
            overflowConstraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out _));

        void AssertClamped(Fixed64 accumulated)
        {
            Assert.True(FixedLever.TryGetCoulombLineResponse(
                constraint,
                first,
                second,
                Vector3d.Forward,
                accumulated,
                Fixed64.Half,
                Fixed64.One / (Fixed64)4,
                out FixedLeverCoulombResponse3d response));
            Assert.True(response.TryGetPrimaryAccumulatedImpulse(
                out Fixed64 completed));
            Assert.Equal((Fixed64)0.75m, completed);
        }
    }

    [Fact]
    public void CoulombDiskResponse_ShouldClampUnrepresentableTangentsWithoutScalarProjection()
    {
        FixedLever lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d tangentVelocity =
            (Vector3d.Right + Vector3d.Forward) * (Fixed64)8;
        Vector3d firstVelocity =
            (Vector3d.Up * (Fixed64)6) + tangentVelocity;
        FixedLeverNormalConstraint3d constraint =
            CreateNormalConstraint(
                lever,
                inverseMass,
                Vector3d.Up,
                firstVelocity);
        FixedLeverResponseOperand3d primaryFirst = CreateTangentOperand(
            lever,
            firstVelocity,
            Vector3d.Left,
            inverseMass);
        FixedLeverResponseOperand3d primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass);
        FixedLeverResponseOperand3d secondaryFirst = CreateTangentOperand(
            lever,
            firstVelocity,
            -Vector3d.Forward,
            inverseMass);
        FixedLeverResponseOperand3d secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass);

        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Half,
            Fixed64.One / (Fixed64)4,
            out FixedLeverCoulombResponse3d response));

        Assert.True(response.HasAppliedImpulse);
        Assert.Equal(
            response.FirstLinearVelocityDelta.X,
            response.FirstLinearVelocityDelta.Z);
        Assert.True(response.FirstLinearVelocityDelta.X < Fixed64.Zero);
        Assert.Equal(
            -response.FirstLinearVelocityDelta,
            response.SecondLinearVelocityDelta);
        Fixed64 magnitude = response.FirstLinearVelocityDelta.Magnitude;
        Assert.True(magnitude >= (Fixed64)0.74999999m);
        Assert.True(magnitude <= (Fixed64)0.75000001m);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldHandleStaticRestingAndZeroDynamicLimits()
    {
        FixedLever lever = CreateLever();
        Vector3d movingVelocity =
            (Vector3d.Up * (Fixed64)6)
            + Vector3d.Right
            + Vector3d.Forward;
        FixedLeverNormalConstraint3d movingConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                movingVelocity);
        FixedLeverResponseOperand3d primaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            Vector3d.Left,
            Fixed64.One);
        FixedLeverResponseOperand3d primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        FixedLeverResponseOperand3d secondaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        FixedLeverResponseOperand3d secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            movingConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverCoulombResponse3d staticResponse));
        Assert.Equal(
            (Vector3d.Left - Vector3d.Forward) * Fixed64.Half,
            staticResponse.FirstLinearVelocityDelta);
        Assert.True(staticResponse.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.Equal(Fixed64.Half, primaryAccumulated);
        Assert.True(staticResponse.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal(Fixed64.Half, secondaryAccumulated);

        Vector3d restingVelocity = Vector3d.Up * (Fixed64)6;
        FixedLeverNormalConstraint3d restingConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                restingVelocity);
        FixedLeverResponseOperand3d restingFirst = CreateTangentOperand(
            lever,
            restingVelocity,
            Vector3d.Left,
            Fixed64.One);
        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            restingConstraint,
            restingFirst,
            primarySecond,
            Vector3d.Right,
            restingFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverCoulombResponse3d restingResponse));
        Assert.False(restingResponse.HasAppliedImpulse);
        Assert.Equal(Vector3d.Zero, restingResponse.FirstLinearVelocityDelta);

        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            movingConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedLeverCoulombResponse3d zeroDynamicResponse));
        Assert.False(zeroDynamicResponse.HasAppliedImpulse);
        Assert.Equal(Vector3d.Zero, zeroDynamicResponse.FirstLinearVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldCombineAngularTangentChanges()
    {
        FixedLever lever = CreateLever(Vector3d.Up);
        Vector3d staticVelocity =
            (Vector3d.Up * (Fixed64)6)
            + Vector3d.Right
            + Vector3d.Forward;
        FixedLeverNormalConstraint3d staticConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                staticVelocity,
                Fixed3x3.Identity);
        FixedLeverResponseOperand3d primaryFirst = CreateTangentOperand(
            lever,
            staticVelocity,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Identity);
        FixedLeverResponseOperand3d primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed3x3.Identity);
        FixedLeverResponseOperand3d secondaryFirst = CreateTangentOperand(
            lever,
            staticVelocity,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Identity);
        FixedLeverResponseOperand3d secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Identity);

        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            staticConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.One,
            Fixed64.One,
            out FixedLeverCoulombResponse3d staticResponse));
        Assert.NotEqual(Vector3d.Zero, staticResponse.FirstAngularVelocityDelta);
        Assert.Equal(
            -staticResponse.FirstAngularVelocityDelta,
            staticResponse.SecondAngularVelocityDelta);

        Vector3d dynamicVelocity =
            (Vector3d.Up * (Fixed64)6)
            + ((Vector3d.Right + Vector3d.Forward) * (Fixed64)8);
        FixedLeverNormalConstraint3d dynamicConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                dynamicVelocity,
                Fixed3x3.Identity);
        primaryFirst = CreateTangentOperand(
            lever,
            dynamicVelocity,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Identity);
        secondaryFirst = CreateTangentOperand(
            lever,
            dynamicVelocity,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Identity);
        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            dynamicConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out FixedLeverCoulombResponse3d dynamicResponse));
        Assert.NotEqual(Vector3d.Zero, dynamicResponse.FirstAngularVelocityDelta);
        Assert.Equal(
            -dynamicResponse.FirstAngularVelocityDelta,
            dynamicResponse.SecondAngularVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldResolveOneTangentAndRejectOverflow()
    {
        FixedLever lever = CreateLever();
        Vector3d movingVelocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Right * (Fixed64)8);
        FixedLeverNormalConstraint3d constraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                movingVelocity);
        FixedLeverResponseOperand3d primaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            Vector3d.Left,
            Fixed64.One);
        FixedLeverResponseOperand3d primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        FixedLeverResponseOperand3d secondaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        FixedLeverResponseOperand3d secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);
        Assert.True(FixedLever.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out FixedLeverCoulombResponse3d response));
        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal(Fixed64.Zero, secondaryAccumulated);

        FixedLeverNormalConstraint3d overflowConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.MaxValue,
                Vector3d.Up,
                Vector3d.Right,
                accumulatedImpulse: Fixed64.MaxValue);
        primaryFirst = CreateTangentOperand(
            lever,
            overflowConstraint.First.LinearVelocity,
            Vector3d.Left,
            Fixed64.MaxValue);
        primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.MaxValue);
        secondaryFirst = CreateTangentOperand(
            lever,
            overflowConstraint.First.LinearVelocity,
            -Vector3d.Forward,
            Fixed64.MaxValue);
        secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.MaxValue);
        Assert.False(FixedLever.TryGetCoulombDiskResponse(
            overflowConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out _));

        FixedLever angularLever = CreateLever(Vector3d.Up);
        var angularOverflowInertia = new Fixed3x3(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.MaxValue, Fixed64.Zero, Fixed64.MinIncrement);
        var angularNormalFirst = new FixedLeverResponseOperand3d(
            angularLever,
            Vector3d.Right,
            Vector3d.Zero,
            -Vector3d.Up,
            Fixed64.Zero,
            angularOverflowInertia);
        var angularNormalSecond = new FixedLeverResponseOperand3d(
            angularLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed3x3.Zero);
        var angularConstraint = new FixedLeverNormalConstraint3d(
            angularNormalFirst,
            angularNormalSecond,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.One);
        var angularPrimaryFirst = new FixedLeverResponseOperand3d(
            angularLever,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            angularOverflowInertia);
        var angularPrimarySecond = new FixedLeverResponseOperand3d(
            angularLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        Assert.False(FixedLever.TryGetCoulombDiskResponse(
            angularConstraint,
            angularPrimaryFirst,
            angularPrimarySecond,
            Vector3d.Right,
            angularPrimaryFirst,
            angularPrimarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CoulombResponses_ShouldValidateInputsAndRemainAllocationFree()
    {
        FixedLever lever = CreateLever();
        FixedLeverNormalConstraint3d constraint =
            CreateNormalConstraint(lever, Fixed64.One);
        var first = new FixedLeverResponseOperand3d(
            lever,
            constraint.First.LinearVelocity,
            Vector3d.Zero,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Zero);
        var second = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Zero);

        Assert.False(FixedLever.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            -Fixed64.One,
            out _));

        var mismatched = new FixedLeverResponseOperand3d(
            CreateLever(Vector3d.Up),
            first.LinearVelocity,
            first.AngularVelocity,
            first.LinearImpulseAxis,
            first.InverseMass,
            first.InverseInertia);
        Assert.False(FixedLever.TryGetCoulombLineResponse(
            constraint,
            mismatched,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        var invalidConstraint = new FixedLeverNormalConstraint3d(
            constraint.First,
            constraint.Second,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        Assert.False(FixedLever.TryGetCoulombLineResponse(
            invalidConstraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        Assert.False(FixedLever.TryGetCoulombDiskResponse(
            constraint,
            first,
            second,
            Vector3d.Forward,
            first,
            second,
            Vector3d.Forward,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetCoulombDiskResponse(
            constraint,
            mismatched,
            second,
            Vector3d.Forward,
            first,
            second,
            Vector3d.Right,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetCoulombDiskResponse(
            invalidConstraint,
            first,
            second,
            Vector3d.Forward,
            first,
            second,
            Vector3d.Right,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            -Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(FixedLever.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        Assert.True(FixedLever.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 8; iteration++)
        {
            Assert.True(FixedLever.TryGetCoulombLineResponse(
                constraint,
                first,
                second,
                Vector3d.Forward,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _));
        }
        Assert.Equal(before, System.GC.GetAllocatedBytesForCurrentThread());
    }

    private static FixedLeverNormalConstraint3d CreateNormalConstraint(
        FixedLever lever,
        Fixed64 inverseMass,
        Vector3d? normal = null,
        Vector3d? firstVelocity = null,
        Fixed3x3? inverseInertia = null,
        Fixed64? restitution = null,
        Fixed64? restitutionVelocityThreshold = null,
        Fixed64? accumulatedImpulse = null)
    {
        Vector3d resolvedNormal = normal ?? Vector3d.Right;
        Fixed3x3 resolvedInverseInertia = inverseInertia ?? Fixed3x3.Zero;
        var first = new FixedLeverResponseOperand3d(
            lever,
            firstVelocity ?? resolvedNormal * (Fixed64)6,
            Vector3d.Zero,
            -resolvedNormal,
            inverseMass,
            resolvedInverseInertia);
        var second = new FixedLeverResponseOperand3d(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            resolvedNormal,
            inverseMass,
            resolvedInverseInertia);
        return new FixedLeverNormalConstraint3d(
            first,
            second,
            resolvedNormal,
            restitution ?? Fixed64.Zero,
            restitutionVelocityThreshold ?? Fixed64.Zero,
            accumulatedImpulse ?? Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
    }

    private static FixedLeverResponseOperand3d CreateTangentOperand(
        FixedLever lever,
        Vector3d velocity,
        Vector3d signedAxis,
        Fixed64 inverseMass,
        Fixed3x3? inverseInertia = null) =>
        new(
            lever,
            velocity,
            Vector3d.Zero,
            signedAxis,
            inverseMass,
            inverseInertia ?? Fixed3x3.Zero);

    private static FixedLever CreateLever() =>
        CreateLever(Vector3d.Zero);

    private static FixedLever CreateLever(Vector3d point)
    {
        var pointAnchor = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var centerAnchor = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Assert.True(pointAnchor.TryGetLeverFrom(
            centerAnchor,
            out FixedLever lever));
        return lever;
    }
}
