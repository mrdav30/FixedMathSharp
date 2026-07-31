using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTransformTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromDouble(0.0001);

    [Fact]
    public void FixedTransform_FromComponents_PreservesLocalComponentsAndNormalizesRotation()
    {
        Vector3d position = new((Fixed64)1, (Fixed64)2, (Fixed64)3);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, (Fixed64)2);
        Vector3d scale = new((Fixed64)(-2), Fixed64.Zero, (Fixed64)4);
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);

        var transform = new FixedTransform(position, rotation, scale, parent);

        Assert.Equal(position, transform.LocalPosition);
        Assert.Equal(FixedQuaternion.Identity, transform.LocalRotation);
        Assert.Equal(scale, transform.LocalScale);
        Assert.Same(parent, transform.Parent);
        Assert.Equal(Fixed4x4.CreateTransform(position, FixedQuaternion.Identity, scale), transform.LocalMatrix);

        transform.LocalPosition = new Vector3d((Fixed64)5, (Fixed64)6, (Fixed64)7);
        transform.LocalRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, (Fixed64)3);
        transform.LocalScale = new Vector3d(Fixed64.Zero, (Fixed64)(-3), (Fixed64)8);

        Assert.Equal(new Vector3d((Fixed64)5, (Fixed64)6, (Fixed64)7), transform.LocalPosition);
        Assert.Equal(FixedQuaternion.Identity, transform.LocalRotation);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)(-3), (Fixed64)8), transform.LocalScale);
    }

    [Fact]
    public void FixedTransform_PlanarConstructorAndProperties_UseLocalXZContract()
    {
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector2d position = new((Fixed64)2, (Fixed64)3);
        Vector2d scale = new((Fixed64)(-4), Fixed64.Zero);
        var transform = new FixedTransform(position, Fixed64.PiOver4, scale, parent);

        Assert.Equal(new Vector3d(position.X, Fixed64.Zero, position.Y), transform.LocalPosition);
        Assert.Equal(new Vector3d(scale.X, Fixed64.One, scale.Y), transform.LocalScale);
        Assert.Same(parent, transform.Parent);
        AssertAnglesEquivalent(Fixed64.PiOver4, transform.LocalRotationXZRadians);

        transform.LocalPosition = new Vector3d((Fixed64)1, (Fixed64)7, (Fixed64)2);
        transform.LocalScale = new Vector3d((Fixed64)3, (Fixed64)5, (Fixed64)4);
        transform.LocalPositionXZ = new Vector2d((Fixed64)(-1), (Fixed64)(-2));
        transform.LocalScaleXZ = new Vector2d((Fixed64)(-3), Fixed64.Zero);
        transform.LocalRotationXZRadians = Fixed64.PiOver3;

        Assert.Equal(new Vector3d((Fixed64)(-1), (Fixed64)7, (Fixed64)(-2)), transform.LocalPosition);
        Assert.Equal(new Vector3d((Fixed64)(-3), (Fixed64)5, Fixed64.Zero), transform.LocalScale);
        Assert.Equal(transform.LocalPosition.ToVector2d(), transform.LocalPositionXZ);
        Assert.Equal(transform.LocalScale.ToVector2d(), transform.LocalScaleXZ);
        AssertRepresentsSameRotation(
            transform.LocalRotation,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.PiOver3));
    }

    [Fact]
    public void FixedTransform_LocalRotationXZRadians_RoundTripsMultiTurnAngles()
    {
        Fixed64[] angles =
        {
            Fixed64.Zero,
            Fixed64.HalfPi,
            -Fixed64.HalfPi,
            Fixed64.PiOver6 + (Fixed64.TwoPi * (Fixed64)7),
            -Fixed64.PiOver4 - (Fixed64.TwoPi * (Fixed64)6),
        };

        foreach (Fixed64 angle in angles)
        {
            var transform = new FixedTransform(Vector2d.Zero, angle, Vector2d.One);
            AssertAnglesEquivalent(angle, transform.LocalRotationXZRadians);
            Assert.True(
                transform.LocalRotation.Rotate(Vector3d.Right).ToVector2d()
                    .FuzzyEqual(Vector2d.Rotate(Vector2d.Right, angle), Tolerance));
        }
    }

    [Fact]
    public void FixedTransform_LocalRotationXZRadians_ReplacesPitchAndRollAndIgnoresScale()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAngles(Fixed64.PiOver6, Fixed64.PiOver4, Fixed64.PiOver3),
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)3));

        transform.LocalRotationXZRadians = Fixed64.PiOver3;

        FixedQuaternion expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.PiOver3).Normalized;
        AssertRepresentsSameRotation(transform.LocalRotation, expected);
        AssertAnglesEquivalent(Fixed64.PiOver3, transform.LocalRotationXZRadians);
    }

    [Fact]
    public void FixedTransform_LocalRotationXZRadians_ZeroProjectedRightReturnsZero()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One),
            new Vector3d((Fixed64)9, (Fixed64)8, (Fixed64)(-7)));
        Vector3d localRight = transform.LocalRotation.Rotate(Vector3d.Right);

        Assert.Equal(Fixed64.Zero, localRight.X);
        Assert.Equal(Fixed64.Zero, localRight.Z);
        Assert.Equal(Fixed64.Zero, transform.LocalRotationXZRadians);
    }

    [Fact]
    public void FixedTransform_LocalEulerAngles_UsesDegreeBasedQuaternion()
    {
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector3d eulerAngles = new((Fixed64)90, Fixed64.Zero, Fixed64.Zero);

        transform.LocalEulerAngles = eulerAngles;

        Assert.True(transform.LocalEulerAngles.FuzzyEqual(eulerAngles, Tolerance));
        AssertRepresentsSameRotation(
            transform.LocalRotation,
            FixedQuaternion.FromEulerAnglesInDegrees(eulerAngles.X, eulerAngles.Y, eulerAngles.Z));
    }

    [Fact]
    public void FixedTransform_TryCreateFromLocalMatrix_UsesStrictDecomposition()
    {
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector3d position = new((Fixed64)(-5), (Fixed64)6, (Fixed64)7);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-20), (Fixed64)35, (Fixed64)50);
        Vector3d scale = new((Fixed64)(-3), (Fixed64)4, (Fixed64)5);
        Fixed4x4 matrix = Fixed4x4.CreateTransform(position, rotation, scale);

        Assert.True(FixedTransform.TryCreateFromLocalMatrix(matrix, out FixedTransform? transform, parent));
        Assert.NotNull(transform);
        Assert.Equal(position, transform.LocalPosition);
        Assert.True(transform.LocalScale.FuzzyEqual(scale, Tolerance));
        AssertRepresentsSameRotation(transform.LocalRotation, rotation);
        Assert.Same(parent, transform.Parent);

        Fixed4x4 shear = Fixed4x4.Identity;
        shear.M12 = Fixed64.Half;
        Assert.False(FixedTransform.TryCreateFromLocalMatrix(shear, out transform));
        Assert.Null(transform);
        Assert.False(FixedTransform.TryCreateFromLocalMatrix(Fixed4x4.CreateScale(Vector3d.Zero), out transform));
        Assert.Null(transform);
    }

    [Fact]
    public void FixedTransform_RootWorldViews_MatchLocalComponents()
    {
        Vector3d position = new((Fixed64)3, (Fixed64)(-2), (Fixed64)7);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAngles(Fixed64.PiOver6, Fixed64.PiOver4, Fixed64.PiOver3);
        Vector3d scale = new((Fixed64)(-2), Fixed64.Zero, (Fixed64)4);
        var transform = new FixedTransform(position, rotation, scale);

        Assert.Equal(transform.LocalMatrix, transform.LocalToWorldMatrix);
        Assert.Equal(position, transform.WorldPosition);
        AssertRepresentsSameRotation(transform.WorldRotation, transform.LocalRotation);
        Assert.True(transform.WorldRotation.IsNormalized());
        Assert.Equal(transform.LocalMatrix.LossyScale, transform.LossyScale);
        Assert.Equal(position.ToVector2d(), transform.WorldPositionXZ);
    }

    [Fact]
    public void FixedTransform_PositiveRootWorldViews_ReturnExactStoredComponents()
    {
        Vector3d position = new((Fixed64)3, (Fixed64)(-2), (Fixed64)7);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAngles(Fixed64.PiOver6, Fixed64.PiOver4, Fixed64.PiOver3);
        Vector3d scale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        var transform = new FixedTransform(position, rotation, scale);

        Assert.Equal(position, transform.WorldPosition);
        Assert.Equal(transform.LocalRotation, transform.WorldRotation);
        Assert.Equal(scale, transform.LossyScale);
    }

    [Fact]
    public void FixedTransform_NegativeYOrZRootScale_CanonicalizesReflectionToX()
    {
        var negativeY = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, -3, 4));
        var negativeZ = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(2, 3, -4));

        var canonicalScale = new Vector3d(-2, 3, 4);
        Assert.Equal(canonicalScale, negativeY.LossyScale);
        Assert.Equal(canonicalScale, negativeZ.LossyScale);
    }

    [Fact]
    public void FixedTransform_HierarchyReads_UseExactRowVectorOrderAndQuaternionChain()
    {
        var grandParent = new FixedTransform(
            new Vector3d((Fixed64)4, (Fixed64)1, (Fixed64)(-3)),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        var parent = new FixedTransform(
            new Vector3d((Fixed64)(-2), (Fixed64)5, Fixed64.One),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver6),
            new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.One),
            grandParent);
        var child = new FixedTransform(
            new Vector3d(Fixed64.One, (Fixed64)(-1), (Fixed64)2),
            FixedQuaternion.FromAxisAngle(Vector3d.Right, -Fixed64.PiOver4),
            new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)3),
            parent);

        Fixed4x4 expectedWorld = child.LocalMatrix * parent.LocalMatrix * grandParent.LocalMatrix;
        Fixed3x3 expectedRotation = child.LocalRotation.ToMatrix3x3()
            * parent.LocalRotation.ToMatrix3x3()
            * grandParent.LocalRotation.ToMatrix3x3();

        Assert.Equal(expectedWorld, child.LocalToWorldMatrix);
        Assert.Equal(expectedWorld.Translation, child.WorldPosition);
        Assert.True(child.WorldRotation.IsNormalized());
        AssertMatrixApproximatelyEqual(expectedRotation, child.WorldRotation.ToMatrix3x3());
    }

    [Fact]
    public void FixedTransform_StrictHierarchyReads_ReturnExactComposedMatrixAndLossyScale()
    {
        var grandParent = new FixedTransform(
            new Vector3d(4, 1, -3),
            FixedQuaternion.Identity,
            new Vector3d(2, 3, 4));
        var parent = new FixedTransform(
            new Vector3d(-2, 5, 1),
            FixedQuaternion.Identity,
            new Vector3d(3, 2, 1),
            grandParent);
        var child = new FixedTransform(
            new Vector3d(1, -1, 2),
            FixedQuaternion.Identity,
            new Vector3d(2, 1, 3),
            parent);
        Fixed4x4 expectedMatrix = child.LocalMatrix * parent.LocalMatrix * grandParent.LocalMatrix;
        Vector3d expectedLossyScale = Fixed4x4.ExtractLossyScale(expectedMatrix);

        Assert.True(child.TryGetLocalToWorldMatrix(out Fixed4x4 matrix));
        Assert.Equal(expectedMatrix, matrix);
        Assert.True(child.TryGetLossyScale(out Vector3d lossyScale));
        Assert.Equal(expectedLossyScale, lossyScale);
    }

    [Fact]
    public void FixedTransform_StrictHierarchyReads_FailAtomicallyForUnrepresentableComposition()
    {
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            new Vector3d(3, 4, 5),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.One),
            parent);
        Vector3d parentPosition = parent.LocalPosition;
        FixedQuaternion parentRotation = parent.LocalRotation;
        Vector3d parentScale = parent.LocalScale;
        Vector3d childPosition = child.LocalPosition;
        FixedQuaternion childRotation = child.LocalRotation;
        Vector3d childScale = child.LocalScale;

        Assert.False(child.TryGetLocalToWorldMatrix(out Fixed4x4 matrix));
        Assert.Equal(Fixed4x4.Zero, matrix);
        Assert.False(child.TryGetLossyScale(out Vector3d lossyScale));
        Assert.Equal(Vector3d.Zero, lossyScale);
        Assert.Equal(parentPosition, parent.LocalPosition);
        Assert.Equal(parentRotation, parent.LocalRotation);
        Assert.Equal(parentScale, parent.LocalScale);
        Assert.Equal(childPosition, child.LocalPosition);
        Assert.Equal(childRotation, child.LocalRotation);
        Assert.Equal(childScale, child.LocalScale);
    }

    [Fact]
    public void FixedTransform_StrictReads_RejectSaturatedRootLocalTrs()
    {
        var root = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi),
            new Vector3d(Fixed64.MinValue, Fixed64.One, Fixed64.One));

        Assert.Equal(Fixed64.MaxValue, root.LocalMatrix.M11);
        Assert.False(root.TryGetLocalToWorldMatrix(out Fixed4x4 matrix));
        Assert.Equal(Fixed4x4.Zero, matrix);
        Assert.False(root.TryGetLossyScale(out Vector3d scale));
        Assert.Equal(Vector3d.Zero, scale);
    }

    [Fact]
    public void FixedTransform_StrictHierarchyReads_RejectSaturatedAncestorLocalTrs()
    {
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi),
            new Vector3d(Fixed64.MinValue, Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            Vector3d.One,
            FixedQuaternion.Identity,
            Vector3d.One,
            parent);

        Assert.False(child.TryGetLocalToWorldMatrix(out Fixed4x4 matrix));
        Assert.Equal(Fixed4x4.Zero, matrix);
    }

    [Fact]
    public void FixedTransform_TryGetLossyScale_PreservesCanonicalReflection()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)(-2), (Fixed64)3, (Fixed64)4));

        Assert.True(transform.TryGetLossyScale(out Vector3d scale));
        Assert.Equal(new Vector3d((Fixed64)(-2), (Fixed64)3, (Fixed64)4), scale);
    }

    [Fact]
    public void FixedTransform_TryGetLossyScale_PreservesNonnegativeRootScale()
    {
        Vector3d expected = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Half);
        var transform = new FixedTransform(
            new Vector3d(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            expected);

        Assert.True(transform.TryGetLossyScale(out Vector3d scale));
        Assert.Equal(expected, scale);
    }

    [Fact]
    public void FixedTransform_TryGetLossyScale_RejectsUnrepresentableComposedBasisMagnitude()
    {
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d(Fixed64.FromDouble(1.1), Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.One),
            parent);

        Assert.True(child.TryGetLocalToWorldMatrix(out Fixed4x4 matrix));
        Assert.False(Vector3d.TryGetMagnitude(new Vector3d(matrix.M11, matrix.M12, matrix.M13), out _));
        Assert.False(child.TryGetLossyScale(out Vector3d scale));
        Assert.Equal(Vector3d.Zero, scale);
    }

    [Fact]
    public void FixedTransform_ScaleInducedShear_UsesComposedCanonicalLossyScale()
    {
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)(-4)));
        var child = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4),
            new Vector3d((Fixed64)(-2), Fixed64.One, (Fixed64)3),
            parent);

        Fixed4x4 world = child.LocalMatrix * parent.LocalMatrix;
        Vector3d basisX = new(world.M11, world.M12, world.M13);
        Vector3d basisY = new(world.M21, world.M22, world.M23);

        Assert.True(FixedMath.Abs(Vector3d.Dot(basisX, basisY)) > Fixed64.Epsilon);
        Assert.False(Fixed4x4.Decompose(world, out _, out _, out _));
        Assert.Equal(Fixed4x4.ExtractLossyScale(world), child.LossyScale);
    }

    [Fact]
    public void FixedTransform_WorldXZViews_UseComposedRotationAndEstablishedAngleConvention()
    {
        var parent = new FixedTransform(Vector2d.Zero, Fixed64.PiOver4, Vector2d.One);
        var child = new FixedTransform(
            new Vector2d((Fixed64)2, (Fixed64)3),
            Fixed64.PiOver6 + Fixed64.TwoPi * (Fixed64)8,
            Vector2d.One,
            parent);

        Assert.Equal(child.WorldPosition.ToVector2d(), child.WorldPositionXZ);
        AssertAnglesEquivalent(Fixed64.PiOver4 + Fixed64.PiOver6, child.WorldRotationXZRadians);
    }

    [Fact]
    public void FixedTransform_HierarchyTraversal_IsIterativeAndAllocationFree()
    {
        var transform = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One);
        for (int i = 0; i < 10_000; i++)
            transform = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One, transform);

        Assert.Equal(new Vector3d((Fixed64)10_001, Fixed64.Zero, Fixed64.Zero), transform.WorldPosition);

        var depthEight = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One);
        for (int i = 0; i < 8; i++)
            depthEight = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One, depthEight);

        Fixed4x4 sink = depthEight.LocalToWorldMatrix;
        for (int i = 0; i < 1_024; i++)
            sink = depthEight.LocalToWorldMatrix;

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1_024; i++)
            sink = depthEight.LocalToWorldMatrix;
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal((Fixed64)9, sink.M41);
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void FixedTransform_SetParentKeepingLocal_OnlyChangesParentAndSameParentIsNoOp()
    {
        var parent = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One);
        var child = new FixedTransform(
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)4));
        Vector3d position = child.LocalPosition;
        FixedQuaternion rotation = child.LocalRotation;
        Vector3d scale = child.LocalScale;

        child.SetParentKeepingLocal(parent);
        child.SetParentKeepingLocal(parent);

        Assert.Same(parent, child.Parent);
        Assert.Equal(position, child.LocalPosition);
        Assert.Equal(rotation, child.LocalRotation);
        Assert.Equal(scale, child.LocalScale);

        child.SetParentKeepingLocal(null);
        Assert.Null(child.Parent);
        Assert.Equal(position, child.LocalPosition);
    }

    [Fact]
    public void FixedTransform_ParentChanges_RejectSelfAndCyclesAtomically()
    {
        var root = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var child = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, root);
        var grandChild = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, child);

        Assert.Throws<ArgumentException>(() => root.SetParentKeepingLocal(root));
        Assert.Throws<ArgumentException>(() => root.SetParentKeepingLocal(grandChild));
        Assert.Throws<ArgumentException>(() => root.TrySetParentKeepingWorld(grandChild));

        Assert.Null(root.Parent);
        Assert.Same(root, child.Parent);
        Assert.Same(child, grandChild.Parent);
    }

    [Fact]
    public void FixedTransform_TrySetParentKeepingWorld_AttachesAndDetachesWithoutChangingWorld()
    {
        var parent = new FixedTransform(
            new Vector3d((Fixed64)10, (Fixed64)(-4), (Fixed64)2),
            FixedQuaternion.Identity,
            Vector3d.One);
        var child = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)5, (Fixed64)(-7)),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)(-2), (Fixed64)3, (Fixed64)4));
        Fixed4x4 originalWorld = child.LocalToWorldMatrix;

        Assert.True(child.TrySetParentKeepingWorld(parent));
        Assert.Same(parent, child.Parent);
        Assert.True(originalWorld.FuzzyEqualAbsolute(child.LocalToWorldMatrix, Fixed64.Epsilon));
        Assert.True(child.TrySetParentKeepingWorld(parent));
        Assert.True(originalWorld.FuzzyEqualAbsolute(child.LocalToWorldMatrix, Fixed64.Epsilon));

        Assert.True(child.TrySetParentKeepingWorld(null));
        Assert.Null(child.Parent);
        Assert.True(originalWorld.FuzzyEqualAbsolute(child.LocalToWorldMatrix, Fixed64.Epsilon));
    }

    [Fact]
    public void FixedTransform_TrySetParentKeepingWorld_SingularParentFailsAtomically()
    {
        var singularParent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero);
        var child = new FixedTransform(
            new Vector3d((Fixed64)1, (Fixed64)2, (Fixed64)3),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        Vector3d position = child.LocalPosition;
        FixedQuaternion rotation = child.LocalRotation;
        Vector3d scale = child.LocalScale;

        Assert.False(child.TrySetParentKeepingWorld(singularParent));

        Assert.Null(child.Parent);
        Assert.Equal(position, child.LocalPosition);
        Assert.Equal(rotation, child.LocalRotation);
        Assert.Equal(scale, child.LocalScale);
    }

    [Fact]
    public void FixedTransform_TrySetParentKeepingWorld_ShearedRelativeMatrixFailsAtomically()
    {
        var proposedParent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        var child = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Assert.True(proposedParent.TryGetWorldToLocalMatrix(out Fixed4x4 parentInverse));
        Assert.False(Fixed4x4.Decompose(child.LocalToWorldMatrix * parentInverse, out _, out _, out _));

        Assert.False(child.TrySetParentKeepingWorld(proposedParent));

        Assert.Null(child.Parent);
        Assert.Equal(Vector3d.Zero, child.LocalPosition);
        Assert.Equal(FixedQuaternion.Identity, child.LocalRotation);
        Assert.Equal(Vector3d.One, child.LocalScale);
    }

    [Fact]
    public void FixedTransform_TrySetParentKeepingWorld_SaturatedLocalTranslationFailsAtomically()
    {
        var proposedParent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Fixed4x4 world = child.LocalToWorldMatrix;
        Vector3d position = child.LocalPosition;
        FixedQuaternion rotation = child.LocalRotation;
        Vector3d scale = child.LocalScale;

        Assert.True(proposedParent.TryGetWorldToLocalMatrix(out _));
        Assert.False(child.TrySetParentKeepingWorld(proposedParent));

        Assert.Null(child.Parent);
        Assert.Equal(position, child.LocalPosition);
        Assert.Equal(rotation, child.LocalRotation);
        Assert.Equal(scale, child.LocalScale);
        Assert.Equal(world, child.LocalToWorldMatrix);
    }

    [Fact]
    public void FixedTransform_WorldPositionAndPoseSetters_UpdateRootsAndNormalizeRotation()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4),
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)4));
        Vector3d scale = transform.LocalScale;

        Assert.True(transform.TrySetWorldPosition(new Vector3d((Fixed64)3, (Fixed64)4, (Fixed64)5)));
        Assert.Equal(new Vector3d((Fixed64)3, (Fixed64)4, (Fixed64)5), transform.LocalPosition);

        Assert.True(transform.TrySetWorldPose(new Vector3d((Fixed64)6, (Fixed64)7, (Fixed64)8), FixedQuaternion.Zero));
        Assert.Equal(new Vector3d((Fixed64)6, (Fixed64)7, (Fixed64)8), transform.LocalPosition);
        Assert.Equal(FixedQuaternion.Identity, transform.LocalRotation);
        Assert.Equal(scale, transform.LocalScale);
    }

    [Fact]
    public void FixedTransform_WorldSetters_UseInvertibleNonuniformParentAndRotationalHierarchy()
    {
        var grandParent = new FixedTransform(
            new Vector3d((Fixed64)1, (Fixed64)2, (Fixed64)3),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver6),
            Vector3d.One);
        var parent = new FixedTransform(
            new Vector3d((Fixed64)(-4), (Fixed64)5, (Fixed64)2),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4),
            grandParent);
        var child = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d((Fixed64)(-2), Fixed64.One, (Fixed64)3), parent);
        Vector3d expectedLocalPosition = new((Fixed64)2, (Fixed64)(-1), (Fixed64)3);
        Vector3d desiredWorldPosition = Fixed4x4.TransformPoint(parent.LocalToWorldMatrix, expectedLocalPosition);
        FixedQuaternion expectedLocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, -Fixed64.PiOver4);
        FixedQuaternion desiredWorldRotation = (parent.WorldRotation * expectedLocalRotation).Normalized;

        Assert.True(child.TrySetWorldPosition(desiredWorldPosition));
        Assert.True(child.LocalPosition.FuzzyEqualAbsolute(expectedLocalPosition, Tolerance));
        Assert.True(child.TrySetWorldPose(desiredWorldPosition, desiredWorldRotation * (Fixed64)3));

        Assert.True(child.LocalPosition.FuzzyEqualAbsolute(expectedLocalPosition, Tolerance));
        AssertRepresentsSameRotation(child.LocalRotation, expectedLocalRotation);
        Assert.Equal(new Vector3d((Fixed64)(-2), Fixed64.One, (Fixed64)3), child.LocalScale);
    }

    [Fact]
    public void FixedTransform_WorldSetters_SingularParentFailAtomically()
    {
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero);
        var child = new FixedTransform(
            new Vector3d((Fixed64)1, (Fixed64)2, (Fixed64)3),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            Vector3d.One,
            parent);
        Vector3d position = child.LocalPosition;
        FixedQuaternion rotation = child.LocalRotation;

        Assert.False(child.TrySetWorldPosition(new Vector3d((Fixed64)9, (Fixed64)8, (Fixed64)7)));
        Assert.Equal(position, child.LocalPosition);
        Assert.False(child.TrySetWorldPose(Vector3d.Zero, FixedQuaternion.Identity));
        Assert.Equal(position, child.LocalPosition);
        Assert.Equal(rotation, child.LocalRotation);
    }

    [Fact]
    public void FixedTransform_TrySetWorldPosition_SaturatedLocalTranslationFailsAtomically()
    {
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.One),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4),
            parent);
        Vector3d position = child.LocalPosition;
        FixedQuaternion rotation = child.LocalRotation;
        Vector3d scale = child.LocalScale;

        Assert.True(parent.TryGetWorldToLocalMatrix(out _));
        Assert.False(child.TrySetWorldPosition(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero)));

        Assert.Same(parent, child.Parent);
        Assert.Equal(position, child.LocalPosition);
        Assert.Equal(rotation, child.LocalRotation);
        Assert.Equal(scale, child.LocalScale);
    }

    [Fact]
    public void FixedTransform_TrySetWorldPose_SaturatedLocalTranslationFailsAtomically()
    {
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.One),
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver6),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4),
            parent);
        Vector3d position = child.LocalPosition;
        FixedQuaternion rotation = child.LocalRotation;
        Vector3d scale = child.LocalScale;
        FixedQuaternion desiredRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver3) * (Fixed64)3;

        Assert.True(parent.TryGetWorldToLocalMatrix(out _));
        Assert.False(child.TrySetWorldPose(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            desiredRotation));

        Assert.Same(parent, child.Parent);
        Assert.Equal(position, child.LocalPosition);
        Assert.Equal(rotation, child.LocalRotation);
        Assert.Equal(scale, child.LocalScale);
    }

    [Fact]
    public void FixedTransform_TryGetWorldToLocalMatrix_VerifiesBothRoundTrips()
    {
        var transform = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)(-4), (Fixed64)5),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));

        Assert.True(transform.TryGetWorldToLocalMatrix(out Fixed4x4 inverse));
        Assert.True((transform.LocalToWorldMatrix * inverse).FuzzyEqualAbsolute(Fixed4x4.Identity, Fixed64.Epsilon));
        Assert.True((inverse * transform.LocalToWorldMatrix).FuzzyEqualAbsolute(Fixed4x4.Identity, Fixed64.Epsilon));

        var saturated = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.One, Fixed64.One));

        Assert.False(saturated.TryGetWorldToLocalMatrix(out inverse));
        Assert.Equal(Fixed4x4.Identity, inverse);
    }

    private static void AssertAnglesEquivalent(Fixed64 expected, Fixed64 actual)
    {
        Fixed64 difference = (actual - expected) % Fixed64.TwoPi;
        if (difference > Fixed64.Pi)
            difference -= Fixed64.TwoPi;
        else if (difference < -Fixed64.Pi)
            difference += Fixed64.TwoPi;

        Assert.True(FixedMath.Abs(difference) <= Tolerance, $"Expected {expected} modulo TwoPi, got {actual}.");
    }

    private static void AssertRepresentsSameRotation(FixedQuaternion actual, FixedQuaternion expected)
    {
        expected = expected.Normalized;
        Assert.True(
            actual.FuzzyEqualAbsolute(expected, Tolerance)
            || actual.FuzzyEqualAbsolute(-expected, Tolerance),
            $"Expected {actual} to represent the same rotation as {expected}.");
    }

    private static void AssertMatrixApproximatelyEqual(Fixed3x3 expected, Fixed3x3 actual)
    {
        Assert.True(
            expected.FuzzyEqualAbsolute(actual, Tolerance),
            $"Expected {expected}, got {actual}.");
    }
}
