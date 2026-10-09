//=======================================================================
// FixedTriangle.FiniteCone.PlaneSection.Materialization.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ConePlaneRayMaterializationTests
{
    [Theory]
    [InlineData(5L, 3L)]
    [InlineData(6L, 3L)]
    [InlineData(5L, 0L)]
    [InlineData(long.MaxValue, long.MaxValue)]
    public void WholeConeWorldRange_UsesExactDoubledBoundsAtEverySignedAxis(long heightRaw, long radiusRaw)
    {
        BigInteger bound = (BigInteger)heightRaw + 2 * (BigInteger)radiusRaw;
        BigInteger gap = (bound + 1) / 2;
        for (int axis = 0; axis < 3; axis++)
        for (int side = -1; side <= 1; side += 2)
        for (int margin = -1; margin <= 1; margin++)
        {
            BigInteger centerRaw = side > 0 ? (BigInteger)long.MaxValue - gap - margin
                : (BigInteger)long.MinValue + gap + margin;
            if (centerRaw < long.MinValue || centerRaw > long.MaxValue) centerRaw = 0;
            Vector3d center = Vector3d.Zero; center[axis] = Fixed64.FromRaw((long)centerRaw);
            var plane = new FixedTriangle(new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
            var frame = new ConePlaneRayFrame(plane, center, FixedQuaternion.Identity,
                center, FixedQuaternion.Identity, Fixed64.FromRaw(heightRaw), Fixed64.FromRaw(radiusRaw));
            bool expected = true;
            for (int coordinate = 0; coordinate < 3; coordinate++)
            {
                BigInteger doubled = 2 * (BigInteger)center[coordinate].m_rawValue;
                expected &= doubled - bound >= 2 * (BigInteger)long.MinValue
                    && doubled + bound <= 2 * (BigInteger)long.MaxValue;
            }
            Assert.Equal(expected, ConePlaneRayPointMaterialization.IsWorldRangeRepresentable(frame));
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void WholeConeWorldRange_CertifiedRotatedConeMaterializesAllStreamedPointsAndExits(int axis, bool negative)
    {
        Vector3d center = Vector3d.Zero;
        center[axis] = Fixed64.FromRaw(negative ? long.MinValue + 6 : long.MaxValue - 6);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        var plane = new FixedTriangle(new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
        var frame = new ConePlaneRayFrame(plane, center, rotation, center, rotation,
            Fixed64.FromRaw(5), Fixed64.FromRaw(3));
        Assert.True(ConePlaneRayPointMaterialization.IsWorldRangeRepresentable(frame));
        int count = 0;
        void Check(in ConePlaneRayEventSource source, in ConePlaneRayFrame currentFrame, ConePlaneRayEvent descriptor,
            scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(currentFrame, point, root, out _));
            if (positive.HasValue) Assert.True(positive.TryMaterialize(currentFrame, 1, out _, out _, out _));
            if (negative.HasValue) Assert.True(negative.TryMaterialize(currentFrame, -1, out _, out _, out _));
            count++;
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(ConePlaneRayEventSource.Plane, frame, Check));
        Assert.True(count > 0);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ActiveMaterialization_PreservesEquivalentHighQuadraticFieldsAndRoot(bool wide, bool rotated)
    {
        Vector3d center = new(Fixed64.FromRaw(1), Fixed64.FromRaw(3), Fixed64.FromRaw(-5));
        FixedQuaternion rotation = rotated ? new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero) : FixedQuaternion.Identity;
        var frame = Frame(center, rotation);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs); point.Set(frame.Transform(Vector3d.Zero));
        BigInteger rootScale = BigInteger.One << (wide ? 64 * 19 : 0), common = BigInteger.One << (wide ? 64 * 40 : 0);
        // Multiplying every point coordinate by 1+sqrt(2), and replacing
        // sqrt(2) with sqrt(2*S²)/S, preserves the exact admitted point.
        // The high representation exercises both radical products, not merely
        // padded zero banks; its coefficients remain inside retained storage.
        for (int axis = 0; axis < 4; axis++)
        {
            ContactQuadratic field = axis == 0 ? point.X : axis == 1 ? point.Y : axis == 2 ? point.Z : point.Denominator;
            BigInteger original = ReadMagnitude(field.Rational) * field.Signs[0];
            SetField(field, original * common * rootScale, original * common);
        }
        WriteMagnitude(2 * rootScale * rootScale, root);
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRaySelection.FieldWords];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRaySelection.FieldWords);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRaySelection.FieldWords);
        n.Set(Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.AddSigned192(frame.Finite.ShapeFrame.Denominator, frame.Finite.ShapeFrame.Denominator))));
        BigInteger basisScale = ReadMagnitude(n.Rational);
        SetField(n, basisScale * rootScale, basisScale); SetField(d, 3 * rootScale, 0);
        ulong[] originalPoint = words.ToArray(), originalFields = fields.ToArray(), originalRoot = root.ToArray();
        Assert.True(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, point, root, out Vector3d authored));
        Assert.Equal(Vector3d.Zero, authored);
        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, root, out Vector3d world));
        Assert.Equal(center, world);
        // (1+sqrt(2))/3 is strictly between 2/3 and 1 raw unit.
        Assert.True(ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(frame, point, root, n, d, 1, out Vector3d localExit));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero), localExit);
        Assert.True(ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point, root, n, d, 1, out Vector3d worldExit));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(rotated ? 0 : 2), center.Y, center.Z), worldExit);
        Assert.True(ConePlaneRayPointMaterialization.IsExitWorldPointRepresentable(frame, point, root, n, d, 1));
        Assert.True(words.SequenceEqual(originalPoint)); Assert.True(fields.SequenceEqual(originalFields)); Assert.True(root.SequenceEqual(originalRoot));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(110)]
    public void WorldMaterialization_AliasedPaddedCoordinatesRetainCompleteSourceFields(int activeWord)
    {
        const int words = 131;
        Span<ulong> values = stackalloc ulong[2 * words]; Span<int> signs = stackalloc int[2];
        var field = new ContactQuadratic(values, signs);
        SetField(field, BigInteger.One << (64 * activeWord), BigInteger.One << (64 * activeWord));
        ulong[] original = values.ToArray();
        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity,
            field, field, field, field, new ulong[] { 2 }, out Vector3d result));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(1), Fixed64.FromRaw(1), Fixed64.FromRaw(1)), result);
        Assert.True(values.SequenceEqual(original));
    }

    private static void SetField(ContactQuadratic field, BigInteger rational, BigInteger radical)
    {
        WriteMagnitude(rational, field.Rational); WriteMagnitude(radical, field.Radical);
        field.Signs[0] = rational.Sign; field.Signs[1] = radical.Sign;
    }

    private static void WriteMagnitude(BigInteger value, Span<ulong> destination)
    {
        value = BigInteger.Abs(value); destination.Clear();
        for (int index = 0; index < destination.Length; index++) { destination[index] = (ulong)(value & ulong.MaxValue); value >>= 64; }
        Assert.Equal(BigInteger.Zero, value);
    }

    private static BigInteger ReadMagnitude(ReadOnlySpan<ulong> words)
    {
        BigInteger result = 0;
        for (int index = words.Length - 1; index >= 0; index--) result = (result << 64) | words[index];
        return result;
    }

    [Theory]
    [InlineData(1, 0, 2)]
    [InlineData(3, 2, 2)]
    [InlineData(-1, 0, 0)]
    [InlineData(-3, -2, 0)]
    public void LocalAnchors_RoundExactHalfRawTiesWithoutInvertingRoundedWorldAnchors(int halfRaws, long localRaw, long worldRaw)
    {
        // An odd common translation changes world tie parity. Local identity
        // must round the retained fraction directly rather than subtracting
        // that translation from the independently rounded world coordinate.
        Vector3d center = new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero);
        var plane = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = new ConePlaneRayFrame(plane, center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs); point.Set(frame.Transform(Vector3d.Zero));
        point.X.Set(Signed576.ExtendValue(WideArithmetic.MultiplySigned192(
            frame.Finite.ShapeFrame.Denominator, Signed192.Signed(halfRaws))));
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        n.Set(default); d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
        Vector3d expectedLocal = new(Fixed64.FromRaw(localRaw), Fixed64.Zero, Fixed64.Zero);
        Assert.True(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, point, ReadOnlySpan<ulong>.Empty, out Vector3d authored));
        Assert.Equal(expectedLocal, authored);
        Assert.True(ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, 1, out Vector3d coneLocal));
        Assert.Equal(expectedLocal, coneLocal);
        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, ReadOnlySpan<ulong>.Empty, out Vector3d world));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(worldRaw), Fixed64.Zero, Fixed64.Zero), world);
        Assert.NotEqual(authored, world - center);
    }

    [Theory]
    [InlineData(-1, 1, 1)]
    [InlineData(0, 0, 1)]
    [InlineData(0, -1, 1)]
    [InlineData(0, 1, 0)]
    [InlineData(0, 1, -1)]
    public void LocalExit_RejectsNegativeDepthAndNonpositiveHomogeneousDenominators(int numerator, int denominator, int pointDenominator)
    {
        var frame = Frame(Vector3d.Zero, FixedQuaternion.Identity);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs); point.Set(frame.Transform(Vector3d.Zero));
        point.Denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(pointDenominator))));
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        n.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(numerator))));
        d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(denominator))));
        Assert.False(ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, 1, out Vector3d local));
        Assert.Equal(Vector3d.Zero, local);
        if (pointDenominator <= 0)
        {
            Assert.False(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, point,
                ReadOnlySpan<ulong>.Empty, out local));
            Assert.Equal(Vector3d.Zero, local);
        }
    }

    [Theory]
    [InlineData(1, 0, 2)]
    [InlineData(3, 2, 2)]
    [InlineData(5, 2, 4)]
    public void LocalApex_UsesExactOddRawHeightBeforeIndependentLocalAndWorldRounding(long heightRaw, long localRaw, long worldRaw)
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero);
        var plane = new FixedTriangle(new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
        var frame = new ConePlaneRayFrame(plane, center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.FromRaw(heightRaw), Fixed64.One);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs); point.Set(default);
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        n.Set(default); d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
        // Apex Y=0 in the retained frame means centered Y=fullHeight/2,
        // including half-raw heights which have no authored Fixed64 point.
        Vector3d expectedLocal = new(Fixed64.Zero, Fixed64.FromRaw(localRaw), Fixed64.Zero);
        Assert.True(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, point, ReadOnlySpan<ulong>.Empty, out Vector3d authored));
        Assert.Equal(expectedLocal, authored);
        Assert.True(ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, -1, out Vector3d coneLocal));
        Assert.Equal(expectedLocal, coneLocal);
        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, ReadOnlySpan<ulong>.Empty, out Vector3d world));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(worldRaw), Fixed64.Zero), world);
        Assert.NotEqual(authored, world - center);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void ExactLocalAnchors_PreserveRelativePoseAcrossCommonTranslationAndRotation(int orientation)
    {
        Vector3d authoredPoint = new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Half);
        var plane = new FixedTriangle(new Vector3d(authoredPoint.X, (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d(authoredPoint.X, Fixed64.Two, (Fixed64)(-2)), new Vector3d(authoredPoint.X, Fixed64.Zero, Fixed64.Two));
        var localRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        // A half-turn is an exact signed permutation, so both authored and
        // cone frames keep precisely the same relative pose after composition.
        var halfTurn = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Vector3d expectedConeExit = default;
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        for (int pose = 0; pose < 3; pose++)
        {
            Vector3d translation = pose == 0 ? Vector3d.Zero
                : new Vector3d(Fixed64.FromRaw(1), (Fixed64)7 + Fixed64.FromRaw(3), (Fixed64)(-9));
            FixedQuaternion common = pose == 2 ? halfTurn : FixedQuaternion.Identity;
            FixedQuaternion authoredRotation = common * localRotation;
            Vector3d center = translation + (pose == 2 ? -Vector3d.Right : Vector3d.Right) * Fixed64.Half;
            var frame = new ConePlaneRayFrame(plane, translation, authoredRotation,
                center, common, (Fixed64)4, Fixed64.Two);
            var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
            var source = new ConePlaneRayEventSource(new FixedSegment(authoredPoint, authoredPoint));
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame,
                new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0), point, root, ref positive, ref negative));
            ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
            Assert.True(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, selected.Point, selected.Root, out Vector3d local));
            Assert.Equal(authoredPoint, local);
            Assert.True(ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(frame, selected.Point, selected.Root,
                ContactQuadratic.At(selected.Values, selected.Signs, 0, ConePlaneRaySelection.FieldWords),
                ContactQuadratic.At(selected.Values, selected.Signs, 1, ConePlaneRaySelection.FieldWords), orientation, out Vector3d coneExit));
            Assert.True(selected.TryMaterialize(frame, orientation, out _, out Vector3d worldExit, out _));
            if (pose == 0)
            {
                expectedConeExit = coneExit;
                Assert.Equal(worldExit - frame.ConeCenter, coneExit);
                Assert.NotEqual(new Vector3d(-Fixed64.Half, Fixed64.FromRaw(1), Fixed64.Half), coneExit);
            }
            else
            {
                Assert.Equal(expectedConeExit, coneExit);
            }
        }
    }

    [Fact]
    public void AuthoredLocalRange_IsIndependentOfRepresentableWorldAndConeCoordinates()
    {
        var plane = new FixedTriangle(new Vector3d(-2, 2, -2), new Vector3d(2, 2, -2), new Vector3d(0, 2, 2));
        var frame = new ConePlaneRayFrame(plane, new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame,
            new ConePlaneRayEvent(ConePlaneRayEventKind.Axis), point, root, ref positive, ref negative));
        Assert.False(ConePlaneRayPointMaterialization.TryGetAuthoredPoint(frame, point, root, out _));
        Assert.True(ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(frame, positive.Point, positive.Root,
            ContactQuadratic.At(positive.Values, positive.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(positive.Values, positive.Signs, 1, ConePlaneRaySelection.FieldWords), 1, out Vector3d conePoint));
        Assert.Equal(Vector3d.Up * 2, conePoint);
        Assert.True(ConePlaneRayPointMaterialization.TryGetWorldPoint(frame, point, root, out Vector3d worldPoint));
        Assert.Equal(frame.ConeCenter + Vector3d.Up * 2, worldPoint);
    }

    [Theory]
    [InlineData(true, 1, true)]
    [InlineData(true, 2, false)]
    [InlineData(true, 3, false)]
    [InlineData(false, 1, true)]
    [InlineData(false, 2, true)]
    [InlineData(false, 3, false)]
    public void ExitRange_UsesTheSameNearestEvenHalfRawThresholds(bool maximum, int quarterRaws, bool expected)
    {
        Fixed64 limit = Fixed64.FromRaw(maximum ? long.MaxValue : long.MinValue);
        Vector3d center = new(limit, Fixed64.Zero, Fixed64.Zero);
        var frame = Frame(center, FixedQuaternion.Identity);
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(values, signs); point.Set(frame.Transform(Vector3d.Zero));
        Span<ulong> fields = stackalloc ulong[4 * ConePlaneRayCharts.Words];
        Span<int> fieldSigns = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(fields, fieldSigns, 0, ConePlaneRayCharts.Words);
        ContactQuadratic d = ContactQuadratic.At(fields, fieldSigns, 1, ConePlaneRayCharts.Words);
        // Centered raw displacement is N*n/(2*ShapeDen*d). Choose exactly
        // quarterRaws/4 before world rounding, without authoring a rounded q.
        n.Set(Signed576.ExtendValue(WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Denominator, Signed192.Signed(quarterRaws))));
        n.Add(n);
        d.Set(Signed576.ExtendValue(frame.Normal.X)); d.Add(d); d.Add(d);
        int orientation = maximum ? 1 : -1;
        Assert.Equal(expected, ConePlaneRayPointMaterialization.IsExitWorldPointRepresentable(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, orientation));
        Assert.Equal(expected, ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, point,
            ReadOnlySpan<ulong>.Empty, n, d, orientation, out Vector3d q));
        if (expected) Assert.Equal(center, q);
    }

    [Theory]
    [InlineData(-1, -1, false)]
    [InlineData(-1, 1, false)]
    [InlineData(0, -1, false)]
    [InlineData(0, 1, false)]
    [InlineData(1, -1, false)]
    [InlineData(1, 1, false)]
    [InlineData(-1, -1, true)]
    [InlineData(-1, 1, true)]
    [InlineData(0, -1, true)]
    [InlineData(0, 1, true)]
    [InlineData(1, -1, true)]
    [InlineData(1, 1, true)]
    public void IrrationalExitRange_AgreesWithWorldMaterializationAtExtremeTranslations(int limit, int orientation, bool rotated)
    {
        var center = new Vector3d(Fixed64.FromRaw(limit < 0 ? long.MinValue : limit > 0 ? long.MaxValue : 0), Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion rotation = rotated ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5)
            : FixedQuaternion.Identity;
        var frame = Frame(center, rotation);
        Vector3d location = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Half);
        var source = new ConePlaneRayEventSource(new FixedSegment(location, location));
        var item = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref positive, ref negative));
        ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
        Assert.True(selected.HasValue);
        Assert.True(WideArithmetic.GetActiveMagnitudeLength(selected.Root) > 0);
        ContactQuadratic n = ContactQuadratic.At(selected.Values, selected.Signs, 0, ConePlaneRaySelection.FieldWords);
        ContactQuadratic d = ContactQuadratic.At(selected.Values, selected.Signs, 1, ConePlaneRaySelection.FieldWords);
        // Both rotations have a strictly positive world-X component of the
        // authored plane normal. Only the outward exit at either limit fails.
        bool expected = limit == 0 || limit != orientation;
        Assert.Equal(expected, ConePlaneRayPointMaterialization.IsExitWorldPointRepresentable(frame, selected.Point, selected.Root, n, d, orientation));
        Assert.Equal(expected, ConePlaneRayPointMaterialization.TryGetExitWorldPoint(frame, selected.Point, selected.Root, n, d, orientation, out Vector3d q));
        if (expected)
        {
            Assert.True(selected.TryMaterialize(frame, orientation, out _, out Vector3d paired, out _));
            Assert.Equal(paired, q);
            Assert.Equal(Fixed64.Half, q.Z);
            if (limit == 0 && !rotated) Assert.Equal(Fixed64.FromRaw(orientation * 3719550787L), q.X);
        }
    }

    private static ConePlaneRayFrame Frame(Vector3d center, FixedQuaternion rotation)
    {
        var plane = new FixedTriangle(new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
        return new ConePlaneRayFrame(plane, center, rotation, center, rotation, (Fixed64)4, Fixed64.Two);
    }
}
