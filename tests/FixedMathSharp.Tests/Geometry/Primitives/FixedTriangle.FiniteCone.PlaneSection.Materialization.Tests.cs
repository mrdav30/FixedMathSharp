//=======================================================================
// FixedTriangle.FiniteCone.PlaneSection.Materialization.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ConePlaneRayMaterializationTests
{
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
