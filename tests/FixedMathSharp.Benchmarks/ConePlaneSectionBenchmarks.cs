//=======================================================================
// ConePlaneSectionBenchmarks.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

/// <summary>Complete finite plane events, retained descriptors and paired winner reconstruction.</summary>
[MemoryDiagnoser]
public class ConePlaneSectionBenchmarks
{
    private FixedTriangle _first, _second;
    private ConePlaneRayFrame _frame;
    private bool _hasSecond;

    [Params("Interior", "Tilted1000To1", "SideBaseSwitch")]
    public string Scenario { get; set; } = "Interior";

    [GlobalSetup]
    public void Setup()
    {
        _hasSecond = Scenario == "Tilted1000To1";
        FixedQuaternion rotation = FixedQuaternion.Identity;
        Fixed64 height = (Fixed64)4, radius = Fixed64.Two;
        if (_hasSecond)
        {
            _first = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(2, 0, 2));
            _second = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, 2), new Vector3d(-2, 0, 2));
            rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
            height = (Fixed64)1000; radius = Fixed64.One;
        }
        else if (Scenario == "SideBaseSwitch")
            _first = new FixedTriangle(new Vector3d(3, -3, -3), new Vector3d(3, -3, 3), new Vector3d(-3, 3, 0));
        else if (Scenario == "Interior")
            _first = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        else
            throw new InvalidOperationException("Unknown finite plane benchmark fixture.");
        _frame = new ConePlaneRayFrame(_first, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, rotation, height, radius);

        var result = AccumulateAndMaterialize();
        bool valid = result.Found;
        if (_hasSecond)
            valid &= result.Positive >= Fixed64.One && result.Positive < result.Negative
                && result.Negative <= (Fixed64)11 / 10;
        else if (Scenario == "SideBaseSwitch")
            valid &= result.Positive.m_rawValue == 5399112000L && result.Negative.m_rawValue == 12148002000L;
        else
            valid &= result.Positive == Fixed64.Two && result.Negative == Fixed64.Two;
        if (!valid)
            throw new InvalidOperationException("The finite plane benchmark did not produce its expected admitted ray maxima.");
    }

    [Benchmark]
    public (bool Found, Fixed64 Positive, Fixed64 Negative, Vector3d Lower, Vector3d Upper) AccumulateAndMaterialize()
    {
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEventSink.Capacity];
        var sink = new ConePlaneRayEventSink(events);
        bool found = ConePlaneRayEvents.Accumulate(
            _first, _frame, ref positive, ref negative, ref sink);
        if (_hasSecond)
            found &= ConePlaneRayEvents.Accumulate(
                _second, _frame, ref positive, ref negative, ref sink);
        bool materialized = ConePlaneRayEvents.TryMaterializeEvent(positive.MaximumTriangle, _frame,
            positive.MaximumEvent, 1, out Vector3d lower, out Vector3d upper, out Fixed64 positiveDepth);
        bool negativeRounded = negative.TryGetRoundedMaximumDepth(out Fixed64 negativeDepth);
        return (found && materialized && negativeRounded, positiveDepth, negativeDepth, lower, upper);
    }
}
