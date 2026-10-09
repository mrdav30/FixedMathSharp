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
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> candidatePositive = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> candidateNegative = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> candidatePositiveSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> candidateNegativeSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        var p = new ConePlaneRaySelection(candidatePositive, candidatePositiveSigns);
        var n = new ConePlaneRaySelection(candidateNegative, candidateNegativeSigns);
        bool found = false;
        for (int domain = -1; domain < (_hasSecond ? 6 : 3); domain++)
        {
            ConePlaneRayEventSource source = domain < 0 ? ConePlaneRayEventSource.Plane
                : new ConePlaneRayEventSource(domain < 3 ? _first.GetEdge(domain) : _second.GetEdge(domain - 3));
            int count = domain < 0 ? ConePlaneRayEvents.GetIntrinsicEvents(_frame, events) : ConePlaneRayEvents.GetBoundaryEvents(events);
            foreach (ConePlaneRayEvent item in events[..count])
            {
                if (!ConePlaneRayEvents.TryEvaluateEvent(source, _frame, item, point, root, ref p, ref n)
                    || !(ConePlaneRayPointExits.ContainsTrianglePoint(_first, _frame, point, root)
                        || _hasSecond && ConePlaneRayPointExits.ContainsTrianglePoint(_second, _frame, point, root))) continue;
                found = true;
                positive.KeepEvaluated(p, _frame); negative.KeepEvaluated(n, _frame);
            }
        }
        bool materialized = positive.TryMaterialize(_frame, 1,
            out Vector3d lower, out Vector3d upper, out Fixed64 positiveDepth);
        bool negativeRounded = negative.TryGetRoundedMaximumDepth(out Fixed64 negativeDepth);
        return (found && materialized && negativeRounded, positiveDepth, negativeDepth, lower, upper);
    }
}
