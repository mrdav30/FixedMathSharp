//=======================================================================
// ConePlaneRayEvents.Special.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Apex, axial support continua, and the lower base-line stationary stratum.</content>
internal static partial class ConePlaneRayEvents
{
    private static bool AccumulateAxisAndApex(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        Span<ulong> values = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[8];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        ContactQuadratic n = At(values, signs, 0), d = At(values, signs, 1), term = At(values, signs, 2);
        point.X.Clear(); point.Z.Clear();
        bool found = false;
        if (sink.Wants(ConePlaneRayEventKind.Axis))
        {
            ConePlaneRayEvent descriptor;
            if (!frame.Normal.Y.IsZero)
            {
                descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis);
                if (!sink.Wants(descriptor)) return false;
                point.Y.Set(frame.PlaneConstant); point.Y.MultiplySign(frame.Normal.Y.Sign);
                point.Denominator.Set(Extend(frame.Normal.Y)); point.Denominator.MultiplySign(frame.Normal.Y.Sign);
                SetEvent(source, descriptor, ref positive, ref negative);
                bool admitted = ConePlaneRayPointExits.AccumulateRationalPoint(source, frame, point, ref positive, ref negative, sink.PointOnly);
                if (admitted) sink.Keep(descriptor, point, ReadOnlySpan<ulong>.Empty, positive, negative);
                found |= admitted;
            }
            else
            {
                if (frame.PlaneConstant.IsZero && (sink.IsStreaming || (uint)sink.Target.Endpoint <= 1))
                    for (int endpoint = sink.IsStreaming ? 0 : sink.Target.Endpoint; endpoint <= (sink.IsStreaming ? 1 : sink.Target.Endpoint); endpoint++)
                    {
                        descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis, endpoint: endpoint);
                        point.Y.Set(endpoint == 0 ? default : Extend(frame.Finite.FullHeight));
                        point.Denominator.Set(Integer(1));
                        SetEvent(source, descriptor, ref positive, ref negative);
                        bool admitted = ConePlaneRayPointExits.AccumulateRationalPoint(source, frame, point, ref positive, ref negative, sink.PointOnly);
                        // Ny=0 and c=0 contain the whole axis. Its two
                        // finite endpoints satisfy this plane and solid cone
                        // exactly, including radius zero; admission cannot fail.
                        System.Diagnostics.Debug.Assert(admitted);
                        sink.Keep(descriptor, point, ReadOnlySpan<ulong>.Empty, positive, negative);
                        found = true;
                    }
            }
            if (!sink.IsStreaming) return found;
        }
        if (!sink.Wants(ConePlaneRayEventKind.Apex)) return found;
        // Project the apex onto the shared plane. Zero depth is represented by
        // the axis event; a nonzero certified exit retains its orientation.
        n.Set(frame.PlaneConstant); d.Set(frame.NormalSquared);
        ContactQuadratic.Scale(n, Extend(frame.Normal.X), point.X);
        ContactQuadratic.Scale(n, Extend(frame.Normal.Y), point.Y);
        ContactQuadratic.Scale(n, Extend(frame.Normal.Z), point.Z);
        d.CopyTo(point.Denominator);
        int orientation = -frame.PlaneConstant.Sign;
        if (orientation != 0)
        {
            var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.Apex, orientation: orientation);
            n.CopyTo(term, -orientation);
            SetEvent(source, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateCertifiedExit(source, frame, point,
                ReadOnlySpan<ulong>.Empty, term, d, orientation, ref positive, ref negative, sink.PointOnly);
            if (admitted) sink.Keep(descriptor, point, ReadOnlySpan<ulong>.Empty, positive, negative);
            return found | admitted;
        }
        return found;
    }

    private static bool AccumulateBaseStationary(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        Signed576 radialSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(frame.Normal.X, frame.Normal.X),
            WideArithmetic.MultiplySigned320(frame.Normal.Z, frame.Normal.Z));
        if (radialSquared.IsZero)
            return false;
        Span<ulong> values = stackalloc ulong[22 * Words];
        Span<int> signs = stackalloc int[22];
        Span<ulong> root = stackalloc ulong[RootWords];
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[8];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        ContactQuadratic k = At(values, signs, 0), L = At(values, signs, 1), A = At(values, signs, 2);
        ContactQuadratic a = At(values, signs, 3), b = At(values, signs, 4), c = At(values, signs, 5);
        ContactQuadratic term = At(values, signs, 6), work = At(values, signs, 7), t = At(values, signs, 8);
        ContactQuadratic d = At(values, signs, 9), depth = At(values, signs, 10);
        Signed576 h2 = Extend(WideArithmetic.MultiplySigned192(Signed192.Raw(frame.Height), Signed192.Raw(frame.Height)));
        Signed576 r2 = Extend(WideArithmetic.MultiplySigned192(Signed192.Raw(frame.Radius), Signed192.Raw(frame.Radius)));
        k.Set(frame.PlaneConstant);
        term.Set(Extend(frame.Normal.Y));
        ContactQuadratic.Scale(term, Extend(frame.Finite.FullHeight), work); k.Add(work, -1);
        L.Set(radialSquared);
        ContactQuadratic.Scale(k, Extend(frame.Normal.X), point.X);
        ContactQuadratic.Scale(L, Extend(frame.Finite.FullHeight), point.Y);
        ContactQuadratic.Scale(k, Extend(frame.Normal.Z), point.Z);
        L.CopyTo(point.Denominator);
        // On the lower-base line, the stationary upper-side endpoint has
        // radial q parallel to N's radial projection. Thus its lower endpoint
        // is the line's closest point to the axis, k*Nrad/L. Substitution gives
        // h²(k+tL)²-r²L(H+tNy)²=0 without expanding a rational discriminant.
        // The defining coefficients remain <1184bits, root <2370bits.
        Product(frame.Normal, frame.Normal, frame, A);
        ContactQuadratic.Multiply(L, A, ReadOnlySpan<ulong>.Empty, a);
        ContactQuadratic.Scale(k, h2, b);
        term.Set(Extend(frame.Finite.FullHeight));
        ContactQuadratic.Scale(term, Extend(frame.Normal.Y), work);
        ContactQuadratic.Scale(work, r2, term); b.Add(term, -1);
        ContactQuadratic.Multiply(b, L, ReadOnlySpan<ulong>.Empty, work); work.CopyTo(b);
        ContactQuadratic.Multiply(k, k, ReadOnlySpan<ulong>.Empty, work);
        ContactQuadratic.Scale(work, h2, c);
        ContactQuadratic.Scale(L, Extend(frame.Finite.FullHeight), term);
        ContactQuadratic.Scale(term, Extend(frame.Finite.FullHeight), work);
        ContactQuadratic.Scale(work, r2, term); c.Add(term, -1);
        bool found = false;
        for (int branch = sink.IsStreaming ? -1 : sink.Target.Branch; branch <= (sink.IsStreaming ? 1 : sink.Target.Branch); branch += 2)
        {
            if (!ConePlaneRayCharts.TryGetParameter(a, b, c, branch, root, t, d)) continue;
            int orientation = t.Sign(root) < 0 ? -1 : 1;
            var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.BaseStationary, branch: branch, orientation: orientation);
            t.CopyTo(depth, orientation);
            SetEvent(source, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateStationarySideExit(source, frame, point, root,
                depth, d, orientation, ref positive, ref negative, sink.PointOnly);
            if (admitted) sink.Keep(descriptor, point, root, positive, negative);
            found |= admitted;
        }
        return found;
    }
}
