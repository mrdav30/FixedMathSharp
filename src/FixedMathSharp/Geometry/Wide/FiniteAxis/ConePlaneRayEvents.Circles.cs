//=======================================================================
// ConePlaneRayEvents.Circles.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Generator-circle and projected upper-rim boundary events.</content>
internal static partial class ConePlaneRayEvents
{
    private static bool AccumulateCircles(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        if (!sink.IsStreaming)
            return AccumulateCircleChart(source, frame, sink.Target.Kind == ConePlaneRayEventKind.UpperRim,
                sink.Target.Feature, sink.Target.Quadrant, sink.Target.Branch, sink.Target.Branch, ref positive, ref negative, ref sink);
        bool axial = frame.Normal.X.IsZero && frame.Normal.Z.IsZero && !frame.Normal.Y.IsZero;
        bool found = false;
        // Match the compact inventories exactly, including seam ownership and
        // axial pruning. Each chart reuses its defining line for both roots;
        // generator endpoints reuse that same admitted circle parameter.
        for (int upper = source.Scope == ConePlaneRayEventScope.Segment ? 1 : 0; upper < 2; upper++)
        for (int quadrant = 0; quadrant < 4; quadrant++)
        {
            if (source.Scope == ConePlaneRayEventScope.Plane)
                for (int endpoint = 0; endpoint < 2; endpoint++)
                {
                    if (endpoint == 0 ? (quadrant & 2) != 0 : (quadrant & 1) != 0) continue;
                    if (!axial || upper == 0 && frame.Radius != Fixed64.Zero)
                        found |= AccumulateCircleChart(source, frame, upper != 0, 8, quadrant, endpoint, endpoint, ref positive, ref negative, ref sink);
                }
            if (axial && source.Scope == ConePlaneRayEventScope.Plane) continue;
            int lineCount = source.Scope == ConePlaneRayEventScope.Segment ? 1 : upper == 0 ? 6 : 8;
            for (int line = 0; line < lineCount; line++)
            {
                if (source.Scope == ConePlaneRayEventScope.Plane && (upper == 0 ? line >= 2 && line <= 4 : line < 3)) continue;
                found |= AccumulateCircleChart(source, frame, upper != 0, line, quadrant, -1, 1, ref positive, ref negative, ref sink);
            }
        }
        return found;
    }

    private static bool AccumulateCircleChart(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        bool upper, int line, int quadrant, int firstBranch, int lastBranch,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        if (source.Scope == ConePlaneRayEventScope.Segment && (!upper || line != 0)) return false;
        if (!upper && line == 8 && frame.Normal.X.IsZero && frame.Normal.Z.IsZero && !frame.Normal.Y.IsZero)
            return AccumulateAxialCardinal(source, frame, quadrant, firstBranch, ref positive, ref negative, ref sink);
        Span<ulong> values = stackalloc ulong[16 * Words], root = stackalloc ulong[RootWords];
        Span<int> signs = stackalloc int[16];
        ContactQuadratic x = At(values, signs, 0), z = At(values, signs, 1), constant = At(values, signs, 2);
        ContactQuadratic a = At(values, signs, 3), b = At(values, signs, 4), c = At(values, signs, 5);
        ContactQuadratic n = At(values, signs, 6), d = At(values, signs, 7);
        int sx = (quadrant & 1) == 0 ? 1 : -1, sz = (quadrant & 2) == 0 ? 1 : -1;
        ConePlaneRayEventKind kind = upper ? ConePlaneRayEventKind.UpperRim : ConePlaneRayEventKind.LowerCircle;
        if (line == 8)
        {
            // The cohort enumerator owns each cardinal seam exactly once.
            root.Clear(); n.Set(Integer(firstBranch)); d.Set(Integer(1));
            var seam = new ConePlaneRayEvent(kind, line, quadrant, firstBranch, firstBranch);
            return AccumulateCircleParameter(source, frame, upper, sx, sz, n, d, root, true,
                seam, ref positive, ref negative, ref sink);
        }
        GetCircleLine(source, frame, upper, line, x, z, constant);
        constant.CopyTo(a); a.Add(x, -sx); z.CopyTo(b, sz); constant.CopyTo(c); c.Add(x, sx);
        bool found = false;
        for (int branch = firstBranch; branch <= lastBranch; branch += 2)
        {
            if (!ConePlaneRayCharts.TryGetParameter(a, b, c, branch, root, n, d)
                || !ConePlaneRayCharts.IsUnitParameter(n, d, root)) continue;
            n.CopyTo(x); x.Add(d, -1);
            // A seam root already belongs to the intrinsic seam cohort.
            if (n.Sign(root) == 0 || x.Sign(root) == 0) continue;
            var descriptor = new ConePlaneRayEvent(kind, line, quadrant, branch);
            found |= AccumulateCircleParameter(source, frame, upper, sx, sz, n, d, root, line == 0,
                descriptor, ref positive, ref negative, ref sink);
        }
        return found;
    }

    private static bool AccumulateAxialCardinal(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        int quadrant, int branch, scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns); point.Set(default);
        Span<ulong> values = stackalloc ulong[2 * Words]; Span<int> signs = stackalloc int[2];
        ContactQuadratic height = At(values, signs, 0);
        // A primitive axial normal has Ny=+/-1, hence s=c*sign(Ny).
        // With positive homogeneous denominator Hraw, a cardinal generator
        // section is (X,Y,Z)=(+/-s*Rraw,s*Hraw,0), or its Z counterpart.
        // s<198 bits and raw dimensions<63 keep products below261 bits.
        // This cancels the base/frame scale exactly; the shared side owner
        // still checks finite admission and certifies both first exits.
        height.Set(frame.PlaneConstant); height.MultiplySign(frame.Normal.Y.Sign);
        Signed576 rawHeight = Extend(Signed320.ExtendValue(Signed192.Raw(frame.Height)));
        ContactQuadratic.Scale(height, rawHeight, point.Y);
        ContactQuadratic radial = branch == 0 ? point.X : point.Z;
        ContactQuadratic.Scale(height, Extend(Signed320.ExtendValue(Signed192.Raw(frame.Radius))), radial);
        radial.MultiplySign((quadrant & (branch == 0 ? 1 : 2)) == 0 ? 1 : -1);
        point.Denominator.Set(rawHeight);
        var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, quadrant, branch, branch);
        SetEvent(source, descriptor, ref positive, ref negative);
        bool admitted = ConePlaneRayPointExits.AccumulateSidePoint(source, frame, point,
            ReadOnlySpan<ulong>.Empty, ref positive, ref negative, sink.PointOnly);
        if (admitted) sink.Keep(descriptor, point, ReadOnlySpan<ulong>.Empty, positive, negative);
        return admitted;
    }

    // Returns x*cos(theta)+z*sin(theta)+constant. Scaling the symbolic line
    // before chart substitution keeps its defining coefficients below1248bits.
    private static void GetCircleLine(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        bool upper, int line, ContactQuadratic x, ContactQuadratic z, ContactQuadratic constant)
    {
        Span<ulong> values = stackalloc ulong[14 * Words];
        Span<int> signs = stackalloc int[14];
        ContactQuadratic lx = At(values, signs, 0), ly = At(values, signs, 1), lz = At(values, signs, 2);
        ContactQuadratic offset = At(values, signs, 3), A = At(values, signs, 4);
        ContactQuadratic term = At(values, signs, 5), scalar = At(values, signs, 6);
        lx.Clear(); ly.Clear(); lz.Clear(); offset.Clear();
        if (source.Scope == ConePlaneRayEventScope.Segment)
        {
            frame.GetEdgeLine(new FixedSegment(source.A, source.B), out Signed576 wx, out Signed576 wy,
                out Signed576 wz, out Signed576 w);
            lx.Set(wx); ly.Set(wy); lz.Set(wz);
            offset.Set(WideArithmetic.SubtractSigned576(default, w));
        }
        else if (line == (upper ? 7 : 5))
        {
            lx.Set(Extend(frame.Normal.Z)); lz.Set(Extend(WideArithmetic.Negate(frame.Normal.X)));
        }
        else if (upper && (line == 4 || line == 5))
        {
            scalar.Set(Extend(frame.Normal.Y));
            ContactQuadratic.Scale(scalar, Extend(frame.Normal.X), lx); lx.MultiplySign(-1);
            ContactQuadratic.Scale(scalar, Extend(frame.Normal.Y), ly); ly.MultiplySign(-1);
            ContactQuadratic.Scale(scalar, Extend(frame.Normal.Z), lz); lz.MultiplySign(-1);
            ContactQuadratic.Scale(scalar, frame.PlaneConstant, offset);
            if (line == 4)
            {
                scalar.Set(frame.NormalSquared);
                ContactQuadratic.Scale(scalar, Extend(frame.Finite.FullHeight), term); offset.Add(term);
            }
        }
        else if (upper && line == 6)
        {
            Product(frame.Normal, frame.Normal, frame, A);
            Signed576 h2 = Extend(WideArithmetic.MultiplySigned192(Signed192.Raw(frame.Height), Signed192.Raw(frame.Height)));
            Signed576 r2 = Extend(WideArithmetic.MultiplySigned192(Signed192.Raw(frame.Radius), Signed192.Raw(frame.Radius)));
            scalar.Set(frame.NormalSquared);
            ContactQuadratic.Scale(scalar, h2, term); term.Add(term);
            A.CopyTo(lx); lx.Add(term, -1); lx.CopyTo(lz);
            ContactQuadratic.Scale(scalar, r2, term); term.Add(term);
            A.CopyTo(ly); ly.Add(term);
            ContactQuadratic.Scale(lx, Extend(frame.Normal.X), term); term.CopyTo(lx);
            ContactQuadratic.Scale(ly, Extend(frame.Normal.Y), term); term.CopyTo(ly);
            ContactQuadratic.Scale(lz, Extend(frame.Normal.Z), term); term.CopyTo(lz);
            ContactQuadratic.Scale(A, frame.PlaneConstant, offset); offset.MultiplySign(-1);
        }
        else
        {
            lx.Set(Extend(frame.Normal.X)); ly.Set(Extend(frame.Normal.Y)); lz.Set(Extend(frame.Normal.Z));
            if (upper || line == 1)
                offset.Set(WideArithmetic.SubtractSigned576(default, frame.PlaneConstant));
        }
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius,
            frame.Finite.ShapeFrame.Denominator);
        ContactQuadratic.Scale(lx, Extend(radius), x);
        ContactQuadratic.Scale(lz, Extend(radius), z);
        ContactQuadratic.Scale(ly, Extend(frame.Finite.FullHeight), constant); constant.Add(offset);
    }

    private static bool AccumulateCircleParameter(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        bool upper, int sx, int sz, scoped ContactQuadratic n, scoped ContactQuadratic d, scoped ReadOnlySpan<ulong> root, bool retainGenerator,
        ConePlaneRayEvent descriptor, scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        Span<ulong> values = stackalloc ulong[20 * Words];
        Span<int> signs = stackalloc int[20];
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[8];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        ContactQuadratic nn = At(values, signs, 0), dd = At(values, signs, 1), gx = At(values, signs, 2);
        ContactQuadratic gy = At(values, signs, 3), gz = At(values, signs, 4), gd = At(values, signs, 5);
        ContactQuadratic dot = At(values, signs, 6), term = At(values, signs, 7);
        ContactQuadratic t = At(values, signs, 8), td = At(values, signs, 9);
        ContactQuadratic.Multiply(n, n, root, nn); ContactQuadratic.Multiply(d, d, root, dd);
        dd.CopyTo(gd); gd.Add(nn);
        dd.CopyTo(gx); gx.Add(nn, -1);
        Signed320 radius = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Radius,
            frame.Finite.ShapeFrame.Denominator);
        ContactQuadratic.Scale(gx, Extend(radius), term); term.CopyTo(gx, sx);
        ContactQuadratic.Scale(gd, Extend(frame.Finite.FullHeight), gy);
        ContactQuadratic.Multiply(n, d, root, gz); gz.Add(gz);
        ContactQuadratic.Scale(gz, Extend(radius), term); term.CopyTo(gz, sz);
        ContactQuadratic.Scale(gx, Extend(frame.Normal.X), dot);
        ContactQuadratic.Scale(gy, Extend(frame.Normal.Y), term); dot.Add(term);
        ContactQuadratic.Scale(gz, Extend(frame.Normal.Z), term); dot.Add(term);
        if (!upper)
        {
            int divisorSign = dot.Sign(root);
            if (divisorSign == 0)
                return retainGenerator && frame.PlaneConstant.IsZero && AccumulateGenerator(source, frame, gx, gy, gz, gd, root,
                    descriptor, ref positive, ref negative, ref sink);
            if (!sink.Wants(descriptor)) return false;
            ContactQuadratic.Scale(gx, frame.PlaneConstant, point.X); point.X.MultiplySign(divisorSign);
            ContactQuadratic.Scale(gy, frame.PlaneConstant, point.Y); point.Y.MultiplySign(divisorSign);
            ContactQuadratic.Scale(gz, frame.PlaneConstant, point.Z); point.Z.MultiplySign(divisorSign);
            dot.CopyTo(point.Denominator, divisorSign);
            SetEvent(source, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateSidePoint(source, frame, point, root, ref positive, ref negative, sink.PointOnly);
            if (admitted) sink.Keep(descriptor, point, root, positive, negative);
            return admitted;
        }
        dot.CopyTo(t); ContactQuadratic.Scale(gd, frame.PlaneConstant, term); t.Add(term, -1);
        ContactQuadratic.Scale(gd, frame.NormalSquared, td);
        ContactQuadratic.Scale(gx, frame.NormalSquared, point.X);
        ContactQuadratic.Scale(t, Extend(frame.Normal.X), term); point.X.Add(term, -1);
        ContactQuadratic.Scale(gy, frame.NormalSquared, point.Y);
        ContactQuadratic.Scale(t, Extend(frame.Normal.Y), term); point.Y.Add(term, -1);
        ContactQuadratic.Scale(gz, frame.NormalSquared, point.Z);
        ContactQuadratic.Scale(t, Extend(frame.Normal.Z), term); point.Z.Add(term, -1);
        td.CopyTo(point.Denominator);
        int orientation = t.Sign(root);
        descriptor = new ConePlaneRayEvent(descriptor.Kind, descriptor.Feature, descriptor.Quadrant,
            descriptor.Branch, descriptor.Endpoint, orientation);
        SetEvent(source, descriptor, ref positive, ref negative);
        if (orientation == 0)
        {
            bool sideAdmitted = ConePlaneRayPointExits.AccumulateSidePoint(source, frame, point, root, ref positive, ref negative, sink.PointOnly);
            if (sideAdmitted) sink.Keep(descriptor, point, root, positive, negative);
            return sideAdmitted;
        }
        t.MultiplySign(orientation);
        bool certified = ConePlaneRayPointExits.AccumulateCertifiedExit(source, frame, point, root, t, td,
            orientation, ref positive, ref negative, sink.PointOnly);
        if (certified) sink.Keep(descriptor, point, root, positive, negative);
        return certified;
    }

    private static bool AccumulateGenerator(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ContactQuadratic gx, scoped ContactQuadratic gy, scoped ContactQuadratic gz, scoped ContactQuadratic gd,
        scoped ReadOnlySpan<ulong> root, ConePlaneRayEvent generator,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        if (!sink.Wants(ConePlaneRayEventKind.Generator) || !sink.IsStreaming && (uint)sink.Target.Endpoint > 1) return false;
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        // Only the cone's generator endpoints are intrinsic. Its intersection
        // with a finite boundary is already that segment's side root/endpoint.
        for (int endpoint = sink.IsStreaming ? 0 : sink.Target.Endpoint; endpoint <= (sink.IsStreaming ? 1 : sink.Target.Endpoint); endpoint++)
        {
            if (endpoint == 0) point.Set(default);
            else { gx.CopyTo(point.X); gy.CopyTo(point.Y); gz.CopyTo(point.Z); gd.CopyTo(point.Denominator); }
            var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.Generator, generator.Feature,
                generator.Quadrant, generator.Branch, endpoint);
            SetEvent(source, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateSidePoint(source, frame, point, root, ref positive, ref negative, sink.PointOnly);
            // D=N.g=0 and c=0 already certify this intrinsic generator's plane.
            // Its apex has denominator 1; its base-rim endpoint has denominator
            // d*d+n*n>0 and satisfies the finite cone exactly, including R=0.
            System.Diagnostics.Debug.Assert(admitted);
            sink.Keep(descriptor, point, root, positive, negative);
        }
        return true;
    }
}
