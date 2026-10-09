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
    private static bool AccumulateCircles(FixedTriangle triangle, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> signs = stackalloc int[16];
        Span<ulong> root = stackalloc ulong[RootWords];
        ContactQuadratic x = At(values, signs, 0), z = At(values, signs, 1), constant = At(values, signs, 2);
        ContactQuadratic a = At(values, signs, 3), b = At(values, signs, 4), c = At(values, signs, 5);
        ContactQuadratic n = At(values, signs, 6), d = At(values, signs, 7);
        bool found = false;
        for (int upper = 0; upper < 2; upper++)
        for (int quadrant = 0; quadrant < 4; quadrant++)
        {
            ConePlaneRayEventKind kind = upper == 0 ? ConePlaneRayEventKind.LowerCircle : ConePlaneRayEventKind.UpperRim;
            if (sink.IsReconstruction && (sink.Target.Quadrant != quadrant
                || sink.Target.Kind != kind && !(upper == 0 && sink.Target.Kind == ConePlaneRayEventKind.Generator)))
                continue;
            int sx = (quadrant & 1) == 0 ? 1 : -1, sz = (quadrant & 2) == 0 ? 1 : -1;
            // Closed chart endpoints also represent a retained zero-polynomial
            // family. Every nonempty clipped arc has an endpoint among these
            // seams or a nonzero clipping polynomial's roots.
            for (int endpoint = 0; endpoint < 2; endpoint++)
            {
                // Each cardinal seam is owned once. Polynomial roots on a
                // seam are skipped below because this event already owns it.
                if (endpoint == 0 && (quadrant & 2) != 0 || endpoint == 1 && (quadrant & 1) != 0)
                    continue;
                var descriptor = new ConePlaneRayEvent(kind, 8, quadrant, endpoint, endpoint);
                if (sink.IsReconstruction && (sink.Target.Feature != 8 || sink.Target.Branch != endpoint))
                    continue;
                root.Clear(); n.Set(Integer(endpoint)); d.Set(Integer(1));
                found |= AccumulateCircleParameter(triangle, frame, upper != 0, sx, sz, n, d, root, true,
                    descriptor, ref positive, ref negative, ref sink);
            }
            for (int line = 0; line < (upper == 0 ? 6 : 8); line++)
            {
                if (sink.IsReconstruction && sink.Target.Feature != line) continue;
                GetCircleLine(triangle, frame, upper != 0, line, x, z, constant);
                constant.CopyTo(a); a.Add(x, -sx);
                z.CopyTo(b, sz);
                constant.CopyTo(c); c.Add(x, sx);
                for (int branch = -1; branch <= 1; branch += 2)
                {
                    if (sink.IsReconstruction && sink.Target.Branch != branch) continue;
                    if (!ConePlaneRayCharts.TryGetParameter(a, b, c, branch, root, n, d)
                        || !ConePlaneRayCharts.IsUnitParameter(n, d, root))
                        continue;
                    n.CopyTo(x); x.Add(d, -1);
                    if (n.Sign(root) == 0 || x.Sign(root) == 0) continue;
                    var descriptor = new ConePlaneRayEvent(kind, line, quadrant, branch);
                    found |= AccumulateCircleParameter(triangle, frame, upper != 0, sx, sz, n, d, root, line == 0,
                        descriptor, ref positive, ref negative, ref sink);
                }
            }
        }
        return found;
    }

    // Returns x*cos(theta)+z*sin(theta)+constant. Scaling the symbolic line
    // before chart substitution keeps its defining coefficients below1248bits.
    private static void GetCircleLine(FixedTriangle triangle, in ConePlaneRayFrame frame,
        bool upper, int line, ContactQuadratic x, ContactQuadratic z, ContactQuadratic constant)
    {
        Span<ulong> values = stackalloc ulong[14 * Words];
        Span<int> signs = stackalloc int[14];
        ContactQuadratic lx = At(values, signs, 0), ly = At(values, signs, 1), lz = At(values, signs, 2);
        ContactQuadratic offset = At(values, signs, 3), A = At(values, signs, 4);
        ContactQuadratic term = At(values, signs, 5), scalar = At(values, signs, 6);
        lx.Clear(); ly.Clear(); lz.Clear(); offset.Clear();
        int wall = upper ? line < 3 ? line : -1 : line >= 2 && line <= 4 ? line - 2 : -1;
        if (wall >= 0)
        {
            frame.GetTriangleEdgeWall(triangle, wall, out Signed576 wx, out Signed576 wy,
                out Signed576 wz, out Signed576 w);
            lx.Set(wx); ly.Set(wy); lz.Set(wz);
            if (upper)
                offset.Set(WideArithmetic.SubtractSigned576(default, w));
            else
            {
                ContactQuadratic.Scale(lx, frame.PlaneConstant, term); term.CopyTo(lx);
                ContactQuadratic.Scale(ly, frame.PlaneConstant, term); term.CopyTo(ly);
                ContactQuadratic.Scale(lz, frame.PlaneConstant, term); term.CopyTo(lz);
                scalar.Set(w);
                ContactQuadratic.Scale(scalar, Extend(frame.Normal.X), term); lx.Add(term, -1);
                ContactQuadratic.Scale(scalar, Extend(frame.Normal.Y), term); ly.Add(term, -1);
                ContactQuadratic.Scale(scalar, Extend(frame.Normal.Z), term); lz.Add(term, -1);
            }
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

    private static bool AccumulateCircleParameter(FixedTriangle triangle, in ConePlaneRayFrame frame,
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
                return retainGenerator && frame.PlaneConstant.IsZero && AccumulateGenerator(triangle, frame, gx, gy, gz, gd, root,
                    descriptor, ref positive, ref negative, ref sink);
            if (!sink.Wants(descriptor)) return false;
            ContactQuadratic.Scale(gx, frame.PlaneConstant, point.X); point.X.MultiplySign(divisorSign);
            ContactQuadratic.Scale(gy, frame.PlaneConstant, point.Y); point.Y.MultiplySign(divisorSign);
            ContactQuadratic.Scale(gz, frame.PlaneConstant, point.Z); point.Z.MultiplySign(divisorSign);
            dot.CopyTo(point.Denominator, divisorSign);
            SetEvent(triangle, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateSidePoint(triangle, frame, point, root, ref positive, ref negative);
            if (admitted) sink.Keep(descriptor, frame, point, root);
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
        SetEvent(triangle, descriptor, ref positive, ref negative);
        if (orientation == 0)
        {
            bool sideAdmitted = ConePlaneRayPointExits.AccumulateSidePoint(triangle, frame, point, root, ref positive, ref negative);
            if (sideAdmitted) sink.Keep(descriptor, frame, point, root);
            return sideAdmitted;
        }
        t.MultiplySign(orientation);
        bool certified = ConePlaneRayPointExits.AccumulateCertifiedExit(triangle, frame, point, root, t, td,
            orientation, ref positive, ref negative);
        if (certified) sink.Keep(descriptor, frame, point, root);
        return certified;
    }

    private static bool AccumulateGenerator(FixedTriangle triangle, in ConePlaneRayFrame frame,
        scoped ContactQuadratic gx, scoped ContactQuadratic gy, scoped ContactQuadratic gz, scoped ContactQuadratic gd,
        scoped ReadOnlySpan<ulong> root, ConePlaneRayEvent source,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        Span<ulong> values = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[8];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        ContactQuadratic n = At(values, signs, 0), d = At(values, signs, 1), term = At(values, signs, 2);
        bool found = false;
        // The section contains a whole finite generator. Depth on it is the
        // minimum of affine side/cap exits. Its switch is an upper-rim event;
        // all other extrema are endpoints of the clipped generator interval.
        for (int endpoint = 0; endpoint < 5; endpoint++)
        {
            var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.Generator, source.Feature,
                source.Quadrant, source.Branch, endpoint);
            if (!sink.Wants(descriptor)) continue;
            if (endpoint < 2)
            {
                n.Set(Integer(endpoint)); d.Set(Integer(1));
            }
            else
            {
                frame.GetTriangleEdgeWall(triangle, endpoint - 2, out Signed576 wx,
                    out Signed576 wy, out Signed576 wz, out Signed576 offset);
                ContactQuadratic.Scale(gd, offset, n);
                ContactQuadratic.Scale(gx, wx, d);
                ContactQuadratic.Scale(gy, wy, term); d.Add(term);
                ContactQuadratic.Scale(gz, wz, term); d.Add(term);
                int sign = d.Sign(root);
                if (sign == 0)
                    continue;
                n.MultiplySign(sign); d.MultiplySign(sign);
            }
            if (!ConePlaneRayCharts.IsUnitParameter(n, d, root))
                continue;
            ContactQuadratic.Multiply(gx, n, root, point.X);
            ContactQuadratic.Multiply(gy, n, root, point.Y);
            ContactQuadratic.Multiply(gz, n, root, point.Z);
            ContactQuadratic.Multiply(gd, d, root, point.Denominator);
            SetEvent(triangle, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateSidePoint(triangle, frame, point, root, ref positive, ref negative);
            if (admitted) sink.Keep(descriptor, frame, point, root);
            found |= admitted;
        }
        return found;
    }
}
