//=======================================================================
// ConePlaneRayPointExits.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact closed admission and first finite exits of retained plane-section points.</summary>
internal static class ConePlaneRayPointExits
{
    private const int Words = ConePlaneRayPoint.FieldWords;
    private const int AdmissionWords = 144;
    private const int VerificationWords = 176;

    // A point certificate stores four homogeneous quadratic-field integers;
    // their rational/radical coefficients are <3072 bits, root <2498 bits.
    // One cone product needs <2*3072+2498+126+3=8771 bits (144 words).
    // Dot/axial/wall predicates need <3072+526+3=3601 bits (64 words).
    // Depth fields are <4096 bits. Stationary exit verification multiplies
    // one depth, one point, A<650 and the shared root; <10320 bits, 176 words.
    // These are transient products, never retained as point/depth fields.

    /// <summary>Tests closed shared-plane and finite-source admission in the cone.</summary>
    /// <remarks>A side certificate proves F(point)=0 exactly in the supplied root; only that solid predicate may then be skipped.</remarks>
    internal static bool ContainsPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root, bool sideCertificate = false)
    {
        if (frame.NormalSquared.IsZero || point.Denominator.Sign(root) <= 0)
            return false;
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic value = ContactQuadratic.At(work, signs, 0, Words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 1, Words);
        Dot(point, Signed576.ExtendValue(frame.Normal.X), Signed576.ExtendValue(frame.Normal.Y),
            Signed576.ExtendValue(frame.Normal.Z), value, term);
        ContactQuadratic.Scale(point.Denominator, frame.PlaneConstant, term); value.Add(term, -1);
        if (value.Sign(root) != 0 || point.Y.Sign(root) < 0)
            return false;
        ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(frame.Finite.FullHeight), value);
        value.Add(point.Y, -1);
        if (value.Sign(root) < 0)
            return false;
        if (!sideCertificate)
        {
            Span<ulong> coneWork = stackalloc ulong[2 * AdmissionWords];
            Span<int> coneSigns = stackalloc int[2];
            ContactQuadratic cone = ContactQuadratic.At(coneWork, coneSigns, 0, AdmissionWords);
            ConeProduct(point, point, frame, root, cone);
            if (cone.Sign(root) > 0) return false;
        }
        if (source.Scope == ConePlaneRayEventScope.Plane) return true;
        frame.GetEdgeLine(new FixedSegment(source.A, source.B), out Signed576 wx, out Signed576 wy, out Signed576 wz, out Signed576 offset);
        Dot(point, wx, wy, wz, value, term);
        ContactQuadratic.Scale(point.Denominator, offset, term); value.Add(term, -1);
        if (value.Sign(root) != 0) return false;
        WideAxis3 start = frame.Transform(source.A), end = frame.Transform(source.B);
        WideAxis3 delta = new(WideArithmetic.SubtractSigned320(end.X, start.X),
            WideArithmetic.SubtractSigned320(end.Y, start.Y), WideArithmetic.SubtractSigned320(end.Z, start.Z));
        if (delta.IsZero)
        {
            // A point segment has no line or projection direction; its
            // finite domain is exact homogeneous coordinate equality.
            for (int axis = 0; axis < 3; axis++)
            {
                (axis == 0 ? point.X : axis == 1 ? point.Y : point.Z).CopyTo(value);
                ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(axis == 0 ? start.X : axis == 1 ? start.Y : start.Z), term);
                value.Add(term, -1);
                if (value.Sign(root) != 0) return false;
            }
            return true;
        }
        // Point coefficients <3072 and transformed edge <198 keep the
        // complete projection and squared-edge bounds <3472 bits (64 words).
        Dot(point, Signed576.ExtendValue(delta.X), Signed576.ExtendValue(delta.Y), Signed576.ExtendValue(delta.Z), value, term);
        ContactQuadratic.Scale(point.Denominator, WideAxis3.Dot(start, delta), term); value.Add(term, -1);
        if (value.Sign(root) < 0) return false;
        ContactQuadratic.Scale(point.Denominator, delta.SquaredLength, term); term.Add(value, -1);
        return term.Sign(root) >= 0;
    }

    /// <summary>Tests only the closed triangle walls of an already certified finite-cone point in the same plane.</summary>
    internal static bool ContainsTrianglePoint(FixedTriangle triangle, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root)
    {
        triangle.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        if (nx.IsZero && ny.IsZero && nz.IsZero) return false;
        Span<ulong> work = stackalloc ulong[4 * Words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic value = ContactQuadratic.At(work, signs, 0, Words), term = ContactQuadratic.At(work, signs, 1, Words);
        for (int edge = 0; edge < 3; edge++)
        {
            frame.GetTriangleEdgeWall(triangle, edge, out Signed576 x, out Signed576 y, out Signed576 z, out Signed576 offset);
            Dot(point, x, y, z, value, term);
            ContactQuadratic.Scale(point.Denominator, offset, term); value.Add(term, -1);
            if (value.Sign(root) < 0) return false;
        }
        return true;
    }

    internal static bool AccumulateRationalPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        bool pointOnly = false, int requestedOrientation = 0)
    {
        Span<ulong> root = stackalloc ulong[ConePlaneRaySelection.RootWords]; root.Clear();
        if (!ContainsPoint(source, frame, point, root))
            return false;
        if (pointOnly) return true;
        // Only vertices, rational edge endpoints and the axis-plane point use
        // this path: coordinate numerators <526, denominator <326. Then the
        // ray equation A<1302, B<1241, C<1181 has discriminant <2485,
        // and its root fits the 40-word chart.
        if (requestedOrientation >= 0) AccumulateRationalOrientation(point, frame, 1, ref positive);
        if (requestedOrientation <= 0) AccumulateRationalOrientation(point, frame, -1, ref negative);
        return true;
    }

    internal static bool AccumulateSidePoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        bool pointOnly = false, int requestedOrientation = 0)
    {
        if (!ContainsPoint(source, frame, point, root, sideCertificate: true))
            return false;
        if (pointOnly) return true;
        // The chart certifies F(point)=0. The nonzero crossing -2B/A is
        // quadratic-field linear; solving a second quadratic would introduce
        // a needless nested radical. Inward zero crossings are rejected.
        if (requestedOrientation >= 0) AccumulateSideOrientation(point, frame, root, 1, ref positive);
        if (requestedOrientation <= 0) AccumulateSideOrientation(point, frame, root, -1, ref negative);
        return true;
    }

    internal static bool AccumulateCertifiedExit(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, int orientation,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        bool pointOnly = false, bool sideCertificate = false)
    {
        if (!ContainsPoint(source, frame, point, root, sideCertificate) || numerator.Sign(root) < 0 || denominator.Sign(root) <= 0)
            return false;
        // The constructing chart certifies an upper support/base/rim point
        // and its first-exit branch. A side certificate skips only F(point);
        // closed plane/height/source admission and exit signs remain checked.
        if (pointOnly) return true;
        if (orientation > 0) positive.Keep(numerator, denominator, root, frame, point);
        else negative.Keep(numerator, denominator, root, frame, point);
        return true;
    }

    internal static bool AccumulateStationarySideExit(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, int orientation,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative, bool pointOnly = false)
    {
        if (!ContainsPoint(source, frame, point, root) || numerator.Sign(root) < 0 || denominator.Sign(root) <= 0)
            return false;
        Span<ulong> work = stackalloc ulong[10 * VerificationWords];
        Span<int> signs = stackalloc int[10];
        ContactQuadratic y = ContactQuadratic.At(work, signs, 0, VerificationWords);
        ContactQuadratic d = ContactQuadratic.At(work, signs, 1, VerificationWords);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 2, VerificationWords);
        ContactQuadratic derivative = ContactQuadratic.At(work, signs, 3, VerificationWords);
        ContactQuadratic a = ContactQuadratic.At(work, signs, 4, VerificationWords);
        ContactQuadratic.Multiply(point.Y, denominator, root, y);
        ContactQuadratic.Multiply(point.Denominator, numerator, root, term);
        ContactQuadratic.Scale(term, Signed576.ExtendValue(frame.Normal.Y), d); d.MultiplySign(orientation); y.Add(d);
        ContactQuadratic.Multiply(point.Denominator, denominator, root, d);
        if (y.Sign(root) < 0)
            return false;
        ContactQuadratic.Scale(d, Signed576.ExtendValue(frame.Finite.FullHeight), term); term.Add(y, -1);
        if (term.Sign(root) < 0)
            return false;
        GetRayCoefficients(point, frame, root, a, derivative);
        derivative.MultiplySign(orientation);
        ContactQuadratic.Multiply(derivative, denominator, root, y);
        ContactQuadratic.Multiply(a, numerator, root, d);
        ContactQuadratic.Multiply(d, point.Denominator, root, term); y.Add(term);
        // F(q)=0 is certified by the stationary elimination polynomial. A
        // nonnegative derivative selects the exit rather than opposite nappe.
        if (y.Sign(root) < 0 || y.Sign(root) == 0 && a.Sign(root) < 0)
            return false;
        if (pointOnly) return true;
        if (orientation > 0) positive.Keep(numerator, denominator, root, frame, point);
        else negative.Keep(numerator, denominator, root, frame, point);
        return true;
    }

    private static void Dot(scoped ConePlaneRayPoint point, Signed576 x, Signed576 y, Signed576 z,
        ContactQuadratic result, ContactQuadratic term)
    {
        ContactQuadratic.Scale(point.X, x, result);
        ContactQuadratic.Scale(point.Y, y, term); result.Add(term);
        ContactQuadratic.Scale(point.Z, z, term); result.Add(term);
    }

    private static void ConeProduct(ConePlaneRayPoint left, ConePlaneRayPoint right,
        in ConePlaneRayFrame frame, scoped ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        int words = result.FieldWords;
        Span<ulong> work = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic term = ContactQuadratic.At(work, signs, 0, words);
        ContactQuadratic scaled = ContactQuadratic.At(work, signs, 1, words);
        Signed576 hh = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(frame.Height), Signed192.Raw(frame.Height)));
        Signed576 rr = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(frame.Radius), Signed192.Raw(frame.Radius)));
        ContactQuadratic.Multiply(left.X, right.X, root, result);
        ContactQuadratic.Multiply(left.Z, right.Z, root, term); result.Add(term);
        ContactQuadratic.Scale(result, hh, scaled); scaled.CopyTo(result);
        ContactQuadratic.Multiply(left.Y, right.Y, root, term);
        ContactQuadratic.Scale(term, rr, scaled); result.Add(scaled, -1);
    }

    private static void GetRayCoefficients(scoped ConePlaneRayPoint point, in ConePlaneRayFrame frame,
        scoped ReadOnlySpan<ulong> root, ContactQuadratic a, ContactQuadratic b)
    {
        Span<ulong> normalWords = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> normalSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        ConePlaneRayPoint normal = new(normalWords, normalSigns); normal.Set(frame.Normal);
        ConeProduct(normal, normal, frame, root, a);
        ConeProduct(point, normal, frame, root, b);
    }

    private static void AccumulateRationalOrientation(scoped ConePlaneRayPoint point, in ConePlaneRayFrame frame,
        int orientation, scoped ref ConePlaneRaySelection selection)
    {
        if (TryAccumulateAxialOrientation(point, frame, ReadOnlySpan<ulong>.Empty,
            orientation, sidePoint: false, ref selection)) return;
        Span<ulong> work = stackalloc ulong[20 * Words]; Span<int> signs = stackalloc int[20];
        ContactQuadratic a = ContactQuadratic.At(work, signs, 0, Words);
        ContactQuadratic b = ContactQuadratic.At(work, signs, 1, Words);
        ContactQuadratic c = ContactQuadratic.At(work, signs, 2, Words);
        ContactQuadratic term = ContactQuadratic.At(work, signs, 3, Words);
        ContactQuadratic polynomialA = ContactQuadratic.At(work, signs, 4, Words);
        ContactQuadratic polynomialB = ContactQuadratic.At(work, signs, 5, Words);
        ContactQuadratic numerator = ContactQuadratic.At(work, signs, 6, Words);
        ContactQuadratic denominator = ContactQuadratic.At(work, signs, 7, Words);
        ContactQuadratic bestN = ContactQuadratic.At(work, signs, 8, Words);
        ContactQuadratic bestD = ContactQuadratic.At(work, signs, 9, Words);
        Span<ulong> root = stackalloc ulong[ConePlaneRaySelection.RootWords]; root.Clear();
        Span<ulong> bestRoot = stackalloc ulong[ConePlaneRaySelection.RootWords]; bestRoot.Clear();
        bool hasValue = false;
        GetRayCoefficients(point, frame, root, a, b); b.MultiplySign(orientation);
        ConeProduct(point, point, frame, root, c);
        ContactQuadratic.Multiply(point.Denominator, point.Denominator, root, term);
        ContactQuadratic.Multiply(a, term, root, polynomialA);
        ContactQuadratic.Multiply(b, point.Denominator, root, polynomialB);
        for (int branch = -1; branch <= 1; branch += 2)
        {
            if (!ConePlaneRayCharts.TryGetParameter(polynomialA, polynomialB, c, branch, root, numerator, denominator)
                || numerator.Sign(root) < 0)
                continue;
            // Derivative b*d+A*n at a root picks the first outward crossing.
            // A concave repeated root touches F=0 from inside and is not exit.
            ContactQuadratic.Multiply(polynomialB, denominator, root, term);
            ContactQuadratic.Multiply(polynomialA, numerator, root, b); term.Add(b);
            int derivative = term.Sign(root);
            if (derivative < 0 || derivative == 0 && polynomialA.Signs[0] < 0)
                continue;
            KeepFirstExit(numerator, denominator, root, bestN, bestD, bestRoot, ref hasValue);
        }
        root.Clear();
        if (GetAxialExit(point, frame, orientation, numerator, denominator))
            KeepFirstExit(numerator, denominator, root, bestN, bestD, bestRoot, ref hasValue);
        // A nonzero ray from an admitted point in the compact finite cone
        // must meet a side or axial boundary, including zero-length exits.
        System.Diagnostics.Debug.Assert(hasValue);
        selection.Keep(bestN, bestD, bestRoot, frame, point);
    }

    private static void AccumulateSideOrientation(scoped ConePlaneRayPoint point, in ConePlaneRayFrame frame,
        scoped ReadOnlySpan<ulong> root, int orientation, scoped ref ConePlaneRaySelection selection)
    {
        if (TryAccumulateAxialOrientation(point, frame, root,
            orientation, sidePoint: true, ref selection)) return;
        Span<ulong> work = stackalloc ulong[12 * Words]; Span<int> signs = stackalloc int[12];
        ContactQuadratic a = ContactQuadratic.At(work, signs, 0, Words);
        ContactQuadratic b = ContactQuadratic.At(work, signs, 1, Words);
        ContactQuadratic numerator = ContactQuadratic.At(work, signs, 2, Words);
        ContactQuadratic denominator = ContactQuadratic.At(work, signs, 3, Words);
        ContactQuadratic bestN = ContactQuadratic.At(work, signs, 4, Words);
        ContactQuadratic bestD = ContactQuadratic.At(work, signs, 5, Words);
        Span<ulong> bestRoot = stackalloc ulong[ConePlaneRaySelection.RootWords]; bestRoot.Clear();
        bool hasValue = false;
        GetRayCoefficients(point, frame, root, a, b); b.MultiplySign(orientation);
        int bSign = b.Sign(root), aSign = a.Sign(root);
        if (bSign > 0 || bSign == 0 && aSign > 0)
        {
            numerator.Clear(); denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
            KeepFirstExit(numerator, denominator, root, bestN, bestD, bestRoot, ref hasValue);
        }
        else if (bSign < 0 && aSign > 0)
        {
            b.CopyTo(numerator, -1); numerator.Add(numerator);
            ContactQuadratic.Multiply(a, point.Denominator, root, denominator);
            KeepFirstExit(numerator, denominator, root, bestN, bestD, bestRoot, ref hasValue);
        }
        // A<=0 with B<=0 remains in this nappe until the finite axial cap.
        // A=B=0 is a generator interval and likewise keeps its finite endpoint.
        if (GetAxialExit(point, frame, orientation, numerator, denominator))
            KeepFirstExit(numerator, denominator, root, bestN, bestD, bestRoot, ref hasValue);
        // With Ny=0, A=H²(Nx²+Nz²)>0 and a side exit exists. Otherwise
        // the admitted finite axial interval supplies an endpoint.
        System.Diagnostics.Debug.Assert(hasValue);
        selection.Keep(bestN, bestD, bestRoot, frame, point);
    }

    /// <summary>Applies the axial first-exit construction to an already admitted point.</summary>
    /// <remarks>The orientation is +1 or -1. The caller proves exact finite admission; sidePoint also certifies the zero cone polynomial in the supplied root.</remarks>
    internal static bool TryAccumulateAxialOrientation(scoped ConePlaneRayPoint point, in ConePlaneRayFrame frame,
        scoped ReadOnlySpan<ulong> root, int orientation, bool sidePoint, scoped ref ConePlaneRaySelection selection)
    {
        if (!frame.Normal.X.IsZero || !frame.Normal.Z.IsZero) return false;
        bool axisPoint = point.X.Sign(root) == 0 && point.Z.Sign(root) == 0;
        int direction = frame.Normal.Y.Sign * orientation;
        if (!sidePoint && direction < 0 && !axisPoint) return false;
        Span<ulong> work = stackalloc ulong[4 * Words]; Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(work, signs, 0, Words);
        ContactQuadratic d = ContactQuadratic.At(work, signs, 1, Words);
        // Along the axis the widening direction stays inside until the base.
        // A point on the axis reaches either axial endpoint (also for R=0).
        // A nonaxial side point exits immediately in the narrowing direction.
        if (direction > 0 || axisPoint)
        {
            bool found = GetAxialExit(point, frame, orientation, n, d);
            System.Diagnostics.Debug.Assert(found);
        }
        else { n.Clear(); d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)))); }
        selection.Keep(n, d, root, frame, point);
        return true;
    }

    private static bool GetAxialExit(scoped ConePlaneRayPoint point, in ConePlaneRayFrame frame,
        int orientation, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator)
    {
        int direction = frame.Normal.Y.Sign * orientation;
        if (direction == 0)
            return false;
        Signed576 ny = Signed576.ExtendValue(frame.Normal.Y);
        if (direction > 0)
        {
            ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(frame.Finite.FullHeight), numerator);
            numerator.Add(point.Y, -1);
        }
        else point.Y.CopyTo(numerator);
        if (ny.Sign < 0) ny = WideArithmetic.SubtractSigned576(default, ny);
        ContactQuadratic.Scale(point.Denominator, ny, denominator);
        return true;
    }

    private static void KeepFirstExit(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root, ContactQuadratic bestN, ContactQuadratic bestD,
        Span<ulong> bestRoot, ref bool hasValue)
    {
        // The three private constructions establish these signs: admitted
        // quadratic roots, inward side crossings, and finite axial endpoints.
#if DEBUG
        System.Diagnostics.Debug.Assert(numerator.Sign(root) >= 0 && denominator.Sign(root) > 0);
#endif
        if (hasValue && ContactQuadratic.CompareRatios(numerator, denominator, root, bestN, bestD, bestRoot) >= 0)
            return;
        numerator.CopyTo(bestN); denominator.CopyTo(bestD);
        root.CopyTo(bestRoot); bestRoot[root.Length..].Clear(); hasValue = true;
    }
}
