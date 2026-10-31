//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsuleTriangleRay.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact finite capsule boundary rays through retained coincident core/triangle witnesses.</content>
internal static partial class WideFiniteAxisIntersection
{
    private const int CapsuleTriangleRayWords = 64;
    private const int CapsuleTriangleRayRootWords = 40;
    private static Signed576 RayExtend(Signed192 value) => Signed576.ExtendValue(Signed320.ExtendValue(value));
    private static Signed576 RayExtend(Signed320 value) => Signed576.ExtendValue(value);
    private static ContactQuadratic RayField(Span<ulong> values, Span<int> signs, int index) =>
        ContactQuadratic.At(values, signs, index, CapsuleTriangleRayWords);

    private static Signed576 GetRetainedRayDot(Signed576 x, Signed576 y, Signed576 z, Vector3d direction) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(x, Signed192.Raw(direction.X)),
            WideArithmetic.MultiplySigned576(y, Signed192.Raw(direction.Y))),
            WideArithmetic.MultiplySigned576(z, Signed192.Raw(direction.Z)));

    private static void GetCoincidentCapsuleRayExit(in AxisTriangleCandidate point, Vector3d center,
        Vector3d axis, Fixed64 length, Fixed64 radius, Vector3d direction,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root,
        out int endpointSign)
    {
        Span<ulong> values = stackalloc ulong[8 * CapsuleTriangleRayWords]; Span<int> signs = stackalloc int[8];
        var x = RayField(values, signs, 0); var y = RayField(values, signs, 1);
        var z = RayField(values, signs, 2); var d = RayField(values, signs, 3);
        x.Set(SubtractVertex(point.AxisX, center.X, point.Denominator));
        y.Set(SubtractVertex(point.AxisY, center.Y, point.Denominator));
        z.Set(SubtractVertex(point.AxisZ, center.Z, point.Denominator)); d.Set(point.Denominator);
        GetCoincidentCapsuleRayExit(x, y, z, d, axis, length, radius, direction,
            numerator, denominator, root, out endpointSign);
    }

    /// <summary>First body/cap exit from an exact rational core point translated about the capsule center.</summary>
    private static void GetCoincidentCapsuleRayExit(scoped ContactQuadratic px, scoped ContactQuadratic py,
        scoped ContactQuadratic pz, scoped ContactQuadratic pointD, Vector3d axis, Fixed64 length,
        Fixed64 radius, Vector3d direction, scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        scoped Span<ulong> root, out int endpointSign)
    {
        Signed192 aa = GetDot(axis, Vector3d.Zero, axis, Vector3d.Zero),
            ad = GetDot(axis, Vector3d.Zero, direction, Vector3d.Zero),
            dd = GetDot(direction, Vector3d.Zero, direction, Vector3d.Zero);
        Signed320 perpendicular = WideArithmetic.MultiplySubtract(aa, dd, ad, ad);
        int words = numerator.FieldWords;
        Span<ulong> values = stackalloc ulong[24 * words]; Span<int> signs = stackalloc int[24];
        var a = ContactQuadratic.At(values, signs, 0, words); var b = ContactQuadratic.At(values, signs, 1, words);
        var c = ContactQuadratic.At(values, signs, 2, words); var term = ContactQuadratic.At(values, signs, 3, words);
        var work = ContactQuadratic.At(values, signs, 4, words); var bound = ContactQuadratic.At(values, signs, 5, words);
        var projection = ContactQuadratic.At(values, signs, 6, words); var f = ContactQuadratic.At(values, signs, 7, words);
        var x = ContactQuadratic.At(values, signs, 8, words); var y = ContactQuadratic.At(values, signs, 9, words);
        var z = ContactQuadratic.At(values, signs, 10, words); var cap = ContactQuadratic.At(values, signs, 11, words);
        ContactQuadratic.Scale(px, RayExtend(Signed192.Raw(axis.X)), projection);
        ContactQuadratic.Scale(py, RayExtend(Signed192.Raw(axis.Y)), term); projection.Add(term);
        ContactQuadratic.Scale(pz, RayExtend(Signed192.Raw(axis.Z)), term); projection.Add(term);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.Raw(radius));
        if (!perpendicular.IsZero)
        {
            a.Set(Signed576.ExtendValue(perpendicular)); b.Clear(); c.Set(WideArithmetic.MultiplySigned320(radiusSquared, aa)); c.MultiplySign(-1);
            _ = ContactQuadratic.TryGetParameter(a, b, c, 1, root, numerator, denominator);
            ContactQuadratic.Scale(pointD, RayExtend(ad), term);
            ContactQuadratic.Scale(term, RayExtend(DoubleParameterScale), f);
            ContactQuadratic.Multiply(numerator, f, root, work);
            ContactQuadratic.Scale(projection, RayExtend(DoubleParameterScale), f);
            ContactQuadratic.Multiply(denominator, f, root, term); work.Add(term);
            ContactQuadratic.Scale(pointD, RayExtend(Signed192.Raw(length)), f);
            ContactQuadratic.Scale(f, RayExtend(aa), cap);
            ContactQuadratic.Multiply(denominator, cap, root, bound);
            bound.CopyTo(term); term.Add(work, -1); bool belowUpper = term.Sign(root) >= 0;
            bound.CopyTo(term); term.Add(work);
            if (belowUpper && term.Sign(root) >= 0) { endpointSign = 0; return; }
        }
        endpointSign = ad.Sign;
        System.Diagnostics.Debug.Assert(endpointSign != 0);
        ContactQuadratic.Scale(pointD, RayExtend(DoubleParameterScale), f);
        ContactQuadratic.Scale(pointD, RayExtend(Signed192.Raw(endpointSign > 0 ? length : -length)), cap);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            ContactQuadratic source = coordinate == 0 ? px : coordinate == 1 ? py : pz;
            ContactQuadratic target = coordinate == 0 ? x : coordinate == 1 ? y : z;
            ContactQuadratic.Scale(source, RayExtend(DoubleParameterScale), target);
            ContactQuadratic.Scale(cap, RayExtend(Signed192.Raw(axis[coordinate])), term); target.Add(term, -1);
        }
        ContactQuadratic.Multiply(f, f, ReadOnlySpan<ulong>.Empty, work);
        ContactQuadratic.Scale(work, RayExtend(dd), a);
        ContactQuadratic.Scale(x, RayExtend(Signed192.Raw(direction.X)), projection);
        ContactQuadratic.Scale(y, RayExtend(Signed192.Raw(direction.Y)), term); projection.Add(term);
        ContactQuadratic.Scale(z, RayExtend(Signed192.Raw(direction.Z)), term); projection.Add(term);
        ContactQuadratic.Multiply(f, projection, ReadOnlySpan<ulong>.Empty, b);
        c.Clear();
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            ContactQuadratic component = coordinate == 0 ? x : coordinate == 1 ? y : z;
            ContactQuadratic.Multiply(component, component, ReadOnlySpan<ulong>.Empty, term); c.Add(term);
        }
        ContactQuadratic.Scale(work, RayExtend(radiusSquared), term); c.Add(term, -1);
        _ = ContactQuadratic.TryGetParameter(a, b, c, 1, root, numerator, denominator);
    }
    private static void GetCapsuleRayDepth(ContactQuadratic numerator, ContactQuadratic denominator,
        ReadOnlySpan<ulong> root, Vector3d direction, out Fixed64 depth, out bool clamped)
    {
        Span<ulong> metric = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(GetDot(direction, Vector3d.Zero, direction, Vector3d.Zero),
            out metric[2], out metric[1], out metric[0]);
        Span<ulong> one = stackalloc ulong[1] { 1UL };
        clamped = !ContactQuadratic.TryRoundRootRatio(numerator, denominator, root, metric, one, out depth);
        if (clamped) depth = Fixed64.MaxValue;
    }

    private static bool TryMaterializeCapsuleRayPoint(in AxisTriangleCandidate point, Vector3d direction,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root,
        out Vector3d boundary)
    {
        Span<ulong> values = stackalloc ulong[12 * CapsuleTriangleRayWords];
        Span<int> signs = stackalloc int[12];
        ContactQuadratic x = RayField(values, signs, 0), y = RayField(values, signs, 1);
        ContactQuadratic z = RayField(values, signs, 2), d = RayField(values, signs, 3);
        ContactQuadratic term = RayField(values, signs, 4);
        Span<Signed576> coordinates = stackalloc Signed576[3] { point.AxisX, point.AxisY, point.AxisZ };
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            ContactQuadratic target = coordinate == 0 ? x : coordinate == 1 ? y : z;
            ContactQuadratic.Scale(denominator, coordinates[coordinate], target);
            ContactQuadratic.Scale(numerator, WideArithmetic.MultiplySigned576(point.Denominator,
                Signed192.Raw(direction[coordinate])), term); target.Add(term);
        }
        ContactQuadratic.Scale(denominator, point.Denominator, d);
        return QuadraticPointMaterialization.TryGetWorldPoint(Vector3d.Zero, FixedQuaternion.Identity,
            x, y, z, d, root, out boundary);
    }

    private static void MaterializeCapsuleRayComponents(in AxisTriangleCandidate point, Vector3d center,
        Vector3d axis, Fixed64 length, Vector3d direction, int endpointSign,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root,
        FixedQuaternion sourceRotation, FixedQuaternion targetRotation,
        out Fixed64 axisParameter, out Vector3d radialOffset)
    {
        // Match the exit to its closest finite core point. Both components are
        // bounded separately even when their sum is outside the scalar domain.
        Signed192 aa = GetDot(axis, Vector3d.Zero, axis, Vector3d.Zero);
        Signed192 ad = GetDot(axis, Vector3d.Zero, direction, Vector3d.Zero);
        Signed576 rx = SubtractVertex(point.AxisX, center.X, point.Denominator);
        Signed576 ry = SubtractVertex(point.AxisY, center.Y, point.Denominator);
        Signed576 rz = SubtractVertex(point.AxisZ, center.Z, point.Denominator);
        Span<ulong> values = stackalloc ulong[12 * CapsuleTriangleRayWords];
        Span<int> signs = stackalloc int[12];
        ContactQuadratic x = RayField(values, signs, 0), y = RayField(values, signs, 1);
        ContactQuadratic z = RayField(values, signs, 2), d = RayField(values, signs, 3);
        ContactQuadratic term = RayField(values, signs, 4), parameter = RayField(values, signs, 5);
        if (endpointSign == 0)
        {
            Signed576 projection = GetRetainedRayDot(rx, ry, rz, axis);
            ContactQuadratic.Scale(denominator, projection, parameter);
            ContactQuadratic.Scale(numerator, WideArithmetic.MultiplySigned576(point.Denominator, ad), term);
            parameter.Add(term);
            ContactQuadratic.Scale(parameter, RayExtend(ParameterScale), x);
            ContactQuadratic.Scale(denominator, WideArithmetic.MultiplySigned576(point.Denominator, aa), d);
            axisParameter = ContactQuadratic.RoundRatio(x, d, root);
            for (int coordinate = 0; coordinate < 3; coordinate++)
            {
                Signed320 radial = WideArithmetic.MultiplySubtract(
                    Signed192.Raw(direction[coordinate]), aa, Signed192.Raw(axis[coordinate]), ad);
                ContactQuadratic.Scale(numerator, Signed576.ExtendValue(radial),
                    coordinate == 0 ? x : coordinate == 1 ? y : z);
            }
            ContactQuadratic.Scale(denominator, RayExtend(aa), d);
        }
        else
        {
            axisParameter = (endpointSign > 0 ? length : -length) / Fixed64.Two;
            Signed576 f = WideArithmetic.MultiplySigned576(point.Denominator, DoubleParameterScale);
            Signed576 cap = WideArithmetic.MultiplySigned576(point.Denominator,
                Signed192.Raw(endpointSign > 0 ? length : -length));
            Span<Signed576> relative = stackalloc Signed576[3] { rx, ry, rz };
            for (int coordinate = 0; coordinate < 3; coordinate++)
            {
                Signed576 w = WideArithmetic.SubtractSigned576(
                    WideArithmetic.MultiplySigned576(relative[coordinate], DoubleParameterScale),
                    WideArithmetic.MultiplySigned576(cap, Signed192.Raw(axis[coordinate])));
                ContactQuadratic target = coordinate == 0 ? x : coordinate == 1 ? y : z;
                ContactQuadratic.Scale(denominator, w, target);
                ContactQuadratic.Scale(numerator, WideArithmetic.MultiplySigned576(f,
                    Signed192.Raw(direction[coordinate])), term); target.Add(term);
            }
            ContactQuadratic.Scale(denominator, f, d);
        }
        // Keep the exact radial coordinates through the target-frame transform.
        // Rounding in the chart first can turn a Max-radius component into Max+1.
        bool represented = QuadraticPointMaterialization.TryGetOffsetInFrame(sourceRotation,
            targetRotation, x, y, z, d, root, out radialOffset);
        System.Diagnostics.Debug.Assert(represented);
    }
}
