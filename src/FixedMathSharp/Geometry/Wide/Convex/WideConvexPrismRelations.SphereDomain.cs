// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;

namespace FixedMathSharp.Geometry;

/// <content>Admits exact native-prism sphere supports before depth ranking.</content>
internal static partial class WideConvexPrismRelations
{
    private static void KeepSphereProjection(Axis3 axis, bool negate, Vector3d offset, ProjectionDepth depth,
        RigidAxis3 shapeAxis, Vector3d center, Vector3d origin, Fixed64 yaw, ReadOnlySpan<Vector2d> offsets,
        Fixed64 halfThickness, PrismPointAdmission admission, ref PenetrationCandidate best)
    {
        if (best.HasValue && CompareProjectionDepths(depth, best.ExactDepth) >= 0) return;
        var candidate = new PenetrationCandidate(axis, negate, offset, depth);
        FixedPointAnchor unused = default;
        if (TryGetMatchedPrismAnchor(candidate, negate ? Negate(axis) : axis, shapeAxis, center,
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, default, 0, origin, yaw,
            FixedQuaternion.Identity, offsets, halfThickness, ref unused, out _, admission, materialize: false))
            best = candidate;
    }

    private static bool AdmitPrismSpherePoint(Axis3 normal, in ProjectionDepth depth, ReadOnlySpan<Signed832> projected,
        Signed832 pointD, Fixed64 yaw, PrismPointAdmission admission)
    {
        const int words = 32;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var x = ContactQuadratic.At(values, signs, 0, words);
        var y = ContactQuadratic.At(values, signs, 1, words);
        var z = ContactQuadratic.At(values, signs, 2, words);
        var d = ContactQuadratic.At(values, signs, 3, words);
        var first = ContactQuadratic.At(values, signs, 4, words);
        var second = ContactQuadratic.At(values, signs, 5, words);
        var term = ContactQuadratic.At(values, signs, 6, words);
        var frame = new WideConvex2dRelations.RotationFrame2d(yaw);
        Signed320 metric = WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(frame.Cosine, frame.Cosine),
            WideArithmetic.MultiplySigned192(frame.Sine, frame.Sine));
        first.Set(projected[0]); second.Set(projected[2]);
        ContactQuadratic.Scale(first, Signed576.ExtendValue(frame.Cosine), x);
        ContactQuadratic.Scale(second, Signed576.ExtendValue(frame.Sine), term); x.Add(term);
        ContactQuadratic.Scale(first, Signed576.ExtendValue(WideArithmetic.Negate(frame.Sine)), z);
        ContactQuadratic.Scale(second, Signed576.ExtendValue(frame.Cosine), term); z.Add(term);
        x.CopyTo(first); ContactQuadratic.Scale(first, Signed576.ExtendValue(Scale), x);
        z.CopyTo(first); ContactQuadratic.Scale(first, Signed576.ExtendValue(Scale), z);
        first.Set(projected[1]); ContactQuadratic.Scale(first, Signed576.ExtendValue(metric), y);
        first.Set(pointD); ContactQuadratic.Scale(first, Signed576.ExtendValue(metric), d);
        // Native scalar Sin/Cos need not have unit squared length. Divide by
        // their actual metric; never substitute a rounded quaternion inverse.
        return admission(new WideDirection3d(normal.X, normal.Y, normal.Z), depth, x, y, z, d);
    }

    private static bool KeepSphereEdgeAxes(Vector3d center, Fixed64 radius, Vector3d origin, Fixed64 yaw,
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness, RigidAxis3 shapeAxis,
        PrismPointAdmission admission, ref PenetrationCandidate best)
    {
        var frame = new WideConvex2dRelations.RotationFrame2d(yaw);
        Span<Signed192> a = stackalloc Signed192[3], b = stackalloc Signed192[3], edge = stackalloc Signed192[3];
        Span<Signed320> normal = stackalloc Signed320[3];
        for (int index = 0; index < offsets.Length; index++)
          for (int kind = 0; kind < 3; kind++)
          {
            GetPrismEdgeVertex(frame, offsets[index], kind == 1 ? halfThickness : -halfThickness, Signed192.Signed(1), a);
            GetPrismEdgeVertex(frame, kind == 2 ? offsets[index] : offsets[(index + 1) % offsets.Length],
                kind == 0 ? -halfThickness : halfThickness, Signed192.Signed(1), b);
            for (int coordinate = 0; coordinate < 3; coordinate++)
            {
                edge[coordinate] = WideArithmetic.SubtractSigned192(b[coordinate], a[coordinate]);
                a[coordinate] = WideArithmetic.AddSigned192(a[coordinate], Signed192.NarrowProven(
                    WideArithmetic.MultiplySigned192(WideArithmetic.Difference(origin[coordinate], center[coordinate]), Scale)));
            }
            Signed320 denominator = Signed320.NarrowValue(PrismEdgeDot(edge, edge));
            if (denominator.IsZero) continue;
            Signed320 numerator = WideArithmetic.Negate(Signed320.NarrowValue(PrismEdgeDot(a, edge)));
            if (numerator.Sign < 0) numerator = default;
            else if (WideArithmetic.SubtractSigned320(numerator, denominator).Sign > 0) numerator = denominator;
            for (int coordinate = 0; coordinate < 3; coordinate++)
                normal[coordinate] = Signed320.NarrowValue(WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(denominator, a[coordinate]),
                    WideArithmetic.MultiplySigned320(numerator, edge[coordinate])));
            // This internal domain entry is for admitted 4,096-reach shapes
            // with overlapping bounds. Native coordinates need <80 bits, Gram
            // terms <162 and directions <244; their squares fit Signed576.
            // The zero core fixes t=1/2 in the shared witness projection, so
            // its <740-bit half-planes fit Signed832 without the free-core bank.
            if (!TryKeepAxis(new(normal[0], normal[1], normal[2]), shapeAxis, center, Fixed64.Zero, radius,
                origin, yaw, offsets, halfThickness, ref best, admission)) return false;
          }
        return true;
    }
}
