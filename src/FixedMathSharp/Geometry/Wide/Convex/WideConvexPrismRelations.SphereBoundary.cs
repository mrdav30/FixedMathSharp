//=======================================================================
// WideConvexPrismRelations.SphereBoundary.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;



/// <content>Finite spherical face peaks retain exact native-yaw span cuts through ray evaluation.</content>
internal static partial class WideConvexPrismRelations
{
    /// <summary>Finds the sphere's inward ray at the closest point of a retained vertical face rectangle.</summary>
    /// <remarks>
    /// Geometry is admitted within 4096 units. The inside point chooses the face's inward normal;
    /// its finite rectangle must actually meet the sphere. The caller supplies fresh selection banks
    /// with the ordinary dimensions, or wide dimensions when the clip has wide bounds. Selection
    /// retains the exact ray parameter; physical depth squared is NormalSquared*N^2/(4*D^2).
    /// </remarks>
    internal static bool TryGetSphereBoundaryFaceContact(Vector3d center, Fixed64 radius,
        Vector3d origin, Fixed64 yaw, Vector2d start, Vector2d end, Vector2d inside,
        in RationalSegmentInterval clip, Fixed64 lowerY, Fixed64 upperY,
        ref CapsulePlaneRaySelection selection, out CapsulePlaneRayFrame ray, out FixedContactAnchors contact,
        out RationalSegmentInterval selectedParameter, out Fixed64 selectedHeight, int verticalNormal = 0,
        PrismSphereBoundaryAdmission? admission = null)
    {
        ray = default; contact = default; selectedParameter = default; selectedHeight = default; selection.Restore(default);
        if (lowerY > upperY) return false;
        var rotation = new WideConvex2dRelations.RotationFrame2d(yaw);
        WideConvex2dRelations.GetRotatedOffset(rotation, start, out Signed192 ax, out Signed192 az);
        WideConvex2dRelations.GetRotatedOffset(rotation, end, out Signed192 bx, out Signed192 bz);
        Signed192 ex = WideArithmetic.SubtractSigned192(bx, ax), ez = WideArithmetic.SubtractSigned192(bz, az);
        Signed320 edgeSquared = WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(ex, ex),
            WideArithmetic.MultiplySigned192(ez, ez));
        if (edgeSquared.IsZero) return false;
        Signed320 orientation = WideArithmetic.MultiplySubtract(WideArithmetic.Difference(end.X, start.X),
            WideArithmetic.Difference(inside.Y, start.Y), WideArithmetic.Difference(end.Y, start.Y),
            WideArithmetic.Difference(inside.X, start.X));
        if (orientation.IsZero) return false;

        Span<Signed576> normal = stackalloc Signed576[3]
            { Signed576.ExtendValue(WideArithmetic.Negate(ez)), default, Signed576.ExtendValue(ex) };
        if (orientation.Sign < 0)
            for (int axis = 0; axis < normal.Length; axis++) normal[axis] = WideArithmetic.SubtractSigned576(default, normal[axis]);
        if (verticalNormal != 0)
        { normal[0] = normal[2] = default; normal[1] = Signed576.ExtendValue(Signed192.Signed(verticalNormal)); }
        WideArithmetic.ReduceCommonScale(normal);

        ax = WideArithmetic.AddSigned192(ax, Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WideArithmetic.Difference(origin.X, center.X), Scale)));
        az = WideArithmetic.AddSigned192(az, Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            WideArithmetic.Difference(origin.Z, center.Z), Scale)));
        Signed320 projection = WideArithmetic.Negate(WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(ax, ex), WideArithmetic.MultiplySigned192(az, ez)));
        Signed832 n = Signed832.ExtendValue(projection), d = Signed832.ExtendValue(edgeSquared);
        Signed832 lowerN = default, lowerD = Signed832.ExtendValue(Signed192.Signed(1)), upperN = lowerD, upperD = lowerD;
        if (clip.IsClipped)
        {
            lowerN = clip.WideLowerNumerator; lowerD = clip.WideLowerDenominator;
            upperN = clip.WideUpperNumerator; upperD = clip.WideUpperDenominator;
        }
        if (n.Sign < 0 || RationalSegmentInterval.CompareParameters(n, d, lowerN, lowerD) < 0) { n = lowerN; d = lowerD; }
        else if (RationalSegmentInterval.CompareParameters(n, d, upperN, upperD) > 0) { n = upperN; d = upperD; }

        Signed192 y = WideArithmetic.Difference(center.Y, origin.Y), lower = Signed192.Raw(lowerY), upper = Signed192.Raw(upperY);
        if (WideArithmetic.SubtractSigned192(y, lower).Sign < 0) y = lower;
        else if (WideArithmetic.SubtractSigned192(y, upper).Sign > 0) y = upper;
        selectedParameter = new(n, d, n, d);
        // Clamping to a bounded admitted height proves this local coordinate fits.
        selectedHeight = Fixed64.FromRaw(unchecked((long)y.Low));
        y = WideArithmetic.AddSigned192(y, WideArithmetic.Difference(origin.Y, center.Y));

        // Native X/Z coordinates carry Q; the sphere ray consumes doubled raw
        // coordinates. Keep the rational span parameter instead of rounding a cut.
        const int words = 32;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var xPoint = ContactQuadratic.At(values, signs, 0, words);
        var yPoint = ContactQuadratic.At(values, signs, 1, words);
        var zPoint = ContactQuadratic.At(values, signs, 2, words);
        var pointD = ContactQuadratic.At(values, signs, 3, words);
        var parameter = ContactQuadratic.At(values, signs, 4, words); parameter.Set(n);
        var divisor = ContactQuadratic.At(values, signs, 5, words); divisor.Set(d);
        var term = ContactQuadratic.At(values, signs, 6, words);
        ContactQuadratic.Scale(divisor, Signed576.ExtendValue(ax), xPoint);
        ContactQuadratic.Scale(parameter, Signed576.ExtendValue(ex), term); xPoint.Add(term);
        ContactQuadratic.Scale(divisor, Signed576.ExtendValue(az), zPoint);
        ContactQuadratic.Scale(parameter, Signed576.ExtendValue(ez), term); zPoint.Add(term);
        ContactQuadratic.Scale(divisor, Signed576.ExtendValue(Scale), pointD);
        ContactQuadratic.Scale(pointD, Signed576.ExtendValue(y), yPoint);
        Signed832 two = Signed832.ExtendValue(Signed192.Signed(2));
        ContactQuadratic.Scale(xPoint, two, term); term.CopyTo(xPoint);
        ContactQuadratic.Scale(yPoint, two, term); term.CopyTo(yPoint);
        ContactQuadratic.Scale(zPoint, two, term); term.CopyTo(zPoint);

        Signed832 plane;
        if (verticalNormal == 0)
        {
            plane = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(normal[0], Signed320.ExtendValue(ax)),
                WideArithmetic.MultiplySigned576ToSigned832(normal[2], Signed320.ExtendValue(az)));
            plane = WideArithmetic.AddSigned832(plane, plane);
            for (int axis = 0; axis < normal.Length; axis++) normal[axis] = WideArithmetic.MultiplySigned576(normal[axis], Scale);
        }
        else plane = WideArithmetic.MultiplySigned576ToSigned832(normal[1], Signed320.ExtendValue(WideArithmetic.AddSigned192(y, y)));
        ray = CapsulePlaneRayFrame.ForSpherePointRay(radius, new WideDirection3d(normal[0], normal[1], normal[2]), plane, clip.HasWideBounds);

        if (admission != null && !admission(selectedParameter, selectedHeight, ray.Normal)) return false;
        if (!CapsulePlaneRayEvents.AccumulateRationalPoint(ray, xPoint, yPoint, zPoint, pointD, 1, ref selection)) return false;
        bool sphereRepresented = selection.TryGetExitAnchor(ray, 1, center, FixedQuaternion.Identity,
            out FixedPointAnchor sphere, out Fixed64 depth, out bool clamped);
        bool targetRepresented = selection.TryGetPointInRigidFrame(ray, origin, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, out Vector3d source);
        // Actual finite intersection keeps both witnesses in their admitted
        // bounded physical charts, independently of absolute world placement.
        System.Diagnostics.Debug.Assert(sphereRepresented && targetRepresented);
        contact = new FixedContactAnchors(sphere, new FixedPointAnchor(origin, FixedQuaternion.Identity, source),
            ray.Normal.GetNormal(false), depth, clamped);
        return true;
    }
}
