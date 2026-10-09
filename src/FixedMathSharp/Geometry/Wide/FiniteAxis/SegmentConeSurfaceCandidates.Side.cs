//=======================================================================
// SegmentConeSurfaceCandidates.Side.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Finite lateral-generator stationary points and parallel slices.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static void AccumulateSide(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {
        if (geometry.Radius.IsZero)
            return;
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        for (int endpoint = geometry.Edge.IsZero ? 0 : -1; endpoint <= (geometry.Edge.IsZero ? 0 : 1); endpoint++)
        {
            WideAxis3 point = endpoint == 1 ? geometry.B : geometry.A;
            if (endpoint >= 0 && point.X.IsZero && point.Z.IsZero)
            {
                if (geometry.ContainsParameter(endpoint == 0 ? Fixed64.Zero : Fixed64.One)
                    && HasSideCircleDomain(geometry, endpoint, values, signs, root))
                    selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Side,
                        ConeSurfaceFamily.RotationCircle, 1, endpoint, endpoint == 0 ? ConeSurfacePointLocation.Start : ConeSurfacePointLocation.End));
                continue;
            }
            for (int branch = -1; branch <= 1; branch += 2)
            {
                if (!BuildSideNormal(geometry, endpoint, branch, values, signs, root))
                    continue;
                if (!TryBuildSideFoot(geometry, endpoint, values, signs, root, out bool interval))
                    continue;
                selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Side,
                    interval ? ConeSurfaceFamily.SegmentInterval : ConeSurfaceFamily.None, branch, endpoint,
                    interval ? ConeSurfacePointLocation.SegmentIntervalUnspecified : endpoint < 0 ? ConeSurfacePointLocation.Interior
                        : endpoint == 0 ? ConeSurfacePointLocation.Start : ConeSurfacePointLocation.End));
                if (IsZero(root))
                    break;
            }
        }
    }

    private static bool BuildSideNormal(in SegmentConeSurfaceGeometry geometry, int endpoint, int branch,
        Span<ulong> values, Span<int> signs, Span<ulong> root)
    {
        WideAxis3 axis = endpoint < 0 ? geometry.Edge : endpoint == 0 ? geometry.A : geometry.B;
        Signed576 radial = CircularRimContactAlgebra.RadialDot(axis, axis);
        if (radial.IsZero)
            return false;
        Signed192 h = Signed192.Raw(geometry.Input.Height), r = Signed192.Raw(geometry.Input.Radius);
        ContactQuadratic nx = At(values, signs, 0), ny = At(values, signs, 1), nz = At(values, signs, 2);
        if (endpoint >= 0)
        {
            BuildSidePointNormal(geometry, axis, branch, values, signs, root);
            return true;
        }
        Span<ulong> scratch = stackalloc ulong[4 * Words];
        Span<ulong> hSquared = Slot(scratch, 0), rSquared = Slot(scratch, 1), product = Slot(scratch, 2), square = Slot(scratch, 3);
        Import(WideArithmetic.MultiplySigned192(h, h), hSquared);
        Import(WideArithmetic.MultiplySigned192(r, r), rSquared);
        Import(radial, square);
        WideArithmetic.MultiplyMagnitudes(square, hSquared, root);
        Import(WideArithmetic.MultiplySigned320(axis.Y, axis.Y), square);
        WideArithmetic.MultiplyMagnitudes(square, rSquared, product);
        int discriminantSign = 1;
        CylinderContactAlgebra.Add(product, -1, root, ref discriminantSign);
        if (discriminantSign < 0)
            return false;
        if (IsZero(root))
        {
            // At tangency E is parallel to a generator. Cancel the extra
            // radial edge factor before constructing finite interval bounds:
            // n=(H² Ex sign(Ey),-R² |Ey|,H² Ez sign(Ey)). Components <325 bits.
            ContactQuadratic component = At(values, signs, 3);
            component.Set(Signed576.ExtendValue(axis.X)); Scale(component, hSquared, axis.Y.Sign, nx);
            component.Set(Signed576.ExtendValue(axis.Z)); Scale(component, hSquared, axis.Y.Sign, nz);
            component.Set(Signed576.ExtendValue(axis.Y)); Scale(component, rSquared, -axis.Y.Sign, ny);
            return true;
        }
        nx.Set(WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(axis.Y, axis.X), geometry.Input.Radius.m_rawValue));
        ny.Set(WideArithmetic.MultiplySigned576(radial, geometry.Input.Radius.m_rawValue)); ny.MultiplySign(-1);
        nz.Set(WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(axis.Y, axis.Z), geometry.Input.Radius.m_rawValue));
        Import(axis.Z, nx.Radical); nx.Signs[1] = -branch * axis.Z.Sign;
        Import(axis.X, nz.Radical); nz.Signs[1] = branch * axis.X.Sign;
        return true;
    }

    private static void BuildSidePointNormal(in SegmentConeSurfaceGeometry geometry, WideAxis3 radialAxis, int branch,
        Span<ulong> values, Span<int> signs, Span<ulong> root)
    {
        Import(CircularRimContactAlgebra.RadialDot(radialAxis, radialAxis), root);
        ContactQuadratic nx = At(values, signs, 0), ny = At(values, signs, 1), nz = At(values, signs, 2);
        Signed192 h = Signed192.Raw(geometry.Input.Height), r = Signed192.Raw(geometry.Input.Radius);
        nx.Set(WideArithmetic.MultiplySigned320(radialAxis.X, h)); nx.MultiplySign(-branch);
        nz.Set(WideArithmetic.MultiplySigned320(radialAxis.Z, h)); nz.MultiplySign(-branch);
        ny.Clear(); Import(Signed320.ExtendValue(r), ny.Radical); ny.Signs[1] = -1;
    }

    private static bool HasSideCircleDomain(in SegmentConeSurfaceGeometry geometry, int endpoint,
        Span<ulong> values, Span<int> signs, Span<ulong> root)
    {
        ContactQuadratic limit = At(values, signs, 0);
        limit.Set(WideArithmetic.MultiplySigned320(geometry.Edge.Y, Signed192.Raw(geometry.Input.Radius)));
        limit.MultiplySign(-1);
        Import(Signed320.ExtendValue(Signed192.Raw(geometry.Input.Height)), limit.Radical);
        limit.Signs[1] = endpoint == 0 ? -1 : 1;
        Import(CircularRimContactAlgebra.RadialDot(geometry.Edge, geometry.Edge), root);
        int sign = limit.Sign(root);
        return endpoint == 0 ? sign <= 0 : sign >= 0;
    }

    // Slots 0..2 are n, 5..6 are finite segment t numerator/denominator,
    // 7 is n² (rational), 8 is (A-V)·n, and 9 is qY's numerator over tD*n².
    private static bool TryBuildSideFoot(in SegmentConeSurfaceGeometry geometry, int endpoint,
        Span<ulong> values, Span<int> signs, ReadOnlySpan<ulong> root, out bool interval)
    {
        interval = false;
        ContactQuadratic numerator = At(values, signs, 5), denominator = At(values, signs, 6);
        ContactQuadratic metric = At(values, signs, 7), gap = At(values, signs, 8);
        ContactQuadratic product = At(values, signs, 3), temporary = At(values, signs, 4);
        metric.Clear(); gap.Clear();
        WideAxis3 offset = new(geometry.A.X, WideArithmetic.SubtractSigned320(geometry.A.Y, geometry.Finite.ShapeFrame.Cap), geometry.A.Z);
        for (int axis = 0; axis < 3; axis++)
        {
            Multiply(At(values, signs, axis), At(values, signs, axis), root, product); metric.Add(product);
            Scale(At(values, signs, axis), Signed576.ExtendValue(Component(offset, axis)), product); gap.Add(product);
        }
        if (endpoint >= 0)
        {
            numerator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(endpoint))));
            denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
            if (!AdmitsEndpointNormal(geometry.Edge, endpoint, values, signs, root))
                return false;
            if (endpoint == 1)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    Scale(At(values, signs, axis), Signed576.ExtendValue(Component(geometry.Edge, axis)), product);
                    gap.Add(product);
                }
            }
        }
        else
        {
            Scale(At(values, signs, 2), Signed576.ExtendValue(geometry.A.X), numerator);
            Scale(At(values, signs, 0), Signed576.ExtendValue(geometry.A.Z), product); numerator.Add(product, -1); numerator.MultiplySign(-1);
            Scale(At(values, signs, 2), Signed576.ExtendValue(geometry.Edge.X), denominator);
            Scale(At(values, signs, 0), Signed576.ExtendValue(geometry.Edge.Z), product); denominator.Add(product, -1);
            int sign = denominator.Sign(root);
            if (sign == 0)
            {
                if (numerator.Sign(root) != 0)
                    return false;
#if DEBUG
                System.Diagnostics.Debug.Assert(gap.Sign(root) >= 0);
#endif
                interval = true;
                return HasSideInterval(geometry, values, signs, root);
            }
            numerator.MultiplySign(sign); denominator.MultiplySign(sign);
            denominator.CopyTo(temporary); temporary.Add(numerator, -1);
            if (numerator.Sign(root) <= 0 || temporary.Sign(root) <= 0)
                return false;
        }
        // The preflight supplies an inside point s. Interior normals have
        // n.E=0; an admitted endpoint maximizes n's segment projection.
        // Therefore gap>=n.(s-V)>=0 for this inward supporting normal.
#if DEBUG
        System.Diagnostics.Debug.Assert(gap.Sign(root) >= 0);
#endif
        return AdmitsSideParameter(geometry, values, signs, root);
    }

    private static bool AdmitsSideParameter(in SegmentConeSurfaceGeometry geometry,
        Span<ulong> values, Span<int> signs, ReadOnlySpan<ulong> root)
    {
        if (!ContainsQuadraticParameter(geometry, At(values, signs, 5), At(values, signs, 6), root))
            return false;
        ContactQuadratic product = At(values, signs, 3), temporary = At(values, signs, 4);
        ContactQuadratic qY = At(values, signs, 9), common = At(values, signs, 10), margin = At(values, signs, 11);
        Scale(At(values, signs, 6), Signed576.ExtendValue(geometry.A.Y), product);
        Scale(At(values, signs, 5), Signed576.ExtendValue(geometry.Edge.Y), temporary); product.Add(temporary);
        Multiply(product, At(values, signs, 7), root, qY);
        Multiply(At(values, signs, 8), At(values, signs, 1), root, product);
        Multiply(product, At(values, signs, 6), root, temporary); qY.Add(temporary, -1);
        Multiply(At(values, signs, 6), At(values, signs, 7), root, common);
        Scale(common, Signed576.ExtendValue(geometry.Finite.ShapeFrame.Cap), margin);
        // p is in the cone and gap>=0 with inward ny<0, hence qY>=pY>=-cap.
        // Only the upper generator endpoint needs an independent rejection.
        margin.Add(qY, -1);
        return margin.Sign(root) >= 0;
    }

    private static bool ContainsQuadraticParameter(in SegmentConeSurfaceGeometry geometry,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        // Isolated t coefficients <660 bits; a reduced parallel-family
        // bound has <849 bits. Coordinates then have <1048 bits; the cone
        // quadratic (including raw H²/R²) stays below 2225 bits.
        Span<ulong> values = stackalloc ulong[14 * Words];
        Span<int> signs = stackalloc int[14];
        ContactQuadratic coordinate = At(values, signs, 0), temporary = At(values, signs, 1), square = At(values, signs, 2);
        ContactQuadratic radial = At(values, signs, 3), axial = At(values, signs, 4), limit = At(values, signs, 5), result = At(values, signs, 6);
        radial.Clear();
        for (int axis = 0; axis < 3; axis += 2)
        {
            Scale(denominator, Signed576.ExtendValue(Component(geometry.A, axis)), coordinate);
            Scale(numerator, Signed576.ExtendValue(Component(geometry.Edge, axis)), temporary); coordinate.Add(temporary);
            Multiply(coordinate, coordinate, root, square); radial.Add(square);
        }
        Scale(denominator, Signed576.ExtendValue(WideArithmetic.SubtractSigned320(geometry.Finite.ShapeFrame.Cap, geometry.A.Y)), axial);
        Scale(numerator, Signed576.ExtendValue(geometry.Edge.Y), temporary); axial.Add(temporary, -1);
        if (axial.Sign(root) < 0)
            return false;
        Scale(denominator, Signed576.ExtendValue(geometry.Finite.FullHeight), limit); limit.Add(axial, -1);
        if (limit.Sign(root) < 0)
            return false;
        Scale(radial, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(geometry.Input.Height), Signed192.Raw(geometry.Input.Height))), result);
        Multiply(axial, axial, root, square);
        Scale(square, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(geometry.Input.Radius), Signed192.Raw(geometry.Input.Radius))), temporary);
        result.Add(temporary, -1);
        return result.Sign(root) <= 0;
    }

    private static bool HasSideInterval(in SegmentConeSurfaceGeometry geometry,
        Span<ulong> normals, Span<int> normalSigns, ReadOnlySpan<ulong> root)
    {
        // The only dependent tangent case has zero discriminant, hence all
        // bounds are rational. Reducing n in BuildSideNormal keeps clipping
        // weights <849 bits and the subsequent cone query <2225 bits.
        Span<ulong> values = stackalloc ulong[20 * Words];
        Span<int> signs = stackalloc int[20];
        ContactQuadratic low = At(values, signs, 0), lowD = At(values, signs, 1);
        ContactQuadratic high = At(values, signs, 2), highD = At(values, signs, 3);
        ContactQuadratic bound = At(values, signs, 4), boundD = At(values, signs, 5);
        ContactQuadratic constant = At(values, signs, 6), velocity = At(values, signs, 7);
        ContactQuadratic temporary = At(values, signs, 8), unit = At(values, signs, 9);
        Signed576 one = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)));
        low.Clear(); lowD.Set(one); high.Set(one); highD.Set(one); unit.Set(one);
        for (int domain = 0; domain < 2; domain++)
        {
            ContactQuadratic divisor = domain == 0 ? At(normals, normalSigns, 7) : unit;
            Scale(divisor, Signed576.ExtendValue(geometry.A.Y), constant);
            if (domain == 0)
            {
                Multiply(At(normals, normalSigns, 8), At(normals, normalSigns, 1), root, temporary);
                constant.Add(temporary, -1);
            }
            Scale(divisor, Signed576.ExtendValue(geometry.Edge.Y), velocity);
            int velocitySign = velocity.Sign(root);
            System.Diagnostics.Debug.Assert(velocitySign != 0);
            for (int side = -1; side <= 1; side += 2)
            {
                Scale(divisor, Signed576.ExtendValue(geometry.Finite.ShapeFrame.Cap), bound);
                bound.MultiplySign(side); bound.Add(constant, -1);
                bound.MultiplySign(velocitySign); velocity.CopyTo(boundD, velocitySign);
                bool lower = side * velocitySign < 0;
                ContactQuadratic current = lower ? low : high, currentD = lower ? lowD : highD;
                int comparison = CompareRatios(bound, boundD, root, current, currentD, root);
                if (lower ? comparison > 0 : comparison < 0)
                {
                    bound.CopyTo(current); boundD.CopyTo(currentD);
                }
            }
        }
        if (CompareRatios(low, lowD, root, high, highD, root) > 0)
            return false;
        // E is generator-parallel, so H²|E_rad|²-R² E_y²=0.
        // The cone polynomial is linear and reaches its minimum at an end.
        return ContainsQuadraticParameter(geometry, low, lowD, root)
            || ContainsQuadraticParameter(geometry, high, highD, root);
    }
}
