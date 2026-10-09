//=======================================================================
// SegmentConeSurfaceCandidates.MeridionalRim.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>The meridional rim's in-plane and axis-crossing stationary pairs.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static bool IsMeridionalRim(in SegmentConeSurfaceGeometry geometry) =>
        !geometry.Edge.Y.IsZero && (!geometry.Edge.X.IsZero || !geometry.Edge.Z.IsZero)
        && WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(geometry.A.X, geometry.Edge.Z),
            WideArithmetic.MultiplySigned320(geometry.A.Z, geometry.Edge.X)).IsZero;

    private static void AccumulateMeridionalRim(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {
        if (geometry.Radius.IsZero || !IsMeridionalRim(geometry))
            return;
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        for (int mode = -3; mode >= -4; mode--)
        for (int branch = -1; branch <= 1; branch += 2)
        {
            if (!BuildMeridionalRim(geometry, mode, branch, values, signs, root))
                continue;
            ContactQuadratic margin = At(values, signs, 3);
            At(values, signs, 6).CopyTo(margin); margin.Add(At(values, signs, 5), -1);
            ConeSurfacePointLocation location = At(values, signs, 5).Sign(root) == 0 ? ConeSurfacePointLocation.Start
                : margin.Sign(root) == 0 ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Interior;
            bool zero = At(values, signs, 0).Sign(root) == 0 && At(values, signs, 1).Sign(root) == 0 && At(values, signs, 2).Sign(root) == 0;
            if (zero && !HasRimTouchDomain(geometry, new WideAxis3(geometry.Edge.X, default, geometry.Edge.Z), branch,
                    location == ConeSurfacePointLocation.Interior ? -1 : location == ConeSurfacePointLocation.Start ? 0 : 1))
                continue;
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Rim,
                zero ? ConeSurfaceFamily.NormalCone : ConeSurfaceFamily.None, branch > 0 ? -2 : -3, mode, location));
            if (IsZero(root))
                break;
        }
    }

    // Slots 0..2 contain (p-q)*normalScale, 5..6 contain t, 7 normalScale.
    // In-plane normals have <995-bit coefficients and root <398 bits.
    // Axis-crossing root <1595 bits and normal coefficients <996 bits;
    // squaring their complete values needs <2000 coefficient bits.
    private static bool BuildMeridionalRim(in SegmentConeSurfaceGeometry geometry, int mode, int branch,
        Span<ulong> values, Span<int> signs, Span<ulong> root)
    {
        Signed576 radialSquare = CircularRimContactAlgebra.RadialDot(geometry.Edge, geometry.Edge);
        Signed576 metric = geometry.Edge.SquaredLength;
        ContactQuadratic numerator = At(values, signs, 5), denominator = At(values, signs, 6), normalScale = At(values, signs, 7);
        ContactQuadratic product = At(values, signs, 3), temporary = At(values, signs, 4);
        root.Clear();
        if (mode == -3)
        {
            Signed576 dot = WideAxis3.Dot(geometry.BaseOffset, geometry.Edge);
            numerator.Set(WideArithmetic.SubtractSigned576(default, dot));
            Import(geometry.Radius, numerator.Radical); numerator.Signs[1] = branch;
            denominator.Set(metric);
            Import(radialSquare, root);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic normal = At(values, signs, axis);
                normal.Set(Signed576.ExtendValue(Component(geometry.BaseOffset, axis)));
                Scale(normal, metric, product);
                temporary.Set(Signed576.ExtendValue(Component(geometry.Edge, axis)));
                Scale(temporary, dot, normal); product.Add(normal, -1);
                Scale(product, radialSquare, normal);
                product.Set(WideArithmetic.MultiplySigned320(geometry.Radius, Component(geometry.Edge, axis)));
                Scale(product, axis == 1 ? radialSquare : WideArithmetic.SubtractSigned576(radialSquare, metric), temporary);
                temporary.Rational.CopyTo(normal.Radical); normal.Signs[1] = branch * temporary.Signs[0];
            }
            product.Set(metric); Scale(product, radialSquare, normalScale);
        }
        else
        {
            numerator.Set(WideArithmetic.SubtractSigned576(default, CircularRimContactAlgebra.RadialDot(geometry.A, geometry.Edge)));
            denominator.Set(radialSquare);
            ContactQuadratic axial = At(values, signs, 8), discriminant = At(values, signs, 9);
            axial.Set(Signed576.ExtendValue(geometry.BaseOffset.Y)); Scale(axial, radialSquare, product);
            Scale(numerator, Signed576.ExtendValue(geometry.Edge.Y), temporary); product.Add(temporary); product.CopyTo(axial);
            product.Set(radialSquare); Scale(product, radialSquare, normalScale);
            Scale(normalScale, radialSquare, product);
            Scale(product, WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius), discriminant);
            Multiply(axial, axial, root, product);
            Scale(product, WideArithmetic.MultiplySigned320(geometry.Edge.Y, geometry.Edge.Y), temporary); discriminant.Add(temporary, -1);
            if (discriminant.Sign(root) < 0)
                return false;
            discriminant.Rational.CopyTo(root);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic normal = At(values, signs, axis);
                Scale(axial, axis == 1 ? radialSquare
                    : WideArithmetic.MultiplySigned320(geometry.Edge.Y, Component(geometry.Edge, axis)), normal);
                if (axis != 1)
                {
                    normal.MultiplySign(-1);
                    Signed320 cross = axis == 0 ? geometry.Edge.Z : WideArithmetic.Negate(geometry.Edge.X);
                    Import(cross, normal.Radical); normal.Signs[1] = branch * cross.Sign;
                }
            }
        }
        denominator.CopyTo(product); product.Add(numerator, -1);
        return numerator.Sign(root) >= 0 && product.Sign(root) >= 0
            && ContainsQuadraticParameter(geometry, numerator, denominator, root);
    }

    private static FixedContactAnchors GetMeridionalRimContact(SegmentConeSurfaceCandidate candidate)
    {
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        bool admitted = BuildMeridionalRim(geometry, candidate.RootOrdinal, candidate.Chart == -2 ? 1 : -1, values, signs, root);
        System.Diagnostics.Debug.Assert(admitted);
        return MaterializeMeridionalRim(geometry, candidate, values, signs, root);
    }

    private static FixedContactAnchors MaterializeMeridionalRim(in SegmentConeSurfaceGeometry geometry,
        SegmentConeSurfaceCandidate candidate, Span<ulong> normals, Span<int> normalSigns, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic sum = At(values, signs, 0), product = At(values, signs, 1), scale = At(values, signs, 2), divisor = At(values, signs, 3);
        Scale(At(normals, normalSigns, 7), Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), scale);
        Vector3d radial;
        if (candidate.RootOrdinal == -3)
            radial = MaterializeRadialDirection(new WideAxis3(geometry.Edge.X, default, geometry.Edge.Z),
                geometry.Input.Radius, candidate.Chart == -2 ? 1 : -1);
        else
        {
            At(normals, normalSigns, 0).CopyTo(product, -1); Fixed64 x = RoundRatio(product, scale, root);
            At(normals, normalSigns, 2).CopyTo(product, -1); Fixed64 z = RoundRatio(product, scale, root);
            radial = new Vector3d(x, Fixed64.Zero, z);
        }
        sum.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Multiply(At(normals, normalSigns, axis), At(normals, normalSigns, axis), root, product); sum.Add(product);
        }
        Multiply(scale, scale, root, divisor);
        bool represented = TryRoundSquareRootRatio(sum, divisor, root, out Fixed64 depth);
        return new FixedContactAnchors(new FixedPointAnchor(geometry.Input.Origin, geometry.Input.Rotation,
                RoundSegmentPoint(geometry.Input.Segment, At(normals, normalSigns, 5), At(normals, normalSigns, 6), root)),
            TriangleCircularGeometry.GetSupport(geometry.Input.Center, geometry.Input.ConeRotation, Signed192.Raw(geometry.Input.Height), radial, 1),
            MaterializeQuadraticNormal(normals, normalSigns, root, geometry.Input.ConeRotation), represented ? depth : Fixed64.MaxValue, !represented);
    }

}
