//=======================================================================
// SegmentConeSurfaceCandidates.NormalQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Exact linear forms on retained normals, before normal rounding.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    internal static bool TryGetNormalDotSign(SegmentConeSurfaceCandidate candidate, WideAxis3 authoredCovector, out int sign)
    {
        sign = 0;
        if (candidate.Family != ConeSurfaceFamily.None && candidate.Family != ConeSurfaceFamily.SegmentInterval)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        WideRigidProjection.TransformLocalAxis(geometry.Finite.ShapeFrame.Basis, authoredCovector,
            out Signed576 x, out Signed576 y, out Signed576 z);
        if (candidate.Feature == ConeSurfaceFeature.Base)
        {
            sign = y.Sign;
            return true;
        }
        if (candidate.Feature == ConeSurfaceFeature.Apex)
        {
            bool admitted = TryBuildApex(geometry, candidate.RootOrdinal, out _, out _, out Signed832 nx, out Signed832 ny, out Signed832 nz);
            System.Diagnostics.Debug.Assert(admitted);
            sign = RationalNormalDotSign(nx, ny, nz, x, y, z);
            return true;
        }
        if (candidate.Feature == ConeSurfaceFeature.Side && geometry.Radius.IsZero)
            return false;
        if (candidate.Feature == ConeSurfaceFeature.Rim && candidate.Chart >= 0)
        {
            sign = GetRimRootNormalDotSign(geometry, candidate, x, y, z);
            return true;
        }
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        if (candidate.Feature == ConeSurfaceFeature.Side)
        {
            bool admitted = BuildSideNormal(geometry, candidate.RootOrdinal, candidate.Chart, values, signs, root);
            System.Diagnostics.Debug.Assert(admitted);
        }
        else if (candidate.RootOrdinal <= -3)
        {
            bool admitted = BuildMeridionalRim(geometry, candidate.RootOrdinal, candidate.Chart == -2 ? 1 : -1, values, signs, root);
            System.Diagnostics.Debug.Assert(admitted);
        }
        else
        {
            bool admitted = TryGetRimPoint(geometry, candidate.RootOrdinal, out WideAxis3 radial,
                out Signed576 radialCoefficient, out Signed320 axial, out _, out _);
            System.Diagnostics.Debug.Assert(admitted);
            Import(radial.SquaredLength, root);
            BuildRimPointNormal(geometry, radial, radialCoefficient, axial, radial.SquaredLength,
                candidate.Chart == -2 ? 1 : -1, values, signs);
        }
        sign = QuadraticNormalDotSign(values, signs, root, x, y, z);
        return true;
    }

    internal static bool TryGetRadialNormalDotSign(SegmentConeSurfaceCandidate candidate, Vector2d direction,
        WideAxis3 authoredCovector, out int sign)
    {
        sign = 0;
        if (candidate.Family != ConeSurfaceFamily.RotationCircle || direction == Vector2d.Zero)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        WideAxis3 radial = new(Signed320.ExtendValue(Signed192.Raw(direction.X)), default, Signed320.ExtendValue(Signed192.Raw(direction.Y)));
        if (candidate.Feature == ConeSurfaceFeature.Side)
        {
            BuildSidePointNormal(geometry, radial, 1, values, signs, root);
            if (!TryBuildSideFoot(geometry, candidate.RootOrdinal, values, signs, root, out _))
                return false;
        }
        else
        {
            bool admitted = TryGetRimPoint(geometry, candidate.RootOrdinal, out _, out _, out Signed320 axial, out _, out _);
            System.Diagnostics.Debug.Assert(admitted);
            BuildRimCircleNormal(geometry, radial, axial, values, signs, root);
            if (!AdmitsEndpointNormal(geometry.Edge, candidate.RootOrdinal, values, signs, root))
                return false;
        }
        WideRigidProjection.TransformLocalAxis(geometry.Finite.ShapeFrame.Basis, authoredCovector,
            out Signed576 x, out Signed576 y, out Signed576 z);
        sign = QuadraticNormalDotSign(values, signs, root, x, y, z);
        return true;
    }

    private static int QuadraticNormalDotSign(Span<ulong> normals, Span<int> normalSigns, ReadOnlySpan<ulong> root,
        Signed576 x, Signed576 y, Signed576 z)
    {
        // A full Signed320 authored covector transforms to <452 bits.
        // The largest retained normal coefficients are <996 bits, so every
        // coefficient of this linear form fits the existing forty words.
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic sum = At(values, signs, 0), product = At(values, signs, 1);
        sum.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(At(normals, normalSigns, axis), axis == 0 ? x : axis == 1 ? y : z, product);
            sum.Add(product);
        }
        return sum.Sign(root);
    }

    private static int RationalNormalDotSign(Signed832 nx, Signed832 ny, Signed832 nz, Signed576 x, Signed576 y, Signed576 z)
    {
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<ulong> normal = Slot(values, 0), covector = Slot(values, 1), product = Slot(values, 2), sum = Slot(values, 3);
        sum.Clear();
        int sign = 0;
        for (int axis = 0; axis < 3; axis++)
        {
            Signed832 n = axis == 0 ? nx : axis == 1 ? ny : nz;
            Signed576 c = axis == 0 ? x : axis == 1 ? y : z;
            normal.Clear(); covector.Clear();
            WideArithmetic.GetMagnitude(n, normal[..13]); WideArithmetic.GetMagnitude(c, covector[..9]);
            WideArithmetic.MultiplyMagnitudes(normal, covector, product);
            CylinderContactAlgebra.Add(product, n.Sign * c.Sign, sum, ref sign);
        }
        return sign;
    }

    private static int GetRimRootNormalDotSign(in SegmentConeSurfaceGeometry geometry, SegmentConeSurfaceCandidate candidate,
        Signed576 x, Signed576 y, Signed576 z)
    {
        GetRimBasis(geometry.Edge, candidate.Chart, out WideAxis3 first, out WideAxis3 second);
        Span<ulong> data = stackalloc ulong[RimSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[RimSlots];
        TriangleConeRimContacts.BuildConeParameter(geometry.BaseOffset, geometry.Radius, geometry.RawScale,
            geometry.Input.Height, geometry.Input.Radius, first, second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5], candidate.RootOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        Span<ulong> query = stackalloc ulong[2 * Words];
        Span<sbyte> querySigns = stackalloc sbyte[2];
        query.Clear();
        for (int power = 0; power < 2; power++)
        {
            WideAxis3 n = power == 0 ? first : second;
            Signed832 dot = WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, n.X), WideArithmetic.MultiplySigned576ToSigned832(y, n.Y)),
                WideArithmetic.MultiplySigned576ToSigned832(z, n.Z));
            WideArithmetic.GetMagnitude(dot, Slot(query, power)[..13]); querySigns[power] = (sbyte)dot.Sign;
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
    }
}
