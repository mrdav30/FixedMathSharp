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
    internal static bool TryGetNormalDotSign(in SegmentConeSurfaceCandidate candidate, WideAxis3 authoredCovector, out int sign)
    {
        int Words = candidate.Input.Words;
        sign = 0;
        if (candidate.Family != ConeSurfaceFamily.None && candidate.Family != ConeSurfaceFamily.SegmentInterval)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        WideDirection3d transformed = TransformNormalCovector(geometry, authoredCovector);
        Signed832 x = Signed832.ExtendValue(transformed.X), y = Signed832.ExtendValue(transformed.Y), z = Signed832.ExtendValue(transformed.Z);
        if (candidate.Feature == ConeSurfaceFeature.Base)
        {
            sign = y.Sign;
            return true;
        }
        if (candidate.Feature == ConeSurfaceFeature.Apex)
        {
            Span<ulong> data = stackalloc ulong[10 * Words]; Span<int> fieldSigns = stackalloc int[10];
            ContactQuadratic nx = At(data, fieldSigns, 0, Words), ny = At(data, fieldSigns, 1, Words), nz = At(data, fieldSigns, 2, Words),
                sum = At(data, fieldSigns, 3, Words), term = At(data, fieldSigns, 4, Words);
            bool admitted = TryBuildApex(geometry, candidate.RootOrdinal, out _, out _, nx, ny, nz);
            System.Diagnostics.Debug.Assert(admitted);
            Scale(nx, x, sum); Scale(ny, y, term); sum.Add(term); Scale(nz, z, term); sum.Add(term);
            sign = sum.Signs[0];
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
            bool admitted = TryGetRimPoint(geometry, candidate.RootOrdinal, out WideDirection3d radial,
                out Signed832 radialCoefficient, out Signed576 axial, out _, out _);
            System.Diagnostics.Debug.Assert(admitted);
            Import(radial.SquaredLength, root);
            BuildRimPointNormal(geometry, radial, radialCoefficient, axial, radial.SquaredLength,
                candidate.Chart == -2 ? 1 : -1, values, signs);
        }
        sign = QuadraticNormalDotSign(values, signs, root, x, y, z, Words);
        return true;
    }

    private static WideDirection3d TransformNormalCovector(in SegmentConeSurfaceGeometry geometry, in WideAxis3 covector) =>
        new WideDirection3d(covector).Transform(geometry.Input.HasSourceFrame
            ? new WideRationalBasis3d(geometry.Input.ConeRotation.Conjugate()) : geometry.Finite.ShapeFrame.Basis);

    private static int QuadraticNormalDotSign(Span<ulong> normals, Span<int> normalSigns, ReadOnlySpan<ulong> root,
        Signed832 x, Signed832 y, Signed832 z, int words = Words)
    {
        // Full Signed320 covectors transform to <452 bits. Isolated normal
        // coefficients <996 fit forty words; clipped-family coefficients
        // <2220 require the caller's sixty-four-word fields. Three complete
        // products and their signed sum fit those respective result widths.
        Span<ulong> values = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic sum = At(values, signs, 0, words), product = At(values, signs, 1, words);
        sum.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(At(normals, normalSigns, axis, words), axis == 0 ? x : axis == 1 ? y : z, product);
            sum.Add(product);
        }
        return sum.Sign(root);
    }

    private static int GetRimRootNormalDotSign(in SegmentConeSurfaceGeometry geometry, in SegmentConeSurfaceCandidate candidate,
        Signed832 x, Signed832 y, Signed832 z)
    {
        int Words = geometry.Words;
        GetRimBasis(geometry.Edge, candidate.Chart, out WideDirection3d first, out WideDirection3d second);
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
        Span<ulong> fields = stackalloc ulong[6 * Words]; Span<int> fieldSigns = stackalloc int[6];
        ContactQuadratic sum = At(fields, fieldSigns, 0, Words), term = At(fields, fieldSigns, 1, Words), scalar = At(fields, fieldSigns, 2, Words);
        for (int power = 0; power < 2; power++)
        {
            WideDirection3d n = power == 0 ? first : second;
            scalar.Set(x); Scale(scalar, n.X, sum); scalar.Set(y); Scale(scalar, n.Y, term); sum.Add(term);
            scalar.Set(z); Scale(scalar, n.Z, term); sum.Add(term);
            sum.Rational.CopyTo(Slot(query, power, Words)); querySigns[power] = (sbyte)sum.Signs[0];
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
    }
}
