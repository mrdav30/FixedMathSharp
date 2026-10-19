//=======================================================================
// SegmentConeSurfaceCandidates.Apex.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Rational apex feet and exact zero-depth apex normal families.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    private static void AccumulateApex(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {
        int Words = geometry.Words;
        Span<ulong> data = stackalloc ulong[6 * Words]; Span<int> fieldSigns = stackalloc int[6];
        ContactQuadratic x = ContactQuadratic.At(data, fieldSigns, 0, Words), y = ContactQuadratic.At(data, fieldSigns, 1, Words), z = ContactQuadratic.At(data, fieldSigns, 2, Words);
        for (int endpoint = geometry.Edge.IsZero ? 0 : -1; endpoint <= (geometry.Edge.IsZero ? 0 : 1); endpoint++)
        {
            if (!TryBuildApex(geometry, endpoint, out Signed832 numerator, out Signed832 denominator, x, y, z))
                continue;
            ConeSurfacePointLocation location = numerator.IsZero ? ConeSurfacePointLocation.Start
                : WideArithmetic.SubtractSigned832(denominator, numerator).IsZero ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Interior;
            // A point in the finite cone at apex height is the apex itself.
            ConeSurfaceFamily family = y.Signs[0] == 0 ? ConeSurfaceFamily.NormalCone : ConeSurfaceFamily.None;
            if (family == ConeSurfaceFamily.NormalCone && !HasApexTouchDomain(geometry, endpoint))
                continue;
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Apex, family, -1, endpoint, location));
        }
    }

    private static bool HasApexTouchDomain(in SegmentConeSurfaceGeometry geometry, int endpoint)
    {
        int Words = geometry.Words;
        if (geometry.Radius.IsZero)
            return true;
        // Scale an apex normal to ny=-R. Its radial components range over
        // the disk of radius H, so tangent projection spans -R*Ey +/- H|Er|.
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        Span<ulong> root = stackalloc ulong[Words];
        ContactQuadratic limit = ContactQuadratic.At(values, signs, 0, Words);
        Signed832 axial = Signed832.ExtendValue(WideArithmetic.MultiplySigned576(geometry.Edge.Y, Signed192.Raw(geometry.Input.Radius)));
        limit.Set(axial);
        if (endpoint == 1 || endpoint < 0 && axial.Sign > 0)
            limit.MultiplySign(-1);
        Import(Signed576.ExtendValue(Signed192.Raw(geometry.Input.Height)), limit.Radical); limit.Signs[1] = 1;
        Import(CircularRimContactAlgebra.RadialDot(geometry.Edge, geometry.Edge), root);
        return limit.Sign(root) >= 0;
    }

    private static bool TryGetApexParameter(in SegmentConeSurfaceGeometry geometry, int endpoint,
        out Signed832 numerator, out Signed832 denominator)
    {
        int Words = geometry.Words;
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        return TryBuildApex(geometry, endpoint, out numerator, out denominator,
            ContactQuadratic.At(values, signs, 0, Words), ContactQuadratic.At(values, signs, 1, Words), ContactQuadratic.At(values, signs, 2, Words));
    }

    private static bool TryBuildApex(in SegmentConeSurfaceGeometry geometry, int endpoint,
        out Signed832 numerator, out Signed832 denominator, ContactQuadratic x, ContactQuadratic y, ContactQuadratic z)
    {
        int Words = geometry.Words;
        WideDirection3d offset = new(geometry.A.X, WideArithmetic.SubtractSigned576(geometry.A.Y, geometry.Cap), geometry.A.Z);
        denominator = endpoint < 0 ? geometry.Edge.SquaredLength : Signed832.ExtendValue(Signed192.Signed(1));
        numerator = endpoint < 0 ? WideArithmetic.SubtractSigned832(default, WideDirection3d.Dot(offset, geometry.Edge))
            : endpoint == 0 ? default : denominator;
        x.Clear(); y.Clear(); z.Clear();
        if (numerator.Sign < 0 || WideArithmetic.SubtractSigned832(denominator, numerator).Sign < 0
            || endpoint < 0 && (numerator.IsZero || WideArithmetic.SubtractSigned832(denominator, numerator).IsZero)) return false;
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic term = ContactQuadratic.At(values, signs, 0, Words), divisor = ContactQuadratic.At(values, signs, 1, Words), pointY = ContactQuadratic.At(values, signs, 2, Words);
        divisor.Set(denominator);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic coordinate = axis == 0 ? x : axis == 1 ? y : z;
            ContactQuadratic.Scale(divisor, Component(offset, axis), coordinate);
            pointY.Set(numerator); ContactQuadratic.Scale(pointY, Component(geometry.Edge, axis), term); coordinate.Add(term);
        }
        if (y.Signs[0] > 0 || CompareWeightedSquares(x, z, geometry.Input.Radius, y, geometry.Input.Height) > 0) return false;
        y.CopyTo(pointY); ContactQuadratic.Scale(divisor, geometry.Cap, term); pointY.Add(term);
        if (!ContainsCenteredRatio(geometry, x, pointY, z, denominator)) return false;
        if (endpoint >= 0)
        {
            WideDirection3d n = endpoint == 0 ? offset : new(geometry.B.X,
                WideArithmetic.SubtractSigned576(geometry.B.Y, geometry.Cap), geometry.B.Z);
            int dot = WideDirection3d.Dot(n, geometry.Edge).Sign;
            if (endpoint == 0 ? dot > 0 : dot < 0) return false;
        }
        return true;
    }

    private static bool ContainsCenteredRatio(in SegmentConeSurfaceGeometry geometry,
        ContactQuadratic x, ContactQuadratic y, ContactQuadratic z, Signed832 denominator)
    {
        int Words = geometry.Words;
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic axial = ContactQuadratic.At(values, signs, 0, Words), divisor = ContactQuadratic.At(values, signs, 1, Words), bound = ContactQuadratic.At(values, signs, 2, Words);
        divisor.Set(denominator); ContactQuadratic.Scale(divisor, geometry.Cap, axial); axial.Add(y, -1);
        System.Diagnostics.Debug.Assert(axial.Signs[0] >= 0);
        ContactQuadratic.Scale(divisor, geometry.FullHeight, bound); bound.Add(axial, -1);
        return bound.Signs[0] >= 0 && CompareWeightedSquares(x, z, geometry.Input.Height, axial, geometry.Input.Radius) <= 0;
    }

    private static int CompareWeightedSquares(ContactQuadratic x, ContactQuadratic z, Fixed64 radialWeight,
        ContactQuadratic y, Fixed64 axialWeight)
    {
        int Words = x.FieldWords;
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic radial = ContactQuadratic.At(values, signs, 0, Words), axial = ContactQuadratic.At(values, signs, 1, Words), term = ContactQuadratic.At(values, signs, 2, Words), factor = ContactQuadratic.At(values, signs, 3, Words);
        ContactQuadratic.Multiply(x, x, ReadOnlySpan<ulong>.Empty, radial);
        ContactQuadratic.Multiply(z, z, ReadOnlySpan<ulong>.Empty, term); radial.Add(term);
        ContactQuadratic.Scale(radial, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(radialWeight), Signed192.Raw(radialWeight))), term);
        ContactQuadratic.Multiply(y, y, ReadOnlySpan<ulong>.Empty, axial);
        ContactQuadratic.Scale(axial, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(Signed192.Raw(axialWeight), Signed192.Raw(axialWeight))), factor);
        term.Add(factor, -1); return term.Signs[0];
    }

    internal static FixedContactAnchors GetApexContact(in SegmentConeSurfaceCandidate candidate)
    {
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input); int Words = geometry.Words;
        Span<ulong> values = stackalloc ulong[14 * Words]; Span<int> signs = stackalloc int[14];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, Words), y = ContactQuadratic.At(values, signs, 1, Words), z = ContactQuadratic.At(values, signs, 2, Words),
            divisor = ContactQuadratic.At(values, signs, 3, Words), metric = ContactQuadratic.At(values, signs, 4, Words), square = ContactQuadratic.At(values, signs, 5, Words), scaled = ContactQuadratic.At(values, signs, 6, Words);
        bool admitted = TryBuildApex(geometry, candidate.RootOrdinal, out Signed832 numerator, out Signed832 denominator, x, y, z);
        System.Diagnostics.Debug.Assert(admitted);
        Vector3d normal = MaterializeRationalNormal(x, y, z, candidate.Input.ConeRotation);
        metric.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic component = axis == 0 ? x : axis == 1 ? y : z;
            ContactQuadratic.Multiply(component, component, ReadOnlySpan<ulong>.Empty, square); metric.Add(square);
        }
        divisor.Set(denominator); ContactQuadratic.Scale(divisor, Signed576.ExtendValue(geometry.RawScale), scaled);
        ContactQuadratic.Multiply(scaled, scaled, ReadOnlySpan<ulong>.Empty, divisor);
        bool represented = ContactQuadratic.TryRoundSquareRootRatio(metric, divisor, ReadOnlySpan<ulong>.Empty, out Fixed64 depth);
        divisor.Set(denominator); scaled.Set(numerator);
        Vector3d point = RoundSourceSegmentPoint(geometry, scaled, divisor, ReadOnlySpan<ulong>.Empty);
        return new FixedContactAnchors(new FixedPointAnchor(candidate.Input.Origin, candidate.Input.Rotation, point),
            TriangleCircularGeometry.GetSupport(candidate.Input.Center, candidate.Input.ConeRotation, Signed192.Raw(candidate.Input.Height), Vector3d.Zero, -1),
            normal, represented ? depth : Fixed64.MaxValue, !represented);
    }

    private static Vector3d MaterializeRationalNormal(ContactQuadratic x, ContactQuadratic y, ContactQuadratic z, FixedQuaternion rotation)
    {
        int Words = x.FieldWords;
        Span<ulong> local = stackalloc ulong[3 * Words], values = stackalloc ulong[7 * Words];
        Span<int> localSigns = stackalloc int[3], signs = stackalloc int[7]; values.Clear(); signs.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic value = axis == 0 ? x : axis == 1 ? y : z;
            value.Rational.CopyTo(Slot(local, axis, Words)); localSigns[axis] = value.Signs[0];
        }
        WriteWorldDirection(new WideRationalBasis3d(rotation), local, localSigns, values[..(3 * Words)], signs[..3]);
        return WideConvexPrismRelations.GetConvexContactCandidateNormal(new ConvexContactCandidate(values, signs, 0, Words));
    }

    private static Vector3d MaterializeRationalNormal(Signed832 x, Signed832 y, Signed832 z, FixedQuaternion rotation)
    {
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic a = ContactQuadratic.At(values, signs, 0, Words), b = ContactQuadratic.At(values, signs, 1, Words), c = ContactQuadratic.At(values, signs, 2, Words);
        a.Set(x); b.Set(y); c.Set(z); return MaterializeRationalNormal(a, b, c, rotation);
    }
}
