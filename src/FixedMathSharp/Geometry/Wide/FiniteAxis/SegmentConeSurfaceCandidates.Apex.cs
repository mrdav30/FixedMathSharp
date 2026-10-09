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
        for (int endpoint = geometry.Edge.IsZero ? 0 : -1; endpoint <= (geometry.Edge.IsZero ? 0 : 1); endpoint++)
        {
            if (!TryBuildApex(geometry, endpoint, out Signed576 numerator, out Signed576 denominator,
                    out Signed832 x, out Signed832 y, out Signed832 z))
                continue;
            ConeSurfacePointLocation location = numerator.IsZero ? ConeSurfacePointLocation.Start
                : WideArithmetic.SubtractSigned576(denominator, numerator).IsZero ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Interior;
            // A point in the finite cone at apex height is the apex itself.
            ConeSurfaceFamily family = y.IsZero ? ConeSurfaceFamily.NormalCone : ConeSurfaceFamily.None;
            if (family == ConeSurfaceFamily.NormalCone && !HasApexTouchDomain(geometry, endpoint))
                continue;
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Apex, family, -1, endpoint, location));
        }
    }

    private static bool HasApexTouchDomain(in SegmentConeSurfaceGeometry geometry, int endpoint)
    {
        if (geometry.Radius.IsZero)
            return true;
        // Scale an apex normal to ny=-R. Its radial components range over
        // the disk of radius H, so tangent projection spans -R*Ey +/- H|Er|.
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        Span<ulong> root = stackalloc ulong[Words];
        ContactQuadratic limit = ContactQuadratic.At(values, signs, 0);
        Signed576 axial = WideArithmetic.MultiplySigned320(geometry.Edge.Y, Signed192.Raw(geometry.Input.Radius));
        limit.Set(axial);
        if (endpoint == 1 || endpoint < 0 && axial.Sign > 0)
            limit.MultiplySign(-1);
        Import(Signed320.ExtendValue(Signed192.Raw(geometry.Input.Height)), limit.Radical); limit.Signs[1] = 1;
        Import(CircularRimContactAlgebra.RadialDot(geometry.Edge, geometry.Edge), root);
        return limit.Sign(root) >= 0;
    }

    private static bool TryBuildApex(in SegmentConeSurfaceGeometry geometry, int endpoint,
        out Signed576 numerator, out Signed576 denominator, out Signed832 x, out Signed832 y, out Signed832 z)
    {
        WideAxis3 offset = new(geometry.A.X, WideArithmetic.SubtractSigned320(geometry.A.Y, geometry.Finite.ShapeFrame.Cap), geometry.A.Z);
        denominator = endpoint < 0 ? geometry.Edge.SquaredLength : Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)));
        numerator = endpoint < 0 ? WideArithmetic.SubtractSigned576(default, WideAxis3.Dot(offset, geometry.Edge))
            : endpoint == 0 ? default : denominator;
        x = y = z = default;
        if (numerator.Sign < 0 || WideArithmetic.SubtractSigned576(denominator, numerator).Sign < 0)
            return false;
        if (endpoint < 0 && (numerator.IsZero || WideArithmetic.SubtractSigned576(denominator, numerator).IsZero))
            return false;
        x = WeightedCoordinate(offset.X, geometry.Edge.X, numerator, denominator);
        y = WeightedCoordinate(offset.Y, geometry.Edge.Y, numerator, denominator);
        z = WeightedCoordinate(offset.Z, geometry.Edge.Z, numerator, denominator);
        if (!IsApexInwardNormal(x, y, z, geometry.Input.Height, geometry.Input.Radius))
            return false;
        Signed832 pointY = WideArithmetic.AddSigned832(y,
            WideArithmetic.MultiplySigned576ToSigned832(denominator, geometry.Finite.ShapeFrame.Cap));
        if (!ContainsCenteredRatio(geometry, x, pointY, z, denominator))
            return false;
        if (endpoint >= 0)
        {
            // Endpoint normals use unscaled input coordinates, so the local
            // tangent dot is below 398 bits, within the existing axis owner.
            WideAxis3 n = endpoint == 0 ? offset : new WideAxis3(geometry.B.X,
                WideArithmetic.SubtractSigned320(geometry.B.Y, geometry.Finite.ShapeFrame.Cap), geometry.B.Z);
            int dot = WideAxis3.Dot(n, geometry.Edge).Sign;
            if (endpoint == 0 ? dot > 0 : dot < 0)
                return false;
        }
        return true;
    }

    private static Signed832 WeightedCoordinate(Signed320 start, Signed320 delta, Signed576 numerator, Signed576 denominator) =>
        WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(denominator, start),
            WideArithmetic.MultiplySigned576ToSigned832(numerator, delta));

    private static bool IsApexInwardNormal(Signed832 x, Signed832 y, Signed832 z, Fixed64 height, Fixed64 radius) =>
        y.Sign <= 0 && CompareWeightedSquares(x, z, radius, y, height) <= 0;

    private static bool ContainsCenteredRatio(in SegmentConeSurfaceGeometry geometry,
        Signed832 x, Signed832 y, Signed832 z, Signed576 denominator)
    {
        Signed832 axial = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(denominator, geometry.Finite.ShapeFrame.Cap), y);
        // Apex polar admission and the principal rim heights already place
        // every caller at or below the apex; the base bound is independent.
        System.Diagnostics.Debug.Assert(axial.Sign >= 0);
        if (WideArithmetic.SubtractSigned832(axial,
                WideArithmetic.MultiplySigned576ToSigned832(denominator, geometry.Finite.FullHeight)).Sign > 0)
            return false;
        return CompareWeightedSquares(x, z, geometry.Input.Height, axial, geometry.Input.Radius) <= 0;
    }

    private static int CompareWeightedSquares(Signed832 first, Signed832 second, Fixed64 radialWeight,
        Signed832 axial, Fixed64 axialWeight)
    {
        // Even complete Signed832 inputs need at most 1665 square bits and
        // 126 weight bits. The signed sum fits the existing 2560-bit slots.
        Span<ulong> values = stackalloc ulong[5 * Words];
        Span<ulong> a = Slot(values, 0), b = Slot(values, 1), radial = Slot(values, 2);
        Span<ulong> axialSquare = Slot(values, 3), factor = Slot(values, 4);
        a.Clear(); b.Clear();
        WideArithmetic.GetMagnitude(first, a[..13]);
        WideArithmetic.GetMagnitude(second, b[..13]);
        WideArithmetic.MultiplyMagnitudes(a, a, radial);
        WideArithmetic.MultiplyMagnitudes(b, b, axialSquare);
        WideArithmetic.AddMagnitudeInto(axialSquare, radial);
        Import(WideArithmetic.MultiplySigned192(Signed192.Raw(radialWeight), Signed192.Raw(radialWeight)), factor);
        WideArithmetic.MultiplyMagnitudes(radial, factor, a);
        b.Clear(); WideArithmetic.GetMagnitude(axial, b[..13]);
        WideArithmetic.MultiplyMagnitudes(b, b, axialSquare);
        Import(WideArithmetic.MultiplySigned192(Signed192.Raw(axialWeight), Signed192.Raw(axialWeight)), factor);
        WideArithmetic.MultiplyMagnitudes(axialSquare, factor, b);
        return WideArithmetic.CompareMagnitudeEqualLength(a, b);
    }

    internal static FixedContactAnchors GetApexContact(SegmentConeSurfaceCandidate candidate)
    {
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        bool admitted = TryBuildApex(geometry, candidate.RootOrdinal, out Signed576 numerator, out Signed576 denominator,
            out Signed832 x, out Signed832 y, out Signed832 z);
        System.Diagnostics.Debug.Assert(admitted);
        Vector3d normal = MaterializeRationalNormal(x, y, z, candidate.Input.ConeRotation);
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic one = ContactQuadratic.At(values, signs, 0), divisor = ContactQuadratic.At(values, signs, 1);
        one.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)))); divisor.Set(denominator);
        Span<ulong> metric = stackalloc ulong[Words], component = stackalloc ulong[Words], square = stackalloc ulong[Words];
        metric.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            component.Clear(); WideArithmetic.GetMagnitude(axis == 0 ? x : axis == 1 ? y : z, component[..13]);
            WideArithmetic.MultiplyMagnitudes(component, component, square);
            WideArithmetic.AddMagnitudeInto(square, metric);
        }
        Import(Signed320.ExtendValue(geometry.RawScale), component);
        bool represented = ContactQuadratic.TryRoundRootRatio(one, divisor, ReadOnlySpan<ulong>.Empty,
            metric, component, out Fixed64 depth);
        return ApexAnchors(candidate.Input, numerator, denominator, normal, represented ? depth : Fixed64.MaxValue, !represented);
    }

    private static FixedContactAnchors ApexAnchors(SegmentConeSurfaceInput input, Signed576 numerator, Signed576 denominator,
        Vector3d normal, Fixed64 depth, bool clamped)
    {
        Vector3d point = new(RoundSegmentCoordinate(input.Segment.Start.X, input.Segment.End.X, numerator, denominator),
            RoundSegmentCoordinate(input.Segment.Start.Y, input.Segment.End.Y, numerator, denominator),
            RoundSegmentCoordinate(input.Segment.Start.Z, input.Segment.End.Z, numerator, denominator));
        return new FixedContactAnchors(new FixedPointAnchor(input.Origin, input.Rotation, point),
            TriangleCircularGeometry.GetSupport(input.Center, input.ConeRotation, Signed192.Raw(input.Height), Vector3d.Zero, -1),
            normal, depth, clamped);
    }

    private static Fixed64 RoundSegmentCoordinate(Fixed64 start, Fixed64 end, Signed576 numerator, Signed576 denominator)
    {
        // The rational apex foot has <398-bit weights; an authored raw
        // difference adds at most 64 bits, keeping the exact coordinate <464.
        Signed576 value = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned576(denominator, Signed192.Raw(start)),
            WideArithmetic.MultiplySigned576(numerator, WideArithmetic.Difference(end, start)));
        Span<ulong> n = stackalloc ulong[9], d = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(value, n); WideArithmetic.GetMagnitude(denominator, d);
        bool represented = Fixed64.TryGetSignedRawRatio(n, d, value.Sign < 0, out Fixed64 result);
        System.Diagnostics.Debug.Assert(represented);
        return result;
    }

    private static Vector3d MaterializeRationalNormal(Signed832 x, Signed832 y, Signed832 z, FixedQuaternion rotation)
    {
        Span<ulong> local = stackalloc ulong[3 * Words], values = stackalloc ulong[7 * Words];
        Span<int> localSigns = stackalloc int[3], signs = stackalloc int[7];
        local.Clear(); values.Clear(); signs.Clear();
        WideArithmetic.GetMagnitude(x, Slot(local, 0)[..13]); localSigns[0] = x.Sign;
        WideArithmetic.GetMagnitude(y, Slot(local, 1)[..13]); localSigns[1] = y.Sign;
        WideArithmetic.GetMagnitude(z, Slot(local, 2)[..13]); localSigns[2] = z.Sign;
        WriteWorldDirection(new WideRationalBasis3d(rotation), local, localSigns, values, signs);
        return WideConvexPrismRelations.GetConvexContactCandidateNormal(new ConvexContactCandidate(values, signs, 0));
    }
}
