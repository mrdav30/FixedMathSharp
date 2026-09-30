//=======================================================================
// WideConvexPrismRelations.CylinderCapsuleFeatures.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Analytic normal strata and certified endpoint/rim contacts for a finite
/// cylinder and capsule. All intermediate positions retain authored rational
/// axes; only the selected normal and complete contact depth are rounded.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private readonly struct CylinderCapsuleDirection
    {
        internal readonly Signed576 X;
        internal readonly Signed576 Y;
        internal readonly Signed576 Z;

        internal CylinderCapsuleDirection(Signed576 x, Signed576 y, Signed576 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal CylinderCapsuleDirection(Axis3 axis)
            : this(Signed576.ExtendValue(axis.X), Signed576.ExtendValue(axis.Y), Signed576.ExtendValue(axis.Z)) { }

        internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;
    }

    private readonly struct CylinderCapsuleFeatureGeometry
    {
        internal readonly RigidAxis3 Cylinder;
        internal readonly RigidAxis3 Capsule;
        internal readonly Signed192 DifferenceX;
        internal readonly Signed192 DifferenceY;
        internal readonly Signed192 DifferenceZ;
        internal readonly Signed192 Scale;
        internal readonly Signed320 CenterX;
        internal readonly Signed320 CenterY;
        internal readonly Signed320 CenterZ;
        internal readonly Axis3 CylinderHalf;
        internal readonly Axis3 CapsuleHalf;
        internal readonly Signed320 CylinderSquared;
        internal readonly Signed192 CylinderLength;
        internal readonly Signed192 CapsuleLength;
        internal readonly Fixed64 Radius;

        internal CylinderCapsuleFeatureGeometry(
            Vector3d cylinderCenter, RigidAxis3 cylinder, Signed192 cylinderLength, Fixed64 radius,
            Vector3d capsuleCenter, RigidAxis3 capsule, Signed192 capsuleLength)
        {
            Cylinder = cylinder;
            Capsule = capsule;
            CylinderLength = cylinderLength;
            CapsuleLength = capsuleLength;
            Radius = radius;
            DifferenceX = WideArithmetic.Difference(capsuleCenter.X, cylinderCenter.X);
            DifferenceY = WideArithmetic.Difference(capsuleCenter.Y, cylinderCenter.Y);
            DifferenceZ = WideArithmetic.Difference(capsuleCenter.Z, cylinderCenter.Z);
            Signed192 denominators = Signed192.NarrowProven(
                WideArithmetic.MultiplySigned192(cylinder.RotationDenominator, capsule.RotationDenominator));
            Scale = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
                denominators, Signed192.Raw(Fixed64.Two)));
            CenterX = WideArithmetic.MultiplySigned192(DifferenceX, Scale);
            CenterY = WideArithmetic.MultiplySigned192(DifferenceY, Scale);
            CenterZ = WideArithmetic.MultiplySigned192(DifferenceZ, Scale);
            CylinderHalf = GetCylinderCapsuleScaledHalf(cylinder, cylinderLength, capsule.RotationDenominator);
            CapsuleHalf = GetCylinderCapsuleScaledHalf(capsule, capsuleLength, cylinder.RotationDenominator);
            CylinderSquared = GetAxisSquared(cylinder);
        }

        internal Axis3 EndpointOffset(int cylinderSign, int capsuleSign) => new(
            GetCylinderCapsuleEndpointCoordinate(CenterX, CylinderHalf.X, cylinderSign, CapsuleHalf.X, capsuleSign),
            GetCylinderCapsuleEndpointCoordinate(CenterY, CylinderHalf.Y, cylinderSign, CapsuleHalf.Y, capsuleSign),
            GetCylinderCapsuleEndpointCoordinate(CenterZ, CylinderHalf.Z, cylinderSign, CapsuleHalf.Z, capsuleSign));
    }

    private static Axis3 GetCylinderCapsuleScaledHalf(RigidAxis3 axis, Signed192 length, Signed192 otherDenominator) => new(
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(axis.X, length), otherDenominator)),
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(axis.Y, length), otherDenominator)),
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(axis.Z, length), otherDenominator)));

    private static Signed320 GetCylinderCapsuleEndpointCoordinate(
        Signed320 center, Signed320 cylinderHalf, int cylinderSign, Signed320 capsuleHalf, int capsuleSign)
    {
        Signed320 result = cylinderSign == 0 ? center : cylinderSign > 0
            ? WideArithmetic.SubtractSigned320(center, cylinderHalf)
            : WideArithmetic.AddSigned320(center, cylinderHalf);
        return capsuleSign == 0 ? result : capsuleSign > 0
            ? WideArithmetic.SubtractSigned320(result, capsuleHalf)
            : WideArithmetic.AddSigned320(result, capsuleHalf);
    }

    private static Signed576 DotCylinderCapsuleDirection(CylinderCapsuleDirection direction, RigidAxis3 axis) =>
        DotCylinderCapsuleDirection(direction, axis.X, axis.Y, axis.Z);

    private static Signed576 DotCylinderCapsuleDirection(
        CylinderCapsuleDirection direction, Signed192 x, Signed192 y, Signed192 z) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(direction.X, x), WideArithmetic.MultiplySigned576(direction.Y, y)),
            WideArithmetic.MultiplySigned576(direction.Z, z));

    private static CylinderCapsuleDirection ProjectCylinderCapsulePerpendicular(Axis3 offset, RigidAxis3 axis)
    {
        Signed320 squared = GetAxisSquared(axis);
        Signed576 dot = DotCylinderCapsuleDirection(new CylinderCapsuleDirection(offset), axis);
        return new CylinderCapsuleDirection(
            WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(offset.X, squared),
                WideArithmetic.MultiplySigned576(dot, axis.X)),
            WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(offset.Y, squared),
                WideArithmetic.MultiplySigned576(dot, axis.Y)),
            WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(offset.Z, squared),
                WideArithmetic.MultiplySigned576(dot, axis.Z)));
    }

    private static CylinderCapsuleDirection GetCylinderCapsulePerpendicular(
        FixedQuaternion rotation, Vector3d localAxis)
    {
        // A continuous radial tie follows the authored frame, so applying a
        // common rigid rotation also rotates the selected representative.
        Vector3d localPerpendicular = localAxis.X != Fixed64.Zero || localAxis.Y != Fixed64.Zero
            ? new Vector3d(localAxis.Y, -localAxis.X, Fixed64.Zero) : Vector3d.Right;
        WideOrientedBox.GetRotatedLocalAxisNumerators(rotation, localPerpendicular,
            out Signed192 x, out Signed192 y, out Signed192 z, out _);
        return new CylinderCapsuleDirection(new Axis3(Signed320.ExtendValue(x),
            Signed320.ExtendValue(y), Signed320.ExtendValue(z)));
    }

    private static void ImportCylinderCapsuleFeature(Signed192 value, Span<ulong> destination)
    {
        destination.Clear();
        WideArithmetic.GetMagnitude(value, out destination[2], out destination[1], out destination[0]);
    }

    private static void ImportCylinderCapsuleFeature(Signed320 value, Span<ulong> destination)
    {
        destination.Clear();
        WideArithmetic.GetMagnitude(value, out destination[4], out destination[3], out destination[2],
            out destination[1], out destination[0]);
    }

    private static void ImportCylinderCapsuleFeature(Signed576 value, Span<ulong> destination)
    {
        destination.Clear();
        WideArithmetic.GetMagnitude(value, destination[..9]);
    }

    private static void ImportCylinderCapsuleFeature(Signed832 value, Span<ulong> destination)
    {
        destination.Clear();
        WideArithmetic.GetMagnitude(value, destination[..13]);
    }

    private static Span<ulong> CylinderCapsuleFeatureSlot(Span<ulong> values, int slot) =>
        values.Slice(slot * ConvexContactCandidate.Words, ConvexContactCandidate.Words);

    private static bool BuildCylinderCapsuleAxisCandidate(
        in CylinderCapsuleFeatureGeometry geometry, CylinderCapsuleDirection direction,
        Span<ulong> values, Span<int> signs, out int gapSign)
    {
        gapSign = 0;
        if (direction.IsZero)
            return false;
        values.Clear();
        signs.Clear();
        Signed576 cylinderAlignment = DotCylinderCapsuleDirection(direction, geometry.Cylinder);
        Signed576 capsuleAlignment = DotCylinderCapsuleDirection(direction, geometry.Capsule);
        Signed576 centerAlignment = DotCylinderCapsuleDirection(direction,
            geometry.DifferenceX, geometry.DifferenceY, geometry.DifferenceZ);
        int orientation = centerAlignment.Sign < 0 ? -1 : 1;
        ImportCylinderCapsuleFeature(direction.X, CylinderCapsuleFeatureSlot(values, 0));
        ImportCylinderCapsuleFeature(direction.Y, CylinderCapsuleFeatureSlot(values, 1));
        ImportCylinderCapsuleFeature(direction.Z, CylinderCapsuleFeatureSlot(values, 2));
        signs[0] = direction.X.Sign * orientation;
        signs[1] = direction.Y.Sign * orientation;
        signs[2] = direction.Z.Sign * orientation;

        // Every generated rational direction is below 2^440. Dot products are
        // below 2^542, the common rational support below 2^674. The final
        // squared-gap coefficients are below 2^1780 (40 words give 2560 bits).
        Signed832 rational = WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(GetMagnitude576(cylinderAlignment),
                WideArithmetic.MultiplySigned192(geometry.CylinderLength, geometry.Capsule.RotationDenominator)),
            WideArithmetic.MultiplySigned576ToSigned832(GetMagnitude576(capsuleAlignment),
                WideArithmetic.MultiplySigned192(geometry.CapsuleLength, geometry.Cylinder.RotationDenominator))),
            WideArithmetic.MultiplySigned576ToSigned832(GetMagnitude576(centerAlignment), Signed320.ExtendValue(geometry.Scale)));

        const int words = ConvexContactCandidate.Words;
        Span<ulong> work = stackalloc ulong[10 * words];
        Span<ulong> u = CylinderCapsuleFeatureSlot(work, 0);
        Span<ulong> axisSquared = CylinderCapsuleFeatureSlot(work, 1);
        Span<ulong> product = CylinderCapsuleFeatureSlot(work, 2);
        Span<ulong> temporary = CylinderCapsuleFeatureSlot(work, 3);
        Span<ulong> plane = CylinderCapsuleFeatureSlot(work, 4);
        Span<ulong> p = CylinderCapsuleFeatureSlot(work, 5);
        Span<ulong> q = CylinderCapsuleFeatureSlot(work, 6);
        Span<ulong> pSquared = CylinderCapsuleFeatureSlot(work, 7);
        Span<ulong> qSquaredK = CylinderCapsuleFeatureSlot(work, 8);
        Span<ulong> f = CylinderCapsuleFeatureSlot(work, 9);
        axisSquared.Clear();
        for (int component = 0; component < 3; component++)
        {
            Span<ulong> coordinate = CylinderCapsuleFeatureSlot(values, component);
            WideArithmetic.MultiplyMagnitudes(coordinate, coordinate, product);
            WideArithmetic.AddMagnitudeInto(product, axisSquared);
        }
        ImportCylinderCapsuleFeature(geometry.CylinderSquared, u);
        WideArithmetic.MultiplyMagnitudes(axisSquared, u, product);
        ImportCylinderCapsuleFeature(cylinderAlignment, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, temporary, plane);
        WideArithmetic.SubtractEqualMagnitudes(product, plane, temporary);
        temporary.CopyTo(plane);
        ImportCylinderCapsuleFeature(rational, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, u, p);
        ImportCylinderCapsuleFeature(WideArithmetic.MultiplySigned192(
            Signed192.Raw(geometry.Radius), geometry.Scale), q);
        Span<ulong> k = CylinderCapsuleFeatureSlot(values, 9);
        WideArithmetic.MultiplyMagnitudes(u, plane, k);
        WideArithmetic.MultiplyMagnitudes(p, p, pSquared);
        WideArithmetic.MultiplyMagnitudes(q, q, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, k, qSquaredK);
        WideArithmetic.AddEqualMagnitudes(pSquared, qSquaredK, CylinderCapsuleFeatureSlot(values, 7));
        WideArithmetic.MultiplyMagnitudes(p, q, temporary);
        WideArithmetic.AddEqualMagnitudes(temporary, temporary, CylinderCapsuleFeatureSlot(values, 8));
        signs[7] = WideArithmetic.GetActiveMagnitudeLength(CylinderCapsuleFeatureSlot(values, 7)) == 0 ? 0 : 1;
        signs[8] = WideArithmetic.GetActiveMagnitudeLength(CylinderCapsuleFeatureSlot(values, 8)) == 0 ? 0 : rational.Sign;
        signs[9] = WideArithmetic.GetActiveMagnitudeLength(k) == 0 ? 0 : 1;
        ImportCylinderCapsuleFeature(geometry.Scale, f);
        WideArithmetic.MultiplyMagnitudes(f, u, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, temporary, product);
        WideArithmetic.MultiplyMagnitudes(product, axisSquared, CylinderCapsuleFeatureSlot(values, 10));
        signs[10] = 1;
        gapSign = rational.Sign < 0
            ? WideArithmetic.CompareMagnitudeEqualLength(qSquaredK, pSquared)
            : rational.Sign > 0 || WideArithmetic.GetActiveMagnitudeLength(qSquaredK) != 0 ? 1 : 0;
        return true;
    }

    private static bool BuildCylinderCapsuleEndpointRimCandidate(
        in CylinderCapsuleFeatureGeometry geometry, int cylinderSign, int capsuleSign,
        Span<ulong> values, Span<int> signs, out int gapSign)
    {
        gapSign = -1;
        Axis3 offset = geometry.EndpointOffset(cylinderSign, capsuleSign);
        var direction = new CylinderCapsuleDirection(offset);
        Signed576 axial = DotCylinderCapsuleDirection(direction, geometry.Cylinder);
        if (axial.Sign * cylinderSign < 0)
            return false;
        Signed576 g = GetAxisSquared(offset);
        Signed832 q = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(g, geometry.CylinderSquared),
            WideArithmetic.MultiplySigned576ToSigned832(axial, axial));
        // On the radial axis the closest feature is a cap, not a rim. Its
        // normal is already present in the analytic boundary candidates.
        if (q.IsZero)
            return false;
        Signed320 rf = WideArithmetic.MultiplySigned192(Signed192.Raw(geometry.Radius), geometry.Scale);
        Signed576 rfSquared = WideArithmetic.MultiplySigned320(rf, rf);
        if (WideArithmetic.SubtractSigned832(q,
                WideArithmetic.MultiplySigned576ToSigned832(rfSquared, geometry.CylinderSquared)).Sign < 0)
            return false;

        values.Clear();
        signs.Clear();
        const int words = ConvexContactCandidate.Words;
        Span<ulong> work = stackalloc ulong[3 * words];
        Span<ulong> first = CylinderCapsuleFeatureSlot(work, 0);
        Span<ulong> second = CylinderCapsuleFeatureSlot(work, 1);
        Span<ulong> third = CylinderCapsuleFeatureSlot(work, 2);
        ImportCylinderCapsuleFeature(q, first);
        ImportCylinderCapsuleFeature(geometry.CylinderSquared, second);
        Span<ulong> root = CylinderCapsuleFeatureSlot(values, 6);
        WideArithmetic.MultiplyMagnitudes(first, second, root);
        root.CopyTo(CylinderCapsuleFeatureSlot(values, 9));
        signs[6] = signs[9] = 1;

        CylinderCapsuleDirection radial = ProjectCylinderCapsulePerpendicular(offset, geometry.Cylinder);
        Signed832 nx = WideArithmetic.MultiplySigned576ToSigned832(radial.X, rf);
        Signed832 ny = WideArithmetic.MultiplySigned576ToSigned832(radial.Y, rf);
        Signed832 nz = WideArithmetic.MultiplySigned576ToSigned832(radial.Z, rf);
        ImportCylinderCapsuleFeature(nx, CylinderCapsuleFeatureSlot(values, 0));
        ImportCylinderCapsuleFeature(ny, CylinderCapsuleFeatureSlot(values, 1));
        ImportCylinderCapsuleFeature(nz, CylinderCapsuleFeatureSlot(values, 2));
        signs[0] = -nx.Sign;
        signs[1] = -ny.Sign;
        signs[2] = -nz.Sign;
        ImportCylinderCapsuleFeature(offset.X, CylinderCapsuleFeatureSlot(values, 3));
        ImportCylinderCapsuleFeature(offset.Y, CylinderCapsuleFeatureSlot(values, 4));
        ImportCylinderCapsuleFeature(offset.Z, CylinderCapsuleFeatureSlot(values, 5));
        signs[3] = offset.X.Sign;
        signs[4] = offset.Y.Sign;
        signs[5] = offset.Z.Sign;

        if (!geometry.CapsuleLength.IsZero)
        {
            Signed576 projection = DotCylinderCapsuleDirection(direction, geometry.Capsule);
            Signed576 radialProjection = DotCylinderCapsuleDirection(radial, geometry.Capsule);
            Signed832 rational = WideArithmetic.MultiplySigned576ToSigned832(radialProjection, rf);
            ImportCylinderCapsuleFeature(rational, first);
            ImportCylinderCapsuleFeature(projection, second);
            if (GetConvexContactCandidateQuadraticSign(first, -rational.Sign, second, projection.Sign, root) * capsuleSign < 0)
                return false;
        }

        Signed832 gapRational = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.AddSigned576(g, rfSquared), geometry.CylinderSquared);
        ImportCylinderCapsuleFeature(gapRational, CylinderCapsuleFeatureSlot(values, 7));
        ImportCylinderCapsuleFeature(WideArithmetic.AddSigned320(rf, rf), CylinderCapsuleFeatureSlot(values, 8));
        signs[7] = gapRational.Sign;
        signs[8] = -rf.Sign;
        ImportCylinderCapsuleFeature(geometry.Scale, first);
        WideArithmetic.MultiplyMagnitudes(first, first, third);
        ImportCylinderCapsuleFeature(geometry.CylinderSquared, second);
        WideArithmetic.MultiplyMagnitudes(third, second, CylinderCapsuleFeatureSlot(values, 10));
        signs[10] = 1;
        // A zero residual has no normal. Shared cap/side boundaries represent
        // this exact-touch case, including the continuous tie at the rim.
        return GetConvexContactCandidateQuadraticSign(CylinderCapsuleFeatureSlot(values, 7), signs[7],
            CylinderCapsuleFeatureSlot(values, 8), signs[8], root) > 0;
    }

    private static bool BuildCylinderCapsuleProjectedCornerCandidate(
        in CylinderCapsuleFeatureGeometry geometry, Axis3 major, int cylinderSign,
        Span<ulong> values, Span<int> signs, out int gapSign)
    {
        gapSign = -1;
        Axis3 offset = geometry.EndpointOffset(cylinderSign, 0);
        Signed576 axial = DotCylinderCapsuleDirection(new CylinderCapsuleDirection(offset), geometry.Cylinder);
        if (axial.Sign * cylinderSign <= 0)
            return false;
        Signed576 e = GetAxisSquared(major);
        Signed576 majorProjection = DotCylinderCapsuleDirection(new CylinderCapsuleDirection(major),
            geometry.DifferenceX, geometry.DifferenceY, geometry.DifferenceZ);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(geometry.Radius), Signed192.Raw(geometry.Radius));
        if (WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(majorProjection, majorProjection),
                WideArithmetic.MultiplySigned576ToSigned832(e, radiusSquared)).Sign <= 0)
            return false;

        // With perpendicular axes the projected cylinder is a rectangle.
        // Both coordinate excesses are positive, so its nearest corner gives
        // the sole non-boundary minimum on the capsule-interior normal plane.
        values.Clear();
        signs.Clear();
        CylinderCapsuleDirection projected = ProjectCylinderCapsulePerpendicular(offset, geometry.Capsule);
        Signed320 b = GetAxisSquared(geometry.Capsule);
        Signed320 rf = WideArithmetic.MultiplySigned192(Signed192.Raw(geometry.Radius), geometry.Scale);
        Signed576 rfb = WideArithmetic.MultiplySigned320(rf, b);
        Signed832 nx = WideArithmetic.MultiplySigned576ToSigned832(rfb, major.X);
        Signed832 ny = WideArithmetic.MultiplySigned576ToSigned832(rfb, major.Y);
        Signed832 nz = WideArithmetic.MultiplySigned576ToSigned832(rfb, major.Z);
        ImportCylinderCapsuleFeature(nx, CylinderCapsuleFeatureSlot(values, 0));
        ImportCylinderCapsuleFeature(ny, CylinderCapsuleFeatureSlot(values, 1));
        ImportCylinderCapsuleFeature(nz, CylinderCapsuleFeatureSlot(values, 2));
        signs[0] = -nx.Sign * majorProjection.Sign;
        signs[1] = -ny.Sign * majorProjection.Sign;
        signs[2] = -nz.Sign * majorProjection.Sign;
        ImportCylinderCapsuleFeature(projected.X, CylinderCapsuleFeatureSlot(values, 3));
        ImportCylinderCapsuleFeature(projected.Y, CylinderCapsuleFeatureSlot(values, 4));
        ImportCylinderCapsuleFeature(projected.Z, CylinderCapsuleFeatureSlot(values, 5));
        signs[3] = projected.X.Sign;
        signs[4] = projected.Y.Sign;
        signs[5] = projected.Z.Sign;
        ImportCylinderCapsuleFeature(e, CylinderCapsuleFeatureSlot(values, 6));
        ImportCylinderCapsuleFeature(e, CylinderCapsuleFeatureSlot(values, 9));
        signs[6] = signs[9] = 1;

        const int words = ConvexContactCandidate.Words;
        Span<ulong> work = stackalloc ulong[5 * words];
        Span<ulong> squared = CylinderCapsuleFeatureSlot(work, 0);
        Span<ulong> product = CylinderCapsuleFeatureSlot(work, 1);
        Span<ulong> first = CylinderCapsuleFeatureSlot(work, 2);
        Span<ulong> second = CylinderCapsuleFeatureSlot(work, 3);
        Span<ulong> third = CylinderCapsuleFeatureSlot(work, 4);
        squared.Clear();
        for (int component = 3; component < 6; component++)
        {
            Span<ulong> coordinate = CylinderCapsuleFeatureSlot(values, component);
            WideArithmetic.MultiplyMagnitudes(coordinate, coordinate, product);
            WideArithmetic.AddMagnitudeInto(product, squared);
        }
        ImportCylinderCapsuleFeature(rfb, first);
        WideArithmetic.MultiplyMagnitudes(first, first, product);
        WideArithmetic.AddMagnitudeInto(product, squared);
        WideArithmetic.MultiplyMagnitudes(squared, CylinderCapsuleFeatureSlot(values, 9),
            CylinderCapsuleFeatureSlot(values, 7));
        Signed832 dot = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned576(GetMagnitude576(majorProjection), geometry.Scale), b);
        ImportCylinderCapsuleFeature(dot, second);
        WideArithmetic.MultiplyMagnitudes(first, second, product);
        WideArithmetic.AddEqualMagnitudes(product, product, CylinderCapsuleFeatureSlot(values, 8));
        signs[7] = 1;
        // Zero-radius cylinders use the segment feature family instead.
        signs[8] = -1;
        ImportCylinderCapsuleFeature(geometry.Scale, first);
        ImportCylinderCapsuleFeature(b, second);
        WideArithmetic.MultiplyMagnitudes(first, second, third);
        WideArithmetic.MultiplyMagnitudes(third, third, product);
        WideArithmetic.MultiplyMagnitudes(product, CylinderCapsuleFeatureSlot(values, 9),
            CylinderCapsuleFeatureSlot(values, 10));
        signs[10] = 1;
        return true;
    }

    private static void KeepCylinderCapsuleCandidate(
        ReadOnlySpan<ulong> values, ReadOnlySpan<int> signs, int gapSign,
        Span<ulong> bestValues, Span<int> bestSigns, ref int bestGapSign, ref bool hasBest)
    {
        var candidate = new ConvexContactCandidate(values, signs, gapSign);
        if (hasBest && CompareConvexContactCandidates(candidate,
                new ConvexContactCandidate(bestValues, bestSigns, bestGapSign)) >= 0)
            return;
        values.CopyTo(bestValues);
        signs.CopyTo(bestSigns);
        bestGapSign = gapSign;
        hasBest = true;
    }

    private static void KeepCylinderCapsuleDirection(
        in CylinderCapsuleFeatureGeometry geometry, CylinderCapsuleDirection direction,
        Span<ulong> values, Span<int> signs, Span<ulong> bestValues, Span<int> bestSigns,
        ref int bestGapSign, ref bool hasBest)
    {
        if (BuildCylinderCapsuleAxisCandidate(geometry, direction, values, signs, out int gapSign))
            KeepCylinderCapsuleCandidate(values, signs, gapSign, bestValues, bestSigns, ref bestGapSign, ref hasBest);
    }

    internal static bool TryGetCenteredFiniteCylinderCapsuleContact(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength, Fixed64 cylinderRadius,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation, Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength, Fixed64 capsuleRadius, out FixedContactAnchors contact)
    {
        if (!TryGetCenteredFiniteCylinderCapsulePenetration(
                cylinderCenter, cylinderRotation, cylinderLocalAxis, Signed192.Raw(cylinderLength), cylinderRadius,
                capsuleCenter, capsuleRotation, capsuleLocalAxis, capsuleLength, capsuleRadius,
                out Vector3d normal, out Fixed64 depth, out bool depthIsClamped))
        {
            contact = default;
            return false;
        }
        contact = CreateCylinderCapsuleFeatureContact(normal, depth, depthIsClamped,
            cylinderCenter, cylinderRotation, cylinderLocalAxis, cylinderLength, cylinderRadius,
            capsuleCenter, capsuleRotation, capsuleLocalAxis, capsuleLength, capsuleRadius);
        return true;
    }

    /// <summary>
    /// Exact closed relation for an admitted finite cylinder and capsule. The
    /// positive cylinder full length may occupy 64 unsigned raw bits, including
    /// twice a slab half-thickness; all other authored inputs retain their
    /// ordinary admission bounds. The cylinder-to-capsule normal and complete
    /// nearest-even depth are rounded only after exact feature selection.
    /// Separation returns default outputs; clamping reports conceptual overflow.
    /// </summary>
    internal static bool TryGetCenteredFiniteCylinderCapsulePenetration(
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Vector3d cylinderLocalAxis,
        Signed192 cylinderLength, Fixed64 cylinderRadius,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation, Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength, Fixed64 capsuleRadius,
        out Vector3d normal, out Fixed64 depth, out bool depthIsClamped)
    {
        WideOrientedBox.GetRotatedLocalAxisNumerators(cylinderRotation, cylinderLocalAxis,
            out Signed192 ax, out Signed192 ay, out Signed192 az, out Signed192 ad);
        WideOrientedBox.GetRotatedLocalAxisNumerators(capsuleRotation, capsuleLocalAxis,
            out Signed192 bx, out Signed192 by, out Signed192 bz, out Signed192 bd);
        var cylinder = new RigidAxis3(ax, ay, az, ad);
        var capsule = new RigidAxis3(bx, by, bz, bd);
        // Full length < 2^64 keeps scaled half-axes < 2^229 and endpoint
        // offsets < 2^231; the existing direction/candidate widths still hold.
        var geometry = new CylinderCapsuleFeatureGeometry(cylinderCenter, cylinder, cylinderLength,
            cylinderRadius, capsuleCenter, capsule, Signed192.Raw(capsuleLength));
        const int valueCount = ConvexContactCandidate.Words * ConvexContactCandidate.Slots;
        Span<ulong> values = stackalloc ulong[valueCount];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> bestValues = stackalloc ulong[valueCount];
        Span<int> bestSigns = stackalloc int[ConvexContactCandidate.Slots];
        int bestGapSign = 0;
        bool hasBest = false;

        // Stable ties: cap poles, capsule-interior principal boundaries, side
        // endpoint directions, degenerate segment endpoints, then ellipse.
        KeepCylinderCapsuleDirection(geometry, new CylinderCapsuleDirection(cylinder.ToWide()),
            values, signs, bestValues, bestSigns, ref bestGapSign, ref hasBest);
        int capGapSign = bestGapSign;
        Axis3 major = Cross(cylinder.ToWide(), capsule.ToWide());
        if (capsuleLength != Fixed64.Zero && !major.IsZero)
        {
            Axis3 minor = Cross(capsule.ToWide(), major);
            KeepCylinderCapsuleDirection(geometry, new CylinderCapsuleDirection(major),
                values, signs, bestValues, bestSigns, ref bestGapSign, ref hasBest);
            KeepCylinderCapsuleDirection(geometry, new CylinderCapsuleDirection(minor),
                values, signs, bestValues, bestSigns, ref bestGapSign, ref hasBest);
        }

        for (int capsuleSign = -1; capsuleSign <= 1; capsuleSign += 2)
        {
            CylinderCapsuleDirection side = ProjectCylinderCapsulePerpendicular(
                geometry.EndpointOffset(0, capsuleSign), cylinder);
            _ = BuildCylinderCapsuleAxisCandidate(geometry,
                side.IsZero ? GetCylinderCapsulePerpendicular(cylinderRotation, cylinderLocalAxis) : side,
                values, signs, out int sideGapSign);
            KeepCylinderCapsuleCandidate(values, signs, sideGapSign,
                bestValues, bestSigns, ref bestGapSign, ref hasBest);
            if ((major.IsZero || capsuleLength == Fixed64.Zero)
                && (capGapSign >= 0 || sideGapSign >= 0))
            {
                // Parallel cores sum to one finite cylinder. If the query is
                // inside either its axial slab or radial cylinder, the nearest
                // boundary is a cap or side; a rim cannot improve either gap.
                return MaterializeCylinderCapsulePenetration(
                    new ConvexContactCandidate(bestValues, bestSigns, bestGapSign), capsuleRadius,
                    out normal, out depth, out depthIsClamped);
            }
            for (int cylinderSign = -1; cylinderSign <= 1; cylinderSign += 2)
            {
                if (BuildCylinderCapsuleEndpointRimCandidate(geometry, cylinderSign, capsuleSign,
                        values, signs, out int rimGapSign))
                {
                    // The support point plus its outward residual proves a
                    // closest point of the whole convex Minkowski sum. Unlike
                    // an arbitrary separating direction, this is a global MTD.
                    return MaterializeCylinderCapsulePenetration(
                        new ConvexContactCandidate(values, signs, rimGapSign), capsuleRadius,
                        out normal, out depth, out depthIsClamped);
                }
                if (cylinderRadius == Fixed64.Zero)
                {
                    KeepCylinderCapsuleDirection(geometry,
                        new CylinderCapsuleDirection(geometry.EndpointOffset(cylinderSign, capsuleSign)),
                        values, signs, bestValues, bestSigns, ref bestGapSign, ref hasBest);
                }
            }
            // A zero-length core already returned through the cap/side
            // certificate or its unique outward endpoint/rim residual.
        }

        if (capsuleLength != Fixed64.Zero && !major.IsZero)
        {
            if (cylinderRadius == Fixed64.Zero)
            {
                for (int cylinderSign = -1; cylinderSign <= 1; cylinderSign += 2)
                    KeepCylinderCapsuleDirection(geometry,
                        ProjectCylinderCapsulePerpendicular(geometry.EndpointOffset(cylinderSign, 0), capsule),
                        values, signs, bestValues, bestSigns, ref bestGapSign, ref hasBest);
            }
            else if (GetAxisProjection(cylinder.ToWide(), capsule).IsZero)
            {
                for (int cylinderSign = -1; cylinderSign <= 1; cylinderSign += 2)
                    if (BuildCylinderCapsuleProjectedCornerCandidate(geometry, major, cylinderSign,
                            values, signs, out int cornerGapSign))
                        KeepCylinderCapsuleCandidate(values, signs, cornerGapSign,
                            bestValues, bestSigns, ref bestGapSign, ref hasBest);
            }
            else if (TryPrepareCylinderCapsuleEllipse(cylinderCenter, cylinder, cylinderLength,
                    cylinderRadius, capsuleCenter, capsule, out CylinderCapsuleEllipse ellipse)
                && TryImproveCylinderCapsuleEllipse(ellipse,
                    new ConvexContactCandidate(bestValues, bestSigns, bestGapSign), cylinderRadius, capsuleRadius,
                    out bool intersects, out normal, out depth, out depthIsClamped))
            {
                return intersects;
            }
        }
        return MaterializeCylinderCapsulePenetration(
            new ConvexContactCandidate(bestValues, bestSigns, bestGapSign), capsuleRadius,
            out normal, out depth, out depthIsClamped);
    }

    private static bool MaterializeCylinderCapsulePenetration(
        in ConvexContactCandidate candidate, Fixed64 capsuleRadius,
        out Vector3d normal, out Fixed64 depth, out bool depthIsClamped)
    {
        normal = default;
        depth = default;
        depthIsClamped = false;
        int depthSign = CompareConvexContactCandidateDepthToTwiceRaw(candidate, capsuleRadius, default);
        if (depthSign < 0)
            return false;
        if (depthSign > 0)
            GetRoundedConvexContactCandidateDepth(candidate, capsuleRadius, out depth, out depthIsClamped);
        normal = GetConvexContactCandidateNormal(candidate);
        return true;
    }

    private static FixedContactAnchors CreateCylinderCapsuleFeatureContact(
        Vector3d normal, Fixed64 depth, bool depthIsClamped,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength, Fixed64 cylinderRadius,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation, Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength, Fixed64 capsuleRadius) => new(
            WideGeometry.GetCenteredCylinderSupportAnchor(cylinderCenter, cylinderRotation, cylinderLocalAxis,
                cylinderLength, cylinderRadius, normal),
            WideGeometry.GetCenteredCapsuleSupportAnchor(capsuleCenter, capsuleRotation, capsuleLocalAxis,
                capsuleLength, capsuleRadius, -normal), normal, depth, depthIsClamped);
}
