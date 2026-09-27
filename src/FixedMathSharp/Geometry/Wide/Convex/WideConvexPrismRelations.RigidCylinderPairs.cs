//=======================================================================
// WideConvexPrismRelations.RigidCylinderPairs.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains routines for computing contact information between pairs of rigid, finite cylinders.
/// </content>
internal static partial class WideConvexPrismRelations
{
    internal static bool TryGetCenteredFiniteCylindersContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out FixedContactAnchors contact)
    {
        // A zero-radius cylinder is exactly its finite core segment. Keep
        // that family with the complete cylinder/capsule authority, including
        // oblique side/rim minima and zero-depth segment/segment contacts.
        if (secondRadius == Fixed64.Zero)
            return TryGetCenteredFiniteCylinderCapsuleContact(
                firstCenter, firstRotation, firstLocalAxis, firstLength, firstRadius,
                secondCenter, secondRotation, secondLocalAxis, secondLength, Fixed64.Zero,
                out contact);
        if (firstRadius == Fixed64.Zero)
        {
            if (!TryGetCenteredFiniteCylinderCapsuleContact(
                    secondCenter, secondRotation, secondLocalAxis, secondLength, secondRadius,
                    firstCenter, firstRotation, firstLocalAxis, firstLength, Fixed64.Zero,
                    out FixedContactAnchors reversed))
            {
                contact = default;
                return false;
            }
            contact = new FixedContactAnchors(reversed.SecondAnchor, reversed.FirstAnchor,
                -reversed.Normal, reversed.Depth, reversed.DepthIsClamped);
            return true;
        }
        if (!TryGetPositiveRadiusCylinderPairPenetration(
                firstCenter, firstRotation, firstLocalAxis, Signed192.Raw(firstLength), firstRadius,
                secondCenter, secondRotation, secondLocalAxis, Signed192.Raw(secondLength), secondRadius,
                out Vector3d normal, out Fixed64 depth, out bool depthIsClamped))
        {
            contact = default;
            return false;
        }
        contact = new FixedContactAnchors(
            WideGeometry.GetCenteredCylinderSupportAnchor(firstCenter, firstRotation,
                firstLocalAxis, firstLength, firstRadius, normal),
            WideGeometry.GetCenteredCylinderSupportAnchor(secondCenter, secondRotation,
                secondLocalAxis, secondLength, secondRadius, -normal),
            normal, depth, depthIsClamped);
        return true;
    }

    /// <summary>
    /// Exact relation for admitted rigid cylinders with positive radii. Raw full
    /// lengths may occupy 64 unsigned bits, including twice a slab half-thickness.
    /// This does not widen authored positions, radii, or quaternion admission.
    /// </summary>
    internal static bool TryGetPositiveRadiusCylinderPairPenetration(
        Vector3d firstCenter, FixedQuaternion firstRotation, Vector3d firstLocalAxis,
        Signed192 firstLength, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Vector3d secondLocalAxis,
        Signed192 secondLength, Fixed64 secondRadius,
        out Vector3d normal, out Fixed64 depth, out bool depthIsClamped)
    {
        normal = default;
        depth = default;
        depthIsClamped = false;
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            firstRotation,
            firstLocalAxis,
            out Signed192 firstAxisX,
            out Signed192 firstAxisY,
            out Signed192 firstAxisZ,
            out Signed192 firstDenominator);
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            secondRotation,
            secondLocalAxis,
            out Signed192 secondAxisX,
            out Signed192 secondAxisY,
            out Signed192 secondAxisZ,
            out Signed192 secondDenominator);
        var firstAxis = new RigidAxis3(
            firstAxisX,
            firstAxisY,
            firstAxisZ,
            firstDenominator);
        var secondAxis = new RigidAxis3(
            secondAxisX,
            secondAxisY,
            secondAxisZ,
            secondDenominator);
        Axis3 firstCandidate = firstAxis.ToWide();
        Axis3 secondCandidate = secondAxis.ToWide();
        if (!Cross(firstCandidate, secondCandidate).IsZero)
        {
            return TryGetNonparallelCylinderPairPenetration(firstCenter, firstRotation, firstLocalAxis,
                firstLength, firstRadius, secondCenter, secondRotation, secondLocalAxis,
                secondLength, secondRadius, out normal, out depth, out depthIsClamped);
        }
        var best = default(CylinderCylinderPenetration);
        if (!TryKeepCylinderCylinderAxis(
                firstCandidate,
                firstCenter,
                firstAxis,
                firstLength,
                firstRadius,
                secondCenter,
                secondAxis,
                secondLength,
                secondRadius,
                ref best))
        {
            return false;
        }
        // Coincident or overlapping coaxial cores have a zero closest-core
        // direction. Their radial minimum still needs a representative;
        // reuse the cylinder/capsule authored-frame tie direction. For an
        // off-axis pair the closest-core radial candidate remains minimal.
        CylinderCapsuleDirection perpendicular = GetCylinderCapsulePerpendicular(firstRotation, firstLocalAxis);
        if (!TryKeepCylinderCylinderAxis(
                new Axis3(Signed320.NarrowValue(perpendicular.X),
                    Signed320.NarrowValue(perpendicular.Y), Signed320.NarrowValue(perpendicular.Z)),
                firstCenter, firstAxis, firstLength, firstRadius,
                secondCenter, secondAxis, secondLength, secondRadius, ref best))
        {
            return false;
        }
        // Parallel finite cylinders need their cap axis and the perpendicular
        // center difference, not clamped closest points on their core segments.
        // |A_i|<2^100 and |d_i|<2^64 bound the two cross products by
        // 2^167 and 2^269 per component, including subtraction headroom.
        // The squared norm is <2^540, alignment <2^371, and radial plane
        // product <2^742, within the existing 576/832-bit slots.
        var difference = new Axis3(
            Signed320.ExtendValue(WideArithmetic.Difference(secondCenter.X, firstCenter.X)),
            Signed320.ExtendValue(WideArithmetic.Difference(secondCenter.Y, firstCenter.Y)),
            Signed320.ExtendValue(WideArithmetic.Difference(secondCenter.Z, firstCenter.Z)));
        Axis3 radial = Cross(firstCandidate, Cross(difference, firstCandidate));
        if (!TryKeepCylinderCylinderAxis(radial,
                firstCenter, firstAxis, firstLength, firstRadius,
                secondCenter, secondAxis, secondLength, secondRadius, ref best))
        {
            return false;
        }
        Axis3 orientedAxis = best.Negate
            ? new Axis3(WideArithmetic.Negate(best.Axis.X),
                WideArithmetic.Negate(best.Axis.Y), WideArithmetic.Negate(best.Axis.Z))
            : best.Axis;
        normal = WideNormalization.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        GetRoundedCylinderCylinderDepth(best, out depth, out depthIsClamped);
        return true;
    }


    private static bool TryKeepCylinderCylinderAxis(
        Axis3 axis,
        Vector3d firstCenter,
        RigidAxis3 firstAxis,
        Signed192 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        RigidAxis3 secondAxis,
        Signed192 secondLength,
        Fixed64 secondRadius,
        ref CylinderCylinderPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 firstAlignment =
            GetAxisProjection(axis, firstAxis);
        Signed576 secondAlignment =
            GetAxisProjection(axis, secondAxis);
        Signed576 centerProjection = GetDifferenceProjection(
            secondCenter,
            firstCenter,
            axis);
        Signed576 firstAxial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude576(firstAlignment),
                firstLength),
            secondAxis.RotationDenominator);
        Signed576 secondAxial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude576(secondAlignment),
                secondLength),
            firstAxis.RotationDenominator);
        Signed576 scaledCenter = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    GetMagnitude576(centerProjection),
                    firstAxis.RotationDenominator),
                secondAxis.RotationDenominator),
            Signed192.Raw(Fixed64.Two));
        Signed704 rational = Signed704.ExtendValue(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(
                    firstAxial,
                    secondAxial),
                scaledCenter));
        Signed576 axisSquared = GetAxisSquared(axis);
        Signed576 commonWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    firstAxis.RotationDenominator,
                    secondAxis.RotationDenominator)),
            Signed192.Raw(Fixed64.Two));
        _ = Signed192.TryNarrowSigned(
            commonWide,
            out Signed192 common);
        // Only cap normals and perpendiculars reach this parallel selector.
        // The radial support is respectively zero or the sum of both radii.
        // Each admitted radius raw value is <= 2^63-1, so their exact sum
        // is <= 2^64-2 and must not be narrowed through Fixed64.
        ulong radiusRaw = firstAlignment.IsZero
            ? unchecked((ulong)firstRadius.m_rawValue + (ulong)secondRadius.m_rawValue)
            : 0UL;
        var depth = new CylinderPairDepth(rational, common, axisSquared, radiusRaw);
        if (GetCylinderPairDepthSign(depth) < 0)
            return false;
        if (!best.HasValue
            || CompareCylinderPairDepths(
                depth,
                best.Depth) < 0)
        {
            best = new CylinderCylinderPenetration(
                axis,
                centerProjection.Sign < 0,
                depth);
        }
        return true;
    }

    private static int GetCylinderPairDepthSign(in CylinderPairDepth depth) =>
        WideArithmetic.CompareSignedLinearRadicalToZero(
            Signed832.ExtendValue(depth.Rational),
            Signed704.ExtendValue(Signed576.ExtendValue(depth.RadialCoefficient)),
            depth.AxisSquared,
            Signed320.ExtendValue(Signed192.Signed(1L)));

    private static int CompareCylinderPairDepths(
        in CylinderPairDepth left,
        in CylinderPairDepth right)
    {
        // All candidates in this query share the same positive Common.
        Signed576 unit = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1L)));
        return WideArithmetic.CompareRadialProjectionDepths(
            left.Rational, left.RadialCoefficient,
            Signed832.ExtendValue(left.AxisSquared), unit, left.AxisSquared,
            right.Rational, right.RadialCoefficient,
            Signed832.ExtendValue(right.AxisSquared), unit, right.AxisSquared);
    }
}
