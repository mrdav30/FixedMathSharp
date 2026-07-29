//=======================================================================
// WideFiniteAxisIntersection.RigidPointRelation.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Computes high-precision point-to-axis relations (projection, radial distance, and
/// associated denominators) for a point expressed in the local space of a rotated rigid frame.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    #region Nested Types

    private readonly struct RigidLocalPointRelation
    {
        internal readonly Signed192 X;
        internal readonly Signed192 Y;
        internal readonly Signed192 Z;
        internal readonly Signed192 Denominator;
        internal readonly Signed192 AxisSquared;
        internal readonly Signed320 Projection;
        internal readonly Signed576 RadialNumerator;
        internal readonly Signed320 RadialDenominator;

        internal RigidLocalPointRelation(
            Signed192 x,
            Signed192 y,
            Signed192 z,
            Signed192 denominator,
            Signed192 axisSquared,
            Signed320 projection,
            Signed576 radialNumerator,
            Signed320 radialDenominator)
        {
            X = x;
            Y = y;
            Z = z;
            Denominator = denominator;
            AxisSquared = axisSquared;
            Projection = projection;
            RadialNumerator = radialNumerator;
            RadialDenominator = radialDenominator;
        }
    }

    #endregion

    private static void GetRigidLocalPointRelation(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        out RigidLocalPointRelation relation)
    {
        WideOrientedBox.GetRelativeLocalPointNumerators(
            point,
            center,
            frameRotation,
            out Signed192 x,
            out Signed192 y,
            out Signed192 z,
            out Signed192 denominator);
        Signed192 axisSquared = GetDot(
            localAxisDirection,
            Vector3d.Zero,
            localAxisDirection,
            Vector3d.Zero);
        Signed320 projection = GetProjection(
            x,
            y,
            z,
            localAxisDirection);
        Signed320 squaredDistance = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(x, x),
                WideArithmetic.MultiplySigned192(y, y)),
            WideArithmetic.MultiplySigned192(z, z));
        Signed576 radialNumerator = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                squaredDistance,
                Signed320.ExtendValue(axisSquared)),
            WideArithmetic.MultiplySigned320(
                projection,
                projection));
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(
            denominator,
            denominator);
        Signed576 wideRadialDenominator = WideArithmetic.MultiplySigned320(
            denominatorSquared,
            Signed320.ExtendValue(axisSquared));
        _ = Signed320.TryNarrowSigned(
            wideRadialDenominator,
            out Signed320 radialDenominator);
        relation = new RigidLocalPointRelation(
            x,
            y,
            z,
            denominator,
            axisSquared,
            projection,
            radialNumerator,
            radialDenominator);
    }
}
