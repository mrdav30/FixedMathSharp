//=======================================================================
// ConeFiniteSectionFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact finite-cone coordinates, closed admission, and rational segment clipping.</summary>
internal readonly struct ConeFiniteSectionFrame
{
    internal readonly CylinderPolytopeFrame ShapeFrame;
    internal readonly Fixed64 Height, Radius;
    internal readonly Signed320 FullHeight;

    internal ConeFiniteSectionFrame(Vector3d origin, FixedQuaternion rotation,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!rotation.IsNormalized())
            throw new ArgumentException("The authored rotation must be normalized.", nameof(rotation));
        if (!coneRotation.IsNormalized())
            throw new ArgumentException("The cone rotation must be normalized.", nameof(coneRotation));
        Height = height; Radius = radius;
        ShapeFrame = new CylinderPolytopeFrame(coneCenter, coneRotation, height, radius, origin, rotation);
        FullHeight = WideArithmetic.AddSigned320(ShapeFrame.Cap, ShapeFrame.Cap);
    }

    internal WideAxis3 Transform(Vector3d point)
    {
        WideAxis3 centered = ShapeFrame.Transform(point);
        return new WideAxis3(centered.X, WideArithmetic.SubtractSigned320(ShapeFrame.Cap, centered.Y), centered.Z);
    }

    internal bool IntersectsTransformedSegment(WideAxis3 start, WideAxis3 end)
    {
        WideAxis3 delta = new(WideArithmetic.SubtractSigned320(end.X, start.X),
            WideArithmetic.SubtractSigned320(end.Y, start.Y), WideArithmetic.SubtractSigned320(end.Z, start.Z));
        if (!TryGetAxialInterval(start.Y, delta.Y, out Signed320 lowerN, out Signed320 lowerD,
                out Signed320 upperN, out Signed320 upperD))
            return false;
        Signed576 a = QuadraticProduct(delta, delta), b = QuadraticProduct(start, delta), c = QuadraticProduct(start, start);
        if (WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(Signed832.ExtendValue(a), Signed832.ExtendValue(b), Signed832.ExtendValue(c), lowerN, lowerD) <= 0
            || WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(Signed832.ExtendValue(a), Signed832.ExtendValue(b), Signed832.ExtendValue(c), upperN, upperD) <= 0)
            return true;
        // A concave/linear quadratic attains its minimum at an endpoint.
        // The remaining convex quadratic has one stationary parameter -b/a.
        if (a.Sign <= 0 || b.Sign >= 0)
            return false;
        Signed576 numerator = WideArithmetic.SubtractSigned576(default, b);
        if (CompareProducts(numerator, lowerD, a, lowerN) < 0
            || CompareProducts(numerator, upperD, a, upperN) > 0)
            return false;
        // F decreases from t=0 to -b/a. The clipped lower endpoint lies
        // in that interval and has F>0, so F(0)=c must also be positive.
        System.Diagnostics.Debug.Assert(c.Sign > 0);
        return WideArithmetic.CompareNonNegativeProducts(Signed832.ExtendValue(b), Signed832.ExtendValue(b),
            Signed832.ExtendValue(a), Signed832.ExtendValue(c)) >= 0;
    }

    internal bool ContainsTransformedPoint(WideAxis3 point) => point.Y.Sign >= 0
        && WideArithmetic.SubtractSigned320(point.Y, FullHeight).Sign <= 0
        && QuadraticProduct(point, point).Sign <= 0;

    internal Signed576 QuadraticProduct(WideAxis3 left, WideAxis3 right)
    {
        // Unreduced doubled rigid coordinates and axial differences <198
        // bits; raw H/R <63. Both products and their signed sum <528 bits.
        // Axial clipping excludes the opposite nappe before F<=0 admission.
        Signed576 radial = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(left.X, right.X),
            WideArithmetic.MultiplySigned320(left.Z, right.Z));
        Signed576 axial = WideArithmetic.MultiplySigned320(left.Y, right.Y);
        return WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(radial, Signed192.Raw(Height), Signed192.Raw(Height)),
            WideArithmetic.MultiplySigned576(axial, Signed192.Raw(Radius), Signed192.Raw(Radius)));
    }

    internal bool TryGetAxialInterval(Signed320 start, Signed320 velocity,
        out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
    {
        lowerN = default;
        lowerD = upperN = upperD = Signed320.ExtendValue(Signed192.Signed(1));
        if (velocity.IsZero)
            return start.Sign >= 0 && WideArithmetic.SubtractSigned320(start, FullHeight).Sign <= 0;
        Signed320 first = WideArithmetic.Negate(start);
        Signed320 second = WideArithmetic.SubtractSigned320(FullHeight, start);
        Signed320 divisor = velocity;
        if (velocity.Sign < 0)
        {
            (first, second) = (WideArithmetic.Negate(second), WideArithmetic.Negate(first));
            divisor = WideArithmetic.Negate(divisor);
        }
        if (second.Sign < 0 || WideArithmetic.SubtractSigned320(first, divisor).Sign > 0)
            return false;
        if (first.Sign > 0) { lowerN = first; lowerD = divisor; }
        if (WideArithmetic.SubtractSigned320(second, divisor).Sign < 0) { upperN = second; upperD = divisor; }
        return true;
    }

    private static int CompareProducts(Signed576 left, Signed320 leftScale, Signed576 right, Signed320 rightScale)
    {
        // <528+198 bits fits Signed832; endpoint polynomial evaluation uses
        // the existing wide owner's transient products rather than narrowing.
        return WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(left, leftScale),
            WideArithmetic.MultiplySigned576ToSigned832(right, rightScale)).Sign;
    }
}
