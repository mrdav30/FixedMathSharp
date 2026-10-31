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

    /// <summary>Exact cone-local identity chart supplied by a retained rational native source.</summary>
    internal ConeFiniteSectionFrame(Fixed64 height, Fixed64 radius, Signed320 coordinateDenominator)
    {
        Height = height; Radius = radius;
        ShapeFrame = new CylinderPolytopeFrame(height, radius, coordinateDenominator);
        FullHeight = WideArithmetic.AddSigned320(ShapeFrame.Cap, ShapeFrame.Cap);
    }

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

    internal ConeFiniteSectionFrame(in ScaledRigidFrame3d sourceFrame,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        if (sourceFrame.IsRigid)
        {
            this = new ConeFiniteSectionFrame(sourceFrame.RigidOrigin, sourceFrame.Rotation,
                coneCenter, coneRotation, height, radius);
            return;
        }
        if (height <= Fixed64.Zero || height > (Fixed64)8192)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero || radius > (Fixed64)4096)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!coneRotation.IsNormalized())
            throw new ArgumentException("The cone rotation must be normalized.", nameof(coneRotation));
        Height = height; Radius = radius;
        ShapeFrame = new CylinderPolytopeFrame(sourceFrame, coneCenter, coneRotation,
            Signed192.Raw(height), radius);
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
        if (ShapeFrame.HasExactScale)
            return IntersectsWideQuadratic(start, delta, lowerN, lowerD, upperN, upperD);
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

    /// <summary>Tests a pretransformed source subsegment, retaining its exact original parameter.</summary>
    internal bool IntersectsTransformedSegment(WideAxis3 start, WideAxis3 end, in RationalSegmentInterval clip)
    {
        if (!clip.IsClipped) return IntersectsTransformedSegment(start, end);
        WideAxis3 delta = new(WideArithmetic.SubtractSigned320(end.X, start.X),
            WideArithmetic.SubtractSigned320(end.Y, start.Y), WideArithmetic.SubtractSigned320(end.Z, start.Z));
        return TryGetAxialInterval(start.Y, delta.Y, out var ln, out var ld, out var un, out var ud)
            && CapsuleFiniteSectionFrame.IntersectsQuadratic(QuadraticProductWide(delta, delta),
                QuadraticProductWide(start, delta), QuadraticProductWide(start, start), ln, ld, un, ud, clip);
    }

    internal bool ContainsTransformedPoint(WideAxis3 point) => point.Y.Sign >= 0
        && WideArithmetic.SubtractSigned320(point.Y, FullHeight).Sign <= 0
        && (ShapeFrame.HasExactScale ? QuadraticProductWide(point, point).Sign : QuadraticProduct(point, point).Sign) <= 0;

    internal Signed832 QuadraticProductWide(WideAxis3 left, WideAxis3 right)
    {
        if (!ShapeFrame.HasExactScale)
            return Signed832.ExtendValue(QuadraticProduct(left, right));
        // Admitted doubled coordinates are <306 bits and H/R raw <46.
        // Three squared terms and two extent factors stay below 707 bits.
        Signed832 radial = WideArithmetic.AddSigned832(Product(left.X, right.X), Product(left.Z, right.Z));
        Signed832 axial = Product(left.Y, right.Y);
        Signed192 height = Signed192.Raw(Height), radius = Signed192.Raw(Radius);
        return WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned832(radial, height), height),
            WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned832(axial, radius), radius));
    }

    private bool IntersectsWideQuadratic(WideAxis3 start, WideAxis3 delta,
        Signed320 lowerN, Signed320 lowerD, Signed320 upperN, Signed320 upperD)
    {
        Signed832 a = QuadraticProductWide(delta, delta), b = QuadraticProductWide(start, delta),
            c = QuadraticProductWide(start, start);
        return CapsuleFiniteSectionFrame.IntersectsQuadratic(a, b, c, lowerN, lowerD, upperN, upperD);
    }

    private static Signed832 Product(Signed320 first, Signed320 second) =>
        WideArithmetic.MultiplySigned576ToSigned832(Signed576.ExtendValue(first), second);

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
