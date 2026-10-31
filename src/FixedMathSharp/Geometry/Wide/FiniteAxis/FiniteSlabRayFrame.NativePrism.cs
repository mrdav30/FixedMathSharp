//=======================================================================
// FiniteSlabRayFrame.NativePrism.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Admits exact native affine source charts without rebuilding rounded triangles.</content>
internal readonly partial struct FiniteSlabRayFrame
{
    internal FiniteSlabRayFrame(Fixed64 height, Fixed64 radius, Signed320 coordinateDenominator,
        in WideDirection3d normal, Signed832 planeConstant)
    {
        this = default;
        Section = new(height, radius, coordinateDenominator);
        Plane = CapsulePlaneRayFrame.ForCapsulePointRay(Fixed64.Zero, radius, coordinateDenominator, normal, planeConstant);
        Origin = Center = Vector3d.Zero; Rotation = SlabRotation = FixedQuaternion.Identity;
        Radius = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(coordinateDenominator,
            Signed320.ExtendValue(Section.RadialFrame.ShapeFrame.Radius)));
        RadiusValue = radius; HalfThickness = FullHeight = height;
        Signed320 bound = WideArithmetic.AddSigned320(Radius, Section.Cap);
        Span<ulong> magnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(bound), magnitude);
        int boundBits = WideArithmetic.GetMagnitudeBitLength(magnitude);
        WideArithmetic.GetMagnitude(normal.SquaredLength, magnitude);
        int normalPower = (WideArithmetic.GetMagnitudeBitLength(magnitude) - 1) / 2;
        int shift = Math.Max(0, boundBits + 2 - normalPower);
        RayNormal = new(ScalePowerOfTwo(Signed320.NarrowValue(normal.X), shift),
            ScalePowerOfTwo(Signed320.NarrowValue(normal.Y), shift), ScalePowerOfTwo(Signed320.NarrowValue(normal.Z), shift));
        RayNormalSquared = RayNormal.SquaredLength;
    }
}
