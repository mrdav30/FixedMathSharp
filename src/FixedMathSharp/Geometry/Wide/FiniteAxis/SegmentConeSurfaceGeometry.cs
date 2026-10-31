//=======================================================================
// SegmentConeSurfaceGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>One exact centered frame shared by the segment's support charts.</summary>
internal readonly struct SegmentConeSurfaceGeometry
{
    internal readonly SegmentConeSurfaceInput Input;
    internal readonly ConeFiniteSectionFrame Finite;
    internal readonly WideDirection3d A, B, Edge, BaseOffset;
    internal readonly Signed576 Radius;
    // Affine coordinates have at most 306 bits, dots at most 615. Weighted
    // points can exceed 832 bits and remain in the existing borrowed banks.
    internal readonly Signed320 RawScale;
    internal readonly Signed576 Cap;
    internal Signed576 FullHeight => WideArithmetic.AddSigned576(Cap, Cap);
    internal int Words => Input.Words;
    internal int FamilyWords => Input.FamilyWords;
    internal int ValueWords => Input.ValueWords;
    internal readonly int ValueShift;

    internal SegmentConeSurfaceGeometry(in SegmentConeSurfaceInput input)
    {
        Input = input;
        Finite = input.HasSourceFrame
            ? new ConeFiniteSectionFrame(input.SourceFrame, input.Center, input.ConeRotation, input.Height, input.Radius)
            : new ConeFiniteSectionFrame(input.Origin, input.Rotation, input.Center, input.ConeRotation, input.Height, input.Radius);
        WideAxis3 first = Finite.ShapeFrame.Transform(input.Segment.Start), second = Finite.ShapeFrame.Transform(input.Segment.End);
        Span<Signed320> components = stackalloc Signed320[9]
        {
            WideArithmetic.AddSigned320(Finite.ShapeFrame.CoordinateDenominator, Finite.ShapeFrame.CoordinateDenominator),
            first.X, first.Y, first.Z, second.X, second.Y, second.Z, Finite.ShapeFrame.Cap,
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(Finite.ShapeFrame.CoordinateDenominator, Finite.ShapeFrame.Radius))
        };
        // One common reduction preserves the entire line and shape together.
        // It removes harmless composed-frame factors before fixed-degree charts.
        if (input.IsAffine) WideArithmetic.ReduceCommonScale(components);
        RawScale = components[0]; A = new(components[1], components[2], components[3]); B = new(components[4], components[5], components[6]);
        Cap = Signed576.ExtendValue(components[7]); Radius = Signed576.ExtendValue(components[8]);
        Edge = TriangleCircularGeometry.Subtract(B, A);
        BaseOffset = new(A.X, WideArithmetic.AddSigned576(A.Y, Cap), A.Z);
        Span<Signed576> coordinates = stackalloc Signed576[8]
        { A.X, A.Y, A.Z, B.X, B.Y, B.Z, Cap, Radius };
        ValueShift = TriangleCircularGeometry.GetValueShift(coordinates, 4);
    }
}
