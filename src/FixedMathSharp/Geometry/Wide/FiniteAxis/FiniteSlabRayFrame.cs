//=======================================================================
// FiniteSlabRayFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>One canonical source-plane metric shared by all admitted finite domains.</summary>
/// <content>Prepares exact slab charts and dispatches the shared finite-domain active sets.</content>
internal readonly partial struct FiniteSlabRayFrame
{
    internal readonly FiniteSlabSectionFrame Section;
    internal readonly CapsulePlaneRayFrame Plane;
    internal readonly Vector3d Origin, Center;
    internal readonly FixedQuaternion Rotation, SlabRotation;
    internal readonly Signed320 Radius, CoreCap;
    internal readonly bool HasCore;
    internal readonly FixedTriangle SourcePlane;
    internal readonly WideDirection3d RayNormal;
    internal readonly Signed832 RayNormalSquared;
    private readonly bool _wideParameters;
    internal bool UsesWideBanks => Section.RadialFrame.ShapeFrame.HasExactScale || _wideParameters;
    internal int FieldWords => UsesWideBanks ? FiniteSlabPolynomial.AffineWords : FiniteSlabPolynomial.Words;
    internal int SelectionStorageWords => FiniteSlabRaySelection.GetStorageWords(FieldWords);
    internal FiniteSlabRayFrame WithWideParameters() => UsesWideBanks ? this : new FiniteSlabRayFrame(this);
    private FiniteSlabRayFrame(in FiniteSlabRayFrame source) { this = source; _wideParameters = true; }
    // HalfThickness retains the supplied extent; FullHeight is zero for authored half-thickness slabs.
    internal readonly Fixed64 RadiusValue, HalfThickness, FullHeight;

    internal FiniteSlabRayFrame(FixedTriangle sourcePlane, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness, bool fullHeight = false)
        : this(sourcePlane, origin, rotation, center, slabRotation, length, radius, halfThickness, fullHeight, default, false)
    { }

    internal FiniteSlabRayFrame(FixedTriangle sourcePlane, in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness, bool fullHeight = false)
        : this(sourcePlane, sourceFrame.RigidOrigin, sourceFrame.Rotation, center, slabRotation, length, radius,
            halfThickness, fullHeight, sourceFrame, !sourceFrame.IsRigid)
    { }

    private FiniteSlabRayFrame(FixedTriangle sourcePlane, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
        bool fullHeight, in ScaledRigidFrame3d sourceFrame, bool exact)
    {
        _wideParameters = false;
        Section = exact ? new(sourceFrame, center, slabRotation, length, radius, halfThickness, fullHeight)
            : new(origin, rotation, center, slabRotation, length, radius, halfThickness, fullHeight);
        Plane = exact ? new(sourcePlane, sourceFrame, Section.RadialFrame) : new(sourcePlane, Section.RadialFrame);
        Origin = origin; Rotation = rotation; Center = center; SlabRotation = slabRotation;
        Radius = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
            Section.RadialFrame.ShapeFrame.CoordinateDenominator, Section.RadialFrame.ShapeFrame.Radius));
        CoreCap = Section.RadialFrame.ShapeFrame.Cap;
        HasCore = length != Fixed64.Zero;
        SourcePlane = sourcePlane;
        RadiusValue = radius; HalfThickness = halfThickness; FullHeight = fullHeight ? halfThickness : Fixed64.Zero;
        // Match the established plane-ray variable conditioning: numerator
        // coordinates include the rigid frame denominator, so an unscaled
        // primitive normal can make ordinary t enormous and u nearly one.
        Signed320 bound = WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(Radius, CoreCap), Section.Cap);
        Span<ulong> magnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(bound), magnitude);
        int boundBits = WideArithmetic.GetMagnitudeBitLength(magnitude);
        WideArithmetic.GetMagnitude(Plane.NormalSquared, magnitude);
        int normalPower = (WideArithmetic.GetMagnitudeBitLength(magnitude) - 1) / 2;
        int shift = Math.Max(0, boundBits + 2 - normalPower);
        // The resulting components need at most max(original bits,bound+3),
        // below 268 rigid or 352 affine bits; conditioning never adds a
        // fixed shift to an already maximal normal.
        RayNormal = exact ? new(ScalePowerOfTwo(Plane.Normal.X, shift), ScalePowerOfTwo(Plane.Normal.Y, shift),
            ScalePowerOfTwo(Plane.Normal.Z, shift))
            : new WideDirection3d(new WideAxis3(ScalePowerOfTwo(Signed320.NarrowValue(Plane.Normal.X), shift),
                ScalePowerOfTwo(Signed320.NarrowValue(Plane.Normal.Y), shift),
                ScalePowerOfTwo(Signed320.NarrowValue(Plane.Normal.Z), shift)));
        RayNormalSquared = RayNormal.SquaredLength;
    }

    internal Vector3d GetNormal(int orientation) =>
        Plane.GetWorldNormal(Rotation) * (Fixed64)(-orientation);

    /// <summary>Compares the positive and negative whole-plane exit depths before materialization.</summary>
    /// <remarks>The caller must establish plane/slab intersection before selecting a nonnegative exit.</remarks>
    internal int ComparePlaneExitDepths(ReadOnlySpan<Vector2d> offsets)
    {
        Signed832 sum = default;
        if (!offsets.IsEmpty)
        {
            Signed320 scale = WideArithmetic.AddSigned320(Section.RadialFrame.ShapeFrame.CoordinateDenominator,
                Section.RadialFrame.ShapeFrame.CoordinateDenominator);
            Signed832 low = default, high = default;
            for (int index = 0; index < offsets.Length; index++)
            {
                WideAxis3 point = new(Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(offsets[index].X))),
                    default, Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(offsets[index].Y))));
                Signed832 value = WideDirection3d.Dot(Plane.Normal, point);
                if (index == 0 || WideArithmetic.SubtractSigned832(value, low).Sign < 0) low = value;
                if (index == 0 || WideArithmetic.SubtractSigned832(value, high).Sign > 0) high = value;
            }
            sum = WideArithmetic.AddSigned832(low, high);
        }
        // Height caps and rounded footprints are symmetric. Affine coordinates
        // and normal components are <352 bits; these dot sums fit within 832 bits.
        return WideArithmetic.SubtractSigned832(sum,
            WideArithmetic.AddSigned832(Plane.PlaneConstant, Plane.PlaneConstant)).Sign;
    }

    /// <summary>Affine dimension of the shape face maximizing the oriented plane functional.</summary>
    internal int GetSupportDimension(ReadOnlySpan<Vector2d> prismOffsets, int orientation)
    {
        if (orientation != -1 && orientation != 1) throw new ArgumentOutOfRangeException(nameof(orientation));
        // A zero normal coefficient leaves that factor of the extrusion free.
        // The stadium disk and core share the X/Z plane, whose dimension is at most two.
        int height = Plane.Normal.Y.IsZero && Section.Cap.Sign > 0 ? 1 : 0;
        bool axial = Plane.Normal.X.IsZero && Plane.Normal.Z.IsZero;
        if (prismOffsets.IsEmpty)
        {
            int footprint = axial && RadiusValue > Fixed64.Zero ? 2
                : HasCore && Plane.Normal.Z.IsZero ? 1 : 0;
            return height + footprint;
        }
        _ = FiniteSlabSectionFrame.GetPrismWinding(prismOffsets);
        if (axial) return height + 2;
        Signed576 maximum = default; Vector2d vertex = default; bool tied = false;
        for (int index = 0; index < prismOffsets.Length; index++)
        {
            Vector2d offset = prismOffsets[index];
            Signed576 projection = WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(Plane.Normal.X, offset.X.m_rawValue),
                WideArithmetic.MultiplySigned576(Plane.Normal.Z, offset.Y.m_rawValue));
            if (orientation < 0) projection = WideArithmetic.SubtractSigned576(default, projection);
            int order = WideArithmetic.SubtractSigned576(projection, maximum).Sign;
            if (index == 0 || order > 0) { maximum = projection; vertex = offset; tied = false; }
            else if (order == 0 && offset != vertex) tied = true;
        }
        return height + (tied ? 1 : 0);
    }

    private static Signed320 ScalePowerOfTwo(Signed320 value, int shift)
    {
        while (shift > 0)
        {
            int step = Math.Min(63, shift);
            WideArithmetic.ShiftLeft(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0, step,
                out ulong w4, out ulong w3, out ulong w2, out ulong w1, out ulong w0);
            value = new(w4, w3, w2, w1, w0); shift -= step;
        }
        return value;
    }

    private static Signed576 ScalePowerOfTwo(Signed576 value, int shift)
    {
        while (shift > 0)
        {
            int step = Math.Min(62, shift);
            value = WideArithmetic.MultiplySigned576(value, 1L << step);
            shift -= step;
        }
        return value;
    }
}
