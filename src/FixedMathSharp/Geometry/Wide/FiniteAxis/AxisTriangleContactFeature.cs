//=======================================================================
// AxisTriangleContactFeature.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Retains exact closest-point feature identity and separation before contact rounding.</summary>
internal readonly struct AxisTriangleContactFeature
{
    private readonly Signed832 _separationX, _separationY, _separationZ;
    private readonly bool _wide;

    internal AxisTriangleContactFeature(int edgeMask, Signed576 separationX,
        Signed576 separationY, Signed576 separationZ, int triangleVertex = -1)
    {
        _wide = false;
        EdgeMask = edgeMask;
        TriangleVertex = triangleVertex;
        _separationX = Signed832.ExtendValue(separationX); _separationY = Signed832.ExtendValue(separationY); _separationZ = Signed832.ExtendValue(separationZ);
    }

    internal AxisTriangleContactFeature(int edgeMask, Signed832 x, Signed832 y, Signed832 z, int triangleVertex = -1)
    {
        _wide = true;
        EdgeMask = edgeMask; TriangleVertex = triangleVertex;
        _separationX = x; _separationY = y; _separationZ = z;
    }

    /// <summary>Maps a retained rigid-local separation into world orientation without scalar rounding.</summary>
    internal AxisTriangleContactFeature GetWorldFeatureFromRigidLocal(FixedQuaternion rotation)
    {
        if (rotation == FixedQuaternion.Identity) return this;
        // This ingress is used only for the legacy local closest chart, whose
        // complete components fit Signed576. One normalized basis adds <65
        // bits; the resulting world direction fits Signed832.
        var basis = new WideRationalBasis3d(rotation);
        Signed576 x = Signed576.NarrowValue(_separationX), y = Signed576.NarrowValue(_separationY), z = Signed576.NarrowValue(_separationZ);
        Signed832 Component(Signed192 a, Signed192 b, Signed192 c) =>
            WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, Signed320.ExtendValue(a)),
                WideArithmetic.MultiplySigned576ToSigned832(y, Signed320.ExtendValue(b))),
                WideArithmetic.MultiplySigned576ToSigned832(z, Signed320.ExtendValue(c)));
        return new AxisTriangleContactFeature(EdgeMask, Component(basis.Xx, basis.Yx, basis.Zx),
            Component(basis.Xy, basis.Yy, basis.Zy), Component(basis.Xz, basis.Yz, basis.Zz), TriangleVertex);
    }
    /// <summary>Zero edge halfspaces: AB=1, BC=2, CA=4; 7 declines degenerate classification.</summary>
    internal int EdgeMask { get; }

    /// <summary>Exact authored vertex ownership; -1 denotes a nonvertex witness.</summary>
    internal int TriangleVertex { get; }

    /// <summary>Whether the retained core and triangle witnesses coincide exactly.</summary>
    internal bool IsZeroSeparation => _separationX.IsZero && _separationY.IsZero && _separationZ.IsZero;

    /// <summary>Gets the exact separation sign along a wide direction in the selected contact chart.</summary>
    internal int CompareSeparationProjection(Signed192 x, Signed192 y, Signed192 z)
    {
        if (_wide) return CompareSeparationProjection(new WideAxis3(Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z)));
        return WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(_separationX), Signed320.ExtendValue(x)),
            WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(_separationY), Signed320.ExtendValue(y))),
            WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(_separationZ), Signed320.ExtendValue(z))).Sign;
    }

    /// <summary>Gets the same exact sign for a rigidly transformed wide axis.</summary>
    internal int CompareSeparationProjection(in WideAxis3 direction)
    {
        // Full 832-by-320 products plus carries need 1154 signed bits. The
        // existing quadratic owner retains this rational-only dot without
        // narrowing exceptional world-chart plane normals to Signed192.
        int words = _wide ? 19 : 15;
        Span<ulong> values = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic sum = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic product = ContactQuadratic.At(values, signs, 2, words);
        sum.Clear();
        Span<Signed832> components = stackalloc Signed832[3] { _separationX, _separationY, _separationZ };
        Span<Signed320> axes = stackalloc Signed320[3] { direction.X, direction.Y, direction.Z };
        for (int index = 0; index < 3; index++)
        {
            if (_wide) term.Set(components[index]); else term.Set(Signed576.NarrowValue(components[index]));
            ContactQuadratic.Scale(term, Signed576.ExtendValue(axes[index]), product);
            sum.Add(product);
        }
        return sum.Sign(ReadOnlySpan<ulong>.Empty);
    }

    /// <summary>Gets the exact sign of separation dotted with a ray in the selected contact chart.</summary>
    internal int CompareSeparationProjection(Vector3d first, Vector3d second)
    {
        Signed320 x = Signed320.ExtendValue(WideArithmetic.Difference(second.X, first.X));
        Signed320 y = Signed320.ExtendValue(WideArithmetic.Difference(second.Y, first.Y));
        Signed320 z = Signed320.ExtendValue(WideArithmetic.Difference(second.Z, first.Z));
        if (!_wide)
            return WideArithmetic.AddSigned704(WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(Signed576.NarrowValue(_separationX), x),
                WideArithmetic.MultiplySigned576ToSigned704(Signed576.NarrowValue(_separationY), y)),
                WideArithmetic.MultiplySigned576ToSigned704(Signed576.NarrowValue(_separationZ), z)).Sign;
        return CompareSeparationProjection(new WideAxis3(x, y, z));
    }
}
