//=======================================================================
// WideLever3dValue.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Stores one exact 3D point-offset ratio for policy-neutral wide geometry.
/// </summary>
internal readonly struct WideLever3dValue
{
    internal WideLever3dValue(
        Signed576 xNumerator,
        Signed576 yNumerator,
        Signed576 zNumerator,
        Signed576 denominator)
    {
        XNumerator = xNumerator;
        YNumerator = yNumerator;
        ZNumerator = zNumerator;
        Denominator = denominator;
    }

    internal Signed576 XNumerator { get; }

    internal Signed576 YNumerator { get; }

    internal Signed576 ZNumerator { get; }

    internal Signed576 Denominator { get; }
}
