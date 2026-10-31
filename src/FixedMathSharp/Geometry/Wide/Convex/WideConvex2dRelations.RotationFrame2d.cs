//=======================================================================
// WideConvex2dRelations.RotationFrame2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>RotationFrame2d declaration for WideConvex2dRelations.</content>
internal static partial class WideConvex2dRelations
{
    internal readonly struct RotationFrame2d
    {
        internal static readonly RotationFrame2d Identity = new(
            Signed192.One,
            default);

        internal readonly Signed192 Cosine;
        internal readonly Signed192 Sine;

        internal RotationFrame2d(Fixed64 rotation)
            : this(
                Signed192.Raw(FixedMath.Cos(rotation)),
                Signed192.Raw(FixedMath.Sin(rotation)))
        {
        }

        private RotationFrame2d(
            Signed192 cosine,
            Signed192 sine)
        {
            Cosine = cosine;
            Sine = sine;
        }
    }
}
