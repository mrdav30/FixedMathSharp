//=======================================================================
// CurvedPrismRimContacts.Selection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>Selection declaration for CurvedPrismRimContacts.</content>
internal static partial class CurvedPrismRimContacts
{
    internal struct Selection
    {
        internal bool HasValue, IsZero, Physical;
        internal int Edge, CapSign, ParameterOrdinal, Order;
        internal WideDirection3d First, Second;
        internal bool IsRational;
        internal int ValueOrdinal, ValueShift;
    }
}
