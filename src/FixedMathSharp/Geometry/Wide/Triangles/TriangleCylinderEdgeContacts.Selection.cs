//=======================================================================
// TriangleCylinderEdgeContacts.Selection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.CircularRimContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Selection declaration for TriangleCylinderEdgeContacts.</content>
internal static partial class TriangleCylinderEdgeContacts
{
    internal struct Selection
    {
        internal bool HasValue, IsZero, IsRational;
        internal WideDirection3d First, Second;
        internal int Edge, Cap, CoreSign, ParameterOrdinal, ValueOrdinal, ValueShift, Triangle;
    }
}
