//=======================================================================
// PrismCurvedBoundaryContactVisitor.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Streams a line's finite contact and exact depth order relative to the native exit.</summary>
internal delegate void PrismCurvedBoundaryContactVisitor(int ordinal, in FixedContactAnchors contact, int compareToNative);
