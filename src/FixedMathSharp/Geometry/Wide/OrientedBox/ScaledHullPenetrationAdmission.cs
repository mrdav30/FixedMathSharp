//=======================================================================
// ScaledHullPenetrationAdmission.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Admits an exact scaled-hull exit before depth ranking.</summary>
internal delegate bool ScaledHullPenetrationAdmission(in WideScaledHullPenetration candidate);
