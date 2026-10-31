//=======================================================================
// PrismPointAdmission.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Admits an exact support point expressed as native-local raw-coordinate ratios.</summary>
internal delegate bool PrismPointAdmission(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth, scoped ContactQuadratic x,
    scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic denominator);
