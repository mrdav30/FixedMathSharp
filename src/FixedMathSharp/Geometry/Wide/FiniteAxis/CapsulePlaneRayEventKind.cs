//=======================================================================
// CapsulePlaneRayEventKind.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

internal enum CapsulePlaneRayEventKind : byte { Support, Sagittal, Seam, Endpoint, Section, Stationary, Circle }
