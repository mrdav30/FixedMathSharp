//=======================================================================
// ConeSurfaceFamilyEventKind.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

internal enum ConeSurfaceFamilyEventKind { Interval, CircleSeam, CircleBoundary, DiskCenter, DiskPair, RimBound, Axis, AxisPlane, AxisPair }
