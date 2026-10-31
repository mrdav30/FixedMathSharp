//=======================================================================
// SegmentConeSurfaceInput.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>Neutral source provenance sufficient to reconstruct one exact chart.</summary>
internal readonly struct SegmentConeSurfaceInput
{
    internal readonly FixedSegment Segment;
    internal readonly Vector3d Origin, Center;
    internal readonly FixedQuaternion Rotation, ConeRotation;
    internal readonly Fixed64 Height, Radius;
    internal readonly ScaledRigidFrame3d SourceFrame;
    internal readonly bool HasSourceFrame;
    internal bool IsAffine => HasSourceFrame && !SourceFrame.IsRigid;
    internal int Words => IsAffine ? 120 : CylinderContactAlgebra.Words;
    internal int FamilyWords => IsAffine ? 3 * SegmentConeSurfaceCandidates.FamilyWords : SegmentConeSurfaceCandidates.FamilyWords;
    internal int ValueWords => IsAffine ? 3 * CircularRimContactAlgebra.ValueWords : CircularRimContactAlgebra.ValueWords;

    internal SegmentConeSurfaceInput(FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        SourceFrame = default; HasSourceFrame = false;
        Segment = segment; Origin = origin; Rotation = rotation; Center = center;
        ConeRotation = coneRotation; Height = height; Radius = radius;
    }

    internal SegmentConeSurfaceInput(FixedSegment segment, in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius)
    {
        SourceFrame = sourceFrame; HasSourceFrame = true; Segment = segment;
        Origin = sourceFrame.IsRigid ? sourceFrame.RigidOrigin : center;
        Rotation = sourceFrame.IsRigid ? sourceFrame.Rotation : rotation;
        Center = center; ConeRotation = rotation; Height = height; Radius = radius;
    }
}
