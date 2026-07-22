//=======================================================================
// FixedTriangle.FiniteCone.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

public partial struct FixedTriangle
{
    /// <summary>
    /// Finds the intersecting point with the smallest axial parameter in an
    /// apex-authored finite cone.
    /// </summary>
    /// <remarks>
    /// Boundary candidates are visited in AB, BC, CA, face order. Triangle-face
    /// roots use a <see cref="Fixed64.MaxValue"/>-scaled parameter lattice;
    /// exact wide predicates select the witness before one final point rounding.
    /// </remarks>
    public bool TryGetFiniteConeIntersectionMinimumAxialPoint(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d point)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));

        bool found = false;
        Vector3d bestPoint = default;
        Fixed64 bestAxial = Fixed64.MaxValue;
        KeepEdgeCandidate(GetEdge(0), apex, apexToBaseDirection, height, baseRadius, ref found, ref bestPoint, ref bestAxial);
        KeepEdgeCandidate(GetEdge(1), apex, apexToBaseDirection, height, baseRadius, ref found, ref bestPoint, ref bestAxial);
        KeepEdgeCandidate(GetEdge(2), apex, apexToBaseDirection, height, baseRadius, ref found, ref bestPoint, ref bestAxial);

        GetExactNormal(out Signed192 normalX, out Signed192 normalY, out Signed192 normalZ, out Signed320 normalSquared);
        if (!WideGeometry.IsQ128MagnitudeAtMostEpsilon(normalSquared)
            && WideTriangleConeIntersection.TryGetFaceMinimumAxialPoint(
                this,
                normalX,
                normalY,
                normalZ,
                normalSquared,
                apex,
                apexToBaseDirection,
                height,
                baseRadius,
                found,
                out Vector3d facePoint,
                out Fixed64 faceAxial)
            && (!found || faceAxial < bestAxial))
        {
            found = true;
            bestPoint = facePoint;
        }

        point = bestPoint;
        return found;
    }

    private static void KeepEdgeCandidate(
        FixedSegment edge,
        Vector3d apex,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        ref bool found,
        ref Vector3d bestPoint,
        ref Fixed64 bestAxial)
    {
        if (!edge.TryGetFiniteConeIntersectionMinimumAxialPoint(
                apex,
                axisDirection,
                height,
                baseRadius,
                out Vector3d candidate))
        {
            return;
        }

        Fixed64 axial = FixedMath.Min(
            Vector3d.ProjectNonNegativeDifferenceParameter(candidate, apex, axisDirection),
            height);
        if (found && axial >= bestAxial)
            return;

        found = true;
        bestPoint = candidate;
        bestAxial = axial;
    }

    private static void ValidateFiniteCone(
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        string axisParameterName,
        string heightParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", axisParameterName);
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(heightParameterName);
        if (baseRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(baseRadius));
    }
}
