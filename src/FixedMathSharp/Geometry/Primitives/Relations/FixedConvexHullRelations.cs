//=======================================================================
// FixedConvexHullRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Provides full-domain relations for rigidly transformed 3D convex hulls.
/// </summary>
public static class FixedConvexHullRelations
{
    /// <summary>
    /// Tests whether a point lies in or on a closed convex hull. The supplied
    /// interior point determines the accepted side of consistently wound faces.
    /// </summary>
    /// <param name="hullOrigin">The hull frame's origin in world coordinates.</param>
    /// <param name="hullRotation">The hull's normalized local-to-world rotation.</param>
    /// <param name="hullLocalPoints">At least three hull vertices in local coordinates. The span is read during the call.</param>
    /// <param name="triangleVertexIndices">
    /// A nonempty sequence of index triples into the corresponding hull vertex span, one triple per face triangle.
    /// </param>
    /// <param name="hullInteriorLocalPoint">
    /// A point strictly inside the hull, in hull-local coordinates. It selects the interior side of each face.
    /// </param>
    /// <param name="point">The query point retained in its own rigid world frame.</param>
    /// <returns>
    /// <see langword="true"/> when the point is inside or on the boundary of the closed shape and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">A topology index is outside its vertex span.</exception>
    /// <exception cref="ArgumentException">
    /// The hull rotation is not normalized, fewer than three vertices are supplied, or triangle indices are missing
    /// or do not form complete triples.
    /// </exception>
    public static bool ContainsPoint(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        Vector3d hullInteriorLocalPoint,
        FixedPointAnchor point)
    {
        ValidateRotation(hullRotation, nameof(hullRotation));
        ValidateHull(
            hullLocalPoints,
            triangleVertexIndices,
            ReadOnlySpan<int>.Empty);
        return WideOrientedBox.ContainsConvexHullPoint(
            hullOrigin,
            hullRotation,
            hullLocalPoints,
            triangleVertexIndices,
            hullInteriorLocalPoint,
            point);
    }

    /// <summary>
    /// Attempts to obtain the minimum-translation contact between two rigidly
    /// transformed convex hulls.
    /// </summary>
    /// <param name="firstOrigin">The first shape frame's origin in world coordinates.</param>
    /// <param name="firstRotation">The first shape's normalized local-to-world rotation.</param>
    /// <param name="firstLocalPoints">
    /// At least three vertices of the first hull in local coordinates. The span is read during the call.
    /// </param>
    /// <param name="firstTriangleVertexIndices">
    /// A nonempty sequence of index triples into the corresponding hull vertex span, one triple per face triangle.
    /// </param>
    /// <param name="firstEdgeVertexPairs">
    /// A nonempty sequence of index pairs into the matching hull vertex span, one pair per hull edge.
    /// </param>
    /// <param name="secondOrigin">The second shape frame's origin in world coordinates.</param>
    /// <param name="secondRotation">The second shape's normalized local-to-world rotation.</param>
    /// <param name="secondLocalPoints">
    /// At least three vertices of the second hull in local coordinates. The span is read during the call.
    /// </param>
    /// <param name="secondTriangleVertexIndices">
    /// A nonempty sequence of index triples into the corresponding hull vertex span, one triple per face triangle.
    /// </param>
    /// <param name="secondEdgeVertexPairs">
    /// A nonempty sequence of index pairs into the matching hull vertex span, one pair per hull edge.
    /// </param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">A topology index is outside its vertex span.</exception>
    /// <exception cref="ArgumentException">
    /// A rotation or local axis is not normalized, a hull has fewer than three vertices, or the face or edge
    /// topology is missing or incomplete.
    /// </exception>
    public static bool TryGetContact(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        ReadOnlySpan<Vector3d> firstLocalPoints,
        ReadOnlySpan<int> firstTriangleVertexIndices,
        ReadOnlySpan<int> firstEdgeVertexPairs,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        ReadOnlySpan<Vector3d> secondLocalPoints,
        ReadOnlySpan<int> secondTriangleVertexIndices,
        ReadOnlySpan<int> secondEdgeVertexPairs,
        out FixedContactAnchors contact)
    {
        ValidateRotation(firstRotation, nameof(firstRotation));
        ValidateRotation(secondRotation, nameof(secondRotation));
        ValidateHull(
            firstLocalPoints,
            firstTriangleVertexIndices,
            firstEdgeVertexPairs);
        ValidateHull(
            secondLocalPoints,
            secondTriangleVertexIndices,
            secondEdgeVertexPairs);
        if (firstEdgeVertexPairs.Length == 0)
        {
            throw new ArgumentException(
                "A convex-hull contact requires first-hull edge topology.",
                nameof(firstEdgeVertexPairs));
        }
        if (secondEdgeVertexPairs.Length == 0)
        {
            throw new ArgumentException(
                "A convex-hull contact requires second-hull edge topology.",
                nameof(secondEdgeVertexPairs));
        }
        return WideOrientedBox.TryGetConvexHullContact(
            firstOrigin,
            firstRotation,
            firstLocalPoints,
            firstTriangleVertexIndices,
            firstEdgeVertexPairs,
            secondOrigin,
            secondRotation,
            secondLocalPoints,
            secondTriangleVertexIndices,
            secondEdgeVertexPairs,
            out contact);
    }

    /// <summary>
    /// Attempts to obtain the minimum-translation contact between a rigidly
    /// transformed convex hull and a centered capsule.
    /// </summary>
    /// <param name="hullOrigin">The hull frame's origin in world coordinates.</param>
    /// <param name="hullRotation">The hull's normalized local-to-world rotation.</param>
    /// <param name="hullLocalPoints">At least three hull vertices in local coordinates. The span is read during the call.</param>
    /// <param name="triangleVertexIndices">
    /// A nonempty sequence of index triples into the corresponding hull vertex span, one triple per face triangle.
    /// </param>
    /// <param name="edgeVertexPairs">
    /// A nonempty sequence of index pairs into the matching hull vertex span, one pair per hull edge.
    /// </param>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleRotation">The capsule's normalized local-to-world rotation.</param>
    /// <param name="capsuleLocalAxisDirection">The normalized capsule axis in shape-local coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="capsuleAxisLength"/> is negative, <paramref name="capsuleRadius"/> is negative, or a
    /// topology index is outside its vertex span.
    /// </exception>
    /// <remarks>
    /// The returned normal points from the hull toward the capsule. Hull
    /// points remain in their canonical local frame throughout the exact SAT.
    /// Face and edge indices must describe the convex hull's complete boundary;
    /// structural validation checks indices, not convexity or topology completeness.
    /// A zero-length capsule core is a sphere, including volume containment.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// A rotation or local axis is not normalized, a hull has fewer than three vertices, or the face or edge
    /// topology is missing or incomplete.
    /// </exception>
    public static bool TryGetCenteredCapsuleContact(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        ValidateRotation(hullRotation, nameof(hullRotation));
        ValidateRotation(capsuleRotation, nameof(capsuleRotation));
        ValidateHull(
            hullLocalPoints,
            triangleVertexIndices,
            edgeVertexPairs);
        if (edgeVertexPairs.Length == 0)
        {
            throw new ArgumentException(
                "A convex-hull capsule contact requires hull edge topology.",
                nameof(edgeVertexPairs));
        }
        if (!capsuleLocalAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "The capsule local axis direction must be normalized.",
                nameof(capsuleLocalAxisDirection));
        }
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));

        return WideOrientedBox.TryGetConvexHullCenteredCapsuleContact(
            hullOrigin,
            hullRotation,
            hullLocalPoints,
            triangleVertexIndices,
            edgeVertexPairs,
            capsuleCenter,
            capsuleRotation,
            capsuleLocalAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            out contact);
    }

    private static void ValidateRotation(
        FixedQuaternion rotation,
        string parameterName)
    {
        if (!rotation.IsNormalized())
            throw new ArgumentException("The hull rotation must be normalized.", parameterName);
    }

    private static void ValidateHull(
        ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs)
    {
        if (points.Length < 3)
            throw new ArgumentException("A convex hull requires at least three points.", nameof(points));
        if (triangleVertexIndices.Length == 0
            || triangleVertexIndices.Length % 3 != 0)
        {
            throw new ArgumentException(
                "Triangle indices must be a nonempty multiple of three.",
                nameof(triangleVertexIndices));
        }
        if (edgeVertexPairs.Length % 2 != 0)
        {
            throw new ArgumentException(
                "Edge indices must contain complete vertex pairs.",
                nameof(edgeVertexPairs));
        }

        ValidateIndices(points.Length, triangleVertexIndices, nameof(triangleVertexIndices));
        ValidateIndices(points.Length, edgeVertexPairs, nameof(edgeVertexPairs));
    }

    private static void ValidateIndices(
        int pointCount,
        ReadOnlySpan<int> indices,
        string parameterName)
    {
        for (int index = 0; index < indices.Length; index++)
        {
            if ((uint)indices[index] >= (uint)pointCount)
                throw new ArgumentOutOfRangeException(parameterName);
        }
    }
}
