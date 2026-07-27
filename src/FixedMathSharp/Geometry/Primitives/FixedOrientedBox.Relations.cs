//=======================================================================
// FixedOrientedBox.Relations.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Contains methods for finding exact or representable-domain contacts between oriented boxes and other shapes.
/// </content>
public readonly partial struct FixedOrientedBox
{
    /// <summary>
    /// Attempts to construct an exact contact against another oriented box.
    /// </summary>
    public bool TryGetContact(
        FixedOrientedBox other,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        other.EnsureValid();
        return WideOrientedBox.TryGetContact(
            Center,
            Orientation,
            HalfExtents,
            other.Center,
            other.Orientation,
            other.HalfExtents,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a nondegenerate
    /// triangle.
    /// </summary>
    /// <param name="triangleOrigin">
    /// The triangle's rigid-frame origin.
    /// </param>
    /// <param name="triangleRotation">The triangle's local-to-world rotation.</param>
    /// <param name="triangle">The triangle in its rigid-frame coordinates.</param>
    /// <param name="contact">
    /// The box-center-relative and triangle-origin-relative constraint.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the shapes overlap and both relative
    /// contact anchors are representable; otherwise <see langword="false"/>.
    /// </returns>
    public bool TryGetTriangleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!triangleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The triangle rotation must be normalized.",
                nameof(triangleRotation));
        }
        return WideOrientedBox.TryGetTriangleContact(
            Center,
            Orientation,
            HalfExtents,
            triangleOrigin,
            triangleRotation,
            triangle,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact triangle contact and, for a parallel
    /// box-face/triangle-face feature, clips stable box corners to the
    /// triangle.
    /// </summary>
    /// <remarks>
    /// <paramref name="faceContacts"/> must provide capacity for four
    /// contacts. <paramref name="contact"/> always contains the primary exact
    /// constraint when the shapes overlap. <paramref name="faceContactCount"/>
    /// is zero for non-face features.
    /// </remarks>
    public bool TryGetTriangleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        Span<FixedContactLocalPoints> faceContacts,
        out FixedContactAnchors contact,
        out int faceContactCount)
    {
        EnsureValid();
        if (faceContacts.Length < 4)
        {
            throw new ArgumentException(
                "Triangle face-contact output requires capacity for four contacts.",
                nameof(faceContacts));
        }
        if (!triangleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The triangle rotation must be normalized.",
                nameof(triangleRotation));
        }
        if (!WideOrientedBox.TryGetTriangleContact(
                Center,
                Orientation,
                HalfExtents,
                triangleOrigin,
                triangleRotation,
                triangle,
                out contact))
        {
            faceContactCount = 0;
            return false;
        }

        WideOrientedBox.GetTriangleFaceContacts(
            Center,
            Orientation,
            HalfExtents,
            triangleOrigin,
            triangleRotation,
            triangle,
            contact,
            faceContacts,
            out faceContactCount);
        return true;
    }

    /// <summary>
    /// Attempts to construct an exact contact against a convex point span.
    /// </summary>
    /// <remarks>
    /// Hull points are origin-relative offsets expressed in the hull's local
    /// orientation. Triangle and edge indices provide the complete convex SAT
    /// topology. The method performs no absolute world-point materialization.
    /// </remarks>
    public bool TryGetConvexHullContact(
        Vector3d hullOrigin,
        FixedQuaternion hullOrientation,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!hullOrientation.IsNormalized())
        {
            throw new ArgumentException(
                "The convex-hull rotation must be normalized.",
                nameof(hullOrientation));
        }
        if (hullLocalOffsets.Length < 4)
        {
            throw new ArgumentException(
                "A convex hull requires at least four origin-relative points.",
                nameof(hullLocalOffsets));
        }
        if (triangleVertexIndices.Length == 0
            || triangleVertexIndices.Length % FixedTriangle.VertexCount != 0)
        {
            throw new ArgumentException(
                "Convex-hull triangle topology must be a nonempty multiple of three.",
                nameof(triangleVertexIndices));
        }
        if (edgeVertexPairs.Length == 0
            || (edgeVertexPairs.Length & 1) != 0)
        {
            throw new ArgumentException(
                "Convex-hull edge topology must contain complete vertex-index pairs.",
                nameof(edgeVertexPairs));
        }
        ValidateConvexHullIndices(
            hullLocalOffsets.Length,
            triangleVertexIndices,
            nameof(triangleVertexIndices));
        ValidateConvexHullIndices(
            hullLocalOffsets.Length,
            edgeVertexPairs,
            nameof(edgeVertexPairs));
        return WideOrientedBox.TryGetConvexHullContact(
            Center,
            Orientation,
            HalfExtents,
            hullOrigin,
            hullOrientation,
            hullLocalOffsets,
            triangleVertexIndices,
            edgeVertexPairs,
            out contact);
    }

    private static void ValidateConvexHullIndices(
        int vertexCount,
        ReadOnlySpan<int> indices,
        string parameterName)
    {
        for (int index = 0; index < indices.Length; index++)
        {
            if ((uint)indices[index] >= (uint)vertexCount)
            {
                throw new ArgumentOutOfRangeException(
                    parameterName,
                    "Convex-hull topology contains an invalid vertex index.");
            }
        }
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered 3D capsule.
    /// </summary>
    public bool TryGetCenteredCapsuleContact(
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!capsuleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule rotation must be normalized.",
                nameof(capsuleRotation));
        }
        if (!localCapsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Local capsule axis direction must be normalized.",
                nameof(localCapsuleAxisDirection));
        }
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        return WideOrientedBox.TryGetCenteredCapsuleContact(
            Center,
            Orientation,
            HalfExtents,
            capsuleCenter,
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered finite
    /// cylinder.
    /// </summary>
    public bool TryGetCenteredCylinderContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        ValidateCenteredCylinder(
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius);
        return WideOrientedBox.TryGetCenteredCylinderContact(
            Center,
            Orientation,
            HalfExtents,
            cylinderCenter,
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered finite
    /// cylinder and, for a parallel cap-to-face feature, clips a stable
    /// allocation-free manifold from the disk/rectangle intersection.
    /// </summary>
    /// <remarks>
    /// <paramref name="capFaceContacts"/> must provide capacity for four
    /// contacts. <paramref name="contact"/> always contains the primary exact
    /// constraint when the shapes overlap. <paramref name="capFaceContactCount"/>
    /// is zero for non-cap-face features, so callers should use the primary
    /// contact in that case.
    /// </remarks>
    public bool TryGetCenteredCylinderContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Span<FixedContactLocalPoints> capFaceContacts,
        out FixedContactAnchors contact,
        out int capFaceContactCount)
    {
        EnsureValid();
        ValidateCenteredCylinder(
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius);
        if (capFaceContacts.Length < 4)
        {
            throw new ArgumentException(
                "Cylinder cap-face contact output requires capacity for four contacts.",
                nameof(capFaceContacts));
        }

        if (!WideOrientedBox.TryGetCenteredCylinderContact(
                Center,
                Orientation,
                HalfExtents,
                cylinderCenter,
                cylinderRotation,
                localCylinderAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                out contact,
                out WideOrientedBox.CenteredCylinderContactFeature feature))
        {
            capFaceContactCount = 0;
            return false;
        }

        WideOrientedBox.GetCenteredCylinderCapFaceContacts(
            Center,
            Orientation,
            HalfExtents,
            cylinderCenter,
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            contact,
            feature,
            capFaceContacts,
            out capFaceContactCount);
        return true;
    }

    private static void ValidateCenteredCylinder(
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius)
    {
        if (!cylinderRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Cylinder rotation must be normalized.",
                nameof(cylinderRotation));
        }
        if (!localCylinderAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Local cylinder axis direction must be normalized.",
                nameof(localCylinderAxisDirection));
        }
        if (cylinderAxisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(cylinderAxisLength));
        if (cylinderRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(cylinderRadius));
    }

    /// <summary>
    /// Attempts to construct an exact box-to-sphere contact using only
    /// center-relative constraint offsets.
    /// </summary>
    /// <param name="sphereCenter">The sphere center.</param>
    /// <param name="sphereRotation">
    /// The normalized rigid frame used to retain stable sphere-local contact
    /// feature identity.
    /// </param>
    /// <param name="sphereRadius">The nonnegative sphere radius.</param>
    /// <param name="contact">The relative contact constraint when overlapping.</param>
    /// <returns>
    /// <see langword="true"/> when the conceptual shapes overlap and both
    /// required relative offsets are representable.
    /// </returns>
    public bool TryGetSphereContact(
        Vector3d sphereCenter,
        FixedQuaternion sphereRotation,
        Fixed64 sphereRadius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!sphereRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Sphere rotation must be normalized.",
                nameof(sphereRotation));
        }
        if (sphereRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphereRadius));
        return WideOrientedBox.TryGetSphereContact(
            Center,
            Orientation,
            HalfExtents,
            sphereCenter,
            sphereRotation,
            sphereRadius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a rotated convex polygon
    /// extruded through a closed world-Y slab.
    /// </summary>
    /// <param name="prismOrigin">The representable prism origin.</param>
    /// <param name="prismRotation">
    /// The prism's counterclockwise X/Z rotation in radians.
    /// </param>
    /// <param name="prismLocalOffsets">
    /// Ordered clockwise or counterclockwise X/Z boundary offsets in the
    /// prism's local frame.
    /// </param>
    /// <param name="prismHalfThickness">The positive world-Y half-thickness.</param>
    /// <param name="contact">The frame-relative contact anchors when overlapping.</param>
    public bool TryGetConvexPrismContact(
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (prismLocalOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex prism requires at least three ordered boundary offsets.",
                nameof(prismLocalOffsets));
        }
        if (prismHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(prismHalfThickness));
        return WideOrientedBox.TryGetConvexPrismContact(
            Center,
            Orientation,
            HalfExtents,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a vertical circular
    /// prism centered in X/Z and extruded through a closed world-Y slab.
    /// </summary>
    public bool TryGetCircleSlabContact(
        Vector3d slabCenter,
        Fixed64 circleFrameRotation,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.TryGetCircleSlabContact(
            Center,
            Orientation,
            HalfExtents,
            slabCenter,
            circleFrameRotation,
            slabHalfThickness,
            radius,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact against a centered planar
    /// capsule extruded through a closed world-Y slab.
    /// </summary>
    public bool TryGetCenteredCapsuleSlabContact(
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        out FixedContactAnchors contact)
    {
        EnsureValid();
        if (!localCapsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Local capsule axis direction must be normalized.",
                nameof(localCapsuleAxisDirection));
        }
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        return WideOrientedBox.TryGetCenteredCapsuleSlabContact(
            Center,
            Orientation,
            HalfExtents,
            slabCenter,
            capsuleFrameRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness,
            out contact);
    }

    /// <summary>
    /// Finds the first X/Z travel distance at which a circle extruded through
    /// a fixed world-Y slab intersects this oriented box.
    /// </summary>
    /// <remarks>
    /// The slab-clipped box projection remains an internal wide rational
    /// relation. No box corner, slab intersection, or projected polygon must
    /// be representable. Tangency and start overlap are inclusive, and the
    /// returned distance uses round-half-to-even.
    /// </remarks>
    public bool TryGetCircleSlabSweepDistance(
        Vector3d slabStartCenter,
        Vector2d normalizedDirection,
        Fixed64 maxDistance,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out Fixed64 distance)
    {
        EnsureValid();
        if (!normalizedDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Sweep direction must be normalized.",
                nameof(normalizedDirection));
        }
        if (maxDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maxDistance));
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.TryGetCircleSlabSweepDistance(
            Center,
            Orientation,
            HalfExtents,
            slabStartCenter,
            normalizedDirection,
            maxDistance,
            slabHalfThickness,
            radius,
            out distance);
    }

    /// <summary>
    /// Returns a conservative X/Z separation lower bound between this box and
    /// a circle extruded through a fixed world-Y slab.
    /// </summary>
    /// <remarks>
    /// Projected edge witnesses are certified against every rational
    /// half-plane. When the closest feature is a projected vertex, the result
    /// uses the full-domain Euclidean distance floor before subtracting the
    /// circle radius. The result never overestimates true projected
    /// separation. When the Y intervals are disjoint, it returns the
    /// conservative vertical interval gap instead.
    /// </remarks>
    public Fixed64 GetCircleSlabSeparationLowerBound(
        Vector3d slabCenter,
        Fixed64 slabHalfThickness,
        Fixed64 radius)
    {
        EnsureValid();
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        return WideOrientedBox.GetCircleSlabSeparationLowerBound(
            Center,
            Orientation,
            HalfExtents,
            slabCenter,
            slabHalfThickness,
            radius);
    }
}
