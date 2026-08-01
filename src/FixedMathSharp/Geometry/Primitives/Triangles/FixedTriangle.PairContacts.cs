//=======================================================================
// FixedTriangle.PairContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides exact full-domain contact generation between two rigidly
/// transformed triangles.
/// </content>
public partial struct FixedTriangle
{
    /// <summary>
    /// Attempts to construct an exact full-domain contact relation between
    /// this triangle and <paramref name="second"/> in their respective rigid
    /// frames.
    /// </summary>
    /// <remarks>
    /// Exact touching is a contact with zero depth. The normal points from the
    /// first triangle toward the second, and each returned anchor remains in
    /// its input frame. The winning depth is rounded half to even once; an
    /// unrepresentable positive depth is returned as <see cref="Fixed64.MaxValue"/>
    /// with <see cref="FixedContactAnchors.DepthIsClamped"/> set.
    /// </remarks>
    /// <returns>
    /// <see langword="false"/> when either triangle has an exact-zero normal or
    /// when a separating axis has negative overlap; otherwise
    /// <see langword="true"/>.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstRotation"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentException">
    /// <paramref name="secondRotation"/> is not normalized.
    /// </exception>
    public readonly bool TryGetContact(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        FixedTriangle second,
        out FixedContactAnchors contact)
    {
        if (!firstRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The first triangle rotation must be normalized.",
                nameof(firstRotation));
        }
        if (!secondRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The second triangle rotation must be normalized.",
                nameof(secondRotation));
        }

        return WideTriangleRelations.TryGetContact(
            this,
            firstOrigin,
            firstRotation,
            second,
            secondOrigin,
            secondRotation,
            out contact);
    }
}
