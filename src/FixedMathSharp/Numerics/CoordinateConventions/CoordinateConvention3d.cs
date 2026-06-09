//=======================================================================
// CoordinateConvention3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Describes a stateless 3D direction convention relative to FixedMathSharp's canonical basis.
/// </summary>
/// <remarks>
/// FixedMathSharp's canonical 3D basis is <c>+X</c> right, <c>+Y</c> up, and <c>+Z</c> forward.
/// This type covers the common <c>+Y</c>-up distinction between positive-Z-forward and
/// negative-Z-forward semantic spaces. Broader engine or tool conventions that change the up
/// axis, right axis, handedness, matrix layout, or clip-space depth should convert through an
/// adapter-specific basis transform instead of changing core math semantics.
/// </remarks>
public readonly struct CoordinateConvention3d : IEquatable<CoordinateConvention3d>
{
    /// <summary>
    /// The FixedMathSharp canonical convention: <c>+X</c> right, <c>+Y</c> up, <c>+Z</c> forward.
    /// </summary>
    public static CoordinateConvention3d PositiveZForward => new(ForwardAxis.PositiveZ);

    /// <summary>
    /// A <c>+Y</c>-up convention where semantic forward is <c>-Z</c>.
    /// </summary>
    public static CoordinateConvention3d NegativeZForward => new(ForwardAxis.NegativeZ);

    /// <summary>
    /// Initializes a new convention with the specified forward axis.
    /// </summary>
    /// <param name="forwardAxis">The signed Z axis used as semantic forward.</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="forwardAxis"/> is not a defined <see cref="ForwardAxis"/> value.
    /// </exception>
    public CoordinateConvention3d(ForwardAxis forwardAxis)
    {
        if (!IsValidForwardAxis(forwardAxis))
            throw new ArgumentOutOfRangeException(nameof(forwardAxis), forwardAxis, "Forward axis must be PositiveZ or NegativeZ.");

        ForwardAxis = forwardAxis;
    }

    /// <summary>
    /// Gets the signed Z axis used as semantic forward.
    /// </summary>
    public ForwardAxis ForwardAxis { get; }

    /// <summary>
    /// Gets the semantic right direction expressed in FixedMathSharp's canonical basis.
    /// </summary>
    public Vector3d Right
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector3d.Right;
    }

    /// <summary>
    /// Gets the semantic left direction expressed in FixedMathSharp's canonical basis.
    /// </summary>
    public Vector3d Left
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector3d.Left;
    }

    /// <summary>
    /// Gets the semantic up direction expressed in FixedMathSharp's canonical basis.
    /// </summary>
    public Vector3d Up
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector3d.Up;
    }

    /// <summary>
    /// Gets the semantic down direction expressed in FixedMathSharp's canonical basis.
    /// </summary>
    public Vector3d Down
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Vector3d.Down;
    }

    /// <summary>
    /// Gets the semantic forward direction expressed in FixedMathSharp's canonical basis.
    /// </summary>
    public Vector3d Forward
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ForwardAxis == ForwardAxis.PositiveZ ? Vector3d.Forward : Vector3d.Backward;
    }

    /// <summary>
    /// Gets the semantic backward direction expressed in FixedMathSharp's canonical basis.
    /// </summary>
    public Vector3d Backward
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ForwardAxis == ForwardAxis.PositiveZ ? Vector3d.Backward : Vector3d.Forward;
    }

    /// <summary>
    /// Converts a direction vector from this convention into FixedMathSharp's canonical convention.
    /// </summary>
    /// <param name="direction">The direction vector expressed in this convention.</param>
    /// <returns>The same semantic direction expressed in the canonical <c>+Z</c>-forward convention.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ToCanonicalDirection(Vector3d direction) =>
        ForwardAxis == ForwardAxis.PositiveZ ? direction : FlipZ(direction);

    /// <summary>
    /// Converts a direction vector from FixedMathSharp's canonical convention into this convention.
    /// </summary>
    /// <param name="canonicalDirection">The direction vector expressed in FixedMathSharp's canonical convention.</param>
    /// <returns>The same semantic direction expressed in this convention.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d FromCanonicalDirection(Vector3d canonicalDirection) =>
        ToCanonicalDirection(canonicalDirection);

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(CoordinateConvention3d other) => ForwardAxis == other.ForwardAxis;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is CoordinateConvention3d other && Equals(other);

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() => (int)ForwardAxis;

    /// <summary>
    /// Determines whether two conventions are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(CoordinateConvention3d left, CoordinateConvention3d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two conventions are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(CoordinateConvention3d left, CoordinateConvention3d right) => !left.Equals(right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d FlipZ(Vector3d direction) => new(direction.X, direction.Y, -direction.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidForwardAxis(ForwardAxis forwardAxis) =>
        forwardAxis == ForwardAxis.PositiveZ || forwardAxis == ForwardAxis.NegativeZ;
}
