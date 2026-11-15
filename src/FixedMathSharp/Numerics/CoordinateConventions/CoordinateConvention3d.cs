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
/// This type maps direction vector components between another explicit right/up/forward axis
/// basis and the FixedMathSharp canonical basis. Matrix storage, transform APIs, clip-space
/// depth, units, and origins remain adapter-level concerns.
/// Use the constructor or a named preset to obtain a valid basis. The default struct has three positive X axes.
/// Mapping permutes and signs components without normalizing them. Negating <see cref="Fixed64.MinValue"/>
/// saturates to <see cref="Fixed64.MaxValue"/>.
/// </remarks>
public readonly struct CoordinateConvention3d : IEquatable<CoordinateConvention3d>
{
    /// <summary>
    /// The FixedMathSharp canonical convention: <c>+X</c> right, <c>+Y</c> up, <c>+Z</c> forward.
    /// </summary>
    /// <value>The same convention as <see cref="PositiveZForward"/>.</value>
    public static CoordinateConvention3d Canonical => PositiveZForward;

    /// <summary>
    /// The FixedMathSharp canonical convention: <c>+X</c> right, <c>+Y</c> up, <c>+Z</c> forward.
    /// </summary>
    /// <value>The signed axes +X right, +Y up, and +Z forward.</value>
    public static CoordinateConvention3d PositiveZForward => new(Axis3d.PositiveX, Axis3d.PositiveY, Axis3d.PositiveZ);

    /// <summary>
    /// A convention with <c>+X</c> right, <c>+Y</c> up, and <c>-Z</c> forward.
    /// </summary>
    /// <value>The signed axes +X right, +Y up, and −Z forward.</value>
    public static CoordinateConvention3d NegativeZForward => new(Axis3d.PositiveX, Axis3d.PositiveY, Axis3d.NegativeZ);

    /// <summary>
    /// A convention with <c>+Y</c> right, <c>+Z</c> up, and <c>+X</c> forward.
    /// </summary>
    /// <value>The signed axes +Y right, +Z up, and +X forward.</value>
    public static CoordinateConvention3d XForwardZUp => new(Axis3d.PositiveY, Axis3d.PositiveZ, Axis3d.PositiveX);

    /// <summary>
    /// Initializes a new convention with the specified semantic right, up, and forward axes.
    /// </summary>
    /// <param name="rightAxis">The signed axis used as semantic right.</param>
    /// <param name="upAxis">The signed axis used as semantic up.</param>
    /// <param name="forwardAxis">The signed axis used as semantic forward.</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when any axis is not a defined <see cref="Axis3d"/> value.
    /// </exception>
    /// <exception cref="ArgumentException">
    /// Thrown when two semantic axes use the same absolute axis.
    /// </exception>
    public CoordinateConvention3d(Axis3d rightAxis, Axis3d upAxis, Axis3d forwardAxis)
    {
        if (!IsDefinedAxis(rightAxis))
            throw new ArgumentOutOfRangeException(nameof(rightAxis), rightAxis, "Right axis must be a defined 3D axis.");

        if (!IsDefinedAxis(upAxis))
            throw new ArgumentOutOfRangeException(nameof(upAxis), upAxis, "Up axis must be a defined 3D axis.");

        if (!IsDefinedAxis(forwardAxis))
            throw new ArgumentOutOfRangeException(nameof(forwardAxis), forwardAxis, "Forward axis must be a defined 3D axis.");

        int absoluteRightAxis = GetAbsoluteAxis(rightAxis);
        int absoluteUpAxis = GetAbsoluteAxis(upAxis);
        int absoluteForwardAxis = GetAbsoluteAxis(forwardAxis);

        if (absoluteRightAxis == absoluteUpAxis)
            throw new ArgumentException("Right and up axes must use different absolute axes.", nameof(upAxis));

        if (absoluteRightAxis == absoluteForwardAxis)
            throw new ArgumentException("Right and forward axes must use different absolute axes.", nameof(forwardAxis));

        if (absoluteUpAxis == absoluteForwardAxis)
            throw new ArgumentException("Up and forward axes must use different absolute axes.", nameof(forwardAxis));

        RightAxis = rightAxis;
        UpAxis = upAxis;
        ForwardAxis = forwardAxis;
    }

    /// <summary>
    /// Gets the signed axis used as semantic right in this convention.
    /// </summary>
    /// <value>The signed component axis assigned to semantic right.</value>
    public Axis3d RightAxis { get; }

    /// <summary>
    /// Gets the signed axis used as semantic up in this convention.
    /// </summary>
    /// <value>The signed component axis assigned to semantic up.</value>
    public Axis3d UpAxis { get; }

    /// <summary>
    /// Gets the signed axis used as semantic forward in this convention.
    /// </summary>
    /// <value>The signed component axis assigned to semantic forward.</value>
    public Axis3d ForwardAxis { get; }

    /// <summary>
    /// Gets the semantic right direction expressed in this convention's component basis.
    /// </summary>
    /// <value>The signed unit vector for <see cref="RightAxis"/>.</value>
    public Vector3d Right
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => AxisToVector(RightAxis);
    }

    /// <summary>
    /// Gets the semantic left direction expressed in this convention's component basis.
    /// </summary>
    /// <value>The unit vector opposing <see cref="Right"/>.</value>
    public Vector3d Left
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -Right;
    }

    /// <summary>
    /// Gets the semantic up direction expressed in this convention's component basis.
    /// </summary>
    /// <value>The signed unit vector for <see cref="UpAxis"/>.</value>
    public Vector3d Up
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => AxisToVector(UpAxis);
    }

    /// <summary>
    /// Gets the semantic down direction expressed in this convention's component basis.
    /// </summary>
    /// <value>The unit vector opposing <see cref="Up"/>.</value>
    public Vector3d Down
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -Up;
    }

    /// <summary>
    /// Gets the semantic forward direction expressed in this convention's component basis.
    /// </summary>
    /// <value>The signed unit vector for <see cref="ForwardAxis"/>.</value>
    public Vector3d Forward
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => AxisToVector(ForwardAxis);
    }

    /// <summary>
    /// Gets the semantic backward direction expressed in this convention's component basis.
    /// </summary>
    /// <value>The unit vector opposing <see cref="Forward"/>.</value>
    public Vector3d Backward
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -Forward;
    }

    /// <summary>
    /// Converts a direction vector from this convention into FixedMathSharp's canonical convention.
    /// </summary>
    /// <param name="direction">The direction vector expressed in this convention.</param>
    /// <returns>The same semantic direction expressed in the canonical <c>+Z</c>-forward convention.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ToCanonicalDirection(Vector3d direction) => new(
        GetSignedComponent(direction, RightAxis),
        GetSignedComponent(direction, UpAxis),
        GetSignedComponent(direction, ForwardAxis));

    /// <summary>
    /// Converts a direction vector from FixedMathSharp's canonical convention into this convention.
    /// </summary>
    /// <param name="canonicalDirection">The direction vector expressed in FixedMathSharp's canonical convention.</param>
    /// <returns>The same semantic direction expressed in this convention.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d FromCanonicalDirection(Vector3d canonicalDirection) =>
        (AxisToVector(RightAxis) * canonicalDirection.X) +
        (AxisToVector(UpAxis) * canonicalDirection.Y) +
        (AxisToVector(ForwardAxis) * canonicalDirection.Z);

    /// <summary>Compares this value with another instance.</summary>
    /// <param name="other">The value to compare.</param>
    /// <returns><see langword="true"/> if the signed right, up, and forward axes match, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(CoordinateConvention3d other) =>
        RightAxis == other.RightAxis &&
        UpAxis == other.UpAxis &&
        ForwardAxis == other.ForwardAxis;

    /// <summary>Compares this value with an object.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if <paramref name="obj"/> is a <see cref="CoordinateConvention3d"/> and the signed right, up, and forward axes match, otherwise <see langword="false"/>.</returns>
    public override bool Equals(object? obj) => obj is CoordinateConvention3d other && Equals(other);

    /// <summary>Computes the hash code for this value.</summary>
    /// <returns>A hash combining the three signed axes in right, up, forward order.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() =>
        ((int)RightAxis * 397) ^
        ((int)UpAxis * 31) ^
        (int)ForwardAxis;

    /// <summary>Tests two values for equality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if the signed right, up, and forward axes match, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(CoordinateConvention3d left, CoordinateConvention3d right) => left.Equals(right);

    /// <summary>Tests two values for inequality.</summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if a corresponding signed axis differs, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(CoordinateConvention3d left, CoordinateConvention3d right) => !left.Equals(right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d AxisToVector(Axis3d axis)
    {
        Fixed64 value = IsPositiveAxis(axis) ? Fixed64.One : -Fixed64.One;
        int absoluteAxis = GetAbsoluteAxis(axis);

        if (absoluteAxis == 0)
            return new Vector3d(value, Fixed64.Zero, Fixed64.Zero);

        return absoluteAxis == 1
            ? new Vector3d(Fixed64.Zero, value, Fixed64.Zero)
            : new Vector3d(Fixed64.Zero, Fixed64.Zero, value);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetSignedComponent(Vector3d direction, Axis3d axis)
    {
        int absoluteAxis = GetAbsoluteAxis(axis);
        Fixed64 component = absoluteAxis == 0
            ? direction.X
            : absoluteAxis == 1
                ? direction.Y
                : direction.Z;

        return IsPositiveAxis(axis) ? component : -component;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetAbsoluteAxis(Axis3d axis) => (int)axis >> 1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsDefinedAxis(Axis3d axis) => axis >= Axis3d.PositiveX && axis <= Axis3d.NegativeZ;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPositiveAxis(Axis3d axis) => ((int)axis & 1) == 0;
}
