//=======================================================================
// FixedChronicleHashWriterExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using Chronicler.Hashing;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Chronicler;

/// <summary>Appends FixedMathSharp values to a deterministic <see cref="ChronicleHashWriter"/>.</summary>
/// <remarks>
/// Calls append to the caller's writer in call order. Every scalar component contributes
/// its signed 64-bit Q32.32 payload in little-endian byte order. Keep the sequence of calls
/// and the documented field order stable across producers of the same replay hash.
/// </remarks>
public static class FixedChronicleHashWriterExtensions
{
    /// <summary>
    /// Writes a fixed-point value by its raw Q32.32 payload.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The fixed-point value to append. All 64 raw bits are preserved.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void WriteFixed64(this ref ChronicleHashWriter writer, Fixed64 value)
    {
        writer.WriteInt64(value.m_rawValue);
    }

    /// <summary>
    /// Writes a 2D vector in X, Y component order.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The vector whose components are appended as stored.</param>
    public static void WriteVector2d(this ref ChronicleHashWriter writer, Vector2d value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
    }

    /// <summary>
    /// Writes a 3D vector in X, Y, Z component order.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The vector whose components are appended as stored.</param>
    public static void WriteVector3d(this ref ChronicleHashWriter writer, Vector3d value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
        writer.WriteFixed64(value.Z);
    }

    /// <summary>
    /// Writes a 4D vector in X, Y, Z, W component order.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The vector whose components are appended as stored.</param>
    public static void WriteVector4d(this ref ChronicleHashWriter writer, Vector4d value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
        writer.WriteFixed64(value.Z);
        writer.WriteFixed64(value.W);
    }

    /// <summary>
    /// Writes a quaternion in X, Y, Z, W component order.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The quaternion to append. Its components retain their original scale and sign.</param>
    public static void WriteQuaternion(this ref ChronicleHashWriter writer, FixedQuaternion value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
        writer.WriteFixed64(value.Z);
        writer.WriteFixed64(value.W);
    }

    /// <summary>
    /// Writes authoritative local position, local rotation, then local scale.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The transform whose local components are read during this call.</param>
    /// <remarks>
    /// Parent identity and all derived world views are intentionally excluded. Adopting this local
    /// contract is a replay/hash compatibility boundary for hashes produced by older package versions.
    /// </remarks>
    /// <exception cref="ArgumentNullException"><paramref name="value"/> is null. The writer is unmodified.</exception>
    public static void WriteTransform(this ref ChronicleHashWriter writer, FixedTransform value)
    {
        if (value == null)
            throw new ArgumentNullException(nameof(value));

        writer.WriteVector3d(value.LocalPosition);
        writer.WriteQuaternion(value.LocalRotation);
        writer.WriteVector3d(value.LocalScale);
    }

    /// <summary>
    /// Writes a 3x3 matrix in row-major order.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The matrix whose entries are appended as stored.</param>
    public static void WriteFixed3x3(this ref ChronicleHashWriter writer, Fixed3x3 value)
    {
        writer.WriteFixed64(value.M11);
        writer.WriteFixed64(value.M12);
        writer.WriteFixed64(value.M13);
        writer.WriteFixed64(value.M21);
        writer.WriteFixed64(value.M22);
        writer.WriteFixed64(value.M23);
        writer.WriteFixed64(value.M31);
        writer.WriteFixed64(value.M32);
        writer.WriteFixed64(value.M33);
    }

    /// <summary>
    /// Writes a 4x4 matrix in row-major order.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The matrix whose entries are appended as stored.</param>
    public static void WriteFixed4x4(this ref ChronicleHashWriter writer, Fixed4x4 value)
    {
        writer.WriteFixed64(value.M11);
        writer.WriteFixed64(value.M12);
        writer.WriteFixed64(value.M13);
        writer.WriteFixed64(value.M14);
        writer.WriteFixed64(value.M21);
        writer.WriteFixed64(value.M22);
        writer.WriteFixed64(value.M23);
        writer.WriteFixed64(value.M24);
        writer.WriteFixed64(value.M31);
        writer.WriteFixed64(value.M32);
        writer.WriteFixed64(value.M33);
        writer.WriteFixed64(value.M34);
        writer.WriteFixed64(value.M41);
        writer.WriteFixed64(value.M42);
        writer.WriteFixed64(value.M43);
        writer.WriteFixed64(value.M44);
    }

    /// <summary>
    /// Writes a 2D bounding area by canonical minimum then maximum corners.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The area whose two corners are appended in its own coordinate space.</param>
    public static void WriteBoundArea(this ref ChronicleHashWriter writer, FixedBoundArea value)
    {
        writer.WriteVector2d(value.Min);
        writer.WriteVector2d(value.Max);
    }

    /// <summary>
    /// Writes a bounding box by canonical minimum then maximum corners.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The box whose two corners are appended in its own coordinate space.</param>
    public static void WriteBoundBox(this ref ChronicleHashWriter writer, FixedBoundBox value)
    {
        writer.WriteVector3d(value.Min);
        writer.WriteVector3d(value.Max);
    }

    /// <summary>
    /// Writes a 2D bounding circle as center then normalized radius.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The circle whose stored center and radius are appended.</param>
    public static void WriteBoundCircle(this ref ChronicleHashWriter writer, FixedBoundCircle value)
    {
        writer.WriteVector2d(value.Center);
        writer.WriteFixed64(value.Radius);
    }

    /// <summary>
    /// Writes a bounding sphere as center then radius.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The sphere whose stored center and radius are appended.</param>
    public static void WriteBoundSphere(this ref ChronicleHashWriter writer, FixedBoundSphere value)
    {
        writer.WriteVector3d(value.Center);
        writer.WriteFixed64(value.Radius);
    }

    /// <summary>
    /// Writes a ray as position then direction.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The ray whose stored origin and direction are appended.</param>
    public static void WriteRay(this ref ChronicleHashWriter writer, FixedRay value)
    {
        writer.WriteVector3d(value.Position);
        writer.WriteVector3d(value.Direction);
    }

    /// <summary>
    /// Writes a plane as normal then distance component.
    /// </summary>
    /// <param name="writer">The caller-owned hash writer to append to in place.</param>
    /// <param name="value">The plane whose stored normal and plane-equation constant are appended.</param>
    public static void WritePlane(this ref ChronicleHashWriter writer, FixedPlane value)
    {
        writer.WriteVector3d(value.Normal);
        writer.WriteFixed64(value.D);
    }
}
