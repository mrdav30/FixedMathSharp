using System;
using System.Runtime.CompilerServices;
using FixedMathSharp.Bounds;

namespace FixedMathSharp.Chronicler;

/// <summary>
/// Provides deterministic <see cref="global::Chronicler.ChronicleHashWriter"/> extensions for FixedMathSharp types.
/// </summary>
public static class FixedMathChronicleHashWriterExtensions
{
    /// <summary>
    /// Writes a fixed-point value by its raw Q32.32 payload.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void WriteFixed64(this ref global::Chronicler.ChronicleHashWriter writer, Fixed64 value)
    {
        writer.WriteInt64(value.m_rawValue);
    }

    /// <summary>
    /// Writes a 2D vector in X, Y component order.
    /// </summary>
    public static void WriteVector2d(this ref global::Chronicler.ChronicleHashWriter writer, Vector2d value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
    }

    /// <summary>
    /// Writes a 3D vector in X, Y, Z component order.
    /// </summary>
    public static void WriteVector3d(this ref global::Chronicler.ChronicleHashWriter writer, Vector3d value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
        writer.WriteFixed64(value.Z);
    }

    /// <summary>
    /// Writes a 4D vector in X, Y, Z, W component order.
    /// </summary>
    public static void WriteVector4d(this ref global::Chronicler.ChronicleHashWriter writer, Vector4d value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
        writer.WriteFixed64(value.Z);
        writer.WriteFixed64(value.W);
    }

    /// <summary>
    /// Writes a quaternion in X, Y, Z, W component order.
    /// </summary>
    public static void WriteQuaternion(this ref global::Chronicler.ChronicleHashWriter writer, FixedQuaternion value)
    {
        writer.WriteFixed64(value.X);
        writer.WriteFixed64(value.Y);
        writer.WriteFixed64(value.Z);
        writer.WriteFixed64(value.W);
    }

    /// <summary>
    /// Writes a transform as position, rotation, then scale. Parent identity and derived Euler angles are not hashed.
    /// </summary>
    public static void WriteTransform(this ref global::Chronicler.ChronicleHashWriter writer, FixedTransform value)
    {
        if (value == null)
            throw new ArgumentNullException(nameof(value));

        writer.WriteVector3d(value.Position);
        writer.WriteQuaternion(value.Rotation);
        writer.WriteVector3d(value.Scale);
    }

    /// <summary>
    /// Writes a 3x3 matrix in row-major order.
    /// </summary>
    public static void WriteFixed3x3(this ref global::Chronicler.ChronicleHashWriter writer, Fixed3x3 value)
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
    public static void WriteFixed4x4(this ref global::Chronicler.ChronicleHashWriter writer, Fixed4x4 value)
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
    /// Writes a bounding box by canonical minimum then maximum corners.
    /// </summary>
    public static void WriteBoundBox(this ref global::Chronicler.ChronicleHashWriter writer, FixedBoundBox value)
    {
        writer.WriteVector3d(value.Min);
        writer.WriteVector3d(value.Max);
    }

    /// <summary>
    /// Writes a bounding area by canonical minimum then maximum corners.
    /// </summary>
    public static void WriteBoundArea(this ref global::Chronicler.ChronicleHashWriter writer, FixedBoundArea value)
    {
        writer.WriteVector3d(value.Min);
        writer.WriteVector3d(value.Max);
    }

    /// <summary>
    /// Writes a bounding sphere as center then radius.
    /// </summary>
    public static void WriteBoundSphere(this ref global::Chronicler.ChronicleHashWriter writer, FixedBoundSphere value)
    {
        writer.WriteVector3d(value.Center);
        writer.WriteFixed64(value.Radius);
    }

    /// <summary>
    /// Writes a ray as position then direction.
    /// </summary>
    public static void WriteRay(this ref global::Chronicler.ChronicleHashWriter writer, FixedRay value)
    {
        writer.WriteVector3d(value.Position);
        writer.WriteVector3d(value.Direction);
    }

    /// <summary>
    /// Writes a plane as normal then distance component.
    /// </summary>
    public static void WritePlane(this ref global::Chronicler.ChronicleHashWriter writer, FixedPlane value)
    {
        writer.WriteVector3d(value.Normal);
        writer.WriteFixed64(value.D);
    }
}
