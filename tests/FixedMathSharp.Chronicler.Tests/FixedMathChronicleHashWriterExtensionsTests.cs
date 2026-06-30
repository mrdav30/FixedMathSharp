using System;
using Chronicler;
using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Chronicler.Tests;

public sealed class FixedMathChronicleHashWriterExtensionsTests
{
    [Fact]
    public void WriteFixed64_UsesRawFixedPointPayload()
    {
        Fixed64 value = Raw(unchecked((long)0x0102030405060708UL));

        ChronicleHash actual = Hash((ref ChronicleHashWriter writer) => writer.WriteFixed64(value));
        ChronicleHash expected = Hash((ref ChronicleHashWriter writer) => writer.WriteInt64(value.m_rawValue));

        Assert.Equal(expected, actual);
    }

    [Fact]
    public void WriteFixed64_DistinguishesAdjacentRawValues()
    {
        Fixed64 first = Raw(Fixed64.One.m_rawValue);
        Fixed64 second = Raw(Fixed64.One.m_rawValue + 1);

        Assert.NotEqual(
            Hash((ref ChronicleHashWriter writer) => writer.WriteFixed64(first)),
            Hash((ref ChronicleHashWriter writer) => writer.WriteFixed64(second)));
    }

    [Fact]
    public void VectorWriters_UseCanonicalComponentOrder()
    {
        var vector2d = new Vector2d(Raw(1), Raw(2));
        var vector3d = new Vector3d(Raw(3), Raw(4), Raw(5));
        var vector4d = new Vector4d(Raw(6), Raw(7), Raw(8), Raw(9));

        Assert.Equal(
            HashRaw(1, 2),
            Hash((ref ChronicleHashWriter writer) => writer.WriteVector2d(vector2d)));
        Assert.Equal(
            HashRaw(3, 4, 5),
            Hash((ref ChronicleHashWriter writer) => writer.WriteVector3d(vector3d)));
        Assert.Equal(
            HashRaw(6, 7, 8, 9),
            Hash((ref ChronicleHashWriter writer) => writer.WriteVector4d(vector4d)));
    }

    [Fact]
    public void RotationTransformAndMatrixWriters_UseCanonicalFieldOrder()
    {
        var quaternion = new FixedQuaternion(Raw(1), Raw(2), Raw(3), Raw(4));
        var transform = new FixedTransform(
            new Vector3d(Raw(5), Raw(6), Raw(7)),
            FixedQuaternion.Identity,
            new Vector3d(Raw(8), Raw(9), Raw(10)));
        var matrix3 = new Fixed3x3(
            Raw(11), Raw(12), Raw(13),
            Raw(14), Raw(15), Raw(16),
            Raw(17), Raw(18), Raw(19));
        var matrix4 = new Fixed4x4(
            Raw(20), Raw(21), Raw(22), Raw(23),
            Raw(24), Raw(25), Raw(26), Raw(27),
            Raw(28), Raw(29), Raw(30), Raw(31),
            Raw(32), Raw(33), Raw(34), Raw(35));

        Assert.Equal(
            HashRaw(1, 2, 3, 4),
            Hash((ref ChronicleHashWriter writer) => writer.WriteQuaternion(quaternion)));
        Assert.Equal(
            HashRaw(
                transform.Position.X.m_rawValue,
                transform.Position.Y.m_rawValue,
                transform.Position.Z.m_rawValue,
                transform.Rotation.X.m_rawValue,
                transform.Rotation.Y.m_rawValue,
                transform.Rotation.Z.m_rawValue,
                transform.Rotation.W.m_rawValue,
                transform.Scale.X.m_rawValue,
                transform.Scale.Y.m_rawValue,
                transform.Scale.Z.m_rawValue),
            Hash((ref ChronicleHashWriter writer) => writer.WriteTransform(transform)));
        Assert.Equal(
            HashRaw(11, 12, 13, 14, 15, 16, 17, 18, 19),
            Hash((ref ChronicleHashWriter writer) => writer.WriteFixed3x3(matrix3)));
        Assert.Equal(
            HashRaw(20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35),
            Hash((ref ChronicleHashWriter writer) => writer.WriteFixed4x4(matrix4)));
    }

    [Fact]
    public void WriteTransform_RejectsNullTransform()
    {
        var writer = new ChronicleHashWriter();

        Assert.Throws<ArgumentNullException>(() => writer.WriteTransform(null!));
    }

    [Fact]
    public void GeometryWriters_UseCanonicalDeterministicFields()
    {
        var area = FixedBoundArea.FromCenterAndSize(
            new Vector2d(Raw(5), Raw(6)),
            new Vector2d(Raw(2), Raw(4)));
        var box = FixedBoundBox.FromCenterAndSize(
            new Vector3d(Raw(10), Raw(20), Raw(30)),
            new Vector3d(Raw(8), Raw(10), Raw(12)));
        var sphere = new FixedBoundSphere(new Vector3d(Raw(21), Raw(22), Raw(23)), Raw(24));
        var ray = new FixedRay(
            new Vector3d(Raw(31), Raw(32), Raw(33)),
            new Vector3d(Raw(34), Raw(35), Raw(36)));
        var plane = new FixedPlane(new Vector3d(Raw(41), Raw(42), Raw(43)), Raw(44));

        Assert.Equal(
            HashVector2Pair(area.Min, area.Max),
            Hash((ref ChronicleHashWriter writer) => writer.WriteBoundArea(area)));
        Assert.Equal(
            HashVector3Pair(box.Min, box.Max),
            Hash((ref ChronicleHashWriter writer) => writer.WriteBoundBox(box)));
        Assert.Equal(
            HashRaw(21, 22, 23, 24),
            Hash((ref ChronicleHashWriter writer) => writer.WriteBoundSphere(sphere)));
        Assert.Equal(
            HashRaw(31, 32, 33, 34, 35, 36),
            Hash((ref ChronicleHashWriter writer) => writer.WriteRay(ray)));
        Assert.Equal(
            HashRaw(41, 42, 43, 44),
            Hash((ref ChronicleHashWriter writer) => writer.WritePlane(plane)));
    }

    [Fact]
    public void ExtensionWriters_DoNotAllocateAfterWarmup()
    {
        var transform = new FixedTransform(
            new Vector3d(Raw(1), Raw(2), Raw(3)),
            FixedQuaternion.Identity,
            new Vector3d(Raw(4), Raw(5), Raw(6)));
        var area = FixedBoundArea.FromCenterAndSize(
            new Vector2d(Raw(5), Raw(6)),
            new Vector2d(Raw(2), Raw(4)));
        var box = FixedBoundBox.FromCenterAndSize(
            new Vector3d(Raw(7), Raw(8), Raw(9)),
            new Vector3d(Raw(2), Raw(4), Raw(6)));
        var sphere = new FixedBoundSphere(new Vector3d(Raw(16), Raw(17), Raw(18)), Raw(19));
        var ray = new FixedRay(
            new Vector3d(Raw(20), Raw(21), Raw(22)),
            new Vector3d(Raw(23), Raw(24), Raw(25)));
        var plane = new FixedPlane(new Vector3d(Raw(26), Raw(27), Raw(28)), Raw(29));

        for (int i = 0; i < 512; i++)
        {
            var warmup = new ChronicleHashWriter();
            WriteAll(ref warmup, transform, area, box, sphere, ray, plane);
            _ = warmup.ToHash();
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 4096; i++)
        {
            var writer = new ChronicleHashWriter();
            WriteAll(ref writer, transform, area, box, sphere, ray, plane);
            _ = writer.ToHash();
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, allocated);
    }

    private delegate void WriterAction(ref ChronicleHashWriter writer);

    private static ChronicleHash Hash(WriterAction action)
    {
        var writer = new ChronicleHashWriter();
        action(ref writer);
        return writer.ToHash();
    }

    private static ChronicleHash HashRaw(params long[] rawValues)
    {
        var writer = new ChronicleHashWriter();
        for (int i = 0; i < rawValues.Length; i++)
        {
            writer.WriteInt64(rawValues[i]);
        }

        return writer.ToHash();
    }

    private static ChronicleHash HashVector2Pair(Vector2d first, Vector2d second)
    {
        var writer = new ChronicleHashWriter();
        writer.WriteInt64(first.X.m_rawValue);
        writer.WriteInt64(first.Y.m_rawValue);
        writer.WriteInt64(second.X.m_rawValue);
        writer.WriteInt64(second.Y.m_rawValue);
        return writer.ToHash();
    }

    private static ChronicleHash HashVector3Pair(Vector3d first, Vector3d second)
    {
        var writer = new ChronicleHashWriter();
        writer.WriteInt64(first.X.m_rawValue);
        writer.WriteInt64(first.Y.m_rawValue);
        writer.WriteInt64(first.Z.m_rawValue);
        writer.WriteInt64(second.X.m_rawValue);
        writer.WriteInt64(second.Y.m_rawValue);
        writer.WriteInt64(second.Z.m_rawValue);
        return writer.ToHash();
    }

    private static void WriteAll(
        ref ChronicleHashWriter writer,
        FixedTransform transform,
        FixedBoundArea area,
        FixedBoundBox box,
        FixedBoundSphere sphere,
        FixedRay ray,
        FixedPlane plane)
    {
        writer.WriteFixed64(Raw(100));
        writer.WriteVector2d(new Vector2d(Raw(101), Raw(102)));
        writer.WriteVector3d(new Vector3d(Raw(103), Raw(104), Raw(105)));
        writer.WriteVector4d(new Vector4d(Raw(106), Raw(107), Raw(108), Raw(109)));
        writer.WriteQuaternion(FixedQuaternion.Identity);
        writer.WriteTransform(transform);
        writer.WriteFixed3x3(Fixed3x3.Identity);
        writer.WriteFixed4x4(Fixed4x4.Identity);
        writer.WriteBoundArea(area);
        writer.WriteBoundBox(box);
        writer.WriteBoundSphere(sphere);
        writer.WriteRay(ray);
        writer.WritePlane(plane);
    }

    private static Fixed64 Raw(long value)
    {
        return Fixed64.FromRaw(value);
    }
}
