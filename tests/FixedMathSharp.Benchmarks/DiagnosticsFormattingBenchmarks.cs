using BenchmarkDotNet.Attributes;
using System;
using System.Globalization;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class DiagnosticsFormattingBenchmarks
{
    private readonly Fixed64[] _scalars = BenchmarkFixtures.ScalarsA;
    private readonly Vector3d[] _vectors = BenchmarkFixtures.VectorsA;
    private readonly FixedQuaternion[] _quaternions = BenchmarkFixtures.RotationsA;
    private readonly Fixed4x4[] _matrices = BenchmarkFixtures.Matrices;
    private readonly char[] _buffer = new char[512];

    [Benchmark]
    public int Fixed64ToString()
    {
        int length = 0;
        for (int i = 0; i < _scalars.Length; i++)
            length += _scalars[i].ToString("0.00", CultureInfo.InvariantCulture).Length;

        return length;
    }

    [Benchmark]
    public int Fixed64TryFormat()
    {
        int length = 0;
        Span<char> destination = _buffer.AsSpan();
        for (int i = 0; i < _scalars.Length; i++)
        {
            if (_scalars[i].TryFormat(destination, out int charsWritten, "0.00", CultureInfo.InvariantCulture))
                length += charsWritten;
        }

        return length;
    }

    [Benchmark]
    public int Vector3dToString()
    {
        int length = 0;
        for (int i = 0; i < _vectors.Length; i++)
            length += _vectors[i].ToString("0.00", CultureInfo.InvariantCulture).Length;

        return length;
    }

    [Benchmark]
    public int Vector3dTryFormat()
    {
        int length = 0;
        Span<char> destination = _buffer.AsSpan();
        for (int i = 0; i < _vectors.Length; i++)
        {
            if (_vectors[i].TryFormat(destination, out int charsWritten, "0.00", CultureInfo.InvariantCulture))
                length += charsWritten;
        }

        return length;
    }

    [Benchmark]
    public int FixedQuaternionToString()
    {
        int length = 0;
        for (int i = 0; i < _quaternions.Length; i++)
            length += _quaternions[i].ToString("0.00", CultureInfo.InvariantCulture).Length;

        return length;
    }

    [Benchmark]
    public int FixedQuaternionTryFormat()
    {
        int length = 0;
        Span<char> destination = _buffer.AsSpan();
        for (int i = 0; i < _quaternions.Length; i++)
        {
            if (_quaternions[i].TryFormat(destination, out int charsWritten, "0.00", CultureInfo.InvariantCulture))
                length += charsWritten;
        }

        return length;
    }

    [Benchmark]
    public int Fixed4x4ToString()
    {
        int length = 0;
        for (int i = 0; i < _matrices.Length; i++)
            length += _matrices[i].ToString("0.00", CultureInfo.InvariantCulture).Length;

        return length;
    }

    [Benchmark]
    public int Fixed4x4TryFormat()
    {
        int length = 0;
        Span<char> destination = _buffer.AsSpan();
        for (int i = 0; i < _matrices.Length; i++)
        {
            if (_matrices[i].TryFormat(destination, out int charsWritten, "0.00", CultureInfo.InvariantCulture))
                length += charsWritten;
        }

        return length;
    }
}
