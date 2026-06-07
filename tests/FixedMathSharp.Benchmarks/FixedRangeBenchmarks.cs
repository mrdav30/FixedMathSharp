using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class FixedRangeBenchmarks
{
    private readonly Fixed64[] _scalars = BenchmarkFixtures.ScalarsA;
    private readonly FixedRange[] _rangesA = BenchmarkFixtures.RangesA;
    private readonly FixedRange[] _rangesB = BenchmarkFixtures.RangesB;
    private readonly Vector3d[] _origins = BenchmarkFixtures.NormalizedAxes;

    [Benchmark]
    public int InRangeExclusive()
    {
        int count = 0;
        for (int i = 0; i < _rangesA.Length; i++)
        {
            if (_rangesA[i].InRange(_scalars[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int InRangeInclusive()
    {
        int count = 0;
        for (int i = 0; i < _rangesA.Length; i++)
        {
            if (_rangesA[i].InRange(_scalars[i], includeMax: true))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int InRangeDouble()
    {
        int count = 0;
        for (int i = 0; i < _rangesA.Length; i++)
        {
            if (_rangesA[i].InRange((double)_scalars[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int Overlaps()
    {
        int count = 0;
        for (int i = 0; i < _rangesA.Length; i++)
        {
            if (_rangesA[i].Overlaps(_rangesB[i]))
                count++;
        }

        return count;
    }

    [Benchmark]
    public Fixed64 GetDirection()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rangesA.Length; i++)
        {
            if (FixedRange.GetDirection(_rangesA[i], _rangesB[i], out Fixed64? sign) && sign.HasValue)
                accumulator += sign.Value;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 ComputeOverlapDepth()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _rangesA.Length; i++)
            accumulator += FixedRange.ComputeOverlapDepth(_rangesA[i], _rangesB[i]);

        return accumulator;
    }

    [Benchmark]
    public Vector3d CheckOverlap()
    {
        Vector3d accumulator = Vector3d.Zero;
        for (int i = 0; i < _rangesA.Length; i++)
        {
            if (FixedRange.CheckOverlap(_origins[i], _rangesA[i], _rangesB[i], Fixed64.MaxValue, Fixed64.One, out (Vector3d Vector, Fixed64 Depth)? output) && output.HasValue)
                accumulator += output.Value.Vector + new Vector3d(output.Value.Depth, output.Value.Depth, output.Value.Depth);
        }

        return accumulator;
    }

    [Benchmark]
    public FixedRange Add()
    {
        FixedRange accumulator = new(Fixed64.Zero, Fixed64.Zero);
        for (int i = 0; i < _rangesA.Length; i++)
            accumulator += _rangesA[i] + _rangesB[i];

        return accumulator;
    }

    [Benchmark]
    public FixedRange Subtract()
    {
        FixedRange accumulator = new(Fixed64.Zero, Fixed64.Zero);
        for (int i = 0; i < _rangesA.Length; i++)
            accumulator += _rangesA[i] - _rangesB[i];

        return accumulator;
    }

    [Benchmark]
    public FixedRange AddInPlace()
    {
        FixedRange accumulator = new(Fixed64.Zero, Fixed64.Zero);
        for (int i = 0; i < _rangesA.Length; i++)
            accumulator.AddInPlace(_rangesA[i].Length);

        return accumulator;
    }
}
