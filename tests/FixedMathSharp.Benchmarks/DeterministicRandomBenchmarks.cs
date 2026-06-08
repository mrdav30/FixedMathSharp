using BenchmarkDotNet.Attributes;
using FixedMathSharp.Random;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class DeterministicRandomBenchmarks
{
    private readonly byte[] _bytes = new byte[1024];

    [Benchmark]
    public ulong NextU64()
    {
        var rng = new DeterministicRandom(0xD15EA5EUL);
        ulong accumulator = 0UL;
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator ^= rng.NextU64();

        return accumulator;
    }

    [Benchmark]
    public int Next()
    {
        var rng = new DeterministicRandom(0xD15EA5EUL);
        int accumulator = 0;
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator ^= rng.Next();

        return accumulator;
    }

    [Benchmark]
    public int NextBoundedInt()
    {
        var rng = new DeterministicRandom(0xB0075EEDUL);
        int accumulator = 0;
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator += rng.Next(97);

        return accumulator;
    }

    [Benchmark]
    public int NextIntRange()
    {
        var rng = new DeterministicRandom(0xB0075EEDUL);
        int accumulator = 0;
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator += rng.Next(-128, 128);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 NextFixed6401()
    {
        var rng = new DeterministicRandom(0xF164UL);
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator += rng.NextFixed6401();

        return accumulator;
    }

    [Benchmark]
    public Fixed64 NextFixed64Max()
    {
        var rng = new DeterministicRandom(0xF164UL);
        Fixed64 accumulator = Fixed64.Zero;
        Fixed64 max = new Fixed64(128);
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator += rng.NextFixed64(max);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 NextFixed64Range()
    {
        var rng = new DeterministicRandom(0xF164UL);
        Fixed64 accumulator = Fixed64.Zero;
        Fixed64 min = new Fixed64(-64);
        Fixed64 max = new Fixed64(64);
        for (int i = 0; i < BenchmarkFixtures.SampleCount * 4; i++)
            accumulator += rng.NextFixed64(min, max);

        return accumulator;
    }

    [Benchmark]
    public byte NextBytes()
    {
        var rng = new DeterministicRandom(0xB17E5UL);
        rng.NextBytes(_bytes);
        return _bytes[_bytes.Length - 1];
    }

    [Benchmark]
    public ulong SeededStreams()
    {
        ulong accumulator = 0UL;
        for (ulong i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            var rng = new DeterministicRandom(0x51A7E000UL + i);
            accumulator ^= rng.NextU64();
        }

        return accumulator;
    }

    [Benchmark]
    public ulong FeatureDerivedStreams()
    {
        ulong accumulator = 0UL;
        for (ulong i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            var rng = DeterministicRandom.FromWorldFeature(0xDEADBEEFCAFEBABEUL, 0x4F5245UL, i);
            accumulator ^= rng.NextU64();
        }

        return accumulator;
    }
}
