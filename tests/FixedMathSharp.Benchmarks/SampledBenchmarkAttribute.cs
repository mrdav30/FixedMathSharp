using System;
using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Benchmarks;

/// <summary>
/// Normalizes loop-style bounds benchmarks so BenchmarkDotNet reports one
/// fixture operation instead of the whole sampled batch.
/// </summary>
[AttributeUsage(AttributeTargets.Method)]
internal sealed class SampledBenchmarkAttribute : BenchmarkAttribute
{
    public SampledBenchmarkAttribute()
        : this(BenchmarkFixtures.SampleCount)
    {
    }

    public SampledBenchmarkAttribute(int operationsPerInvoke)
    {
        OperationsPerInvoke = operationsPerInvoke;
    }
}
