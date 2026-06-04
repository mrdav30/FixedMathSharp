```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method              | Mean        | Error     | StdDev    | Allocated |
|-------------------- |------------:|----------:|----------:|----------:|
| InRangeExclusive    |    425.1 ns |   8.42 ns |  14.53 ns |         - |
| InRangeInclusive    |    438.0 ns |   9.71 ns |  28.64 ns |         - |
| InRangeDouble       |    594.3 ns |  11.71 ns |  24.70 ns |         - |
| Overlaps            |    355.4 ns |   7.09 ns |   9.22 ns |         - |
| GetDirection        |    696.0 ns |   5.59 ns |   5.22 ns |         - |
| ComputeOverlapDepth |    944.4 ns |  12.88 ns |  11.42 ns |         - |
| CheckOverlap        | 10,076.2 ns | 190.94 ns | 212.23 ns |         - |
| Add                 |  2,190.5 ns |  30.47 ns |  27.01 ns |         - |
| Subtract            |  2,311.7 ns |  45.42 ns |  55.78 ns |         - |
| AddInPlace          |    709.4 ns |  11.36 ns |  10.07 ns |         - |
