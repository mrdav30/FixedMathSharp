```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method           | Mean        | Error     | StdDev    | Allocated |
|----------------- |------------:|----------:|----------:|----------:|
| Add              |    686.1 ns |   5.45 ns |   4.55 ns |         - |
| AddOut           |    689.1 ns |   3.11 ns |   2.76 ns |         - |
| AddScale         |  8,852.9 ns | 160.61 ns | 150.24 ns |         - |
| Dot              |  2,650.4 ns |  19.28 ns |  17.09 ns |         - |
| Magnitude        | 34,212.8 ns | 546.80 ns | 511.48 ns |         - |
| Normal           | 54,734.5 ns | 247.80 ns | 219.67 ns |         - |
| NormalizeInPlace | 56,000.0 ns | 495.87 ns | 463.83 ns |         - |
| GetNormalized    | 58,083.7 ns | 772.33 ns | 722.43 ns |         - |
| Distance         | 17,014.8 ns | 330.13 ns | 483.91 ns |         - |
| Lerp             |  9,591.2 ns | 124.58 ns | 116.54 ns |         - |
| LerpOut          |  9,418.9 ns |  79.24 ns |  74.13 ns |         - |
