```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method           | Mean      | Error     | StdDev    | Allocated |
|----------------- |----------:|----------:|----------:|----------:|
| Add              |  1.571 μs | 0.0182 μs | 0.0170 μs |         - |
| AddOut           |  1.542 μs | 0.0058 μs | 0.0054 μs |         - |
| AddScale         | 12.830 μs | 0.0837 μs | 0.0783 μs |         - |
| Dot              |  5.590 μs | 0.0803 μs | 0.0751 μs |         - |
| Magnitude        | 36.335 μs | 0.1779 μs | 0.1577 μs |         - |
| Normal           | 86.907 μs | 1.1197 μs | 0.9350 μs |         - |
| NormalizeInPlace | 87.263 μs | 1.4656 μs | 1.3710 μs |         - |
| GetNormalized    | 89.416 μs | 1.1059 μs | 0.9804 μs |         - |
| Distance         | 27.698 μs | 0.3340 μs | 0.3124 μs |         - |
| Lerp             | 14.017 μs | 0.2748 μs | 0.4279 μs |         - |
