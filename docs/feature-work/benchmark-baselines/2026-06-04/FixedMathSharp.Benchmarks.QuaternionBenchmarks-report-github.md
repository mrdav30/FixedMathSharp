```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method          | Mean      | Error    | StdDev   | Allocated |
|---------------- |----------:|---------:|---------:|----------:|
| FromAxisAngle   | 216.72 μs | 3.788 μs | 3.721 μs |         - |
| AngleAxis       | 168.05 μs | 2.118 μs | 1.878 μs |         - |
| FromEulerAngles | 468.24 μs | 6.703 μs | 5.942 μs |         - |
| Multiply        |  82.31 μs | 1.031 μs | 0.914 μs |         - |
| Lerp            | 143.19 μs | 2.170 μs | 1.924 μs |         - |
| Slerp           | 256.60 μs | 1.443 μs | 1.205 μs |         - |
| Normalize       |  93.87 μs | 1.774 μs | 1.742 μs |         - |
| ToEulerAngles   | 297.75 μs | 2.442 μs | 2.039 μs |         - |
| FromDirection   | 443.02 μs | 5.805 μs | 5.430 μs |         - |
| RotateVector    |  48.20 μs | 0.841 μs | 0.745 μs |         - |
