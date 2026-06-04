```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method                      | Mean      | Error     | StdDev    | Allocated |
|---------------------------- |----------:|----------:|----------:|----------:|
| LinearEvaluate              | 11.900 μs | 0.2138 μs | 0.2545 μs |         - |
| StepEvaluate                |  9.377 μs | 0.1524 μs | 0.1425 μs |         - |
| SmoothEvaluate              | 17.898 μs | 0.2109 μs | 0.1973 μs |         - |
| CubicEvaluate               | 30.218 μs | 0.5858 μs | 0.5193 μs |         - |
| LinearEvaluateTwoKeys       | 14.711 μs | 0.2801 μs | 0.3334 μs |         - |
| LinearEvaluateThirtyTwoKeys | 12.404 μs | 0.2391 μs | 0.2236 μs |         - |
