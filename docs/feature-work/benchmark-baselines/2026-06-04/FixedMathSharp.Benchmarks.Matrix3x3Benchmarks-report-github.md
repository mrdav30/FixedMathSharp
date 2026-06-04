```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method             | Mean      | Error    | StdDev   | Allocated |
|------------------- |----------:|---------:|---------:|----------:|
| CreateRotation     | 467.73 μs | 3.659 μs | 3.055 μs |         - |
| Multiply           |  75.75 μs | 0.630 μs | 0.526 μs |         - |
| TransformDirection |  18.97 μs | 0.371 μs | 0.456 μs |         - |
| Determinant        |  16.09 μs | 0.312 μs | 0.334 μs |         - |
| Invert             | 155.49 μs | 2.916 μs | 3.120 μs |         - |
