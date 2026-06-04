```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method               | Mean      | Error    | StdDev   | Allocated |
|--------------------- |----------:|---------:|---------:|----------:|
| CreateTranslation    |  49.34 μs | 0.737 μs | 0.689 μs |         - |
| CreateRotation       | 121.37 μs | 1.843 μs | 1.634 μs |         - |
| CreateScale          |  50.01 μs | 0.986 μs | 1.283 μs |         - |
| CreateTransform      | 201.88 μs | 3.500 μs | 2.922 μs |         - |
| ScaleRotateTranslate | 267.88 μs | 4.872 μs | 5.415 μs |         - |
| TranslateRotateScale | 287.10 μs | 3.752 μs | 3.326 μs |         - |
| Multiply             | 100.19 μs | 1.999 μs | 2.669 μs |         - |
| TransformPoint       |  25.57 μs | 0.160 μs | 0.142 μs |         - |
| InvertAffine         | 215.20 μs | 3.838 μs | 3.590 μs |         - |
| InvertFull           | 390.78 μs | 3.940 μs | 3.290 μs |         - |
