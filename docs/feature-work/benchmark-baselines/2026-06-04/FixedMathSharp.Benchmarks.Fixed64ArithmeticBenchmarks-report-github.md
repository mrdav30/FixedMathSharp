```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method              | Mean       | Error     | StdDev    | Allocated |
|-------------------- |-----------:|----------:|----------:|----------:|
| AddSubtractMultiply |   7.299 μs | 0.1412 μs | 0.1251 μs |         - |
| Divide              |   6.263 μs | 0.1076 μs | 0.0954 μs |         - |
| Sqrt                |  33.880 μs | 0.6426 μs | 0.5697 μs |         - |
| Sin                 |  39.855 μs | 0.5704 μs | 0.4763 μs |         - |
| Cos                 |  45.510 μs | 0.8679 μs | 0.8524 μs |         - |
| Tan                 | 103.154 μs | 1.8465 μs | 1.7272 μs |         - |
| Acos                | 124.290 μs | 1.0053 μs | 0.8395 μs |         - |
| Asin                |  77.112 μs | 1.0192 μs | 0.9035 μs |         - |
| Atan                |  92.729 μs | 1.6797 μs | 1.5712 μs |         - |
| Atan2               |  93.539 μs | 1.7278 μs | 1.6162 μs |         - |
| Pow                 | 136.806 μs | 2.2170 μs | 2.9596 μs |         - |
| Log2                |  37.717 μs | 0.7290 μs | 0.8102 μs |         - |
| Ln                  |  44.761 μs | 0.7827 μs | 1.0714 μs |         - |
