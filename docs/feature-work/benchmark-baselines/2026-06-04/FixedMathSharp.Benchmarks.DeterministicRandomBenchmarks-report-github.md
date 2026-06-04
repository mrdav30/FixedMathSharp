```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method                | Mean       | Error    | StdDev   | Allocated |
|---------------------- |-----------:|---------:|---------:|----------:|
| NextU64               | 1,083.6 ns | 21.65 ns | 28.16 ns |         - |
| Next                  | 1,040.8 ns | 11.92 ns |  9.95 ns |         - |
| NextBoundedInt        | 1,585.9 ns | 28.58 ns | 29.35 ns |         - |
| NextIntRange          | 1,211.2 ns | 23.35 ns | 28.68 ns |         - |
| NextDouble            | 1,320.5 ns | 24.67 ns | 23.08 ns |         - |
| NextFixed6401         | 1,427.4 ns | 25.22 ns | 21.06 ns |         - |
| NextFixed64Max        | 1,425.9 ns | 26.93 ns | 25.19 ns |         - |
| NextFixed64Range      | 1,875.7 ns | 28.90 ns | 24.14 ns |         - |
| NextBytes             |   230.5 ns |  4.57 ns |  6.41 ns |         - |
| SeededStreams         |   540.8 ns |  8.80 ns |  8.23 ns |         - |
| FeatureDerivedStreams | 1,095.2 ns |  9.73 ns |  8.13 ns |         - |
