```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method           | Mean        | Error     | StdDev    | Allocated |
|----------------- |------------:|----------:|----------:|----------:|
| Add              |    970.2 ns |  12.83 ns |  12.00 ns |         - |
| AddOut           |  1,053.7 ns |   8.58 ns |   8.03 ns |         - |
| AddScale         | 11,047.5 ns |  79.59 ns |  62.14 ns |         - |
| Dot              |  3,821.2 ns |  25.41 ns |  22.53 ns |         - |
| Cross            |  8,857.5 ns |  75.84 ns |  70.94 ns |         - |
| Magnitude        | 37,997.8 ns | 229.25 ns | 214.44 ns |         - |
| Normal           | 72,625.3 ns | 522.02 ns | 488.30 ns |         - |
| NormalizeInPlace | 71,258.2 ns | 330.18 ns | 308.85 ns |         - |
| GetNormalized    | 72,913.1 ns | 673.06 ns | 596.65 ns |         - |
| Distance         | 21,386.0 ns | 427.58 ns | 690.47 ns |         - |
| Lerp             | 11,969.2 ns |  71.65 ns |  67.02 ns |         - |
| LerpOut          | 12,326.3 ns |  73.25 ns |  64.94 ns |         - |
