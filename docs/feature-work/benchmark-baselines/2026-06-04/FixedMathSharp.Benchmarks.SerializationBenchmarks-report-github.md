```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method                             | Mean           | Error        | StdDev       | Gen0     | Gen1   | Allocated |
|----------------------------------- |---------------:|-------------:|-------------:|---------:|-------:|----------:|
| MemoryPackSerializeMathTypes       |    18,787.5 ns |    374.03 ns |  1,011.21 ns |  19.5618 |      - |  122880 B |
| MemoryPackSerializeBoundsAndCurves |    39,552.7 ns |    790.37 ns |  1,384.28 ns |  20.2026 |      - |  126976 B |
| MemoryPackDeserializeScalar        |       754.7 ns |     14.87 ns |     15.91 ns |        - |      - |         - |
| MemoryPackDeserializeVector3d      |     1,207.8 ns |     24.19 ns |     26.88 ns |        - |      - |         - |
| MemoryPackRoundTripCurve           |   110,426.3 ns |  1,941.17 ns |  2,310.82 ns |  59.0820 | 0.1221 |  370688 B |
| JsonSerializeMathTypes             | 1,498,327.4 ns | 21,671.17 ns | 19,210.93 ns | 193.3594 |      - | 1223096 B |
| JsonSerializeBoundsAndCurves       | 1,907,895.9 ns | 27,828.09 ns | 26,030.41 ns | 216.7969 |      - | 1362672 B |
| JsonDeserializeScalar              |    50,521.5 ns |    825.10 ns |    771.80 ns |   6.4697 |      - |   40960 B |
| JsonDeserializeVector3d            |   253,339.2 ns |  4,287.85 ns |  4,010.85 ns |  46.3867 |      - |  292864 B |
| JsonRoundTripCurve                 | 4,115,785.4 ns | 72,146.87 ns | 63,956.32 ns | 484.3750 |      - | 3041448 B |
