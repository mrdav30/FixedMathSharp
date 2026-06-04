```

BenchmarkDotNet v0.15.8, Linux Ubuntu 24.04.1 LTS (Noble Numbat)
Intel Core i7-9700K CPU 3.60GHz (Coffee Lake), 1 CPU, 8 logical and 8 physical cores
.NET SDK 10.0.203
  [Host]     : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3
  DefaultJob : .NET 8.0.26 (8.0.26, 8.0.2626.16921), X64 RyuJIT x86-64-v3


```
| Method                      | Mean         | Error        | StdDev       | Gen0    | Allocated |
|---------------------------- |-------------:|-------------:|-------------:|--------:|----------:|
| BoxContainsPoint            |     433.9 ns |      4.74 ns |      4.44 ns |       - |         - |
| BoxIntersectsBox            |   1,078.4 ns |     18.99 ns |     16.83 ns |       - |         - |
| BoxIntersectsArea           |   1,086.4 ns |     21.58 ns |     28.80 ns |       - |         - |
| BoxIntersectsSphere         |   7,844.4 ns |    111.23 ns |    104.04 ns |       - |         - |
| BoxClampPoint               |   1,491.5 ns |     29.76 ns |     65.33 ns |       - |         - |
| SphereContainsPoint         |   5,069.3 ns |     64.71 ns |     57.36 ns |       - |         - |
| SphereIntersectsSphere      |   6,267.1 ns |    102.65 ns |     96.02 ns |       - |         - |
| SphereIntersectsBox         |  13,748.3 ns |    226.67 ns |    200.94 ns |       - |         - |
| SphereClampPoint            |  84,802.8 ns |    922.69 ns |    863.09 ns |       - |         - |
| SphereCreateFromBoundingBox |  86,630.3 ns |    987.45 ns |    923.66 ns |       - |         - |
| SphereTransform             | 110,208.5 ns |  1,563.10 ns |  1,385.65 ns |       - |         - |
| AreaContainsPoint           |     622.9 ns |      6.70 ns |      5.94 ns |       - |         - |
| AreaIntersectsArea          |   2,679.3 ns |     47.78 ns |     44.69 ns |       - |         - |
| AreaIntersectsSphere        |   5,957.2 ns |     55.17 ns |     46.07 ns |       - |         - |
| AreaClampPoint              |   2,017.6 ns |     40.09 ns |     47.72 ns |       - |         - |
| FrustumCreateFromMatrix     | 933,380.6 ns | 17,742.16 ns | 19,720.36 ns | 26.3672 |  165888 B |
| FrustumContainsPoint        |  19,828.2 ns |    300.86 ns |    281.42 ns |       - |         - |
| FrustumIntersectsBox        |  35,012.0 ns |    699.96 ns |    748.95 ns |       - |         - |
| FrustumIntersectsSphere     |  30,039.8 ns |    596.56 ns |    558.03 ns |       - |         - |
| FrustumIntersectsRay        | 123,809.7 ns |  1,971.63 ns |  2,191.46 ns |       - |         - |
| FrustumGetCornersIntoArray  |   2,634.4 ns |     35.87 ns |     33.56 ns |       - |         - |
| FrustumGetPlanesIntoArray   |   2,376.3 ns |     38.68 ns |     34.29 ns |       - |         - |
| RayIntersectsBox            |  37,827.9 ns |    263.18 ns |    205.47 ns |       - |         - |
| RayIntersectsSphere         |  22,233.8 ns |    378.95 ns |    335.93 ns |       - |         - |
| RayIntersectsPlane          |  24,347.2 ns |    469.20 ns |    460.82 ns |       - |         - |
| PlaneDotCoordinate          |   6,200.6 ns |    122.35 ns |    130.91 ns |       - |         - |
| PlaneIntersectsBox          |  14,310.7 ns |    279.80 ns |    363.82 ns |       - |         - |
| PlaneIntersectsSphere       |   6,604.2 ns |    129.67 ns |    337.03 ns |       - |         - |
