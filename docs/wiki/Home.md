# FixedMathSharp Wiki

Welcome. This wiki explains the choices and behavioral contracts behind
FixedMathSharp. Use the [API reference](https://mrdav30.github.io/FixedMathSharp/)
when you need an exact signature; use these pages when you need to understand
why an API behaves the way it does.

## Start here

New to the library? Follow [Getting Started](getting-started.md). The shortest
path is:

```bash
dotnet add package FixedMathSharp
```

```csharp
using FixedMathSharp;

Fixed64 tick = Fixed64.One / 60;
Vector3d movement = new Vector3d(6, 0, 2) * tick;
```

For Unity, install from the dedicated
[FixedMathSharp-Unity repository](https://github.com/mrdav30/FixedMathSharp-Unity).

## Find the right guide

| I want to... | Read... |
| --- | --- |
| Install a package and run a first example | [Getting Started](getting-started.md) |
| Understand Q32.32 range, precision, and conversions | [Fixed64 Representation](fixed64-representation.md) |
| Understand fused operations and extreme intermediates | [Full-Domain Arithmetic](full-domain-wide-arithmetic.md) |
| Work with vectors, matrices, quaternions, or engine adapters | [Coordinate Conventions](coordinate-conventions.md) |
| Choose bounds or geometry APIs | [Bounds and Geometry](bounds-and-geometry.md) |
| Format values for logs or persist them safely | [Diagnostics Formatting](diagnostics-formatting.md) |
| See how the pieces fit together | [Technical Overview](Overview.md) |

## Project links

- [API reference](https://mrdav30.github.io/FixedMathSharp/)
- [GitHub repository](https://github.com/mrdav30/FixedMathSharp)
- [NuGet package](https://www.nuget.org/packages/FixedMathSharp)
- [Core test-suite coverage](https://mrdav30.github.io/FixedMathSharp/coverage/)
- [Migration guide](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/MIGRATION.md)
- [Discord community](https://discord.gg/mhwK2QFNBA)
