# FixedMathSharp

[FixedMathSharp](https://github.com/mrdav30/FixedMathSharp) is a deterministic
Q32.32 fixed-point math library for .NET games, simulations, procedural
generation, tooling, and replays.

## Install

```bash
dotnet add package FixedMathSharp
```

Use [`FixedMathSharp.Lean`](https://www.nuget.org/packages/FixedMathSharp.Lean)
when you want the same math surface without a direct MemoryPack dependency.
Both packages target .NET Standard 2.1 and .NET 8.

For Unity projects, use the dedicated
[FixedMathSharp-Unity packages](https://github.com/mrdav30/FixedMathSharp-Unity).

## Example

```csharp
using FixedMathSharp;
using FixedMathSharp.Geometry;

Fixed64 tick = Fixed64.One / 60;
Vector3d velocity = new(6, 0, 2);
Vector3d position = velocity * tick;

FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
    Vector3d.Up,
    Fixed64.PiOver4);

Vector3d heading = rotation.Rotate(Vector3d.Forward);
FixedBoundSphere world = new(Vector3d.Zero, new Fixed64(100));
bool inside = world.Contains(position);
```

The package includes `Fixed64`, `FixedMath`, 2D/3D/4D vectors,
`FixedQuaternion`, `Fixed3x3`, `Fixed4x4`, `FixedTransform`, deterministic
random streams, curves, ranges, bounds, and computational geometry.

## Learn more

- [Getting started](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/wiki/getting-started.md)
- [Behavioral guides](https://github.com/mrdav30/FixedMathSharp/wiki)
- [API reference](https://mrdav30.github.io/FixedMathSharp/)
- [Migration guide](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/MIGRATION.md)
- [Source and issues](https://github.com/mrdav30/FixedMathSharp)

FixedMathSharp is licensed under the
[MIT License](https://github.com/mrdav30/FixedMathSharp/blob/main/LICENSE).
