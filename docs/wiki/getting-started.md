# Getting Started

This guide gets a .NET project using FixedMathSharp, then points you to the
deeper behavior contracts when you need them.

## 1. Choose a package

Most applications should start here:

```bash
dotnet add package FixedMathSharp
```

| If you need...                                               | Choose...                                                               |
| ------------------------------------------------------------ | ----------------------------------------------------------------------- |
| Core math with MemoryPack support                            | `FixedMathSharp`                                                        |
| Core math without a direct MemoryPack dependency             | `FixedMathSharp.Lean`                                                   |
| Deterministic record hashes for replay or conformance checks | `FixedMathSharp.Chronicler` or `.Lean`                                  |
| FluentAssertions helpers for fixed-point tests               | `FixedMathSharp.FluentAssertions` or `.Lean`                            |
| Unity packages and Unity-specific interop                    | [FixedMathSharp-Unity](https://github.com/mrdav30/FixedMathSharp-Unity) |

Pair standard companion packages with `FixedMathSharp`, and Lean companion
packages with `FixedMathSharp.Lean`.

## 2. Run a fixed-point calculation

The main numeric types live in `FixedMathSharp`. Geometry types live in
`FixedMathSharp.Geometry`, and deterministic random streams live in
`FixedMathSharp.Random`.

```csharp
using FixedMathSharp;
using FixedMathSharp.Geometry;
using FixedMathSharp.Random;

Fixed64 tick = Fixed64.One / 60;
Vector3d position = Vector3d.Zero;
Vector3d velocity = new(6, 0, 2);

position += velocity * tick;

var random = DeterministicRandom.FromWorldFeature(
    worldSeed: 123456789UL,
    featureKey: 0xC0FFEEUL);

int variant = random.Next(0, 4); // [0, 4)

FixedBoundSphere playableWorld = new(
    Vector3d.Zero,
    new Fixed64(100));

bool positionIsValid = playableWorld.Contains(position);
```

Creating the same random stream with the same seed, feature key, and index
produces the same sequence. Keep stream ownership and call order explicit; a
deterministic generator cannot make a changing call sequence deterministic.

## 3. Create values intentionally

```csharp
Fixed64 whole = new Fixed64(12);
Fixed64 decimalValue = Fixed64.FromDecimal(12.5m);
Fixed64 ratio = Fixed64.One / 60;
Fixed64 boundaryValue = Fixed64.FromDouble(externalDouble);
```

Use `FromDouble` and floating-point conversions at engine, UI, or file-format
boundaries. Prefer fixed-point constants and operators once data enters the
deterministic runtime.

`Fixed64.FromRaw(long)` is different: it interprets the input as an already
scaled Q32.32 payload. Use it only for raw-value protocols and exact fixtures.
Read [Fixed64 Representation](fixed64-representation.md) before working with raw
values, domain limits, or overflow behavior.

## 4. Know the core conventions

- 3D semantic forward is `+Z`; right is `+X`; up is `+Y`.
- 4x4 transforms use row vectors, with translation in `M41`, `M42`, and `M43`.
- Default geometry containment and intersection include boundaries.
- `Try*` methods report the failure described by that member. Invalid arguments
  may still throw, so check the
  [API reference](https://mrdav30.github.io/FixedMathSharp/) for the exact
  contract.
- Ordinary operators round or saturate at each operator boundary. Fused and
  full-domain APIs preserve wider intermediates until the final result.

Read [Coordinate Conventions](coordinate-conventions.md),
[Bounds and Geometry](bounds-and-geometry.md), and
[Full-Domain Arithmetic](full-domain-wide-arithmetic.md) when those rules affect
your design.

## 5. Choose serialization deliberately

The standard package includes MemoryPack integration. The Lean package excludes
the MemoryPack-specific partial files and replaces the direct MemoryPack
dependency with the `Chronicler.MemoryPackShim` compatibility package.

- Use MemoryPack for compact binary state, snapshots, and hot serialization
  paths.
- Use JSON for tooling, diagnostics, configuration, and interoperability.
- Do not use `ToString()` as a persistence format. See
  [Diagnostics Formatting](diagnostics-formatting.md).

## Build from source

Install the .NET 10 SDK selected by `global.json` and the .NET 8 runtime used by
the test and benchmark targets.

```bash
git clone https://github.com/mrdav30/FixedMathSharp.git
cd FixedMathSharp
dotnet restore FixedMathSharp.slnx --property:Configuration=Debug
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug --no-build
```

The contributor workflow and release-configuration checks live in
[CONTRIBUTING.md](https://github.com/mrdav30/FixedMathSharp/blob/main/CONTRIBUTING.md).

## Next steps

- [Technical Overview](Overview.md) — how the library is divided and which type
  owns each responsibility
- [API Reference](https://mrdav30.github.io/FixedMathSharp/) — exact signatures,
  exceptions, and member behavior
- [Migration Guide](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/MIGRATION.md)
  — breaking changes between major versions
- [Benchmark Guide](https://github.com/mrdav30/FixedMathSharp/blob/main/tests/FixedMathSharp.Benchmarks/README.md)
  — repeatable performance measurements
