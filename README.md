# FixedMathSharp

![FixedMathSharp Icon](https://raw.githubusercontent.com/mrdav30/fixedmathsharp/main/icon.png)

[![Build](https://github.com/mrdav30/FixedMathSharp/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/mrdav30/FixedMathSharp/actions/workflows/build-and-test.yml)
[![Coverage](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fmrdav30.github.io%2FFixedMathSharp%2FSummary.json&query=%24.summary.linecoverage&suffix=%25&label=coverage&color=brightgreen)](https://mrdav30.github.io/FixedMathSharp/)
[![NuGet](https://img.shields.io/nuget/v/FixedMathSharp.svg)](https://www.nuget.org/packages/FixedMathSharp)
[![NuGet Downloads](https://img.shields.io/nuget/dt/FixedMathSharp.svg)](https://www.nuget.org/packages/FixedMathSharp)
[![License](https://img.shields.io/github/license/mrdav30/FixedMathSharp.svg)](https://github.com/mrdav30/FixedMathSharp/blob/main/LICENSE)
[![Frameworks](https://img.shields.io/badge/frameworks-netstandard2.1%20%7C%20net8.0-512BD4.svg)](https://github.com/mrdav30/FixedMathSharp)

**Deterministic fixed-point math for .NET simulations, games, tools, and procedural systems.**

FixedMathSharp gives you a practical Q32.32 fixed-point numeric stack: scalar math, vectors, matrices, quaternions, bounds, curves, and deterministic random generation. It is built for code where the same inputs should produce the same results across machines, runs, replays, and networked clients.

---

## Why Fixed-Point?

Floating-point math is fast, hardware-accelerated, and the right choice for rendering, visual effects, and many everyday calculations. It is also allowed to vary in small ways across runtimes, processors, compiler settings, instruction sets, and evaluation order.

Fixed-point math stores numbers as scaled integers. In FixedMathSharp, `Fixed64` uses a Q32.32 layout: 32 bits for the whole-number side and 32 bits for the fractional side. That trade gives you deterministic arithmetic with predictable rounding behavior, at the cost of less dynamic range than `double` and less raw throughput than native floating point.

For the exact raw layout, range, and precision trade-offs, see [`docs/wiki/fixed64-representation.md`](docs/wiki/fixed64-representation.md).

Use FixedMathSharp when you need:

- Lockstep multiplayer, replay systems, rollback, or deterministic simulation.
- Procedural generation that must be reproducible from the same seed.
- Gameplay, physics-adjacent, or tooling logic where drift and platform differences are painful.
- Serializable math values with stable behavior across .NET targets.

Use floating point when you need:

- Maximum numeric range or hardware throughput.
- Rendering, shader, animation, and visual-only calculations.
- Interop with engines or APIs that already own their float-based math pipeline.

---

## Features

- **`Fixed64` scalar arithmetic** with deterministic Q32.32 representation, guarded overflow behavior, parsing, formatting, and common math helpers.
- **2D, 3D, and 4D vectors** via `Vector2d`, `Vector3d`, and `Vector4d`, including dot products, distances, normalization, transforms, fuzzy equality, and component operations.
- **Rotations and matrices** with `FixedQuaternion`, `Fixed3x3`, and `Fixed4x4` for deterministic transforms and orientation math.
- **Coordinate convention helpers** with `Axis3d` and `CoordinateConvention3d` for explicit signed-axis adapter boundaries.
- **Geometry and bounds** with `FixedBoundBox`, `FixedBoundSphere`, `FixedBoundFrustum`, `FixedPlane`, and `FixedRay`.
- **Curves and ranges** with `FixedCurve`, `FixedCurveKey`, and `FixedRange`.
- **Deterministic RNG** with `DeterministicRandom` streams derived from seeds, feature keys, and indices.
- **Serialization-friendly structs** with MemoryPack support in the standard package and a Lean package when you do not want that dependency.
- **Chronicler record-hash extensions** through a companion package for deterministic replay and conformance signals.
- **Testing helpers** through the companion FluentAssertions package.

---

## Installation

For most .NET projects:

```bash
dotnet add package FixedMathSharp
```

Choose the package that fits your runtime:

| Package | Best For | Install |
| --- | --- | --- |
| `FixedMathSharp` | Most .NET applications. Includes MemoryPack support. | `dotnet add package FixedMathSharp` |
| `FixedMathSharp.Lean` | Projects that want the same math API without a MemoryPack dependency, including custom serializers and Burst AOT-sensitive workflows. | `dotnet add package FixedMathSharp.Lean` |
| `FixedMathSharp.Chronicler` | Replay or conformance tooling that hashes FixedMathSharp values with Chronicler's deterministic record-hash writer. | `dotnet add package FixedMathSharp.Chronicler` |
| `FixedMathSharp.Chronicler.Lean` | Chronicler hash extensions paired with the Lean package graph. | `dotnet add package FixedMathSharp.Chronicler.Lean` |
| `FixedMathSharp.FluentAssertions` | Tests that use FluentAssertions with `Fixed64`, vectors, quaternions, and matrices. | `dotnet add package FixedMathSharp.FluentAssertions` |
| `FixedMathSharp.FluentAssertions.Lean` | FluentAssertions helpers paired with the Lean package. | `dotnet add package FixedMathSharp.FluentAssertions.Lean` |

### Serialization Guidance

- **MemoryPack:** best for hot paths, snapshots, networking payloads, deterministic state transfer, and binary saves.
- **JSON:** best for interoperability, debugging, editor tooling, human-readable configs, and low-frequency persistence.

### Unity

FixedMathSharp is maintained separately for Unity-specific packaging and workflows:
[FixedMathSharp-Unity](https://github.com/mrdav30/FixedMathSharp-Unity).

If you are evaluating this .NET package for Unity-adjacent tooling or Burst AOT-sensitive code, prefer `FixedMathSharp.Lean`.

---

## Quick Start

```csharp
using FixedMathSharp;

Fixed64 speed = Fixed64.FromDouble(3.5);
Fixed64 deltaTime = Fixed64.FromFraction(1, 60);
Fixed64 step = speed * deltaTime;

Vector3d position = new Vector3d(0, 0, 0);
Vector3d velocity = new Vector3d(step, Fixed64.Zero, Fixed64.One);

FixedQuaternion turn = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
Vector3d rotated = turn.Rotate(velocity);

FixedBoundSphere sensor = new FixedBoundSphere(position, new Fixed64(5));
bool inRange = sensor.Contains(rotated);

Console.WriteLine($"{rotated} in range: {inRange}");
```

### Deterministic Random Streams

```csharp
ulong worldSeed = 123456789UL;

var oreRng = DeterministicRandom.FromWorldFeature(worldSeed, featureKey: 0xC0FFEEUL);
var riverRng = DeterministicRandom.FromWorldFeature(worldSeed, featureKey: 0xBADC0DEUL, index: 3);

Fixed64 oreRichness = oreRng.NextFixed64(Fixed64.Zero, new Fixed64(10));
Fixed64 riverBend = riverRng.NextFixed64(-Fixed64.One, Fixed64.One);
int lootCount = oreRng.Next(1, 5); // [1, 5)
```

### Chronicler Record Hashes

```csharp
using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Chronicler;

var writer = new ChronicleHashWriter();
writer.WriteVector3d(new Vector3d(1, 2, 3));
writer.WriteQuaternion(FixedQuaternion.Identity);

ChronicleHash hash = writer.ToHash();
```

### Bounds and Geometry

```csharp
FixedBoundBox room = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, new Vector3d(10, 4, 10));
FixedRay ray = new FixedRay(new Vector3d(-20, 0, 0), Vector3d.Right);

Fixed64? hitDistance = ray.Intersects(room);

if (hitDistance.HasValue)
{
    Vector3d hitPoint = ray.Position + (ray.Direction * hitDistance.Value);
    Console.WriteLine(hitPoint);
}
```

### Matrices and Transforms

```csharp
FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);
Fixed4x4 transform = Fixed4x4.CreateTranslation(new Vector3d(10, 0, 0)) *
                     Fixed4x4.CreateRotation(rotation);

Vector3d transformed = Fixed4x4.TransformPoint(transform, new Vector3d(1, 0, 0));
```

---

## Library Map

- `Fixed64`: deterministic scalar type backed by a signed 64-bit raw value.
- `FixedMath`: constants, rounding, interpolation, trigonometry, powers, square roots, and utility math.
- `Vector2d`, `Vector3d`, `Vector4d`: deterministic vector math and transform helpers.
- `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`: rotations, orientations, matrices, and transform operations.
- `Axis3d`, `CoordinateConvention3d`: stateless helpers for documenting and converting signed-axis 3D direction conventions.
- `FixedBoundBox`, `FixedBoundSphere`, `FixedBoundFrustum`: containment, intersection, clamping, and projection queries.
- `FixedPlane`, `FixedRay`: geometric primitives for plane classification and ray intersections.
- `FixedCurve`, `FixedCurveKey`, `FixedRange`: interpolation and range helpers.
- `DeterministicRandom`: repeatable random streams for simulations and procedural generation.
- `FixedMathSharp.Chronicler`: deterministic `ChronicleHashWriter` extensions for fixed-point, vector, matrix, transform, and bounds values.
- `FixedMathSharp.FluentAssertions`: expressive test assertions for FixedMathSharp types.

### API Shape Notes

- `FixedMath` is the canonical scalar algorithm surface; `Fixed64` owns Q32.32 representation, conversions, parsing, operators, and raw-value helpers.
- Core 3D direction semantics are `+X` right, `+Y` up, and `+Z` forward. See [`docs/wiki/coordinate-conventions.md`](docs/wiki/coordinate-conventions.md) before adding adapter-facing conversions.
- Numeric types expose clear return-by-value statics/operators plus explicit `*InPlace` methods when mutation is useful.
- Extension methods are curated receiver-shaped conveniences that forward to canonical APIs; factories and convention-heavy operations stay on the owning type.
- `Fast*` helpers are expert APIs for proven hot paths. They skip some guarded operator behavior, so prefer normal operators unless benchmarks and invariants justify the fast path.
- Countable hot-path data may use array or `ReadOnlySpan<T>` overloads, while `IEnumerable<T>` remains useful for broader interoperability.

### Coordinate Conventions

FixedMathSharp's core 3D convention is `+X` right, `+Y` up, and `+Z` forward.
Use `CoordinateConvention3d.NegativeZForward` when a `+Y`-up adapter boundary
needs to translate `-Z`-forward direction semantics, such as MonoGame's
`Vector3.Forward`, into the core convention. Use
`CoordinateConvention3d.XForwardZUp` for `+X` forward, `+Y` right, `+Z` up
adapter boundaries, such as Unreal-style coordinate spaces. XNA is legacy
context for MonoGame's maintained XNA-compatible API. Engine and toolchain
differences beyond direction basis mapping, such as matrix multiplication
convention, storage layout, transform APIs, origins, units, or clip-space depth,
belong in adapter code.

---

## Build From Source

Install the .NET 10 SDK before building from source. `global.json` pins the
repo tooling to SDK 10 so `.slnx` workflows behave consistently; the packages
still target .NET Standard 2.1 and .NET 8 for consumers.

```bash
git clone https://github.com/mrdav30/FixedMathSharp.git
cd FixedMathSharp
dotnet restore
dotnet build --configuration Debug --no-restore
dotnet test --configuration Debug --no-build
```

Release build configurations:

- `Release` builds the standard package.
- `ReleaseLean` builds the Lean package with MemoryPack excluded.

The helper script `.assets/scripts/set-version-and-build.ps1` builds both release configurations and writes release archives to `artifacts/releases/`.

### Benchmarks

FixedMathSharp includes a BenchmarkDotNet project for measuring hot-path scalar, vector, quaternion, matrix, and bounds behavior:
[`tests/FixedMathSharp.Benchmarks`](tests/FixedMathSharp.Benchmarks).

See [`tests/FixedMathSharp.Benchmarks/README.md`](tests/FixedMathSharp.Benchmarks/README.md) for benchmark setup, aliases, and baseline guidance.

---

## Compatibility

Upgrading from v4.x? See the [v5 migration guide](docs/MIGRATION.md).

- .NET Standard 2.1
- .NET 8
- Windows, Linux, and macOS

---

## Quality Notes

The library is covered by xUnit tests for arithmetic, vectors, matrices, quaternions, bounds, curves, serialization, deterministic random behavior, and FluentAssertions helpers.

Cyclomatic complexity exceptions are tracked in [`docs/complexity-exceptions.md`](docs/complexity-exceptions.md). The register explains why specific hot-path or fixed-shape methods exceed the review threshold and what should trigger revisiting them.

---

## Contributing

Contributions, bug reports, feature requests, and real-world determinism stories are welcome. Please read the [CONTRIBUTING](https://github.com/mrdav30/FixedMathSharp/blob/main/CONTRIBUTING.md) guide before opening a pull request.

For questions and discussion, join the official Discord community:
[Join the Discord Server](https://discord.gg/mhwK2QFNBA)

---

## License

FixedMathSharp is licensed under the MIT License.

See these repository files for details:

- `LICENSE` - standard MIT license.
- `NOTICE` - additional terms regarding project branding and redistribution.
- `COPYRIGHT` - authorship information.
