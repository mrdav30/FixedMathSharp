# FixedMathSharp

![FixedMathSharp icon](icon.png)

[![Build](https://github.com/mrdav30/FixedMathSharp/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/mrdav30/FixedMathSharp/actions/workflows/build-and-test.yml)
[![Core Branch Coverage](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fmrdav30.github.io%2FFixedMathSharp%2Fcoverage%2FSummary.json&query=%24.summary.branchcoverage&suffix=%25&label=core%20branch%20coverage&color=brightgreen)](https://mrdav30.github.io/FixedMathSharp/coverage/)
[![NuGet](https://img.shields.io/nuget/v/FixedMathSharp.svg)](https://www.nuget.org/packages/FixedMathSharp)
[![License](https://img.shields.io/github/license/mrdav30/FixedMathSharp.svg)](LICENSE)
[![API](https://img.shields.io/badge/docs-API-f4511e)](https://mrdav30.github.io/FixedMathSharp/)
[![Discord](https://img.shields.io/badge/discord-join%20community-5865F2?logo=discord&logoColor=white)](https://discord.gg/mhwK2QFNBA)

**Deterministic fixed-point math for .NET games, simulations, procedural
generation, tooling, and replays.**

FixedMathSharp gives you a practical Q32.32 numeric stack—from scalar math and
random streams to vectors, transforms, and geometry—without tying your runtime
to a game engine. The same inputs follow the same integer-backed math path,
which makes the library a strong fit for lockstep simulation, reproducible world
generation, and replayable systems.

## Why use it?

- **Deterministic by design.** `Fixed64` uses an explicit Q32.32 representation
  with documented rounding and overflow behavior.
- **A complete math toolbox.** Vectors, matrices, quaternions, transforms,
  curves, bounds, rays, segments, triangles, and deterministic random streams
  share one numeric model.
- **Built for difficult inputs.** Full-domain intermediate arithmetic prevents
  premature saturation in fused math, transforms, and geometry queries.
- **Engine-agnostic.** The core stays portable; engine conventions and interop
  live at adapter boundaries.
- **Serialization choices.** Use the standard MemoryPack-enabled package or the
  Lean variant with no direct MemoryPack dependency.

## Install

```bash
dotnet add package FixedMathSharp
```

Targeting Unity? Use the dedicated
[FixedMathSharp-Unity packages](https://github.com/mrdav30/FixedMathSharp-Unity),
including a Lean package for Burst-oriented projects.

## Quick start

```csharp
using FixedMathSharp;
using FixedMathSharp.Geometry;

Fixed64 tick = Fixed64.One / 60;
Vector3d velocity = new(6, 0, 2);
Vector3d nextPosition = velocity * tick;

FixedQuaternion turn = FixedQuaternion.FromAxisAngle(
    Vector3d.Up,
    Fixed64.PiOver4);

Vector3d heading = turn.Rotate(Vector3d.Forward);
FixedBoundSphere playArea = new(Vector3d.Zero, new Fixed64(100));

bool inside = playArea.Contains(nextPosition);
```

All values in the simulation-facing calculation remain fixed point. Convert to
`float` or `double` only at rendering, editor, or external API boundaries.

## Pick a package

| Package                                                                                                       | Use it for                                                     |
| ------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| [`FixedMathSharp`](https://www.nuget.org/packages/FixedMathSharp)                                             | Core math with MemoryPack support                              |
| [`FixedMathSharp.Lean`](https://www.nuget.org/packages/FixedMathSharp.Lean)                                   | The same math surface without a direct MemoryPack dependency   |
| [`FixedMathSharp.Chronicler`](https://www.nuget.org/packages/FixedMathSharp.Chronicler)                       | Deterministic record hashes for replay and conformance tooling |
| [`FixedMathSharp.Chronicler.Lean`](https://www.nuget.org/packages/FixedMathSharp.Chronicler.Lean)             | Chronicler extensions on the Lean dependency graph             |
| [`FixedMathSharp.FluentAssertions`](https://www.nuget.org/packages/FixedMathSharp.FluentAssertions)           | Fluent assertions for fixed-point tests                        |
| [`FixedMathSharp.FluentAssertions.Lean`](https://www.nuget.org/packages/FixedMathSharp.FluentAssertions.Lean) | Fluent assertions paired with the Lean package                 |

Published packages target .NET Standard 2.1 and .NET 8.

## Explore the library

- [Getting started](docs/wiki/getting-started.md) — package choice, setup,
  examples, serialization, and source builds
- [Wiki](docs/wiki/Home.md) — numeric behavior, transforms, conventions,
  geometry, and formatting
- [API reference](https://mrdav30.github.io/FixedMathSharp/) — public types and
  members generated from the source XML documentation
- [Migration guide](docs/MIGRATION.md) — intentional breaking changes and
  upgrade guidance
- [Benchmarks](tests/FixedMathSharp.Benchmarks/README.md) — reproducible
  BenchmarkDotNet workflows and evidence rules

## Community and contributions

Bug reports, feature requests, performance evidence, and real-world determinism
stories are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a
pull request, or join the [Discord community](https://discord.gg/mhwK2QFNBA).

FixedMathSharp is available under the [MIT License](LICENSE). See
[NOTICE](NOTICE) for the project branding and redistribution terms.
