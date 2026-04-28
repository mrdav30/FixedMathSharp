# FixedMathSharp

![FixedMathSharp Icon](https://raw.githubusercontent.com/mrdav30/fixedmathsharp/main/icon.png)

[![.NET CI](https://github.com/mrdav30/FixedMathSharp/actions/workflows/dotnet.yml/badge.svg)](https://github.com/mrdav30/FixedMathSharp/actions/workflows/dotnet.yml)
[![Coverage](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fmrdav30.github.io%2FFixedMathSharp%2FSummary.json&query=%24.summary.linecoverage&suffix=%25&label=coverage&color=brightgreen)](https://mrdav30.github.io/FixedMathSharp/)
[![NuGet](https://img.shields.io/nuget/v/FixedMathSharp.svg)](https://www.nuget.org/packages/FixedMathSharp)
[![NuGet Downloads](https://img.shields.io/nuget/dt/FixedMathSharp.svg)](https://www.nuget.org/packages/FixedMathSharp)
[![License](https://img.shields.io/github/license/mrdav30/FixedMathSharp.svg)](https://github.com/mrdav30/FixedMathSharp/blob/main/LICENSE)
[![Frameworks](https://img.shields.io/badge/frameworks-netstandard2.1%20%7C%20net8.0-512BD4.svg)](https://github.com/mrdav30/FixedMathSharp)

**A deterministic fixed-point math library for .NET.**  
Built for simulations, games, and physics-heavy code that needs reliable results without floating-point drift.

---

## 🛠️ Key Features

- **Deterministic fixed-point arithmetic:** Consistent results across platforms with `Fixed64`.
- **Core math types included:** `Vector2d`, `Vector3d`, `FixedQuaternion`, `Fixed3x3`, and `Fixed4x4`.
- **Spatial helpers:** `BoundingBox`, `BoundingSphere`, and `BoundingArea` for lightweight bounds checks.
- **Shared math utilities:** Common math and trigonometry helpers via `FixedMath` and `FixedTrigonometry`.
- **Deterministic RNG:** `DeterministicRandom` for repeatable procedural generation and simulations.
- **Flexible packaging:** Use the default package with `MemoryPack`, or the `NoMemoryPack` package when you want the same API without that dependency.
- **Broad .NET compatibility:** Targets modern .NET while remaining friendly to engine and tooling workflows.

---

## 🚀 Installation

For most .NET projects, start with the standard package:

```bash
dotnet add package FixedMathSharp
```

### Non-Unity Projects

Choose the package that fits your runtime:

| Package | Best for | Install |
| --- | --- | --- |
| `FixedMathSharp` | Most .NET applications. Includes built-in `MemoryPack` support. | `dotnet add package FixedMathSharp` |
| `FixedMathSharp.NoMemoryPack` | Projects that want the same math API without a `MemoryPack` dependency, including custom serializer setups and Burst AOT-sensitive workflows. | `dotnet add package FixedMathSharp.NoMemoryPack` |

If you're using `FluentAssertions` in your test project, the companion assertions package is available here:
[FixedMathSharp.FluentAssertions](https://www.nuget.org/packages/FixedMathSharp.FluentAssertions)

### Build From Source

Clone the repository and build locally:

```bash
git clone https://github.com/mrdav30/FixedMathSharp.git
dotnet restore
dotnet build --configuration Debug --no-restore
```

You can also reference the project directly or consume the generated package artifacts in your own build process.

### Package Variants

The published NuGet packages map directly to the source-build configurations below.

If you build from source, the repository also provides matching release configurations:

- `Release` builds the standard `FixedMathSharp` package and archives.
- `ReleaseNoMemoryPack` builds the `FixedMathSharp.NoMemoryPack` package and archives.

If you use Unity Burst AOT, prefer the `NoMemoryPack` variant.

### Unity Integration

FixedMathSharp is maintained as a separate Unity package. For Unity-specific implementations, refer to:

🔗 [FixedMathSharp-Unity Repository](https://github.com/mrdav30/FixedMathSharp-Unity).

If you are evaluating this .NET package for Unity-adjacent tooling using Burst AOT, prefer `FixedMathSharp.NoMemoryPack`.

---

## 📖 Usage Examples

### Basic Arithmetic with `Fixed64`

```csharp
Fixed64 a = new Fixed64(1.5);
Fixed64 b = new Fixed64(2.5);
Fixed64 result = a + b;
Console.WriteLine(result); // Output: 4.0
```

### Vector Operations

```csharp
Vector3d v1 = new Vector3d(1, 2, 3);
Vector3d v2 = new Vector3d(4, 5, 6);
Fixed64 dotProduct = Vector3d.Dot(v1, v2);
Console.WriteLine(dotProduct); // Output: 32
```

### Quaternion Rotation

```csharp
FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2); // 90 degrees around Y-axis
Vector3d point = new Vector3d(1, 0, 0);
Vector3d rotatedPoint = rotation.Rotate(point);
Console.WriteLine(rotatedPoint); // Output: (0, 0, -1)
```

### Matrix Transformations

```csharp
Fixed4x4 matrix = Fixed4x4.Identity;
Vector3d position = new Vector3d(1, 2, 3);
matrix.SetTransform(position, Vector3d.One, FixedQuaternion.Identity);
Console.WriteLine(matrix);
```

### Bounding Shapes and Intersection

```csharp
BoundingBox box = new BoundingBox(new Vector3d(0, 0, 0), new Vector3d(5, 5, 5));
BoundingSphere sphere = new BoundingSphere(new Vector3d(3, 3, 3), new Fixed64(1));
bool intersects = box.Intersects(sphere);
Console.WriteLine(intersects); // Output: True
```

### Trigonometry Example

```csharp
Fixed64 angle = FixedMath.PiOver4; // 45 degrees
Fixed64 sinValue = FixedTrigonometry.Sin(angle);
Console.WriteLine(sinValue); // Output: ~0.707
```

### Deterministic Random Generation

Use `DeterministicRandom` when you need reproducible random values across runs, worlds, or features.  
Streams are derived from a seed and remain deterministic regardless of threading or platform.

```csharp
// Simple constructor-based stream:
var rng = new DeterministicRandom(42UL);

// Deterministic integer:
int value = rng.Next(1, 10); // [1,10)

// Deterministic Fixed64 in [0,1):
Fixed64 ratio = rng.NextFixed6401();

// One stream per “feature” that’s stable for the same worldSeed + key:
var rngOre = DeterministicRandom.FromWorldFeature(worldSeed: 123456789UL, featureKey: 0xORE);
var rngRivers = DeterministicRandom.FromWorldFeature(123456789UL, 0xRIV, index: 0);

// Deterministic Fixed64 draws:
Fixed64 h = rngOre.NextFixed64(Fixed64.One);                      // [0, 1)
Fixed64 size = rngOre.NextFixed64(Fixed64.Zero, 5 * Fixed64.One); // [0, 5)
Fixed64 posX = rngRivers.NextFixed64(-Fixed64.One, Fixed64.One);  // [-1, 1)

// Deterministic integers:
int loot = rngOre.Next(1, 5); // [1,5)
```

---

## 📦 Library Structure

- **`Fixed64` Struct:** Core Q32.32 fixed-point scalar type.
- **`Vector2d` and `Vector3d` Structs:** 2D and 3D vector math.
- **`FixedQuaternion` Struct:** Deterministic quaternion rotations.
- **`Fixed4x4` and `Fixed3x3`:** Matrix math for transforms and orientation.
- **`IBound` Interface and bounds types:** `BoundingBox`, `BoundingArea`, and `BoundingSphere` for intersection, containment, and projection queries.
- **`FixedMath` and `FixedTrigonometry`:** Shared numeric and trigonometric helpers.
- **`DeterministicRandom` Struct:** Seedable, allocation-free RNG for repeatable procedural generation.

### Fixed64 Struct

`Fixed64` is the center of the library: a deterministic fixed-point number type backed by integer arithmetic. It is the type used throughout the vector, matrix, quaternion, bounds, and helper APIs.

---

## ⚡ Performance Considerations

FixedMathSharp is optimized for high-performance deterministic calculations:

- **Inline methods and bit-shifting optimizations** keep hot paths lightweight.
- **Deterministic arithmetic** avoids floating-point drift in lockstep or replay-driven systems.
- **Fuzzy equality helpers** are available where precision tolerances are useful.

---

## 🧪 Testing and Validation

The library is covered by xUnit tests for core arithmetic, vectors, bounds, serialization, and deterministic random behavior. Fuzzy comparisons are used where a tolerance-based check is more appropriate than exact equality.

To run the tests:

```bash
dotnet test --configuration Debug
```

---

## 🛠️ Compatibility

- **.NET Standard** 2.1
- **.NET** 8
- **Unity 2020+** (via [FixedMathSharp-Unity](https://github.com/mrdav30/FixedMathSharp-Unity))
- **Cross-Platform Support** (Windows, Linux, macOS)

---

## 🤝 Contributing

We welcome contributions! Please see our [CONTRIBUTING](https://github.com/mrdav30/FixedMathSharp/blob/main/CONTRIBUTING.md) guide for details on how to propose changes, report issues, and interact with the community.

---

## 👥 Contributors

- **mrdav30** - Lead Developer
- Contributions are welcome! Feel free to submit pull requests or report issues.

---

## 💬 Community & Support

For questions, discussions, or general support, join the official Discord community:

👉 **[Join the Discord Server](https://discord.gg/mhwK2QFNBA)**

For bug reports or feature requests, please open an issue in this repository.

We welcome feedback, contributors, and community discussion across all projects.

---

## 📄 License

This project is licensed under the MIT License.

See the following files for details:

- LICENSE – standard MIT license
- NOTICE – additional terms regarding project branding and redistribution
- COPYRIGHT – authorship information
