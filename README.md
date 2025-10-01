FixedMathSharp
==============

![FixedMathSharp Icon](https://raw.githubusercontent.com/mrdav30/fixedmathsharp/main/icon.png)

[![.NET CI](https://github.com/mrdav30/FixedMathSharp/actions/workflows/dotnet.yml/badge.svg)](https://github.com/mrdav30/FixedMathSharp/actions/workflows/dotnet.yml)

**A high-precision, deterministic fixed-point math library for .NET.**  
Ideal for simulations, games, and physics engines requiring reliable arithmetic without floating-point inaccuracies.

---

## 🛠️ Key Features

- **Deterministic Calculations:** Ensures consistent results across different platforms. 
- **High Precision Arithmetic:** Uses fixed-point math to eliminate floating-point inaccuracies.
- **Comprehensive Vector Support:** Includes 2D and 3D vector operations (`Vector2d`, `Vector3d`).
- **Quaternion Rotations:** Leverage `FixedQuaternion` for smooth rotations without gimbal lock.
- **Matrix Operations:** Supports transformations with `Fixed4x4` and `Fixed3x3` matrices.
- **Bounding Shapes:** Includes `IBound` structs `BoundingBox`, `BoundingSphere`, and `BoundingArea` for lightweight spatial calculations.
- **Advanced Math Functions:** Includes trigonometry and common math utilities.
- **Framework Agnostic:** Works with **.NET, Unity, and other game engines**.
- **Full Serialization Support:** Out-of-the-box round-trip serialization via BinaryFormatter (for .NET Framework 4.8+), System.Text.Json (for .NET 8+), and MessagePack across all serializable structs.

---

## 🚀 Installation


Clone the repository and add it to your project:

### Non-Unity Projects

1. **Install via NuGet**:
   - Add FixedMathSharp to your project using the following command:
   
     ```bash
     dotnet add package FixedMathSharp
     ```

2. **Or Download/Clone**:
   - Clone the repository or download the source code.
   
     ```bash
     git clone https://github.com/mrdav30/FixedMathSharp.git
     ```

3. **Add to Project**:

   - Include the FixedMathSharp project or its DLLs in your build process.

### Unity Integration

FixedMathSharp is now maintained as a separate Unity package.For Unity-specific implementations, refer to:

🔗 [FixedMathSharp-Unity Repository](https://github.com/mrdav30/FixedMathSharp-Unity).

---

## 📖 Usage Examples

### Basic Arithmetic with `Fixed64`:
```csharp
Fixed64 a = new Fixed64(1.5);
Fixed64 b = new Fixed64(2.5);
Fixed64 result = a + b;
Console.WriteLine(result); // Output: 4.0
```

### Vector Operations:
```csharp
Vector3d v1 = new Vector3d(1, 2, 3);
Vector3d v2 = new Vector3d(4, 5, 6);
Fixed64 dotProduct = Vector3d.Dot(v1, v2);
Console.WriteLine(dotProduct); // Output: 32
```

### Quaternion Rotation:
```csharp
FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2); // 90 degrees around Y-axis
Vector3d point = new Vector3d(1, 0, 0);
Vector3d rotatedPoint = rotation.Rotate(point);
Console.WriteLine(rotatedPoint); // Output: (0, 0, -1)
```

### Matrix Transformations:
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

### Trigonometry Example:
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

- **`Fixed64` Struct:** Represents fixed-point numbers for precise arithmetic.
- **`Vector2d` and `Vector3d` Structs:** Handle 2D and 3D vector operations.
- **`FixedQuaternion` Struct:** Provides rotation handling without gimbal lock, enabling smooth rotations and quaternion-based transformations.
- **`IBound` Interface:** Standard interface for bounding shapes `BoundingBox`, `BoundingArea`, and `BoundingSphere`, each offering intersection, containment, and projection logic.
- **`FixedMath` Static Class:** Provides common math and trigonometric functions using fixed-point math.
- **`Fixed4x4` and `Fixed3x3`:** Support matrix operations for transformations.
- **`DeterministicRandom` Struct:** Seedable, allocation-free RNG for repeatable procedural generation.  

### Fixed64 Struct

**Fixed64** is the core data type representing fixed-point numbers. It provides various mathematical operations, including addition, subtraction, multiplication, division, and more. 
The struct guarantees deterministic behavior by using integer-based arithmetic with a configurable `SHIFT_AMOUNT`.

---

## ⚡ Performance Considerations

FixedMathSharp is optimized for high-performance deterministic calculations:
- **Inline methods and bit-shifting optimizations** ensure minimal overhead.
- **Eliminates floating-point drift**, making it ideal for lockstep simulations.
- **Supports fuzzy equality comparisons** for handling minor precision deviations.

---

## 🧪 Testing and Validation

Unit tests are used extensively to validate the correctness of mathematical operations. 
Special **fuzzy comparisons** are employed where small precision discrepancies might occur, mimicking floating-point behavior.

To run the tests:
```bash
dotnet test --configuration debug
```

---

## 🛠️ Compatibility

- **.NET Framework** 4.7.2+
- **.NET Core / .NET** 6+
- **Unity 2020+** (via [FixedMathSharp-Unity](https://github.com/mrdav30/FixedMathSharp-Unity))
- **Cross-Platform Support** (Windows, Linux, macOS)

---

## 🤝 Contributing

We welcome contributions! Please see our [CONTRIBUTING](https://github.com/mrdav30/FixedMathSharp/blob/main/CONTRIBUTING.md) guide for details on how to propose changes, report issues, and interact with the community.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/mrdav30/FixedMathSharp/blob/main/LICENSE.md) for details.

---

## 👥 Contributors

- **mrdav30** - Lead Developer
- Contributions are welcome! Feel free to submit pull requests or report issues.

---

## 📧 Contact

For questions or support, reach out to **mrdav30** via GitHub or open an issue in the repository.

---
