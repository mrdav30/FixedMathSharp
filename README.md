FixedMathSharp
==============

![FixedMathSharp Icon](https://raw.githubusercontent.com/mrdav30/fixedmathsharp/main/icon.png)

**A high-precision, deterministic fixed-point math library for .NET.**  
Ideal for simulations, games, and physics engines requiring reliable arithmetic without floating-point inaccuracies.

---

## 🛠️ Key Features

- **Deterministic Calculations:** Perfect for simulations, multiplayer games, and physics engines.
- **High Precision Arithmetic:** Uses fixed-point math to eliminate floating-point inaccuracies.
- **Comprehensive Vector Support:** Includes 2D and 3D vector operations (`Vector2d`, `Vector3d`).
- **Quaternion Rotations:** Leverage `FixedQuaternion` for smooth rotations without gimbal lock.
- **Matrix Operations:** Supports transformations with `Fixed4x4` and `Fixed3x3` matrices.
- **Bounding Shapes:** Includes `IBound` structs `BoundingBox`, `BoundingSphere`, and `BoundingArea` for lightweight spatial calculations.
- **Advanced Math Functions:** Includes trigonometry and common math utilities.
- **Unity Integration:** Seamless interoperability with Unity using `FixedMathSharp.Editor`.


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

### Unity

To integrate **FixedMathSharp** into your Unity project:

1. **Download the Package**:
   - Obtain the latest `FixedMathSharp{{VERSION}}.unitypackage` from the [Releases](https://github.com/mrdav30/FixedMathSharp/releases) section of the repository.

2. **Import into Unity**:
   - In Unity, navigate to **Assets > Import Package > Custom Package...**.
   - Select the downloaded `FixedMathSharp{{VERSION}}.unitypackage` file.

3. **Verify the Integration**:
   - After importing, confirm that the `FixedMathSharp` namespace is accessible in your scripts.

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

---

## 📦 Library Structure

- **`Fixed64` Struct:** Represents fixed-point numbers for precise arithmetic.
- **`Vector2d` and `Vector3d` Structs:** Handle 2D and 3D vector operations.
- **`FixedQuaternion` Struct:** Provides rotation handling without gimbal lock, enabling smooth rotations and quaternion-based transformations.
- **`IBound` Interface:** Standard interface for bounding shapes `BoundingBox`, `BoundingArea`, and `BoundingSphere`, each offering intersection, containment, and projection logic.
- **`FixedMath` Static Class:** Provides common math functions.
- **`FixedTrigonometry` Class:** Offers trigonometric functions using fixed-point math.
- **`Fixed4x4` and `Fixed3x3`:** Support matrix operations for transformations.
- **`FixedMathSharp.Editor`:** Extensions for seamless integration with Unity, including property drawers and type conversions.

### Fixed64 Struct

**Fixed64** is the core data type representing fixed-point numbers. It 
provides various mathematical operations, including addition,
subtraction, multiplication, division, and more. The struct guarantees
deterministic behavior by using integer-based arithmetic with a
configurable `SHIFT_AMOUNT`.

---

## ⚡ Performance Considerations

This library leverages **inline methods** and **fixed-point arithmetic** 
to ensure high precision without the pitfalls of floating-point numbers. 
It is optimized for **deterministic behavior**, making it ideal for physics 
engines, multiplayer simulations, and other time-sensitive applications.

---

## 🧪 Testing and Validation

Unit tests are used extensively to validate the correctness of mathematical 
operations. Special **fuzzy comparisons** are employed where small precision 
discrepancies might occur, mimicking floating-point behavior.

To run the tests:
```bash
dotnet test --configuration Release
```

---

## 🛠️ Compatibility

- **.NET Framework:** 4.7.1+
- **Unity3D:** Fully compatible with Unity using the `FixedMathSharp.Editor` extension.
- **Platforms:** Windows, Linux, macOS

---

## 📄 License

This project is licensed under the MIT License - see the `LICENSE` file
for details.

---

## 👥 Contributors

- **mrdav30** - Lead Developer
- Contributions are welcome! Feel free to submit pull requests or report issues.

---

## 📧 Contact

For questions or support, reach out to **mrdav30** via GitHub or open an issue in the repository.

---