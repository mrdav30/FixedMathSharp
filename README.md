# FixedMathSharp

<p align="center">
  <img src=".assets/icon_128x128.png" alt="FixedMathSharp Icon" width="200"/>
</p>

**FixedMathSharp** is a high-precision, fixed-point mathematics library
designed to enable deterministic, efficient arithmetic operations. It
provides robust implementations of mathematical functions, trigonometric
operations, and vector transformations, ensuring precision without the
pitfalls of floating-point errors.

## Key Components

- **Fixed64**: Represents a 64-bit fixed-point number with configurable
  fractional precision.
- **FixedMath**: A partial static class providing core mathematical
  functions for fixed-point numbers, including clamping, rounding, and
  interpolation.
- **FixedTrigonometry**: A module containing trigonometric and
  logarithmic functions, ensuring consistent angle calculations with
  fixed-point precision.

## Usage Overview

FixedMathSharp is useful for applications where determinism is critical,
such as simulations, games, and physics engines. With support for
precise math functions, developers can rely on consistent results across
platforms and eliminate floating-point discrepancies.

### Installation

1.  Clone the repository:
    `git clone https://github.com/your-repo/fixedmathsharp.git`
2.  Add the project as a dependency to your .NET solution.
3.  Reference the `FixedMathSharp` namespace in your files:
    `using FixedMathSharp;`

### Fixed64 Struct

**Fixed64** is the core data type representing fixed-point numbers. It
provides various mathematical operations, including addition,
subtraction, multiplication, division, and more. The struct guarantees
deterministic behavior by using integer-based arithmetic with a
configurable `SHIFT_AMOUNT`.

#### Example

    var fixedValue = new Fixed64(1.5);
    var doubledValue = fixedValue * Fixed64.Two;
    Console.WriteLine(doubledValue); // Outputs: 3.0
        

### FixedMath Static Class

**FixedMath** contains essential functions like clamping, interpolation,
rounding, and utility operations for `Fixed64`. It supports operations
like `Clamp`, `MoveTowards`, and `RoundToPrecision`.

#### Example

    var clampedValue = FixedMath.Clamp(new Fixed64(2.5), Fixed64.Zero, Fixed64.One);
    Console.WriteLine(clampedValue); // Outputs: 1.0
        

### FixedTrigonometry Module

The **FixedTrigonometry** module extends FixedMath with trigonometric
functions like `Sin`, `Cos`, `Tan`, and inverse functions. These
functions ensure accurate angle calculations, even with fixed-point
precision.

#### Example

    var angle = FixedMath.DegToRad(new Fixed64(90));
    var sineValue = FixedMath.Sin(angle);
    Console.WriteLine(sineValue); // Outputs: 1.0
        

## Key Features

- **Deterministic Behavior**: Fixed-point arithmetic ensures consistency
  across platforms.
- **Precision Control**: Configurable fractional bits for optimal
  precision.
- **Efficient Operations**: Optimized for performance-critical scenarios
  with minimal overhead.
- **Comprehensive Trigonometry**: Includes sine, cosine, tangent, and
  their inverse functions.

## Contributing

We welcome contributions to improve FixedMathSharp. Please fork the
repository and submit a pull request with your changes. Ensure that all
existing tests pass and add new tests for any new functionality.

### Testing

Run the unit tests to verify that the library functions correctly:

    dotnet test

## License

This project is licensed under the MIT License - see the `LICENSE` file
for details.
