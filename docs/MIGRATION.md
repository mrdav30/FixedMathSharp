# Migrating From v4.x To v5.0.0

FixedMathSharp v5.0.0 is a major API hardening release. The migration is mostly
source-level cleanup, but several changes affect numeric interpretation,
transform semantics, and public names.

Use this guide when upgrading from v4.0.1 or earlier.

## Upgrade Checklist

- Update package references to `FixedMathSharp` 5.0.0, or `FixedMathSharp.Lean`
  5.0.0 if you use the lean package.
- Rebuild your solution and fix compile errors before chasing runtime behavior.
  Many v5 changes intentionally fail at compile time instead of preserving weak
  v4 shapes.
- Replace renamed geometry and enum types.
- Audit every raw `Fixed64` text conversion.
- Replace removed floating-point helper surfaces with explicit `Fixed64`
  boundary conversions.
- Audit matrix, quaternion, and transform code that assumed column-vector or
  engine-specific semantics.
- Re-run deterministic replay, save/load, and lockstep tests after the code
  compiles.

## Geometry Type Renames

The bounds types now use the same `Fixed*` naming style as the rest of the
library.

| v4.x | v5.0.0 |
| --- | --- |
| `BoundingBox` | `FixedBoundBox` |
| `BoundingSphere` | `FixedBoundSphere` |
| `BoundingArea` | `FixedBoundArea` |
| `BoundingFrustum` | `FixedBoundFrustum` |
| `ContainmentType` | `FixedEnclosureType` |

The core geometry namespace remains `FixedMathSharp`. Most call sites need a
rename only:

```csharp
// v4.x
BoundingBox room = new BoundingBox(center, size);
ContainmentType state = room.Contains(other);

// v5.0.0
FixedBoundBox room = new FixedBoundBox(center, size);
FixedEnclosureType state = room.Contains(other);
```

## Fixed64 Value And Raw Conversions

`Fixed64` now makes value-space and raw-payload conversions explicit.

### Decimal Text Vs Raw Text

In v4.x, `Fixed64.Parse` and `TryParse` interpreted text as a raw Q32.32
`long` payload. In v5.0.0, they parse normal decimal value text.

```csharp
// v4.x raw payload text
Fixed64 one = Fixed64.Parse("4294967296");

// v5.0.0 raw payload text
Fixed64 one = Fixed64.ParseRaw("4294967296");

// v5.0.0 value-space decimal text
Fixed64 value = Fixed64.Parse("1.25");
```

Use `TryParseRaw` for raw payload text and `ToRawString` when writing raw text.
Use `Parse`, `TryParse`, `ToString`, and `TryFormat` for human-readable decimal
diagnostics.

### Constructor And Factory Changes

The public double constructor was removed. Floating-point input now goes through
checked boundary factories.

```csharp
// v4.x
Fixed64 speed = new Fixed64(3.5);
Fixed64 ratio = Fixed64.Fraction(1, 60);

// v5.0.0
Fixed64 speed = Fixed64.FromDouble(3.5);
Fixed64 exactDecimal = Fixed64.FromDecimal(3.5m);
Fixed64 ratio = Fixed64.FromFraction(1, 60);
```

`Vector2d`, `Vector3d`, and `Vector4d` double constructors were also replaced
with `FromDouble` factories:

```csharp
// v4.x
Vector3d point = new Vector3d(1.25, 2.5, 3.75);

// v5.0.0
Vector3d point = Vector3d.FromDouble(1.25, 2.5, 3.75);
```

`FromDouble`, explicit `float`/`double` casts to `Fixed64`, vector `FromDouble`
factories, and `FixedCurveKey.FromDouble` now reject `NaN` and infinities with
`ArgumentOutOfRangeException`. Finite values outside the Q32.32 range throw
`OverflowException`.

### Raw Longs Vs Integer Longs

The `Fixed64` arithmetic operators that accepted `long` operands were removed.
Those overloads were ambiguous because a `long` can mean either a normal
integer value or an already-scaled raw Q32.32 payload.

```csharp
long tileCount = 5;
long rawStep = 1;

Fixed64 integerValue = (Fixed64)tileCount;
Fixed64 rawValue = Fixed64.FromRaw(rawStep);
```

The explicit `long` conversion now saturates to `Fixed64.MinValue` or
`Fixed64.MaxValue` when the source integer is outside the representable Q32.32
whole-number range.

## Fixed-Point Boundary APIs

Several cross-domain helpers were removed so deterministic code stays in
fixed-point land.

| v4.x | v5.0.0 |
| --- | --- |
| `FixedRange.InRange(double)` | Convert once with `Fixed64.FromDouble`, then call `InRange(Fixed64)` |
| `DeterministicRandom.NextDouble()` | Use `NextFixed6401()` or `NextFixed64(...)` |
| `Fixed64.RawToString()` | `Fixed64.ToRawString()` |
| `Fixed64.RawToInt(...)` | `Fixed64.ToInt(...)` |

Keep floating-point conversion at engine, UI, editor, or import/export
boundaries. Core simulation code should pass `Fixed64` values directly.

## Scalar Algorithm Ownership

`FixedMath` is the canonical home for scalar algorithms such as interpolation,
powers, logarithms, trigonometry, square root, rounding, and clamping. `Fixed64`
owns representation, constants, conversions, parsing, operators, equality, and
raw helpers.

For v4 call sites, the verified scalar rename is:

| v4.x | v5.0.0 |
| --- | --- |
| `value.ToDegree()` | `value.ToDegrees()` |

For new or refactored scalar interpolation code, prefer the `FixedMath` static
surface, such as `FixedMath.Lerp`, `FixedMath.CatmullRom`,
`FixedMath.HermiteSpline`, and `FixedMath.BarycentricCoordinate`. The fluent
extension surface is curated and forwards to the canonical implementation.
Factories and convention-heavy methods stay on the owning type.

## Vector API Cleanup

Vector mutation and value-returning APIs now use one naming model.

| v4.x | v5.0.0 |
| --- | --- |
| `vector.x`, `vector.y`, `vector.z`, `vector.w` | `vector.X`, `vector.Y`, `vector.Z`, `vector.W` |
| `vector.Normal` | `vector.Normalized` |
| `vector.Normalize()` | `vector.NormalizeInPlace()` |
| `Vector*d.Normalize(value)` where applicable | `Vector*d.GetNormalized(value)` |
| `SqrMagnitude` | `MagnitudeSquared` |
| `SqrDistance(...)` | `DistanceSquared(...)` |
| `Vector2d.Lerped(...)` | `Vector2d.Lerp(...)` |
| `ScaleInPlace(...)` | `MultiplyInPlace(...)` |
| Public vector-result `out Vector*d` helpers | Return-by-value statics or explicit `*InPlace` methods |

Example:

```csharp
// v4.x
Vector3d normal = velocity.Normal;
velocity.Normalize();
velocity.ScaleInPlace(factor);

// v5.0.0
Vector3d normal = velocity.Normalized;
velocity.NormalizeInPlace();
velocity.MultiplyInPlace(factor);
```

If you persist vectors through JSON using field names, audit payloads that use
lowercase component names. MemoryPack component order remains explicit through
the existing `[MemoryPackOrder]` attributes.

`Normalize(out Fixed64 magnitude)` remains as `NormalizeInPlace(out Fixed64
magnitude)` because it returns a second scalar result.

## Matrix And Transform Semantics

v5.0.0 makes affine transform semantics explicit and consistent:

- FixedMathSharp's 3D basis is `+X` right, `+Y` up, and `+Z` forward.
- 3D transforms use row vectors: points and vectors are transformed as
  `value * matrix`.
- Translation lives in `M41`, `M42`, and `M43`.
- `Fixed4x4` composition is left-to-right under that row-vector convention.
- Quaternion matrix conversion, matrix-to-quaternion extraction, Euler
  extraction, and look-rotation basis construction were aligned to that model.

Audit any code that depended on a column-vector convention, copied matrices
directly from engine APIs, or assumed a `-Z` forward convention.

`Fixed4x4.Decompose` now returns out parameters in
`translation, rotation, scale` order.

```csharp
// v4.x style
Fixed4x4.Decompose(matrix, out Vector3d scale, out FixedQuaternion rotation, out Vector3d translation);

// v5.0.0
Fixed4x4.Decompose(matrix, out Vector3d translation, out FixedQuaternion rotation, out Vector3d scale);
```

## Coordinate Convention Helpers

Use `CoordinateConvention3d` at adapter boundaries instead of changing core
direction constants or adding engine-specific conditionals.

```csharp
CoordinateConvention3d external = CoordinateConvention3d.NegativeZForward;
Vector3d canonicalForward = external.ToCanonicalDirection(external.Forward);
```

Built-in conventions include:

- `CoordinateConvention3d.Canonical`
- `CoordinateConvention3d.PositiveZForward`
- `CoordinateConvention3d.NegativeZForward`
- `CoordinateConvention3d.XForwardZUp`

These helpers map direction vectors and signed axes. Matrix storage, handedness,
clip-space depth, units, and origins remain adapter-specific concerns.

## Bounds And Hot-Path Helpers

`FixedBoundSphere.CreateFromPoints` now has `Vector3d[]` and
`ReadOnlySpan<Vector3d>` overloads for countable, allocation-light call sites.
The `IEnumerable<Vector3d>` overload remains for interoperability.

`Vector2d.CheckDistance` and `Vector3d.CheckDistance` now compare squared
distances after validating the threshold, avoiding an unnecessary square root.
Negative thresholds throw `ArgumentOutOfRangeException`.

`FixedMath.FastAdd`, `FastSub`, `FastMul`, `FastDiv`, and `FastMod` are expert
APIs. Their docs describe the skipped checks or precision caveats. Prefer the
normal operators unless a local benchmark proves that the fast path is correct
for your inputs and worth the narrower contract.

## Diagnostics Formatting

Human-readable formatting is now separated from raw payload representation:

- Use `ToString(...)` or `TryFormat(...)` for logs, editor display, and
  diagnostics.
- Use `ToRawString`, `ParseRaw`, and `TryParseRaw` for raw Q32.32 payload text.
- Use MemoryPack or JSON support for structured serialization.

On `net8.0`, supported types implement `ISpanFormattable`. On
`netstandard2.1`, the same `TryFormat` method shape is exposed where the
interface itself is unavailable.

## Suggested Validation

After migrating source, run:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
```

For consumer applications, also run deterministic replay, save/load, and
network synchronization tests that cover transforms, parsing, serialization,
and random streams.
