# Migrating From v5.x To v6.x

FixedMathSharp v6.x is a geometry and bounds hardening release. The largest
change is dimensional clarity: `FixedBoundBox` is the 3D AABB type, while
`FixedBoundArea` is now a true `Vector2d` 2D AABB. The release also removes
ambiguous bounds construction, removes hidden corner-array storage, adds shared
2D/3D segment and triangle primitives, and adds an optional
`FixedMathSharp.Chronicler` companion package for deterministic replay hash
writers.

Use this guide when upgrading from any v5.x package.

## v6 Upgrade Checklist

- Update package references to `FixedMathSharp` v6.x, or `FixedMathSharp.Lean`
  v6.x if you use the lean package.
- Add `FixedMathSharp.Chronicler` or `FixedMathSharp.Chronicler.Lean` only if
  your project uses Chronicler replay hashing helpers.
- Replace old 3D `FixedBoundArea` usage. Use `FixedBoundBox` for 3D volumes and
  the new `Vector2d`-based `FixedBoundArea` for planar footprints.
- Replace `new FixedBoundBox(center, size)` with named factory calls.
- Replace `FixedBoundBox.Vertices` with `GetCorner` or `CopyCorners`.
- Audit `Intersects` behavior where touching bounds used to be treated as
  separate. Default intersections are now boundary-inclusive.
- Audit serialized or hashed bounds payloads if you persisted `FixedBoundArea`
  or `FixedBoundSphere` state directly.
- Re-run deterministic replay, save/load, spatial-query, and broad-phase tests.

## Dimensional Bounds Split

`FixedBoundArea` no longer represents a 3D shape. It is now a normalized 2D
axis-aligned area backed by `Vector2d`:

```csharp
FixedBoundArea area = FixedBoundArea.FromMinMax(
    new Vector2d(-4, -2),
    new Vector2d(4, 2));
```

Migrate old 3D area usage by intent:

| v5.x usage | v6.x replacement |
| --- | --- |
| 3D volume, collider, frustum, ray, or plane bounds | `FixedBoundBox` |
| Flat footprint in a 3D world | `FixedBoundArea` plus explicit layer, height, or elevation state in the consuming package |
| Pure 2D area query or broad-phase bounds | `FixedBoundArea` |

There is no 3D `FixedBoundArea` compatibility layer. That is intentional: a 3D
area was ambiguous beside `FixedBoundBox`, and higher-level packages should own
layer/elevation semantics explicitly.

3D overloads that previously accepted `FixedBoundArea` were removed. For
example:

```csharp
// v5.x 3D-shaped area usage
// bool hit = ray.Intersects(area) != null;
// FixedPlaneIntersectionType side = plane.Intersects(area);

// v6.x 3D volume usage
FixedBoundBox box = FixedBoundBox.FromMinMax(min3d, max3d);
Fixed64? hit = ray.Intersects(box);
FixedPlaneIntersectionType side = plane.Intersects(box);

// v6.x pure 2D usage
FixedRay2d ray2d = new(origin2d, direction2d);
Fixed64? planarHit = ray2d.Intersects(area);
```

## FixedBoundBox Construction

The public `FixedBoundBox(Vector3d center, Vector3d size)` constructor was
removed because call sites could not tell whether two vectors meant
center/size, center/scope, or min/max. Use named factories:

```csharp
// v5.x
FixedBoundBox box = new FixedBoundBox(center, size);

// v6.x
FixedBoundBox box = FixedBoundBox.FromCenterAndSize(center, size);
FixedBoundBox fromHalfExtents = FixedBoundBox.FromCenterAndScope(center, scope);
FixedBoundBox fromCorners = FixedBoundBox.FromMinMax(min, max);
```

`FromMinMax`, `SetMinMax`, and serialized state population normalize swapped
min/max inputs. `FromCenterAndSize` and `FromCenterAndScope` normalize negative
extents by absolute component value.

The serialization state constructor remains:

```csharp
FixedBoundBox box = new FixedBoundBox(
    new FixedBoundBox.BoundingBoxState(min, max));
```

Use that constructor for explicit state transfer, not for ordinary call-site
construction.

## Allocation-Free Box Corners

`FixedBoundBox.Vertices` was removed. It exposed mutable array storage from a
value type and could allocate when callers only needed a corner.

```csharp
// v5.x
Vector3d corner = box.Vertices[0];

// v6.x
Vector3d corner = box.GetCorner(0);

Span<Vector3d> corners = stackalloc Vector3d[FixedBoundBox.CornerCount];
box.CopyCorners(corners);
```

`GetCorner` and `CopyCorners` use the same stable corner order. Prefer
`GetCorner` for one-off access and caller-owned spans/arrays for bulk copies.

## Intersection Semantics

Default `Intersects` methods now use closed-bound, boundary-inclusive overlap.
Touching edges, faces, corners, or tangent surfaces count as intersections.

Where positive area or positive volume matters, use `IntersectsStrict`:

```csharp
bool touchesOrOverlaps = box.Intersects(otherBox);
bool hasPositiveVolume = box.IntersectsStrict(otherBox);

bool areasTouchOrOverlap = area.Intersects(otherArea);
bool hasPositiveArea = area.IntersectsStrict(otherArea);
```

Strict overlap methods reject boundary-only contact and zero-size inputs. They
exist for box-box, box-sphere, sphere-box, sphere-sphere, area-area,
area-circle, circle-area, and circle-circle pairs.

If your v5.x logic depended on `FixedBoundBox.Intersects(FixedBoundBox)`
returning `false` for face, edge, or corner contact, switch that call site to
`IntersectsStrict`.

## Radius And Serialized Bounds State

`FixedBoundCircle` is new in v6.x. `FixedBoundSphere` was also tightened so
negative radii are normalized through construction, assignment, and serialized
state load.

```csharp
FixedBoundSphere sphere = new FixedBoundSphere(center, new Fixed64(-5));
// sphere.Radius == 5
```

`FixedBoundSphere` now exposes a matching `BoundingSphereState` payload, aligned
with `FixedBoundBox`, `FixedBoundArea`, and `FixedBoundCircle`.

If your project persisted or inspected bounds state directly, audit those
payloads:

- 3D boxes write canonical `Min` then `Max`.
- 2D areas write canonical `Vector2d Min` then `Vector2d Max`.
- 2D circles write `Center` then normalized `Radius`.
- 3D spheres write `Center` then normalized `Radius`.

## New Geometry Primitives

v6.x adds reusable deterministic geometry primitives that downstream packages
can use instead of local one-off structs:

| Domain | New type | Main use |
| --- | --- | --- |
| 2D area bounds | `FixedBoundArea` | Planar AABB, clamp/project, union, area overlap |
| 2D circular bounds | `FixedBoundCircle` | Radius queries, circle/area overlap, projection |
| 2D rays | `FixedRay2d` | Planar ray against area/circle |
| 2D segments | `FixedSegment2d` | Finite edge, closest point, distance, bounds |
| 2D triangles | `FixedTriangle2d` | Area, bounds, containment, closest point, barycentric weights |
| 3D segments | `FixedSegment` | Finite 3D edge, closest point, distance, bounds |
| 3D triangles | `FixedTriangle` | Normal, area, bounds, containment, closest point, projected barycentric weights |

Segments preserve ordered endpoint identity. Reversed endpoints have the same
bounds but do not compare equal.

Rays do not normalize direction by construction. A returned ray parameter is a
physical distance only when the caller supplied a normalized direction.

Triangles preserve ordered vertices. `FixedTriangle2d.TryGetBarycentricWeights`
solves planar weights directly. `FixedTriangle.TryGetProjectedBarycentricWeights`
names the 3D projection behavior explicitly.

`Vector2d.BarycentricCoordinates(...)` now mirrors
`Vector3d.BarycentricCoordinates(...)` for reconstructing points from known B/C
barycentric weights.

## Chronicler Hash Companion Package

FixedMathSharp v6.x includes an optional `FixedMathSharp.Chronicler` companion
package. Add it only when your project uses Chronicler replay hashing:

```csharp
using Chronicler;
using FixedMathSharp.Chronicler;

ChronicleHashWriter writer = new();
writer.WriteFixed64(value);
writer.WriteVector3d(position);
writer.WriteBoundBox(bounds);
```

The extension package writes canonical deterministic payloads for `Fixed64`,
vectors, quaternions, transforms, matrices, bounds, rays, and planes. It keeps
Chronicler-specific code out of the core math package.

Important `FixedBoundArea` change: `WriteBoundArea` now writes the new 2D
`FixedBoundArea`. If old replay code used it for 3D area-like payloads, migrate
that hash input to `WriteBoundBox` or to an explicit 2D area plus separate
layer/elevation fields.

## Suggested Search Patterns

After updating package references, these searches catch the most common v6
migration work:

```bash
rg -n "new FixedBoundBox" src tests
rg -n "Vertices" src tests
rg -n "FixedBoundArea" src tests
rg -n "Intersects" src tests
rg -n "WriteBoundArea" src tests
```

Review each `new FixedBoundBox(...)` match manually. The state constructor is
still valid, while the old center/size constructor should become a named
factory call.

Review each `FixedBoundArea` match by dimension. If the surrounding code uses
`Vector3d`, a ray/plane/frustum, or volumetric bounds, it probably wants
`FixedBoundBox`. If the code is planar, migrate to the new `Vector2d` area.

## v6 Suggested Validation

After migrating source, run:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
```

For consumer applications, also run deterministic replay, save/load,
broad-phase/query, and spatial partition tests that cover bounds construction,
intersection semantics, and serialized geometry state.

---

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
