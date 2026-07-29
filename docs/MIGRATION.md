# FixedMathSharp Migration Guide

## Migrating From v6.x To v7.x

FixedMathSharp v7.x is a deterministic arithmetic, transform, and full-domain
geometry hardening release. The largest source-breaking change is the
`FixedTransform` redesign: local and world values are now explicit, parent
changes have named preservation semantics, and arbitrary matrix import is
strict. Arithmetic and geometry APIs also retain exact wide intermediates until
their final Q32.32 conversion, which can change raw results at rounding,
overflow, normalization, degeneracy, and containment boundaries.

Use this guide when upgrading from any v6.x package.

### v7 Upgrade Checklist

- Update package references to `FixedMathSharp` v7.x, or `FixedMathSharp.Lean`
  v7.x if you use the Lean package.
- Rebuild first and migrate removed `FixedTransform`, matrix-scale, and
  `Vector3d.ClosestPointsOnTwoLines` usages. Replace any direct use of the
  removed `FixedMath.AddOverflowHelper`; it was an unused low-level helper, not
  part of the fixed-point arithmetic contract.
- Decide explicitly whether each transform access is local or world space.
- Audit chained multiply/divide calculations that rely on an intermediate
  saturated result; use `TryMultiplyDivide` when the mathematical expression
  requires one final rounding step.
- Replace `ray.Position + ray.Direction * parameter` reconstruction with
  `ray.GetPoint(parameter)` so each coordinate rounds and saturates once.
- Replace normalize-then-ray finite-axis sweeps with the segment physical-distance
  interval APIs when the authored chord contains small components that must not
  round away during normalization. Reconstruct those hits with
  `segment.GetPointAtDistance(distance, totalDistance)`.
- Replace separate radial and axial finite-cylinder expansion with
  `TryGetSweptSphereFiniteCylinderIntersectionDistance` when the
  intended volume is the exact swept-sphere Minkowski sum rather than a
  sharp-rim affine proxy.
- Replace `FixedBoundCircle.RadiusSquared` and
  `FixedBoundSphere.RadiusSquared` callers with the actual `Radius` or the
  bound's exact containment/intersection methods.
- Handle `OverflowException` from bounding-sphere factories when their
  deterministic containing radius is not representable; v6 could silently
  return a saturated sphere that did not contain its input.
- Re-record deterministic golden values, replay hashes, and serialized expected
  outputs that depend on division, normalization, transforms, segments, or
  triangles. Do not silently compare v6 and v7 simulation hashes as though the
  numeric contract were unchanged.
- Re-run deterministic replay, save/load, collision/query, transform-hierarchy,
  and broad-phase tests after the source migration compiles.

### Geometry Namespace

Bounds and geometry primitives now share the
`FixedMathSharp.Geometry` namespace. Type names are unchanged; update the
namespace import:

```csharp
using FixedMathSharp.Geometry;
```

### Division Rounds Midpoints To Even

`Fixed64` multiplication and division now share the same nearest-even rounding
contract. Division previously rounded exact midpoints away from zero. In v7, the
retained raw value's parity decides the tie:

```csharp
Fixed64 oneRaw = Fixed64.FromRaw(1);
Fixed64 threeRaw = Fixed64.FromRaw(3);

// v7 raw results: 0 and 2.
Fixed64 first = oneRaw / Fixed64.Two;
Fixed64 second = threeRaw / Fixed64.Two;
```

`FixedMath.FastDiv` matches `/` for its supported positive-divisor path.
Divide-by-zero and final saturation behavior are unchanged.

This correction also restores exact binary reciprocal identities:

```csharp
value / Fixed64.Two == value * Fixed64.Half
value / new Fixed64(4) == value * Fixed64.Quarter
value / new Fixed64(8) == value * Fixed64.Eighth
```

Pre-rounded reciprocals are still different mathematical inputs. For example,
`value / new Fixed64(3)` is not required to equal
`value * Fixed64.FromFraction(1, 3)`.

### Exact Try Arithmetic And Fused Multiply-Divide

The normal `Fixed64`, `Vector2d`, and `Vector3d` addition/subtraction operators
remain saturating. Use the new `TryAdd` and `TrySubtract` methods when
saturation must be reported instead of accepted:

```csharp
if (!Vector3d.TrySubtract(end, start, out Vector3d delta))
{
    // The exact component result was outside the Fixed64 range.
}
```

Failure returns `false` with `result = default`. Vector operations are atomic;
they never expose a partially calculated or partially saturated vector.

Parentheses do not fuse overloaded operators. `(left * right) / divisor` still
rounds and saturates the multiplication before division. Use `TryMultiplyDivide`
when the complete expression should round once and fail only when the divisor is
zero or the final result is not representable:

```csharp
if (!Fixed64.TryMultiplyDivide(left, right, divisor, out Fixed64 result))
{
    // Zero divisor or final-result overflow.
}

if (!Fixed64.TryMultiplyDivide(first, second, third, divisor, out result))
{
    // Zero divisor or final-result overflow.
}
```

These are opt-in APIs. Existing operators retain their public saturating
contract.

`FixedMath.AddOverflowHelper` has been removed. For checked integral code, use
the language's `checked` context. A caller that deliberately needs the former
wrapped-sum-plus-flag behavior can keep that policy locally:

```csharp
long sum = unchecked(left + right);
overflow |= ((left ^ sum) & (right ^ sum)) < 0;
```

`Fixed64.MultiplyAdd(left, right, addend)` provides the corresponding fused
multiply-add contract. It rounds once and saturates only the final result;
`TryMultiplyAdd` reports an unrepresentable final value with `false` and a
default result. `FixedRay.GetPoint(parameter)` and `FixedRay2d.GetPoint(parameter)`
use this contract per coordinate. The argument is a ray parameter, not always a
physical distance, because ray directions are not normalized by construction.

### Exact Chord Physical-Distance Intervals

`FixedSegment2d` and `FixedSegment` now expose
`TryGetCapsuleIntersectionDistanceInterval`; `FixedSegment` also exposes
`TryGetFiniteCylinderIntersectionDistanceInterval`. These methods preserve the
original endpoint differences through the exact finite-axis solve, then map the
closed segment parameter range to the caller-supplied nonnegative `totalDistance`
with one final round-half-to-even conversion. This is stronger than normalizing
the chord and calling a bounded-ray method: a small but representable transverse
component cannot disappear before collision classification.

Centered capsule and centered finite-cylinder overloads retain the existing
inclusive-start and strict-end containment flags. Endpoint-authored capsule and
cylinder overloads provide the same classification. Centered cylinder
overloads retain separate authored full axis length, radial expansion, and
axial expansion.

For swept spheres, `FixedSegment` now also exposes
`TryGetSweptSphereFiniteCylinderIntersectionDistance` and
`TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval`. These
methods solve the rounded side, cap, and circular-rim boundary of the exact
finite-cylinder/sphere Minkowski sum. They are not aliases for independently
expanding cylinder radius and height, which produces a larger sharp-rim
volume. The entry-only form avoids refining the toroidal-rim exit root when a
query needs only its first contact; both forms retain exact wide intermediates
through one final round-half-to-even physical-distance conversion.

`FixedSegment.TryGetSweptSphereBoxIntersectionDistance` provides the equivalent
entry-only contract for a `FixedBoundBox`. Use it when the intended volume is
the exact box/sphere Minkowski sum: independently expanding the three box
extents creates sharp edge and corner regions that can report early or false
contacts. The method preserves the authored chord through exact box-feature
transitions and narrows only the final physical distance.

Use `GetPointAtDistance(distance, totalDistance)` to reconstruct a returned hit
from the same exact chord. It rejects negative total distance and distances
outside `[0, totalDistance]`; zero total distance is accepted only for a
zero-length segment and reconstructs its single point. Nondegenerate endpoints
are returned bit-for-bit, while interior coordinates are fused and rounded once.

### Exact Projection, Magnitude, And Rotation Behavior

v7 adds full-domain helpers for calculations that must make decisions before
public saturation:

- `Vector2d.CompareProjection(...)`
- `Vector3d.CompareProjection(...)`
- `Vector3d.ProjectNonNegativeDifference(...)`
- `Vector2d.TryGetMagnitude(...)`
- `Vector3d.TryGetMagnitude(...)`
- `Vector4d.TryGetMagnitude(...)`
- `Vector2d.IsNormalized()`
- `FixedMath.Average(first, second, third)`
- `FixedMath.Midpoint(left, right)`

Vector and quaternion magnitude/normalization paths now preserve complete finite
component ranges. Quaternion construction and conversion are also more robust:

- Nonzero vector and quaternion normalization results now satisfy the matching
  `IsNormalized()` predicate. Inputs already admitted by that predicate are
  preserved, while tiny and full-domain components normalize through a
  scale-safe exact fallback when ordinary Q32.32 magnitude arithmetic cannot
  retain enough information.
- Nonzero axis and direction inputs normalize scale-safely.
- Zero-axis axis-angle construction returns `FixedQuaternion.Identity`.
- Radian and degree constructors accept the complete finite `Fixed64` domain;
  multi-turn inputs reduce deterministically.
- `FixedQuaternion.QuaternionLog` tolerates one-raw-unit normalization drift at
  both `Acos` endpoints.
- `FixedQuaternion.ToMatrix3x3` is independent of a nonzero quaternion's common
  scale; zero still maps to identity.
- `FixedQuaternion.Angle` now returns the full shortest physical rotation in
  degrees and treats `q` and `-q` as the same rotation. v6 returned the
  quaternion half-angle; remove downstream `* 2` corrections.
- `FixedMath.DegToRad` and `RadToDeg` retain full-domain intermediates and round
  once at the public boundary.
- `FixedMath.Sin` and `Cos` use complementary reduced-range polynomials around
  quadrant boundaries. Values adjacent to `0` and `±Pi/2` no longer jump away
  from their exact anchors by the former degree-7 endpoint error.
  `FixedMath.CanonicalSinCosErrorBound` publishes the conservative approximation
  bound for canonical `[-Pi, Pi]` inputs. It intentionally excludes phase error
  accumulated while reducing large multi-turn angles.

Ordinary inputs generally retain their expected meaning, but raw results can
change where v6 saturated, underflowed, rounded an intermediate, or evaluated
trigonometry in the outer half of a quadrant. Refresh golden numeric and replay
expectations rather than adding downstream clamps.

### Semantic Point Anchors

`FixedPointAnchor` and `FixedPointAnchor2d` can now retain relative geometry
after an offset or absolute point leaves the Q32.32 scalar domain. Prefer their
semantic operations over materializing an intermediate vector:

- `CompareSquaredDistance(first, second)` ranks two candidates exactly from the
  reference anchor and preserves exact ties.
- `TryGetPoint` and `TryGetOffsetFrom` remain the honest narrowing boundaries
  when a public point or vector is actually required.
- `TryGetLocalPointIn` and `TryReframe` preserve rigid-frame cancellation
  without first materializing an absolute world point.

The public anchor contract deliberately excludes rigid-body lever,
mass-property, and response policy. Simulation libraries should own those
semantics while reusing FixedMathSharp's general geometry and arithmetic.

### Full-Domain Directions, Interpolation, And Radial Rays

`Vector2d.GetDirection(start, end)` and `Vector3d.GetDirection(start, end)` now
normalize endpoint differences without first narrowing them to one `Fixed64`
component. Equal endpoints return zero. Use these helpers when world-coordinate
endpoints can span more than one representable scalar even though the resulting
unit direction is representable. Very small representable differences also keep
their component ratio; for example, raw deltas `(1, 2)` no longer quantize the
magnitude first and return a non-unit direction.

`Vector2d.TryGetDistance(start, end, out distance)` and
`Vector3d.TryGetDistance(start, end, out distance)` provide the corresponding
full-domain endpoint-distance contract. They round the final Euclidean distance
to the nearest representable `Fixed64`; when that positive distance cannot be
represented, they return `false` and set `distance` to `Fixed64.MaxValue`.

`Vector2d.Lerp` and its in-place form now match the existing full-domain scalar
and 3D interpolation contract: each component retains the complete endpoint
difference until the final nearest-even result. Raw values can therefore change
where the v6 implementation saturated `end - start` or rounded two weighted
products independently.

Circle/sphere point, pair, area, and box predicates now compare exact wide
squared distances against exact radius sums. `Vector2d.CheckDistance` and
`Vector3d.CheckDistance` use the same full-domain contract. Inclusive, strict,
zero-radius, and negative-threshold behavior is unchanged, but classifications
can change where v6 saturated a distance, radius sum, square, or derived bound.

`FixedBoundCircle.RadiusSquared` and `FixedBoundSphere.RadiusSquared` were
removed. A `Fixed64` cannot represent the square of every valid radius, so
those properties could not honor their advertised domain. Keep the actual
radius and use the exact bound predicates when making spatial decisions.

`FixedRay.Intersects(FixedBoundSphere)` and
`FixedRay2d.Intersects(FixedBoundCircle)` now evaluate offset differences,
quadratic products, discriminants, and first-root ordering without saturation.
New first-hit overloads bound the accepted ray parameter and can expand the
target radius in wide arithmetic:

```csharp
Fixed64? hit = ray.Intersects(targetSphere, sourceRadius, maxParameter);
```

The expansion must be nonnegative. Its exact sum with the target radius may
exceed `Fixed64.MaxValue`; this is useful for Minkowski-expanded sweeps without
pre-saturating the geometry decision.

Callers that need both radial roots can use `TryGetIntersectionInterval` on the
same ray types. These methods return the closed overlap interval clipped to an
explicit non-negative maximum parameter and offer overloads that retain radius
expansion exactly instead of pre-adding two saturating `Fixed64` radii.

Bounded capsule and finite-cylinder traversal now has the same direct ray-space
contract. Use `TryGetCapsuleIntersectionInterval` on `FixedRay2d` or `FixedRay`,
and `TryGetFiniteCylinderIntersectionInterval` on `FixedRay`. These methods clip
directly to `[0, maxParameter]`; they do not normalize a long traversal into a
segment parameter and multiply the rounded result back into distance. A
normalized direction therefore preserves physical-distance raw units even on a
very long ray. Endpoint-axis, centered-axis, radial-expansion, and 3D affine
cylinder forms mirror the segment families. Advanced overloads report inclusive
origin containment and strict containment at the maximum parameter.

Use `FixedMath.TryGetCircleCrossSectionRadius` when reducing a sphere at a
signed plane offset. It retains the difference of squares and square root in
wide arithmetic through the final nearest-even radius, returning `false` only
when the plane does not intersect the sphere. Negative source radii throw.

Use `FixedMath.TryGetSphereSlabCrossSectionRadius` when projecting a sphere
through a centered finite slab. It keeps opposite-domain center separation and
the slab offset exact before reducing to the nearest cross-section radius.

### Full-Domain Bounding-Sphere Construction

`FixedBoundSphere.CreateFromBoundingBox`, `CreateFromFrustum`,
`CreateFromPoints`, and `CreateMerged` now keep construction arithmetic wide
until the final Q32.32 center and radius. Extreme-pair ordering no longer
collapses when multiple squared distances saturate, same-sign endpoint
midpoints no longer saturate before halving, and merge radius sums are halved
before representability is decided.

Successful factories return a deterministic sphere that contains every supplied
point or bound. Non-integral required radii round outward rather than to nearest,
which can increase a v7 radius by one raw unit relative to a rounded Euclidean
distance. If the chosen deterministic construction needs a radius outside the
`Fixed64` range, the factory throws `OverflowException`; it never returns a
saturated under-bound result.

`CreateFromPoints` and `CreateFromFrustum` remain deterministic Ritter-style
builders, and a merged center must be selected from the Q32.32 coordinate
lattice. These methods therefore promise containment, not a mathematically
minimum enclosing sphere. Refresh golden values that previously depended on
intermediate saturation or nearest-radius rounding.

### Full-Domain Derived Bounds

`FixedBoundArea` and `FixedBoundBox` now derive centers and extents without
saturating endpoint addition or subtraction. `Center` uses a nearest-even
Q32.32 midpoint. `Size` / `Proportions` return the exact endpoint span and throw
`OverflowException` when a positive component cannot fit in `Fixed64`.
`Scope` rounds an odd raw-unit span outward so it never under-represents the
stored endpoints; only a half-extent outside the positive scalar domain throws.

This also changes centered construction and mutation at raw-unit boundaries:

- odd raw-unit sizes expand by one raw unit when divided into symmetric
  half-extents;
- recentering an odd-span bound can expand that axis by one raw unit so the
  requested lattice center remains exact;
- `Fixed64.MinValue` is valid as a size component because its magnitude can be
  halved before representability is decided, but it is not valid as a scope
  component because its positive magnitude is unrepresentable; and
- centered factories, center/size setters, `Resize`, `Orient`, and
  `SetBoundingBox` validate all endpoints before committing. They throw
  `OverflowException` without partially mutating an existing bound when an
  endpoint would leave the scalar domain.

Code that intentionally models only the representable portion of geometry
extending beyond the Q32.32 coordinate domain must opt into
`FromCenterAndSizeClippedToDomain` or
`FromCenterAndScopeClippedToDomain`. The explicit name distinguishes domain
clipping from an exact centered bound; the ordinary factories no longer clip
silently. `FixedBoundCircle.Bounds` deliberately uses the clipped contract so
a circle crossing a scalar face still returns the AABB of its representable
domain intersection.

Audit callers that assumed every stored min/max interval had a representable
full-size vector, or that expected odd raw-unit sizes to round inward.
`FixedRange.MidPoint` now uses the same full-domain midpoint contract, and
`FixedRange.Length` throws rather than saturating an unrepresentable signed
endpoint difference. Spatial indexes should use
`FixedBoundBox.GetVolumeExpansionCost` instead of multiplying derived
`Proportions`: it retains both union volumes in exact unsigned 192-bit
arithmetic and clamps only the final integer heuristic to `long.MaxValue`.

### Canonical Oriented Boxes

`FixedOrientedBox` is the new invariant-bearing 3D oriented-box primitive. It
stores only a center, normalized orientation, and strictly positive local
half-extents. Construction rejects non-normalized quaternions and non-positive
half-extents; `default(FixedOrientedBox)` is deliberately invalid.

The public feature API stays center-relative. `GetLocalCorner` and
`GetLocalSupportPoint` select stable local features, while
`TryMaterializeLocalPoint` performs the one final exact local-to-world
conversion only when a world-space witness is actually needed.
`GetBoundsClippedToDomain` computes analytical extents without materializing or
deforming saturated world corners.

`Contains`, `TryGetClosestPointOnSurface`, and `GetNearestFaceNormal` use one
exact scale-invariant rational basis derived from the stored quaternion's raw
components. `GetAxes` exposes nearest-even `Fixed64` views of those conceptual
axes; the rounded views are not reused for classification. Support ties retain
the lower corner index, and nearest-face ties select X, then Y, then Z.
Equality remains structural: `q` and `-q` produce identical geometry but remain
distinct stored orientations.

Materialization rounds a conceptual local-to-world result once to its nearest
even Q32.32 lattice point. Because a conceptual face or corner can lie between
lattice points, a rounded boundary witness is not guaranteed to classify as
contained. Use an inset local point when the materialized result must remain
strictly inside.

Both the standard and Lean packages expose the same oriented-box API.
`FixedOrientedBox` supports constructor-validated JSON serialization but is
intentionally not MemoryPack-annotated: MemoryPack serializes unmanaged structs
as raw memory and therefore cannot enforce the constructor invariant. It also
does not advertise field-based binary serialization for the same reason.
Persist authored center/orientation/half-extents in a validated DTO when a
binary payload is required.

### FixedTransform Local And World Contract

`FixedTransform` no longer hides one mutable matrix behind ambiguous component
properties. Its authoritative state is now local position, normalized local
rotation, and exact signed or zero local scale. World values are derived from
the parent chain.

Migrate properties by intent:

| v6.x surface                 | v7.x local replacement       | v7.x world replacement                            |
| ---------------------------- | ---------------------------- | ------------------------------------------------- |
| `Position`                   | `LocalPosition`              | `WorldPosition` or `TrySetWorldPosition(...)`     |
| `Rotation`                   | `LocalRotation`              | `WorldRotation` or `TrySetWorldPose(...)`         |
| `Scale`                      | `LocalScale`                 | `LossyScale`                                      |
| `LossyScale` alias           | `LocalScale`                 | `LossyScale` with hierarchy-derived semantics     |
| `EulerAngles`                | `LocalEulerAngles`           | No writable world-Euler alias                     |
| Writable `Parent`            | `SetParentKeepingLocal(...)` | `TrySetParentKeepingWorld(...)`                   |
| `new FixedTransform(matrix)` | Component constructor        | `TryCreateFromLocalMatrix(...)` for strict import |

For example:

```csharp
// v6.x: names and parent-assignment behavior were ambiguous.
transform.Position = localPosition;
transform.Parent = parent;

// v7.x: local mutation and reparenting intent are explicit.
transform.LocalPosition = localPosition;
transform.SetParentKeepingLocal(parent);

// Use this instead when the current world pose must be preserved.
bool reparented = transform.TrySetParentKeepingWorld(parent);
```

`Parent` is read-only. Reparenting rejects self/ancestor cycles.
`TrySetParentKeepingWorld`, `TrySetWorldPosition`, and `TrySetWorldPose` commit
atomically only when the required inverse and strict TRS decomposition succeed.

The v6 matrix constructor accepted arbitrary matrices. In v7, import is explicit
and strict:

```csharp
if (!FixedTransform.TryCreateFromLocalMatrix(
        localMatrix,
        out FixedTransform? transform,
        parent))
{
    // Perspective, shear, singular/zero scale, unrepresentable, or
    // non-round-trippable matrix.
}
```

Prefer the component constructors at engine-adapter boundaries. They preserve
signed and zero authored local scale without asking FixedMathSharp to infer
components from a matrix. Calls using named constructor arguments must also
rename `position`, `rotation`, and `scale` to `localPosition`, `localRotation`,
and `localScale`.

`LocalToWorldMatrix`, `WorldPosition`, and `LossyScale` traverse parents on
read. `FixedTransform` does not own a scene graph, child collection, matrix
cache, or engine object.

The new `LocalPositionXZ`, `LocalRotationXZRadians`, `LocalScaleXZ`,
`WorldPositionXZ`, and `WorldRotationXZRadians` helpers embed planar `(x, y)` as
3D `(x, 0, y)`. Local position and scale setters preserve the existing Y
component. Positive planar rotation matches `Vector2d.Rotate`; local and world
angle getters report the projected local-right direction modulo `Fixed64.TwoPi`.
Use `LossyScale.ToVector2d()` only when a canonical hierarchy-derived X/Z scale
view fits the consuming system.

### Matrix Scale Naming And Strict Decomposition

Matrix scale APIs now distinguish unsigned basis magnitudes from a canonical
signed lossy view:

| v6.x surface                         | v7.x replacement                             |
| ------------------------------------ | -------------------------------------------- |
| `Fixed3x3.ExtractScale(...)`         | `Fixed3x3.ExtractScaleMagnitudes(...)`       |
| `matrix3x3.ExtractScale()`           | `matrix3x3.ExtractScaleMagnitudes()`         |
| `Fixed4x4.ExtractScale(...)`         | `Fixed4x4.ExtractScaleMagnitudes(...)`       |
| `matrix4x4.ExtractScale()`           | `matrix4x4.ExtractScaleMagnitudes()`         |
| `Fixed4x4.Scale`                     | `Fixed4x4.LossyScale`                        |
| `Fixed3x3.SetLossyScale(scale)`      | `Fixed3x3.CreateScale(scale)`                |
| `Fixed3x3.SetScale(...)`             | `Fixed3x3.CreateScale(...)` for pure scale   |
| `Fixed3x3.ResetScaleToIdentity(...)` | Own rotation and scale components explicitly |
| `Fixed3x3.SetGlobalScale(...)`       | Own rotation and scale components explicitly |
| `Fixed4x4.SetScale(...)`             | Checked `Decompose` + `CreateTransform`      |
| `Fixed4x4.ResetScaleToIdentity(...)` | Checked `Decompose` + `CreateTransform`      |
| `Fixed4x4.SetGlobalScale(...)`       | Checked `Decompose` + `CreateTransform`      |

`ExtractScaleMagnitudes` always returns nonnegative basis magnitudes.
`ExtractLossyScale` and `Fixed4x4.LossyScale` now derive basis magnitudes and
assign an odd reflection's canonical negative sign to X. They no longer return
the matrix diagonal. If a caller genuinely needs diagonal entries, read `M11`,
`M22`, and `M33` explicitly and do not label them scale.

Callers that cannot accept a saturated basis magnitude should use
`Fixed4x4.TryExtractLossyScale`. It preserves the same canonical reflection
contract, accepts singular, sheared, and non-affine bases, and returns `false`
with zero only when a final row magnitude is not representable.
`FixedTransform.TryGetLossyScale` combines this with strict hierarchy
composition. `Fixed4x4.TryTransformAffinePoint` similarly rejects non-affine
matrices and computes each row-vector point coordinate with one exact
round-half-to-even conversion, returning zero atomically when any coordinate is
not representable.

For rigid-frame geometry that also applies a local scale, use
`FixedQuaternion.TryTransformScaledPoint` in 3D or
`Vector2d.TryTransformScaledPoint` in 2D. Their overloads retain
`origin + rotation * (localPoint * scale)` as one wide operation and return
`false` only when a final coordinate is outside the scalar domain; they avoid
the intermediate saturation of separately scaling, rotating, and translating.

`SetScale` and `ResetScaleToIdentity` only overwrote diagonal entries and
corrupted rotated bases. Construct a pure scale matrix with `CreateScale`.
`SetGlobalScale` was also removed: a matrix value has no parent or global
context, and these mutation APIs had no failure channel for reflections, shear,
or singular rows. For 3x3 transforms, keep authored rotation and scale as
explicit components instead. `Fixed3x3.GetNormalized` is not a general
reflected-scale remover.

`Fixed4x4.Decompose(...)` keeps its Boolean signature but now returns `false`
for matrices that are not valid affine orthogonal TRS values. Check the result:

```csharp
if (!Fixed4x4.Decompose(matrix, out Vector3d translation,
        out FixedQuaternion rotation, out _))
{
    throw new InvalidOperationException("Matrix is not a valid affine TRS value.");
}

Fixed4x4 rebuilt = Fixed4x4.CreateTransform(
    translation,
    rotation,
    desiredScale); // Use Vector3d.One to remove scale.
```

### Full-Domain Segment And Triangle Geometry

The existing 2D/3D segment and triangle query surfaces now preserve exact raw
endpoint differences, predicates, ratios, and distance ordering until the final
public conversion. This fixes extreme-coordinate and near-degenerate cases but
can change raw results relative to v6.

New segment APIs replace downstream line/segment solvers:

```csharp
var first2d = new FixedSegment2d(a2d, b2d);
var second2d = new FixedSegment2d(c2d, d2d);

bool unique = first2d.TryGetUniqueIntersection(
    second2d,
    out Fixed64 firstParameter,
    out Fixed64 secondParameter);

(Vector2d firstPoint, Vector2d secondPoint) =
    first2d.GetClosestPoints(second2d);
```

`Vector3d.ClosestPointsOnTwoLines(...)` was removed because it actually solved
finite segments. Replace it with the accurately owned API:

```csharp
// v7.x
var first = new FixedSegment(firstStart, firstEnd);
var second = new FixedSegment(secondStart, secondEnd);
(Vector3d firstPoint, Vector3d secondPoint) = first.GetClosestPoints(second);
```

Finite-axis intersections should also move to the segment-owned interval APIs
instead of normalizing an axis or building quadratic coefficients downstream:

```csharp
bool capsuleHit = query.TryGetCapsuleIntersectionInterval(
    capsuleAxis,
    capsuleRadius,
    out Fixed64 capsuleEntry,
    out Fixed64 capsuleExit);

bool centeredCapsuleHit = query.TryGetCapsuleIntersectionInterval(
    capsuleCenter,
    normalizedCapsuleAxis,
    capsuleAxisLength,
    capsuleRadius,
    radiusExpansion,
    out Fixed64 centeredCapsuleEntry,
    out Fixed64 centeredCapsuleExit);

bool cylinderHit = query.TryGetFiniteCylinderIntersectionInterval(
    cylinderAxis,
    cylinderRadius,
    out Fixed64 cylinderEntry,
    out Fixed64 cylinderExit);

bool centeredCylinderHit = query.TryGetFiniteCylinderIntersectionInterval(
    cylinderCenter,
    normalizedCylinderAxis,
    cylinderAxisLength,
    cylinderRadius,
    radialExpansion,
    axialExpansion,
    out Fixed64 centeredEntry,
    out Fixed64 centeredExit);
```

Expanded overloads accept authored radius and expansion separately. The
centered finite-cylinder overload accepts the positive full authored axis
length and a separate axial expansion, avoiding a narrowed combined radius or
expanded cap center. Capsule zero lengths reduce to a circle or sphere;
cylinder zero axes throw because the segment cannot retain a cap normal.
Returned closed `[0, 1]` parameters are rounded half to even, so deterministic
query goldens produced by an older downstream quadratic should be regenerated.

When the original query is a ray plus a finite travel budget, keep it in ray
space instead of constructing a segment solely to reuse these methods:

```csharp
bool rayCapsuleHit = ray.TryGetCapsuleIntersectionInterval(
    capsuleCenter,
    normalizedCapsuleAxis,
    capsuleAxisLength,
    capsuleRadius,
    radiusExpansion,
    maxParameter,
    out Fixed64 entryDistance,
    out Fixed64 exitDistance,
    out bool originContained,
    out bool maximumContainedStrict);
```

With a normalized ray direction the two returned parameters are physical
distances. With any other direction they remain ordinary ray parameters.
Use the centered capsule or cylinder overload when a physical center and
normalized axis are the source of truth, especially near `Fixed64.MinValue` or
`Fixed64.MaxValue`. Neither contract constructs cap centers, so scalar
saturation cannot silently shorten or rotate the finite axis. A centered
capsule accepts zero length as the circle/sphere limit; a centered cylinder
requires a positive full length. Invalid zero or non-unit axis directions are
rejected.

`FixedBoundArea.FromCenteredCapsuleClippedToDomain` and
`FixedBoundBox.FromCenteredCapsuleClippedToDomain` derive tight analytical 2D
and 3D axis-aligned bounds from that full-length capsule contract. Exact
conceptual endpoints are rounded outward and clipped only at the final scalar
domain boundary.

Centered capsule containment, closest-feature direction, and surface
reconstruction are available through
`ContainsPointInCenteredCapsule`, `GetDirectionFromCenteredAxis`, and
`TryGetSurfacePointOnCenteredCapsule`. The surface helper accepts an explicit
normalized radial direction so callers can choose the otherwise non-unique
on-axis result. It fuses the conceptual axis point and radial offset before one
final half-to-even conversion, returning `false` only when the final surface
coordinate cannot be represented.

Callers that previously cached cap centers or reconstructed convex support
points should keep center, normalized axis, full length, and radius as the
canonical state. Use `TryGetCenteredAxisEndpoint` for an explicitly selected
cap and the centered capsule, finite-cylinder, or finite-cone `Try*Support`
helper for a final support witness. These helpers preserve odd raw lengths and
return `false` atomically when the selected world point is not representable.

Centered capsule containment has explicit `strict` overloads in 2D and 3D;
strict mode excludes the cylindrical side and both rounded-cap boundaries while
retaining the optional radial-expansion contract. Centered finite cylinders
provide the matching `ContainsPointInCenteredFiniteCylinder(..., strict)`
helper, where strict mode also excludes both flat caps.

`GetDistanceToCenteredCapsule` and `TryGetDistanceToCenteredCapsule` provide
the matching exact distance contract without reconstructing a surface point.
They return zero for points inside or on the capsule and round a positive gap
half to even only at the final `Fixed64` conversion. The `Get` form saturates
an unrepresentable final distance to `Fixed64.MaxValue`; the `Try` form returns
`false` and also writes `Fixed64.MaxValue`.

`FixedSegment`, `FixedSegment2d`, `FixedTriangle`, and `FixedTriangle2d` now use
one final nearest-even conversion and final-only saturation for their hardened
query results. Triangle barycentric weights are calculated independently;
degenerate closest-point ties retain stable AB, BC, CA order. No source change
is normally required, but collision, containment, and deterministic golden tests
should be rerun.

### Chronicler Transform Hash Boundary

`FixedMathSharp.Chronicler.WriteTransform` now hashes authoritative local
position, local rotation, and local scale in that order. Parent identity and all
derived world views are excluded.

This is an intentional replay/hash compatibility boundary. Version stored
replays or regenerate v7 golden hashes rather than expecting hashes produced
from the v6 matrix-backed transform representation to match.

### Suggested Search Patterns for v7 Migration

After updating package references, these searches catch the main v7 migration
work:

```bash
rg -n "new FixedTransform\s*\(" src tests
rg -n "\.(Position|Rotation|Scale|EulerAngles|PositionXZ|RotationXZRadians|ScaleXZ)\b" src tests
rg -n "\.Parent\s*=" src tests
rg -n "ExtractScale|SetLossyScale|\.Scale\b" src tests
rg -n "ClosestPointsOnTwoLines" src tests
rg -n "AddOverflowHelper" src tests
rg -n "WriteTransform" src tests
```

Review property matches by intent instead of mechanically adding `Local`.
Physics and simulation state often wants local components, while rendering,
queries, and hierarchy-aware bounds may want derived world values.

---

## Migrating From v5.x To v6.x

FixedMathSharp v6.x is a geometry and bounds hardening release. The largest
change is dimensional clarity: `FixedBoundBox` is the 3D AABB type, while
`FixedBoundArea` is now a true `Vector2d` 2D AABB. The release also removes
ambiguous bounds construction, removes hidden corner-array storage, adds shared
2D/3D segment and triangle primitives, and adds an optional
`FixedMathSharp.Chronicler` companion package for deterministic replay hash
writers.

Use this guide when upgrading from any v5.x package.

### v6 Upgrade Checklist

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

### Dimensional Bounds Split

`FixedBoundArea` no longer represents a 3D shape. It is now a normalized 2D
axis-aligned area backed by `Vector2d`:

```csharp
FixedBoundArea area = FixedBoundArea.FromMinMax(
    new Vector2d(-4, -2),
    new Vector2d(4, 2));
```

Migrate old 3D area usage by intent:

| v5.x usage                                         | v6.x replacement                                                                          |
| -------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| 3D volume, collider, frustum, ray, or plane bounds | `FixedBoundBox`                                                                           |
| Flat footprint in a 3D world                       | `FixedBoundArea` plus explicit layer, height, or elevation state in the consuming package |
| Pure 2D area query or broad-phase bounds           | `FixedBoundArea`                                                                          |

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

### FixedBoundBox Construction

The public `FixedBoundBox(Vector3d center, Vector3d size)` constructor was
removed because call sites could not tell whether two vectors meant center/size,
center/scope, or min/max. Use named factories:

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

### Allocation-Free Box Corners

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

### Intersection Semantics

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

### Radius And Serialized Bounds State

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

### New Geometry Primitives

v6.x adds reusable deterministic geometry primitives that downstream packages
can use instead of local one-off structs:

| Domain             | New type           | Main use                                                                        |
| ------------------ | ------------------ | ------------------------------------------------------------------------------- |
| 2D area bounds     | `FixedBoundArea`   | Planar AABB, clamp/project, union, area overlap                                 |
| 2D circular bounds | `FixedBoundCircle` | Radius queries, circle/area overlap, projection                                 |
| 2D rays            | `FixedRay2d`       | Planar ray against area/circle                                                  |
| 2D segments        | `FixedSegment2d`   | Finite edge, closest point, distance, bounds                                    |
| 2D triangles       | `FixedTriangle2d`  | Area, bounds, containment, closest point, barycentric weights                   |
| 3D segments        | `FixedSegment`     | Finite 3D edge, closest point, distance, bounds                                 |
| 3D triangles       | `FixedTriangle`    | Normal, area, bounds, containment, closest point, projected barycentric weights |

Segments preserve ordered endpoint identity. Reversed endpoints have the same
bounds but do not compare equal.

Rays do not normalize direction by construction. A returned ray parameter is a
physical distance only when the caller supplied a normalized direction.

Triangles preserve ordered vertices. `FixedTriangle2d.TryGetBarycentricWeights`
solves planar weights directly.
`FixedTriangle.TryGetProjectedBarycentricWeights` names the 3D projection
behavior explicitly.

`Vector2d.BarycentricCoordinates(...)` now mirrors
`Vector3d.BarycentricCoordinates(...)` for reconstructing points from known B/C
barycentric weights.

### Chronicler Hash Companion Package

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

### Suggested Search Patterns for v6 Migration

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
still valid, while the old center/size constructor should become a named factory
call.

Review each `FixedBoundArea` match by dimension. If the surrounding code uses
`Vector3d`, a ray/plane/frustum, or volumetric bounds, it probably wants
`FixedBoundBox`. If the code is planar, migrate to the new `Vector2d` area.

## Migrating From v4.x To v5.0.0

FixedMathSharp v5.0.0 is a major API hardening release. The migration is mostly
source-level cleanup, but several changes affect numeric interpretation,
transform semantics, and public names.

Use this guide when upgrading from v4.0.1 or earlier.

### Upgrade Checklist

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

### Geometry Type Renames

The bounds types now use the same `Fixed*` naming style as the rest of the
library.

| v4.x              | v5.0.0               |
| ----------------- | -------------------- |
| `BoundingBox`     | `FixedBoundBox`      |
| `BoundingSphere`  | `FixedBoundSphere`   |
| `BoundingArea`    | `FixedBoundArea`     |
| `BoundingFrustum` | `FixedBoundFrustum`  |
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

### Fixed64 Value And Raw Conversions

`Fixed64` now makes value-space and raw-payload conversions explicit.

#### Decimal Text Vs Raw Text

In v4.x, `Fixed64.Parse` and `TryParse` interpreted text as a raw Q32.32 `long`
payload. In v5.0.0, they parse normal decimal value text.

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

#### Constructor And Factory Changes

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

#### Raw Longs Vs Integer Longs

The `Fixed64` arithmetic operators that accepted `long` operands were removed.
Those overloads were ambiguous because a `long` can mean either a normal integer
value or an already-scaled raw Q32.32 payload.

```csharp
long tileCount = 5;
long rawStep = 1;

Fixed64 integerValue = (Fixed64)tileCount;
Fixed64 rawValue = Fixed64.FromRaw(rawStep);
```

The explicit `long` conversion now saturates to `Fixed64.MinValue` or
`Fixed64.MaxValue` when the source integer is outside the representable Q32.32
whole-number range.

### Fixed-Point Boundary APIs

Several cross-domain helpers were removed so deterministic code stays in
fixed-point land.

| v4.x                               | v5.0.0                                                               |
| ---------------------------------- | -------------------------------------------------------------------- |
| `FixedRange.InRange(double)`       | Convert once with `Fixed64.FromDouble`, then call `InRange(Fixed64)` |
| `DeterministicRandom.NextDouble()` | Use `NextFixed6401()` or `NextFixed64(...)`                          |
| `Fixed64.RawToString()`            | `Fixed64.ToRawString()`                                              |
| `Fixed64.RawToInt(...)`            | `Fixed64.ToInt(...)`                                                 |

Keep floating-point conversion at engine, UI, editor, or import/export
boundaries. Core simulation code should pass `Fixed64` values directly.

### Scalar Algorithm Ownership

`FixedMath` is the canonical home for scalar algorithms such as interpolation,
powers, logarithms, trigonometry, square root, rounding, and clamping. `Fixed64`
owns representation, constants, conversions, parsing, operators, equality, and
raw helpers.

For v4 call sites, the verified scalar rename is:

| v4.x               | v5.0.0              |
| ------------------ | ------------------- |
| `value.ToDegree()` | `value.ToDegrees()` |

For new or refactored scalar interpolation code, prefer the `FixedMath` static
surface, such as `FixedMath.Lerp`, `FixedMath.CatmullRom`,
`FixedMath.HermiteSpline`, and `FixedMath.BarycentricCoordinate`. The fluent
extension surface is curated and forwards to the canonical implementation.
Factories and convention-heavy methods stay on the owning type.

### Vector API Cleanup

Vector mutation and value-returning APIs now use one naming model.

| v4.x                                           | v5.0.0                                                 |
| ---------------------------------------------- | ------------------------------------------------------ |
| `vector.x`, `vector.y`, `vector.z`, `vector.w` | `vector.X`, `vector.Y`, `vector.Z`, `vector.W`         |
| `vector.Normal`                                | `vector.Normalized`                                    |
| `vector.Normalize()`                           | `vector.NormalizeInPlace()`                            |
| `Vector*d.Normalize(value)` where applicable   | `Vector*d.GetNormalized(value)`                        |
| `SqrMagnitude`                                 | `MagnitudeSquared`                                     |
| `SqrDistance(...)`                             | `DistanceSquared(...)`                                 |
| `Vector2d.Lerped(...)`                         | `Vector2d.Lerp(...)`                                   |
| `ScaleInPlace(...)`                            | `MultiplyInPlace(...)`                                 |
| Public vector-result `out Vector*d` helpers    | Return-by-value statics or explicit `*InPlace` methods |

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

`Normalize(out Fixed64 magnitude)` remains as
`NormalizeInPlace(out Fixed64 magnitude)` because it returns a second scalar
result.

### Matrix And Transform Semantics

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

### Coordinate Convention Helpers

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

### Bounds And Hot-Path Helpers

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

### Diagnostics Formatting

Human-readable formatting is now separated from raw payload representation:

- Use `ToString(...)` or `TryFormat(...)` for logs, editor display, and
  diagnostics.
- Use `ToRawString`, `ParseRaw`, and `TryParseRaw` for raw Q32.32 payload text.
- Use MemoryPack or JSON support for structured serialization.

On `net8.0`, supported types implement `ISpanFormattable`. On `netstandard2.1`,
the same `TryFormat` method shape is exposed where the interface itself is
unavailable.
